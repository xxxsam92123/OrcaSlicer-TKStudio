#!/usr/bin/env python3
"""Re-runs the Build all runs that failed only because no hosted runner was free.

GitHub accepts a job, cannot find a hosted runner for it within its queue limit,
and cancels it; the run turns red although nothing in the repository is broken.
The job records why in its check-run annotations, as a failure annotation reading
"The job was not acquired by Runner of type hosted even after multiple attempts"
and notices about macOS arm64 capacity. That annotation, not the job's status, is
the test here: a cancelled job also appears when another job in the same run
failed first and the run cancelled the rest, and those runs must be left alone.

A run is only a candidate when *every* failed job in it carries the annotation.
rerun-failed-jobs reruns all failed jobs of the run, so one genuinely failing job
among the cancelled ones -- 40341179917 has five, failing to download an artifact
-- would be rerun too, hide the failure behind another red run, and burn machine
time for nothing. The annotation alone does not say the run is innocent; the whole
set of failed jobs does.

A rerun costs no review and no code change, so a schedule can make it. Each run is
attempted at most --max-attempts times: run_attempt counts attempts and starts at
one, so the default of two leaves one rerun per run. A run at that count is left
alone, which is what stops a run whose failure has nothing to do with capacity
from being rerun for ever.

A pass runs every twenty minutes and carries nothing between passes. A small
ledger in the actions cache was tried and dropped: an entry keyed by this
workflow's own run id is invisible to every cleanup the repository has --
prune_ci_caches.yml keeps only the build cache prefixes, and its run-tail pattern
does not match a bare run id, so an entry is kept for ever -- so one entry per pass
accumulates until the pool is full, and the prune's ten-page scan stops seeing the
old ones without saying so. The state a pass needs is instead recomputed from
GitHub on every pass, and a rerun is bounded four ways:

  * run_attempt is the durable counter, checked before anything else;
  * the run is read again immediately before the POST, and is skipped unless it
    is still completed, still failed, and still at the attempt count this pass
    saw -- a run that moved on while the pass was reading jobs is not touched;
  * it is skipped unless its updated_at is at least --min-age-minutes old, which
    defaults to sixty, three times the twenty-minute beat. A rerun moves
    updated_at, so this is the backoff that stands in where the ledger stood: a
    run asked for -- accepted, or refused by a gateway that may still have
    forwarded it -- waits an hour instead of coming back on the next pass;
  * at most --max-reruns-per-pass POSTs are made in one pass, which bounds the
    damage of a wide outage that cancels many runs at once.

A POST is verified, not believed: the run is read once more afterwards and the
attempt count must have grown. GitHub answering 201 without rerunning anything, or
refusing the POST, is reported as an error annotation on the run -- nobody reads
the log of a job that went red for an API answer that looked fine. Neither fails
the job, though: only a pass that could not list the runs at all, or that crashed,
exits non-zero. A twenty-minute schedule that went red on a transient 5xx would be
the same red this workflow exists to remove; the annotation is what a reader sees,
and the next pass, an hour later, asks again. A connection that drops while a run
or a job is being read is a warning for the same reason.

The window bounds the work a scheduled pass does, and the API is read newest
first, so a pass stops at the first run created well before the window -- a run
created before it but updated inside it still counts, within the page margin.

Environment: GH_TOKEN, GITHUB_API_URL and GITHUB_REPOSITORY, the last two for the
same reason ci_macos_admission.py reads them -- so this also works on GHES.
"""

import argparse
import datetime
import json
import os
import sys
import urllib.error
import urllib.parse
import urllib.request

# What GitHub writes on a job it never found a hosted runner for. Matched as a
# substring: the message has carried a runner-type suffix before.
NOT_ACQUIRED = "not acquired by Runner of type hosted"

FAILED = ("failure", "cancelled")

# The run list is read a page at a time. A pass that reaches this many pages
# without crossing its window is reading a different repository than expected,
# and stops rather than paging through the whole history.
MAX_PAGES = 10
PER_PAGE = 100

# A run created just before the window but updated inside it is still worth
# looking at, so the read continues this far past the window and the filter
# accepts either timestamp. The list is ordered by creation, so this margin is
# the only slack there is: a run created long before the window and updated
# inside it is not seen.
SCAN_MARGIN = datetime.timedelta(minutes=30)


def parse_time(value):
    return datetime.datetime.fromisoformat(value.replace("Z", "+00:00"))


def age_minutes(now, timestamp):
    return int((now - parse_time(timestamp)).total_seconds() // 60)


class Api:
    def __init__(self, token, url="https://api.github.com"):
        self.token = token
        self.url = url.rstrip("/")
        # The token's hourly limit is shared by every workflow in the repository,
        # so what the last response says is left shows their use too.
        self.calls = 0
        self.remaining = self.limit = "?"

    def request(self, method, path, **params):
        query = urllib.parse.urlencode(params)
        url = f"{self.url}/{path}" + (f"?{query}" if query else "")
        request = urllib.request.Request(url, method=method, headers={
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {self.token}",
            "X-GitHub-Api-Version": "2022-11-28",
            "Content-Type": "application/json",
        }, data=b"{}" if method == "POST" else None)
        self.calls += 1
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                self.note_rate_limit(response.headers)
                body = response.read()
                return response.status, (json.loads(body) if body.strip() else None)
        except urllib.error.HTTPError as error:
            # A rate-limited request fails, and its headers say so.
            self.note_rate_limit(error.headers)
            raise

    def get(self, path, **params):
        return self.request("GET", path, **params)[1]

    def post(self, path):
        return self.request("POST", path)[0]

    def note_rate_limit(self, headers):
        self.remaining = headers.get("X-RateLimit-Remaining", "?")
        self.limit = headers.get("X-RateLimit-Limit", "?")


def describe(error):
    """One line for an HTTPError, with what a rate limit leaves behind."""
    if not isinstance(error, urllib.error.HTTPError):
        # A timeout or a dropped connection: no status, no headers to read.
        return f"{type(error).__name__}: {error}"
    parts = [f"HTTP {error.code}"]
    if error.code in (403, 429):
        headers = error.headers or {}
        parts.append(f"rate limit left {headers.get('X-RateLimit-Remaining', '?')}")
        parts.append(f"retry-after {headers.get('Retry-After', 'none')}")
    try:
        message = json.loads(error.read() or b"{}").get("message")
    except ValueError:
        message = None
    if message:
        parts.append(message)
    return ", ".join(parts)


def in_window(run, cutoff):
    """Created inside the window, or created before it and updated inside it."""
    if parse_time(run["created_at"]) >= cutoff:
        return True
    updated = run.get("updated_at")
    return bool(updated) and parse_time(updated) >= cutoff


def list_runs(api, repo, workflow, cutoff):
    """Runs of one workflow touched since cutoff, newest created first."""
    runs, page = [], 1
    scan_from = cutoff - SCAN_MARGIN
    while page <= MAX_PAGES:
        body = api.get(f"repos/{repo}/actions/workflows/{workflow}/runs",
                       per_page=PER_PAGE, page=page)
        batch = body.get("workflow_runs") or []
        if not batch:
            break
        runs += [run for run in batch if in_window(run, cutoff)]
        # The list is newest first, so a page ending before the window ends the read.
        if parse_time(batch[-1]["created_at"]) < scan_from:
            break
        page += 1
    else:
        print(f"::warning title=rerun-unacquired::Stopped after {MAX_PAGES} pages of runs.",
              flush=True)
    return runs


def dedupe(runs):
    """One entry per run: pages can overlap, and a run may come back twice."""
    seen, unique = set(), []
    for run in runs:
        if run["id"] in seen:
            continue
        seen.add(run["id"])
        unique.append(run)
    return unique


def list_jobs(api, repo, run_id):
    # A build run carries around fifty jobs; the loop is there so a run that
    # grows past a page is read whole rather than silently half-read.
    jobs, page = [], 1
    while page <= MAX_PAGES:
        body = api.get(f"repos/{repo}/actions/runs/{run_id}/jobs",
                       filter="latest", per_page=PER_PAGE, page=page)
        batch = body.get("jobs") or []
        if not batch:
            break
        jobs += batch
        if len(jobs) >= body.get("total_count", len(jobs)):
            break
        page += 1
    return jobs


def annotations(api, repo, job_id):
    body = api.get(f"repos/{repo}/check-runs/{job_id}/annotations", per_page=PER_PAGE)
    # The endpoint answers with a bare list; older API versions wrapped it.
    if isinstance(body, dict):
        return body.get("annotations") or []
    return body or []


def unacquired(run, api, repo, warnings):
    """(names of failed jobs the runner never picked up, number of failed jobs).

    Both halves are needed: a rerun takes every failed job with it, so a run is
    only innocent when the two counts are equal. A job whose annotations cannot
    be read counts as failed but not as a runner problem, which leaves the run
    alone and lets the next pass look again. That read failing is a warning on
    the job, not a failure of the pass.
    """
    names, failed = [], 0
    for job in list_jobs(api, repo, run["id"]):
        if job.get("conclusion") not in FAILED:
            continue
        failed += 1
        try:
            notes = annotations(api, repo, job["id"])
        except urllib.error.URLError as error:
            message = (f"reading annotations of job {job['id']} in run {run['id']}:"
                       f" {describe(error)}")
            warnings.append({"run_id": run["id"], "job_id": job["id"], "warning": message})
            print(f"::warning title=rerun-unacquired::{message}; run left alone this pass",
                  flush=True)
            continue
        for note in notes:
            message = note.get("message") or ""
            if NOT_ACQUIRED in message:
                print(f"    job {job['id']} {job['name']}: {message}", flush=True)
                names.append(job["name"])
                break
    return names, failed


def rerun_candidate(api, repo, candidate, args, now):
    """POST the rerun unless the run moved on.

    Returns (outcome, attempt_after, problem); outcome is "verified" when the
    attempt count grew, "unverified" when GitHub took the POST and did nothing,
    "rejected" when the POST failed, "skipped" when the guard said not to ask.
    """
    run_id, attempt = candidate["run_id"], candidate["attempt"]
    # Read the run again: it may have been rerun, or moved on, while this pass
    # was reading the annotations of fifty jobs.
    try:
        fresh = api.get(f"repos/{repo}/actions/runs/{run_id}")
    except urllib.error.URLError as error:
        return "rejected", None, f"re-reading the run: {describe(error)}"
    if fresh.get("status") != "completed" or fresh.get("conclusion") not in FAILED:
        return "skipped", None, (f"now {fresh.get('status')}/{fresh.get('conclusion')},"
                                " no longer a finished failed run")
    if fresh.get("run_attempt") != attempt:
        return "skipped", None, (f"attempt moved to {fresh.get('run_attempt')}"
                                 f" since it was read as {attempt}")
    updated = fresh.get("updated_at")
    if updated:
        age = age_minutes(now, updated)
        if age < args.min_age_minutes:
            return "skipped", None, (f"updated {age} min ago,"
                                    f" under --min-age-minutes {args.min_age_minutes}")
    try:
        status = api.post(f"repos/{repo}/actions/runs/{run_id}/rerun-failed-jobs")
    except urllib.error.URLError as error:
        return "rejected", None, f"rerun: {describe(error)}"
    # GitHub answering 201 is not the rerun happening. If the attempt count did
    # not move, say so on the run instead of asking again next pass as though
    # nothing had been sent.
    try:
        after = api.get(f"repos/{repo}/actions/runs/{run_id}")
    except urllib.error.URLError as error:
        return "unverified", None, f"HTTP {status}, but verifying it failed: {describe(error)}"
    attempt_after = after.get("run_attempt")
    if not isinstance(attempt_after, int) or attempt_after <= attempt:
        return "unverified", attempt_after, (f"HTTP {status} but run_attempt stayed"
                                            f" {attempt_after} (was {attempt}): the rerun"
                                            " was accepted and not run")
    return "verified", attempt_after, None


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Re-run workflow runs cancelled because no hosted runner was acquired.")
    parser.add_argument("--repo", default=os.environ.get("GITHUB_REPOSITORY"),
                        help="owner/name (default: GITHUB_REPOSITORY)")
    parser.add_argument("--workflow", default="build_all.yml",
                        help="workflow file name (default: build_all.yml)")
    parser.add_argument("--window-hours", type=float, default=6,
                        help="how far back to look, by creation or by update, in hours (default: 6)")
    parser.add_argument("--max-attempts", type=int, default=2,
                        help="attempts a run may have; run_attempt starts at one, so the"
                             " default of two allows one rerun (default: 2)")
    parser.add_argument("--min-age-minutes", type=int, default=60,
                        help="how long a run must have gone untouched -- updated_at older"
                             " than this -- before it is asked for; three times the */20"
                             " cron beat, and, the passes sharing no state, the only"
                             " backoff there is (default: 60)")
    parser.add_argument("--max-reruns-per-pass", type=int, default=3,
                        help="most reruns one pass will ask for (default: 3)")
    parser.add_argument("--dry-run", action="store_true",
                        help="report what would be rerun and rerun nothing")
    args = parser.parse_args(argv)

    if not args.repo:
        parser.error("--repo or GITHUB_REPOSITORY is required")
    token = os.environ.get("GH_TOKEN")
    if not token:
        parser.error("GH_TOKEN is required")

    api = Api(token, os.environ.get("GITHUB_API_URL", "https://api.github.com"))
    now = datetime.datetime.now(datetime.timezone.utc)
    cutoff = now - datetime.timedelta(hours=args.window_hours)

    warnings, errors, candidates, rerun, held = [], [], [], [], []
    scan_failed = False
    try:
        runs = dedupe(list_runs(api, args.repo, args.workflow, cutoff))
    except urllib.error.URLError as error:
        runs = []
        scan_failed = True
        print(f"::error title=rerun-unacquired::listing runs: {describe(error)}", flush=True)

    print(f"{len(runs)} {args.workflow} run(s) since {cutoff:%Y-%m-%dT%H:%M:%SZ}"
          f"{', dry run' if args.dry_run else ''}", flush=True)

    for run in runs:
        if run.get("conclusion") not in FAILED:
            continue
        attempt = run.get("run_attempt", 1)
        if attempt >= args.max_attempts:
            print(f"  skip {run['id']}: attempt {attempt} has reached {args.max_attempts}",
                  flush=True)
            continue
        updated = run.get("updated_at")
        if updated:
            age = age_minutes(now, updated)
            if age < args.min_age_minutes:
                print(f"  skip {run['id']}: updated {age} min ago, under"
                      f" --min-age-minutes {args.min_age_minutes}", flush=True)
                continue
        print(f"  run {run['id']} (attempt {attempt}, {run['conclusion']},"
              f" {run.get('head_branch')})", flush=True)
        try:
            names, failed = unacquired(run, api, args.repo, warnings)
        except urllib.error.URLError as error:
            message = f"listing jobs of run {run['id']}: {describe(error)}"
            warnings.append({"run_id": run["id"], "warning": message})
            print(f"::warning title=rerun-unacquired::{message}; run left alone this pass",
                  flush=True)
            continue
        if not failed:
            print("    no failed job; left alone", flush=True)
            continue
        if not names:
            print("    no runner-acquisition annotation; left alone", flush=True)
            continue
        if len(names) != failed:
            print(f"    {len(names)} of {failed} failed jobs lost a runner; the other"
                  f" {failed - len(names)} failed for a reason of their own, so a rerun"
                  " would carry them along; left alone", flush=True)
            continue
        candidates.append({"run_id": run["id"], "attempt": attempt, "jobs": names})

    if args.dry_run:
        for candidate in candidates:
            print(f"  would rerun {candidate['run_id']}: {', '.join(candidate['jobs'])}",
                  flush=True)
        if len(candidates) > args.max_reruns_per_pass:
            print(f"  {len(candidates) - args.max_reruns_per_pass} of these would wait:"
                  f" --max-reruns-per-pass {args.max_reruns_per_pass}", flush=True)
    else:
        asked = 0
        for candidate in candidates:
            # Every POST counts, including the ones GitHub refuses: the cap is
            # there to bound how much this pass asks of the API, not only how
            # much it gets.
            if asked >= args.max_reruns_per_pass:
                # The candidates are newest first, so these wait for the next pass.
                held.append({"run_id": candidate["run_id"],
                             "reason": f"--max-reruns-per-pass {args.max_reruns_per_pass}"})
                print(f"  hold {candidate['run_id']}: --max-reruns-per-pass"
                      f" {args.max_reruns_per_pass} reached", flush=True)
                continue
            outcome, attempt_after, problem = rerun_candidate(
                api, args.repo, candidate, args, now)
            if outcome == "skipped":
                # The run moved on, or was asked for too recently: nothing wrong,
                # nothing to do.
                print(f"  skip {candidate['run_id']}: {problem}", flush=True)
                continue
            asked += 1
            if outcome in ("verified", "unverified"):
                rerun.append({"run_id": candidate["run_id"],
                              "attempt": candidate["attempt"], "attempt_after": attempt_after})
            if problem:
                errors.append({"run_id": candidate["run_id"], "error": problem})
                print(f"  rerun {candidate['run_id']}: {problem}", flush=True)
                # A note on the run, because nobody reads the log of a job that
                # went red for an API answer that looked fine. It does not fail
                # the job: a transient 5xx on a twenty-minute schedule must not
                # turn this workflow into the red it exists to clear.
                print(f"::error title=rerun-unacquired::run {candidate['run_id']}:"
                      f" {problem}", flush=True)

    print(json.dumps({
        "scanned": len(runs),
        "candidates": candidates,
        "rerun": rerun,
        "held": held,
        "warnings": warnings,
        "errors": errors,
        "api_calls": api.calls,
        "rate_limit_remaining": api.remaining,
    }, indent=2))
    # Only the scan itself can fail the pass. A job whose annotations could not be
    # read is a warning, and a POST that was refused or not honoured is an
    # annotation on the run; neither is a reason for the schedule to go red.
    return 1 if scan_failed else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:  # noqa: BLE001 - a crash is the other way to fail
        print(f"::error title=rerun-unacquired::crashed: {type(error).__name__}: {error}",
              flush=True)
        sys.exit(1)
