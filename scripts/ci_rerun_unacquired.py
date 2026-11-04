#!/usr/bin/env python3
"""Re-runs the Build all runs that failed only because no hosted runner was free.

GitHub accepts a job, cannot find a hosted runner for it within its queue limit,
and cancels it; the run turns red although nothing in the repository is broken.
The job records why in its check-run annotations, as a failure annotation reading
"The job was not acquired by Runner of type hosted even after multiple attempts"
and notices about macOS arm64 capacity. That annotation, not the job's status, is
the test here: a cancelled job also appears when another job in the same run
failed first and the run cancelled the rest, and those runs must be left alone.

A rerun costs no review and no code change, so a schedule can make it. Each run
is attempted at most --max-attempts times: every rerun increments run_attempt, so
the count is what stops a run whose failure has nothing to do with capacity from
being rerun for ever. The window bounds the work a scheduled pass does, and the
API is read newest first, so a pass stops at the first run older than the window.

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


def parse_time(value):
    return datetime.datetime.fromisoformat(value.replace("Z", "+00:00"))


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


def list_runs(api, repo, workflow, cutoff):
    """Runs of one workflow created since cutoff, newest first."""
    runs, page = [], 1
    while page <= MAX_PAGES:
        body = api.get(f"repos/{repo}/actions/workflows/{workflow}/runs",
                       per_page=PER_PAGE, page=page)
        batch = body.get("workflow_runs") or []
        if not batch:
            break
        runs += [run for run in batch if parse_time(run["created_at"]) >= cutoff]
        # The list is newest first, so a page ending before the window ends the read.
        if parse_time(batch[-1]["created_at"]) < cutoff:
            break
        page += 1
    else:
        print(f"::warning title=rerun-unacquired::Stopped after {MAX_PAGES} pages of runs.", flush=True)
    return runs


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


def unacquired(run, api, repo, errors):
    """The names of the run's jobs whose annotations say no runner was found."""
    names = []
    for job in list_jobs(api, repo, run["id"]):
        if job.get("conclusion") not in FAILED:
            continue
        try:
            notes = annotations(api, repo, job["id"])
        except urllib.error.HTTPError as error:
            errors.append({"run_id": run["id"], "job_id": job["id"], "error": describe(error)})
            continue
        for note in notes:
            message = note.get("message") or ""
            if NOT_ACQUIRED in message:
                print(f"    job {job['id']} {job['name']}: {message}", flush=True)
                names.append(job["name"])
                break
    return names


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Re-run workflow runs cancelled because no hosted runner was acquired.")
    parser.add_argument("--repo", default=os.environ.get("GITHUB_REPOSITORY"),
                        help="owner/name (default: GITHUB_REPOSITORY)")
    parser.add_argument("--workflow", default="build_all.yml",
                        help="workflow file name (default: build_all.yml)")
    parser.add_argument("--window-hours", type=float, default=6,
                        help="how far back to look, in hours (default: 6)")
    parser.add_argument("--max-attempts", type=int, default=2,
                        help="rerun runs already attempted at most this many times (default: 2)")
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

    errors, candidates, rerun = [], [], []
    try:
        runs = list_runs(api, args.repo, args.workflow, cutoff)
    except urllib.error.HTTPError as error:
        runs = []
        errors.append({"error": f"listing runs: {describe(error)}"})

    print(f"{len(runs)} {args.workflow} run(s) since {cutoff:%Y-%m-%dT%H:%M:%SZ}"
          f"{', dry run' if args.dry_run else ''}", flush=True)

    for run in runs:
        if run.get("conclusion") not in FAILED:
            continue
        attempt = run.get("run_attempt", 1)
        if attempt > args.max_attempts:
            print(f"  skip {run['id']}: attempt {attempt} is over {args.max_attempts}", flush=True)
            continue
        print(f"  run {run['id']} (attempt {attempt}, {run['conclusion']},"
              f" {run.get('head_branch')})", flush=True)
        try:
            names = unacquired(run, api, args.repo, errors)
        except urllib.error.HTTPError as error:
            errors.append({"run_id": run["id"], "error": f"listing jobs: {describe(error)}"})
            continue
        if not names:
            print("    no runner-acquisition annotation; left alone", flush=True)
            continue
        candidates.append({"run_id": run["id"], "attempt": attempt, "jobs": names})

    for candidate in candidates:
        run_id = candidate["run_id"]
        if args.dry_run:
            print(f"  would rerun {run_id}: {', '.join(candidate['jobs'])}", flush=True)
            continue
        try:
            status = api.post(f"repos/{args.repo}/actions/runs/{run_id}/rerun-failed-jobs")
        except urllib.error.HTTPError as error:
            errors.append({"run_id": run_id, "error": f"rerun: {describe(error)}"})
            print(f"  rerun {run_id}: {describe(error)}", flush=True)
            continue
        rerun.append({"run_id": run_id, "http_status": status})
        print(f"  rerun {run_id}: HTTP {status}", flush=True)

    print(json.dumps({
        "scanned": len(runs),
        "candidates": candidates,
        "rerun": rerun,
        "errors": errors,
        "api_calls": api.calls,
        "rate_limit_remaining": api.remaining,
    }, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
