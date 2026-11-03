#!/usr/bin/env bash
# Lets a pull request's macOS deps build, or its clang-tidy job, wait for a
# build that saves the deps cache it needs, from a Linux job. `check` runs after
# a cache miss and names a queued or running push build of the base branch with
# the same deps/ and .github/ to wait for, unless the pull request changes either.
# `check-or-own` does the same for clang-tidy, and names this pull request's own
# build when the base branch has none. `own` names the pull request's own build
# directly, for clang-tidy when the pull request changes deps/. `wait` waits
# until that build saves KEY under CACHE_REF, or its deps job fails.
# actions/cache then confirms the entry can be restored.
#
# Environment: GH_TOKEN, REPO, WORKFLOW_REF (or WORKFLOW, the build workflow's
# file name) and BASE_REF, plus HEAD_SHA for `own` and `check-or-own`, and
# KEY_INPUTS, WAIT_FOR, WAIT_SINCE, WAIT_MINUTES, KEY, CACHE_REF (default the
# base branch) and ARCH (or JOB, the matrix job's name) for `wait`.
# STRICT=1 makes `own`, `check-or-own` and `wait` exit non-zero rather than leave
# the caller to fall back to an older cache: clang-tidy sets it, because it
# cannot parse against deps the pull request has not built.
# Writes key-inputs, wait-for, wait-since and cache-ref to $GITHUB_OUTPUT.
set -uo pipefail

workflow=${WORKFLOW_REF:-}
workflow=${workflow%%@*}
workflow=${WORKFLOW:-${workflow##*/}}

# Must match the job names in build_all.yml, build_check_cache.yml and build_deps.yml.
job=${JOB:-build_macos_arch (${ARCH:-})}
check_job_name="$job / Check Cache"
deps_job_name="$job / Build Deps / Build Deps"

# The build a pull request is waiting for can be created a moment after it, and
# the runs list can lag behind that. Bounded, so a missing build fails the job
# instead of holding a runner.
producer_attempts=${PRODUCER_ATTEMPTS:-10}
producer_delay=${PRODUCER_DELAY:-15}

# Set by check when it named a build; `check-or-own` falls back to `own` when it
# is empty.
selected=

out() { echo "$1" >> "${GITHUB_OUTPUT:-/dev/stdout}"; }
strict() { [ "${STRICT:-}" = 1 ]; }

# A gh failure must not pass for an empty answer: a strict caller fails now,
# and a caller that can fall back is told what it is falling back from.
api_failed() { # what gh-exit
  if strict; then
    echo "::error::GitHub API call for $1 failed (gh exit $2). Check GH_TOKEN and API access, then rerun this job." >&2
  else
    echo "::warning title=Deps wait::GitHub API call for $1 failed (gh exit $2)." >&2
  fi
}

# The cache key depends on deps/ and on the workflows under .github/.
local_key_inputs() { echo "$(git rev-parse "$1:.github") $(git rev-parse "$1:deps")"; }
remote_key_inputs() {
  local inputs
  inputs=$(gh api "repos/$REPO/git/trees/$1" < /dev/null \
    --jq '[.tree[] | select(.path == ".github" or .path == "deps") | {key: .path, value: .sha}] | from_entries | "\(.[".github"]) \(.deps)"') \
    || return 1
  echo "$inputs"
}

# The ref this pull request's own build saves its cache under.
pr_ref() {
  if [[ "${GITHUB_REF:-}" == refs/pull/*/merge ]]; then
    echo "$GITHUB_REF"
  fi
}

# Prints the id of a push build of the base branch that has not finished, is not
# one of the ids in $1, and has KEY_INPUTS.
matching_base_build() {
  local status runs id sha inputs rc
  runs=$(for status in in_progress queued pending waiting requested; do
    gh api -X GET "repos/$REPO/actions/workflows/$workflow/runs" \
      -f branch="$BASE_REF" -f event=push -f status="$status" -f per_page=10 < /dev/null \
      | jq -r '.workflow_runs[] | "\(.id) \(.head_sha)"' || exit 1
  done)
  rc=$?
  if [ "$rc" -ne 0 ]; then
    api_failed "recent $BASE_REF builds" "$rc"
    return 1
  fi
  while read -r id sha; do
    [ -n "$id" ] || continue
    [[ " ${1:-} " == *" $id "* ]] && continue
    inputs=$(remote_key_inputs "$sha")
    rc=$?
    if [ "$rc" -ne 0 ]; then
      api_failed "the tree of $sha" "$rc"
      return 1
    fi
    if [ "$inputs" = "$KEY_INPUTS" ]; then
      echo "$id"
      return
    fi
  done <<< "$runs"
}

# Prints the id of the newest build of this pull request's HEAD_SHA. Runs of an
# older head are never accepted: they build deps for code this checkout does not
# contain.
matching_pr_build() {
  local attempt id rc
  for ((attempt = 0; attempt < producer_attempts; attempt++)); do
    id=$(gh api -X GET "repos/$REPO/actions/workflows/$workflow/runs" \
      -f head_sha="$HEAD_SHA" -f event=pull_request -f per_page=1 \
      --jq '.workflow_runs[0].id // empty' < /dev/null)
    rc=$?
    if [ "$rc" -ne 0 ]; then
      api_failed "the runs of this pull request's head $HEAD_SHA" "$rc"
      return 1
    fi
    if [ -n "$id" ]; then
      echo "$id"
      return
    fi
    [ "$attempt" -lt $((producer_attempts - 1)) ] && sleep "$producer_delay"
  done
  return 0
}

check() {
  KEY_INPUTS=$(local_key_inputs HEAD)
  out "key-inputs=$KEY_INPUTS"
  if [ "$KEY_INPUTS" != "$(local_key_inputs HEAD^1)" ]; then
    echo "This pull request changes deps/ or .github/, so it builds its own deps."
    return
  fi
  local id
  if ! id=$(matching_base_build); then
    strict && return 1
    return 0
  fi
  if [ -n "$id" ]; then
    echo "Build $id of $BASE_REF has the same deps/ and .github/. Waiting for its cache."
    out "wait-for=$id"
    out "wait-since=$(date +%s)"
    out "cache-ref=refs/heads/$BASE_REF"
    selected=1
  else
    echo "No build of $BASE_REF is building these deps."
  fi
}

own() {
  # check-or-own has already written it; GitHub keeps the last value either way.
  [ -n "${KEY_INPUTS:-}" ] || out "key-inputs=$(local_key_inputs HEAD)"
  local ref id rc
  ref=$(pr_ref)
  if [ -z "$ref" ]; then
    echo "::error::Not a pull request run (GITHUB_REF='${GITHUB_REF:-}'), so there is no build of this head to wait for." >&2
    strict && return 1
    return 0
  fi
  id=$(matching_pr_build)
  rc=$?
  if [ "$rc" -ne 0 ]; then
    strict && return 1
    return 0
  fi
  if [ -n "$id" ]; then
    echo "Waiting for the deps cache of build $id of this pull request ($HEAD_SHA)."
    out "wait-for=$id"
    out "wait-since=$(date +%s)"
    out "cache-ref=$ref"
    selected=1
    return
  fi
  echo "::error::No Build all run exists for this pull request's head $HEAD_SHA after $((producer_attempts * producer_delay))s, so nothing will save its deps cache. Rerun this job once that run exists." >&2
  strict && return 1
  return 0
}

check_or_own() {
  check || return 1
  [ -n "$selected" ] && return
  own
}

# The ref the waited-for build saves its cache under, and the base branch too: a
# pull request can restore an exact cache saved on its base branch, which is why
# the build's deps job can be skipped without saving anything itself.
cache_has_key() {
  local json rc
  json=$(gh cache list -R "$REPO" --ref "$1" --key "$KEY" --json key < /dev/null)
  rc=$?
  if [ "$rc" -ne 0 ]; then
    api_failed "the cache list for $1" "$rc"
    return 2
  fi
  jq -e --arg key "$KEY" 'any(.[]; .key == $key)' <<< "$json" > /dev/null
  rc=$?
  if [ "$rc" -gt 1 ]; then
    # jq, not the API: a response that cannot be read is not an empty one.
    if strict; then
      echo "::error::Could not read the cache list for $1 (exit $rc). Check the jq in the log, then rerun this job." >&2
    else
      echo "::warning title=Deps wait::Could not read the cache list for $1 (exit $rc)." >&2
    fi
    return 2
  fi
  return "$rc"
}

# 0 saved, 1 not saved, 2 the cache list could not be read. Both refs are
# checked unless the first one has the key: the base branch's exact entry is
# restorable from a pull request, and only a real miss returns 1.
key_saved() {
  local primary=${CACHE_REF:-refs/heads/$BASE_REF} rc unknown=1
  cache_has_key "$primary"
  rc=$?
  [ "$rc" -eq 0 ] && return 0
  [ "$rc" -eq 2 ] && unknown=2
  if [ "$primary" != "refs/heads/$BASE_REF" ]; then
    cache_has_key "refs/heads/$BASE_REF"
    rc=$?
    [ "$rc" -eq 0 ] && return 0
    [ "$rc" -eq 2 ] && unknown=2
  fi
  return "$unknown"
}

# The new entry can take a moment to be listed, and a skipped deps job means the
# build found the cache, possibly under the default branch.
key_saved_soon() {
  local attempt rc
  for attempt in 1 2 3; do
    key_saved
    rc=$?
    [ "$rc" -eq 1 ] || return "$rc"
    sleep 20
  done
  key_saved
}

wait_for() {
  # Counts the limit from the check, including time queued behind other waiting
  # pull requests.
  local id=$WAIT_FOR seen=$WAIT_FOR deadline=$((${WAIT_SINCE:-$(date +%s)} + WAIT_MINUTES * 60))
  local jobs state run_state check warned='' rc
  while [ "$(date +%s)" -lt "$deadline" ]; do
    key_saved
    rc=$?
    if [ "$rc" -eq 0 ]; then
      echo "Found $KEY."
      return 0
    fi
    # The cache list already reported why it failed; a strict caller cannot
    # confirm the cache, and a fallback caller keeps waiting as it did before.
    [ "$rc" -eq 2 ] && strict && return 1
    jobs=$(gh api -X GET "repos/$REPO/actions/runs/$id/jobs" -f filter=latest -f per_page=100 < /dev/null)
    state=$(jq -r --arg name "$deps_job_name" 'first(.jobs[] | select(.name == $name) | "\(.status) \(.conclusion)") // ""' <<< "$jobs")
    case "$state" in
      "completed success" | "completed skipped")
        key_saved_soon
        rc=$?
        case "$rc" in
          0)
            echo "Found $KEY."
            return 0 ;;
          1) ;;
          *)
            if strict; then
              return 1
            fi
            echo "Could not confirm $KEY."
            return 0 ;;
        esac
        if strict; then
          echo "::error::Build $id finished its deps job without saving $KEY, so this job has no exact deps cache to restore. Rerun it once that build has saved one." >&2
          return 1
        fi
        echo "Build $id finished its deps without saving $KEY."
        return 0 ;;
      "completed cancelled")
        if strict; then
          echo "::error::Build $id was cancelled before it saved $KEY. Rerun this job once a build of these deps/ finishes." >&2
          return 1
        fi ;;
      completed*)
        if strict; then
          echo "::error::The deps job of build $id ended '$state' instead of succeeding, so it saved no $KEY. Check that build, then rerun this job." >&2
          return 1
        fi
        echo "The deps job of build $id did not succeed."
        return 0 ;;
      # The deps job exists only once the build's cache check has succeeded.
      "")
        run_state=$(gh api "repos/$REPO/actions/runs/$id" --jq '"\(.status) \(.conclusion)"' < /dev/null)
        case "$run_state" in
          "completed action_required")
            if strict; then
              echo "::error::Build $id of this pull request is waiting for a maintainer to approve it, so it cannot save $KEY. Approve Build all, then rerun this job." >&2
              return 1
            fi
            echo "Build $id is waiting for approval, so it saves no $KEY."
            return 0 ;;
          "completed cancelled")
            if strict; then
              echo "::error::Build $id was cancelled before its deps job started, so it saved no $KEY. Rerun this job once the pull request's Build all run succeeds." >&2
              return 1
            fi ;;
          completed*)
            if strict; then
              echo "::error::Build $id finished without a \"$deps_job_name\" job and without saving $KEY. If the build jobs were renamed, update scripts/ci_deps_cache.sh." >&2
              return 1
            fi
            check=$(jq -r --arg name "$check_job_name" 'first(.jobs[] | select(.name == $name) | .conclusion) // ""' <<< "$jobs")
            if [ -z "$warned" ] && { [ -z "$check" ] || [ "$check" = success ]; }; then
              echo "::warning title=Deps wait::Build $id has no job named \"$deps_job_name\". If the build jobs were renamed, update scripts/ci_deps_cache.sh. This does not affect this pull request."
              warned=1
            fi ;;
          *)
            sleep 120
            continue ;;
        esac ;;
      *)
        sleep 120
        continue ;;
    esac
    # Follows another unfinished build of the base branch with the same key
    # inputs, such as the push that replaced a cancelled one. A pull request's
    # own build has no such replacement.
    if [[ "${CACHE_REF:-refs/heads/$BASE_REF}" == refs/heads/* ]]; then
      id=$(matching_base_build "$seen")
      rc=$?
      if [ "$rc" -ne 0 ]; then
        strict && return 1
        break
      fi
      [ -n "$id" ] || break
      seen="$seen $id"
      echo "Following build $id of $BASE_REF."
    else
      sleep 120
    fi
  done
  if strict; then
    echo "::error::No build saved $KEY within $WAIT_MINUTES minutes." >&2
    return 1
  fi
  echo "No build saved $KEY."
}

case "${1:-}" in
  check) check ; exit $? ;;
  own) own ; exit $? ;;
  check-or-own) check_or_own ; exit $? ;;
  wait) wait_for ; exit $? ;;
  *) echo "usage: $0 check|own|check-or-own|wait" >&2 ; exit 2 ;;
esac
