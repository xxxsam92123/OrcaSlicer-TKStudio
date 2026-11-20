#!/usr/bin/env bash
# End-to-end checks that a CLI run survives a log location it cannot write to.
#
# The "logfile" option always exists and defaults to an empty string. An empty name used to leave
# boost::log's file_name at its default "%5N.log" pattern, so every CLI run wrote 00000.log into
# the current working directory; when that directory was not writable the sink threw from inside
# the logging thread and the process died with an uncaught C++ exception before slicing anything.
# Only the real binary reaches this code, so it is checked here.
#
# - A directory named 00000.log in the working directory no longer kills the run.
# - A --logfile that cannot be opened (here: a directory) degrades to a warning, not a crash.
# - A plain run no longer leaves 00000.log behind in the working directory.
#
# usage: test_cli_logfile_cwd.sh <orca-slicer binary> <python3>
set -u

BIN="${1:-}"
PY="${2:-python3}"
# 77 is the test's SKIP_RETURN_CODE.
[ -x "$BIN" ] || { echo "SKIP: orca-slicer binary not found: $BIN"; exit 77; }

# From src/libslic3r/Utils.hpp. main() returns them, so the shell sees them modulo 256.
CLI_SUCCESS=0

WORK="$(mktemp -d "${TMPDIR:-/tmp}/orca-cli-logfile.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$WORK/datadir"

# A standalone printer that resets E at every layer, so relative extruder addressing validates.
cat > "$WORK/machine.json" <<'EOF'
{
    "type": "machine",
    "from": "User",
    "name": "CLI logfile test printer",
    "printable_area": ["0x0", "200x0", "200x200", "0x200"],
    "printable_height": "100",
    "layer_change_gcode": "G92 E0"
}
EOF
cat > "$WORK/process.json" <<'EOF'
{
    "type": "process",
    "from": "User",
    "name": "CLI logfile test process"
}
EOF
cat > "$WORK/filament.json" <<'EOF'
{
    "type": "filament",
    "from": "User",
    "name": "CLI logfile test filament",
    "filament_diameter": ["1.75"],
    "filament_colour": ["#FF0000"]
}
EOF

"$PY" - "$WORK/cube.stl" <<'EOF'
import sys

v = [(x, y, z) for z in (0, 10) for y in (0, 10) for x in (0, 10)]
with open(sys.argv[1], "w") as f:
    f.write("solid cube\n")
    for a, b, c, d in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
        for tri in ((a, b, c), (a, c, d)):
            f.write("facet normal 0 0 0\nouter loop\n")
            for i in tri:
                f.write("vertex %g %g %g\n" % v[i])
            f.write("endloop\nendfacet\n")
    f.write("endsolid cube\n")
EOF

fails=0
fail() { echo "FAIL: $*"; fails=$((fails + 1)); }

# run_in <tag> <cwd> [option...]: run with <cwd> as the working directory.
run_in() {
    local tag="$1" dir="$2"; shift 2
    local out="$WORK/$tag"
    mkdir -p "$out" "$dir"
    ( cd "$dir" && timeout 300 "$BIN" --datadir "$WORK/datadir" --outputdir "$out" "$@" > "$out/log" 2>&1 )
    echo $? > "$out/status"
}

# expect_status <tag> <cli code>
expect_status() {
    local got; got="$(cat "$WORK/$1/status")"
    [ "$got" -eq $(( $2 & 255 )) ] || fail "$1: shell status $got, want $(( $2 & 255 )) (code $2)"
}

# expect_gcode <tag>
expect_gcode() {
    compgen -G "$WORK/$1/*.gcode" > /dev/null || fail "$1: no G-code was exported"
}

presets=(--load-settings "$WORK/machine.json;$WORK/process.json" --load-filaments "$WORK/filament.json" --slice 1)

echo "== a 00000.log directory in the working directory does not kill the run"
mkdir -p "$WORK/cwd-logdir/00000.log"
run_in logdir "$WORK/cwd-logdir" "${presets[@]}" "$WORK/cube.stl"
expect_status logdir $CLI_SUCCESS
expect_gcode logdir
[ -d "$WORK/cwd-logdir/00000.log" ] || fail "logdir: the placeholder directory disappeared"

echo "== a plain run no longer drops 00000.log into the working directory"
mkdir -p "$WORK/cwd-plain"
run_in plain "$WORK/cwd-plain" "${presets[@]}" "$WORK/cube.stl"
expect_status plain $CLI_SUCCESS
expect_gcode plain
[ ! -e "$WORK/cwd-plain/00000.log" ] || fail "plain: 00000.log was created in the working directory"

echo "== a --logfile that cannot be opened warns instead of crashing"
mkdir -p "$WORK/logdir-target.log" "$WORK/cwd-explicit"
run_in explicit "$WORK/cwd-explicit" "${presets[@]}" --logfile "$WORK/logdir-target.log" "$WORK/cube.stl"
expect_status explicit $CLI_SUCCESS
expect_gcode explicit
grep -qF "cannot be written" "$WORK/explicit/log" || fail "explicit: no warning about the unusable --logfile"

if [ "$fails" -ne 0 ]; then
    for log in "$WORK"/*/log; do
        echo "--- $log"
        tail -n 40 "$log"
    done
    exit 1
fi
echo "PASS"
