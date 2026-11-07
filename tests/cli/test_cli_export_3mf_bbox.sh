#!/usr/bin/env bash
# End-to-end check of the first-layer bbox metadata in a --export-3mf project.
#
# --slice-all releases a plate's preview payload and sliced objects as soon as its G-code has been
# exported, and the 3MF export measures the per-object first-layer boxes after that. A released
# print has no PrintObjects left, so the export has to use the boxes measured before the release;
# otherwise the plate carries no bbox file at all.
#
# --export-3mf deliberately comes after --slice on the command line: the slice pass must already
# know that the export will need the boxes, so the capture cannot depend on option order.
#
# The same run reports the per-plate release outcome and the process footprint around it as info
# log records, which only reach a file through --logfile. The test passes one, so the release the
# memory work claims really happened is observable here instead of inferred.
#
# usage: test_cli_export_3mf_bbox.sh <orca-slicer binary> <python3>
set -eu

BIN="${1:-}"
PY="${2:-python3}"
# 77 is the test's SKIP_RETURN_CODE.
[ -x "$BIN" ] || { echo "SKIP: orca-slicer binary not found: $BIN"; exit 77; }

WORK="$(mktemp -d "${TMPDIR:-/tmp}/orca-cli-bbox.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$WORK/datadir" "$WORK/out"

# The binary and python get paths in the platform's own spelling: this shell may produce a POSIX
# path (git-bash on Windows) that a native build cannot resolve.
if command -v cygpath >/dev/null 2>&1; then
    WORK_ARG="$(cygpath -w "$WORK")"
else
    WORK_ARG="$WORK"
fi

# Standalone presets: without "inherits" the CLI loads them as-is, with no preset bundle.
cat > "$WORK/machine.json" <<'EOF'
{
    "type": "machine",
    "from": "User",
    "name": "CLI bbox test printer",
    "printable_area": ["0x0", "200x0", "200x200", "0x200"],
    "printable_height": "100",
    "layer_change_gcode": "G92 E0"
}
EOF
cat > "$WORK/process.json" <<'EOF'
{
    "type": "process",
    "from": "User",
    "name": "CLI bbox test process",
    "enable_support": "0"
}
EOF

# A 20 mm cube as an ASCII STL, so the plate has exactly one object whose first-layer box is known.
# An STL facet is a triangle: three vertices per facet, twelve facets for a closed cube.
"$PY" - "$WORK_ARG/cube.stl" <<'EOF'
import sys

size = 20.0
v = [(x, y, z) for z in (0.0, size) for y in (0.0, size) for x in (0.0, size)]
# (triangle, normal), wound counter-clockwise seen from outside: -z, +z, -y, +y, -x, +x.
faces = (
    ((0, 2, 3), (0.0, 0.0, -1.0)), ((0, 3, 1), (0.0, 0.0, -1.0)),
    ((4, 5, 7), (0.0, 0.0, 1.0)),  ((4, 7, 6), (0.0, 0.0, 1.0)),
    ((0, 1, 5), (0.0, -1.0, 0.0)), ((0, 5, 4), (0.0, -1.0, 0.0)),
    ((2, 6, 7), (0.0, 1.0, 0.0)),  ((2, 7, 3), (0.0, 1.0, 0.0)),
    ((0, 4, 6), (-1.0, 0.0, 0.0)), ((0, 6, 2), (-1.0, 0.0, 0.0)),
    ((1, 3, 7), (1.0, 0.0, 0.0)),  ((1, 7, 5), (1.0, 0.0, 0.0)),
)
with open(sys.argv[1], "w") as f:
    f.write("solid cube\n")
    for triangle, normal in faces:
        f.write("  facet normal %f %f %f\n    outer loop\n" % normal)
        for i in triangle:
            f.write("      vertex %f %f %f\n" % v[i])
        f.write("    endloop\n  endfacet\n")
    f.write("endsolid cube\n")
EOF

# --debug 3 keeps info records and --logfile gives them a sink; without both the CLI writes no log
# at all, and the release record asserted below would not exist.
if ! timeout 300 "$BIN" --datadir "$WORK_ARG/datadir" \
        --load-settings "$WORK_ARG/machine.json;$WORK_ARG/process.json" \
        --debug 3 --logfile "$WORK_ARG/cli.log" \
        --slice 0 --outputdir "$WORK_ARG/out" --export-3mf project.3mf "$WORK_ARG/cube.stl" \
        > "$WORK/log" 2>&1; then
    echo "FAIL: slicing with --export-3mf failed:"
    tail -20 "$WORK/log"
    exit 1
fi

if [ ! -f "$WORK/out/project.3mf" ]; then
    echo "FAIL: no 3MF was exported"
    exit 1
fi

# The release of a finished plate is what the bbox capture exists for, and only the log shows it:
# assert it happened, that it was complete rather than partial, and that it carried a footprint.
if [ ! -f "$WORK/cli.log" ]; then
    echo "FAIL: --logfile produced no log, so the release cannot be observed"
    exit 1
fi
if ! grep -q "after releasing slice resources (complete)" "$WORK/cli.log"; then
    echo "FAIL: no complete plate release was logged:"
    grep -i "releasing slice resources" "$WORK/cli.log" || echo "  (no release record at all)"
    exit 1
fi
if ! grep -Eq "plate 1: RSS [0-9]+ MiB, peak RSS [0-9]+ MiB before releasing slice resources" "$WORK/cli.log"; then
    echo "FAIL: the release records carry no resident-set figures"
    exit 1
fi

# The bbox metadata of a plate is Metadata/plate_<n>.json, written only when the export measured at
# least one object box. Assert that the released plate still carries the box of the 20 mm cube and
# its first-layer area - a zero or degenerate box would pass a mere "non-empty" check.
"$PY" - "$WORK_ARG/out/project.3mf" <<'EOF'
import json, sys, zipfile

with zipfile.ZipFile(sys.argv[1]) as z:
    names = [n for n in z.namelist() if n.startswith("Metadata/plate_") and n.endswith(".json")]
    if not names:
        sys.exit("FAIL: the exported 3MF has no Metadata/plate_N.json bbox file")
    plate = json.loads(z.read(names[0]))
objects = plate.get("bbox_objects") or []
if len(objects) != 1:
    sys.exit("FAIL: expected one bbox object, got %d" % len(objects))
if len(plate.get("bbox_all") or []) != 4:
    sys.exit("FAIL: %s has no bbox_all" % names[0])
x0, y0, x1, y1 = plate["bbox_all"]
if abs((x1 - x0) - 20.0) > 0.01 or abs((y1 - y0) - 20.0) > 0.01:
    sys.exit("FAIL: bbox_all is not the 20 mm cube: %r" % (plate["bbox_all"],))
expected_area = 20.0 * 20.0
area = objects[0].get("area") or 0
if abs(area - expected_area) > 0.01 * expected_area:
    sys.exit("FAIL: first-layer area is %r, expected %r" % (area, expected_area))
EOF

echo "PASS: --export-3mf kept the first-layer bbox metadata"
