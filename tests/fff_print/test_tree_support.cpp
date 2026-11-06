#include <catch2/catch_all.hpp>

#include <algorithm>
#include <initializer_list>
#include "libslic3r/Config.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Point.hpp"
#include <cstddef>
#include "libslic3r/libslic3r.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/Layer.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_helpers.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Support/TreeSupport3D.hpp"
#include "libslic3r/Support/TreeSupportContact.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Support/TreeSupportCommon.hpp"
#include "libslic3r/Support/SupportParameters.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <utility>
#include <math.h>
#include <optional>
#include <set>
#include <stdio.h>
#include <string>
#include <string_view>
#include <sstream>
#include <vector>
#ifdef _WIN32
    #include <io.h>
#else
    #include <unistd.h>
#endif
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/PrintBase.hpp"

using namespace Slic3r::Test;
using namespace Slic3r;

namespace {

// The upper plate overhangs both the lower plate and open air, so branches land on the model and on
// the bed in the same slice.
TriangleMesh two_tier_mesh()
{
    TriangleMesh lower  = make_cube(30, 30, 3);
    TriangleMesh column = make_cube(8, 8, 15);
    TriangleMesh upper  = make_cube(50, 50, 3);
    // Each part overlaps the one below rather than resting on it; a coplanar join slices ambiguously.
    column.translate(11.f, 11.f, 2.f);
    upper.translate(-10.f, -10.f, 16.f);
    TriangleMesh mesh = lower;
    mesh.merge(column);
    mesh.merge(upper);
    return mesh;
}

TriangleMesh scaled(TestMesh id, float scale)
{
    TriangleMesh mesh = Slic3r::Test::mesh(id);
    mesh.scale(scale);
    return mesh;
}

// What the exact-gap contract says the printed support under `face_z` ends at. `grid_top` is the top of
// the highest object layer that keeps the requested gap - the contact layer. When the requested Z lies
// above that layer by at least the profile's minimum layer height, one thin contact layer finishes at
// that Z (`top == target`, `exact`); otherwise the request cannot be printed on this profile and the
// support really ends on the whole layer (`top == grid_top`), i.e. the air gap is wider than asked.
struct ContactExpectation
{
    double grid_top { 0. };  // top of the object layer that carries the contact
    double target   { 0. };  // face_z - gap
    double top      { 0. };  // Z the printed support really ends at
    bool   exact    { false };
};

ContactExpectation expected_contact(const PrintObject &po, const double face_z, const double gap)
{
    ContactExpectation expected;
    expected.target = face_z - gap;
    for (const Layer *layer : po.layers())
        if (layer->print_z <= expected.target + TreeSupport3D::TreeSupportLayerGrid::z_roundoff)
            expected.grid_top = layer->print_z;
    const double remainder = expected.target - expected.grid_top;
    // The same test add_exact_contact_area() applies: a layer thinner than the profile's minimum layer
    // height is refused, and with it the exact gap.
    expected.exact = remainder > EPSILON &&
        remainder + EPSILON >= po.slicing_parameters().min_layer_height;
    expected.top = expected.exact ? expected.target : expected.grid_top;
    return expected;
}

// The object layer printed at `print_z`, or nullptr when no object layer is there.
const Layer *object_layer_at(const PrintObject &po, const double print_z)
{
    for (const Layer *layer : po.layers())
        if (std::abs(layer->print_z - print_z) < 1e-9)
            return layer;
    return nullptr;
}

// The object layer directly above `print_z`, or nullptr at the top of the stack.
const Layer *object_layer_above(const PrintObject &po, const double print_z)
{
    const Layer *above = nullptr;
    for (const Layer *layer : po.layers())
        if (layer->print_z > print_z + 1e-9 && (above == nullptr || layer->print_z < above->print_z))
            above = layer;
    return above;
}

// `extra` is applied last, so a caller can add or override any key.
void slice_with_tree_support(const TriangleMesh &mesh, Slic3r::Print &print, const char *style,
                             int threshold_angle = 30, int build_plate_only = 0, int raft_layers = 0,
                             std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> extra = {})
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_support",              1 },
        { "support_type",                "tree(auto)" },
        { "support_style",               style },
        { "support_on_build_plate_only", build_plate_only },
        { "support_threshold_angle",     threshold_angle },
        { "raft_layers",                 raft_layers },
        { "layer_height",                0.2 },
    });
    config.set_deserialize_strict(extra);
    Slic3r::Test::init_and_process_print({ mesh }, print, config);
}

Points support_points(const Slic3r::Print &print)
{
    Points points;
    for (const SupportLayer *layer : print.objects().front()->support_layers())
        layer->support_fills.collect_points(points);
    return points;
}

size_t support_point_count(const TriangleMesh &mesh, const char *style, int threshold_angle = 30,
                           int build_plate_only = 0)
{
    Slic3r::Print print;
    slice_with_tree_support(mesh, print, style, threshold_angle, build_plate_only);
    return support_points(print).size();
}

// Index of the first differing point, or the common length when they match. An index keeps a
// failure readable; comparing the vectors themselves dumps thousands of points.
size_t first_difference(const Points &a, const Points &b)
{
    const size_t common = std::min(a.size(), b.size());
    for (size_t i = 0; i < common; ++i)
        if (a[i] != b[i])
            return i;
    return common;
}

// Slice `mesh` twice and require an identical support point sequence. Point counts and total
// length are order insensitive, so the sequence is what a reordering shows up in.
void sliced_twice_matches(const TriangleMesh &mesh, int build_plate_only, const char *style = "tree_slim",
                          std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> extra = {})
{
    Slic3r::Print first_print, second_print;
    slice_with_tree_support(mesh, first_print, style, 30, build_plate_only, 0, extra);
    slice_with_tree_support(mesh, second_print, style, 30, build_plate_only, 0, extra);
    const Points first  = support_points(first_print);
    const Points second = support_points(second_print);
    REQUIRE(first.size() > 1000); // without support the comparison below passes vacuously
    REQUIRE(second.size() == first.size());
    REQUIRE(first_difference(first, second) == first.size());
}

// The organic generator reports a branch it could not keep connected by printing to stdout, and the
// geometry it returns is unchanged when that happens - so a test that only looks at Zs and heights stays
// green while branches are missing. Redirect stdout for the duration of a call and assert on the text
// afterwards instead of trusting the exit code.
class ScopedStdoutCapture
{
public:
    ScopedStdoutCapture() :
        m_file(std::tmpfile())
    {
        if (m_file == nullptr)
            FAIL("could not open a temporary file to capture the support diagnostics");
        std::fflush(stdout);
        m_saved = fd_dup(file_fd(stdout));
        fd_dup2(file_fd(m_file), file_fd(stdout));
    }
    ~ScopedStdoutCapture() { finish(); }
    ScopedStdoutCapture(const ScopedStdoutCapture &) = delete;
    ScopedStdoutCapture &operator=(const ScopedStdoutCapture &) = delete;

    // Ends the capture and returns everything printed to stdout in between.
    std::string finish()
    {
        if (m_file == nullptr)
            return {};
        std::fflush(stdout);
        fd_dup2(m_saved, file_fd(stdout));
        fd_close(m_saved);
        std::rewind(m_file);
        std::string text;
        char        buffer[4096];
        for (size_t read; (read = std::fread(buffer, 1, sizeof(buffer), m_file)) > 0;)
            text.append(buffer, read);
        std::fclose(m_file);
        m_file = nullptr;
        return text;
    }

private:
#ifdef _WIN32
    static int  file_fd(std::FILE *file) { return _fileno(file); }
    static int  fd_dup(int fd) { return _dup(fd); }
    static int  fd_dup2(int from, int to) { return _dup2(from, to); }
    static void fd_close(int fd) { _close(fd); }
#else
    static int  file_fd(std::FILE *file) { return fileno(file); }
    static int  fd_dup(int fd) { return dup(fd); }
    static int  fd_dup2(int from, int to) { return dup2(from, to); }
    static void fd_close(int fd) { close(fd); }
#endif

    std::FILE *m_file  { nullptr };
    int        m_saved { -1 };
};

size_t occurrences(const std::string &text, const std::string &needle)
{
    size_t count = 0;
    for (size_t pos = text.find(needle); pos != std::string::npos; pos = text.find(needle, pos + needle.size()))
        ++ count;
    return count;
}

// Support diagnostics are printed by tree_supports_show_error() as "Error: <message>, critical: <0|1>".
// Counting the records rather than a list of messages catches a critical one the test does not know the
// wording of; every critical diagnostic of the generator goes through that one function.
size_t diagnostic_count(const std::string &text) { return occurrences(text, "Error: "); }

size_t critical_diagnostic_count(const std::string &text) { return occurrences(text, ", critical: 1"); }

// The grid entry whose top is `print_z`, or null when that Z is not a layer of the grid.
const Slic3r::TreeSupport3D::TreeSupportLayerGrid::Entry *grid_entry_at(
    const Slic3r::TreeSupport3D::TreeSupportLayerGrid &grid, double print_z)
{
    for (const Slic3r::TreeSupport3D::TreeSupportLayerGrid::Entry &entry : grid.entries)
        if (std::abs(entry.print_z - print_z) < 1e-9)
            return &entry;
    return nullptr;
}

// The layer counts of two prints have to match: a raft or a thicker first layer is not a variable layer
// height, so it may not move a single one of them.
void check_counts_unchanged(const Slic3r::TreeSupport3D::TreeSupportSettings &config,
                            const Slic3r::TreeSupport3D::TreeSupportSettings &reference)
{
    INFO("tip_layers " << config.tip_layers << " vs " << reference.tip_layers);
    CHECK(config.tip_layers == reference.tip_layers);
    INFO("support_bottom_layers " << config.support_bottom_layers << " vs " << reference.support_bottom_layers);
    CHECK(config.support_bottom_layers == reference.support_bottom_layers);
    INFO("min_dtt_to_model " << config.min_dtt_to_model << " vs " << reference.min_dtt_to_model);
    CHECK(config.min_dtt_to_model == reference.min_dtt_to_model);
    INFO("increase_radius_until_layer " << config.increase_radius_until_layer << " vs " << reference.increase_radius_until_layer);
    CHECK(config.increase_radius_until_layer == reference.increase_radius_until_layer);
    INFO("layer_start_bp_radius " << config.layer_start_bp_radius << " vs " << reference.layer_start_bp_radius);
    CHECK(config.layer_start_bp_radius == reference.layer_start_bp_radius);
}

// A prism with a sloped underside that runs into a horizontal shelf, extruded along Y. Its cross section
// in XZ is (0, 2.0) -> (13.5, 3.35) -> (25, 3.35) -> (25, 4.0) -> (0, 4.0), so one overhang area holds two
// downward faces: the slope, whose lowest vertex at 2.0 lies below the window a layer searches, and the
// shelf at 3.35, whose own lowest vertex lies inside that window. The slope rises 0.1 mm per mm, i.e. 5.7
// degrees, which is below the 30 degree support threshold - a steeper face than that leaves no overhang
// area behind the threshold offset at all.
TriangleMesh sloped_shelf_mesh()
{
    const float y0 = 0.f, y1 = 10.f;
    const std::vector<Vec3f> vertices {
        { 0.f, y0, 2.000f }, { 13.5f, y0, 3.35f }, { 25.f, y0, 3.35f }, { 25.f, y0, 4.f }, { 0.f, y0, 4.f },
        { 0.f, y1, 2.000f }, { 13.5f, y1, 3.35f }, { 25.f, y1, 3.35f }, { 25.f, y1, 4.f }, { 0.f, y1, 4.f },
    };
    // Outward facing throughout: the caps close the ends, the walls run along the cross section.
    const std::vector<Vec3i32> indices {
        { 1, 3, 2 }, { 1, 4, 3 }, { 1, 0, 4 },   // the y = 0 cap
        { 6, 7, 8 }, { 6, 8, 9 }, { 6, 9, 5 },   // the y = 10 cap
        { 0, 5, 6 }, { 0, 6, 1 },                // the sloped underside
        { 1, 6, 7 }, { 1, 7, 2 },                // the shelf
        { 2, 7, 8 }, { 2, 8, 3 },                // the wall at x = 25
        { 3, 8, 9 }, { 3, 9, 4 },                // the top
        { 4, 9, 5 }, { 4, 5, 0 },                // the wall at x = 0
    };
    return TriangleMesh(vertices, indices);
}

// Lowest Z at which the mesh has material above `z` at (x, y), i.e. the surface an air gap under a support
// point at that Z has to be measured against. Nothing when the point is outside the object's projection.
std::optional<double> face_above(const TriangleMesh &mesh, const double x, const double y, const double z)
{
    auto cross = [](double ax, double ay, double bx, double by) { return ax * by - ay * bx; };
    std::optional<double> face;
    for (const Vec3i32 &triangle : mesh.its.indices) {
        const Vec3f &a = mesh.its.vertices[triangle[0]];
        const Vec3f &b = mesh.its.vertices[triangle[1]];
        const Vec3f &c = mesh.its.vertices[triangle[2]];
        const double det = cross(double(b.x()) - a.x(), double(b.y()) - a.y(),
                                 double(c.x()) - a.x(), double(c.y()) - a.y());
        if (std::abs(det) < 1e-12)
            continue;
        // Barycentrics of (x, y) in the XY projection of the facet; Z is affine over it.
        const double wa = cross(double(b.x()) - x, double(b.y()) - y, double(c.x()) - x, double(c.y()) - y) / det;
        const double wb = cross(double(c.x()) - x, double(c.y()) - y, double(a.x()) - x, double(a.y()) - y) / det;
        const double wc = 1. - wa - wb;
        if (wa < -1e-9 || wb < -1e-9 || wc < -1e-9)
            continue;
        const double facet_z = wa * double(a.z()) + wb * double(b.z()) + wc * double(c.z());
        if (facet_z > z && (! face || facet_z < *face))
            face = facet_z;
    }
    return face;
}

} // namespace

TEST_CASE("Tree support is generated for an overhang and not for a plain cube", "[TreeSupport]")
{
    REQUIRE(support_point_count(scaled(TestMesh::overhang, 2.f), "tree_slim") > 1000);
    REQUIRE(support_point_count(Slic3r::Test::cube(20), "tree_slim") == 0);
}

TEST_CASE("Restricting tree support to the build plate changes what is generated", "[TreeSupport]")
{
    const TriangleMesh mesh = two_tier_mesh();
    const size_t anywhere    = support_point_count(mesh, "tree_slim", 30, 0);
    const size_t plate_only  = support_point_count(mesh, "tree_slim", 30, 1);
    REQUIRE(anywhere > 1000);
    REQUIRE(plate_only > 1000);
    // The upper plate overhangs the lower one, so some branches would land on the model.
    REQUIRE(plate_only != anywhere);
}

TEST_CASE("Tree support layers rise monotonically within the layer height limits", "[TreeSupport]")
{
    Slic3r::Print print;
    slice_with_tree_support(scaled(TestMesh::overhang, 2.f), print, "tree_slim");
    const double nozzle = print.config().nozzle_diameter.values.front();

    size_t checked = 0;
    double previous = 0;
    bool   previous_was_adjacent = false;
    for (const SupportLayer *layer : print.objects().front()->support_layers()) {
        if (layer->print_z <= 0 || layer->height <= 0) {
            // Layers with no nodes are left at zero. Skipping one leaves a hole, so the next pair
            // spans more than one layer and its gap says nothing about the layer height limit.
            previous_was_adjacent = false;
            continue;
        }
        if (previous > 0) {
            CAPTURE(previous, layer->print_z);
            REQUIRE(layer->print_z > previous);
            if (previous_was_adjacent)
                REQUIRE(layer->print_z - previous <= nozzle + EPSILON);
        }
        previous = layer->print_z;
        previous_was_adjacent = true;
        ++checked;
    }
    REQUIRE(checked > 10);
}

TEST_CASE("A raft is still generated under tree support", "[TreeSupport]")
{
    // The mesh supports itself, so a layer count alone passes with no raft at all.
    Slic3r::Print rafted, unrafted;
    slice_with_tree_support(scaled(TestMesh::overhang, 2.f), rafted, "tree_slim", 30, 0, 3);
    slice_with_tree_support(scaled(TestMesh::overhang, 2.f), unrafted, "tree_slim", 30, 0, 0);
    const PrintObject *rafted_object   = rafted.objects().front();
    const PrintObject *unrafted_object = unrafted.objects().front();
    REQUIRE(rafted_object->support_layers().size() > unrafted_object->support_layers().size());
    // The raft goes under the object.
    REQUIRE(rafted_object->layers().front()->print_z > unrafted_object->layers().front()->print_z);
}

// drop_nodes() decides the node merges and spawns the next layer's nodes in parallel. Every one of
// those decisions has to be applied in a fixed order, or the same model gives different branches on
// each slice.
TEST_CASE("Tree support toolpaths do not depend on thread scheduling", "[TreeSupport][Regression]")
{
    // Scaled up so that a layer holds enough nodes for the parallel range to be split. At stock
    // size it stays in one chunk and the order never varies.
    SECTION("overhang")            { sliced_twice_matches(scaled(TestMesh::overhang, 2.f), 0); }
    SECTION("bridge with hole")    { sliced_twice_matches(scaled(TestMesh::bridge_with_hole, 3.f), 0); }
    // Dropping every branch that cannot reach the bed leaves the survivors dense enough that the
    // neighbour merge fires in bulk.
    SECTION("on the build plate")  { sliced_twice_matches(scaled(TestMesh::overhang, 4.f), 1); }
    // Branches resting on the model are what put nodes in a part group other than 0, which is the
    // only way to reach the prune in the second pass. tree_hybrid additionally builds polygon
    // nodes, so it is the only style that exercises the overhang merge.
    SECTION("resting on the model") { sliced_twice_matches(two_tier_mesh(), 0); }
    SECTION("hybrid on the model")  { sliced_twice_matches(two_tier_mesh(), 0, "tree_hybrid"); }
}

// Prim breaks equal-distance ties by heap address. A 1 mm branch diameter puts neighbours close
// enough to tie, and an explicit line width pins max_move_dist, so the moved tie winner reaches
// the support toolpaths.
TEST_CASE("Tree support toolpaths do not depend on the MST tie order", "[TreeSupport][Regression]")
{
    sliced_twice_matches(two_tier_mesh(), 0, "tree_hybrid", {
        { "tree_support_branch_diameter", 1.0 },
        { "tree_support_branch_distance", 5.0 },
        { "tree_support_branch_angle",    40 },
        { "support_line_width",           0.4 },
    });
}

// The organic generator turns support_top_z_distance into a whole number of layers: the contact is
// placed on the highest support layer that keeps at least the requested distance under the overhang.
// The gap therefore never comes out narrower than requested, but it is quantised upwards - on a fixed
// 0.2 mm profile 0.1 and 0.3 both give 0.2 / 0.4 instead of squeezing the remainder into a layer.
TEST_CASE("Organic support places its top contact at least the configured gap below the overhang", "[TreeSupportOrganicGap]")
{
    // The upper plate of two_tier_mesh() starts at z = 16.0, so every point of its lower face is an
    // overhang bottom.
    const TriangleMesh mesh = two_tier_mesh();
    const double     gap    = GENERATE(0.1, 0.2, 0.3);
    INFO("support_top_z_distance=" << gap);

    Slic3r::Print print;
    slice_with_tree_support(mesh, print, "organic", 30, 0, 0, {
        { "support_top_z_distance",      gap },
        { "support_interface_top_layers", 2 },
    });

    const PrintObject &po = *print.objects().front();
    REQUIRE(! po.support_layers().empty());
    // The contact is the highest support layer, and its top is its own print_z: the support material
    // ends there, so the air gap under the overhang is 16.0 - print_z. The requested Z is printed by a
    // thin contact layer on top of the whole object layer that carries it, unless that remainder is below
    // the profile's minimum layer height.
    const ContactExpectation expected = expected_contact(po, 16.0, gap);
    const SupportLayer      &contact  = *po.support_layers().back();
    INFO("contact layer print_z=" << contact.print_z << " height=" << contact.height
                                  << " expected top=" << expected.top << " grid top=" << expected.grid_top
                                  << " exact=" << expected.exact);
    CHECK_THAT(contact.print_z, Catch::Matchers::WithinAbs(expected.top, 1e-9));
    if (expected.exact) {
        // The gap is the requested distance, not an upward quantisation of it ...
        CHECK_THAT(16.0 - contact.print_z, Catch::Matchers::WithinAbs(gap, 1e-9));
        // ... and the thin layer prints exactly the remainder it covers.
        CHECK_THAT(contact.height, Catch::Matchers::WithinAbs(expected.top - expected.grid_top, 1e-6));
    } else {
        // The profile cannot print the remainder: the support stays on the whole layer, so the gap is a
        // lower bound and at most one layer wider - never narrower.
        CHECK(contact.print_z <= 16.0 - gap + 1e-9);
        CHECK(contact.print_z > 16.0 - gap - 0.2 - 1e-9);
        const Layer *object_layer = object_layer_at(po, contact.print_z);
        REQUIRE(object_layer != nullptr);
        CHECK_THAT(contact.height, Catch::Matchers::WithinAbs(object_layer->height, 1e-12));
    }
}

TEST_CASE("Support layer grid resolves Z and places the top contact on whole layers", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;
    // Tops 0.2, 0.4, 0.8, 1.2, 1.4, 1.6: two 0.2 layers, two 0.4 layers, two 0.2 layers.
    const std::vector<double> heights { 0.2, 0.2, 0.4, 0.4, 0.2, 0.2 };
    TreeSupportLayerGrid grid;
    double               bottom = 0.;
    for (const double h : heights) {
        TreeSupportLayerGrid::Entry entry;
        entry.bottom_z     = bottom;
        entry.obj_bottom_z = bottom; // build_layer_grid() starts the object's material at the layer bottom
        entry.print_z      = bottom + h;
        entry.height       = h;
        bottom         = entry.print_z;
        grid.entries.emplace_back(entry);
    }

    CHECK_THAT(grid.z(2), Catch::Matchers::WithinAbs(0.8, 1e-9));
    CHECK_THAT(grid.h(2), Catch::Matchers::WithinAbs(0.4, 1e-9));
    CHECK(grid.idx_ceil(0.7) == 2);
    CHECK(grid.idx_floor(0.7) == 1);
    CHECK_THAT(grid.z_span(1, 4), Catch::Matchers::WithinAbs(1.0, 1e-9));

    SECTION("A gap that lands on a layer top takes that layer") {
        // Overhang bottom 1.6 and a 0.2 gap give 1.4, which is layer 4's own top.
        CHECK(grid.place_top_contact(1.6, 0.2) == 4);
        CHECK_THAT(grid.z(4), Catch::Matchers::WithinAbs(1.4, 1e-9));
        CHECK_THAT(grid.h(4), Catch::Matchers::WithinAbs(0.2, 1e-9));
    }

    SECTION("A gap of zero puts the contact on the layer whose top is the face") {
        CHECK(grid.place_top_contact(1.6, 0.) == 5);
    }

    SECTION("A gap inside a layer takes the layer below, widening it by less than a layer") {
        // Overhang bottom 1.6 and a 0.3 gap ask for 1.3, which is inside layer 4 (1.2 .. 1.4). The
        // contact goes to layer 3, whose top 1.2 leaves 0.4 of air: wider than asked, never narrower.
        CHECK(grid.place_top_contact(1.6, 0.3) == 3);
        CHECK_THAT(grid.z(3), Catch::Matchers::WithinAbs(1.2, 1e-9));
        CHECK_THAT(grid.h(3), Catch::Matchers::WithinAbs(0.4, 1e-9));
    }

    SECTION("Placing a contact never rewrites the object layer stack") {
        // The stack is the object's: Z, height and the material start stay exactly what was built,
        // whatever gap is requested. A rewritten stack is what put 0.14 / 0.23 into the layer height
        // legend.
        const std::vector<TreeSupportLayerGrid::Entry> before = grid.entries;
        CHECK(grid.place_top_contact(1.6, 0.3) == 3);
        // 0.5 is inside layer 2 (0.4 .. 0.8), so the contact lands on layer 1.
        CHECK(grid.place_top_contact(1.6, 1.1) == 1);
        REQUIRE(grid.entries.size() == before.size());
        for (size_t i = 0; i < before.size(); ++ i) {
            INFO("layer " << i);
            CHECK_THAT(grid.entries[i].print_z, Catch::Matchers::WithinAbs(before[i].print_z, 0.));
            CHECK_THAT(grid.entries[i].bottom_z, Catch::Matchers::WithinAbs(before[i].bottom_z, 0.));
            CHECK_THAT(grid.entries[i].height, Catch::Matchers::WithinAbs(before[i].height, 0.));
            CHECK_THAT(grid.entries[i].obj_bottom_z, Catch::Matchers::WithinAbs(before[i].obj_bottom_z, 0.));
        }
    }

    SECTION("Layer counts follow the grid heights") {
        // Tops 0.2, 0.4, 0.8, 1.2, 1.4, 1.6: 0.6 is covered by 0.2 + 0.2 + 0.4 but only 0.2 + 0.2 fit into it.
        CHECK(grid.layers_fitting(0.6) == 2);
        CHECK(grid.layers_covering(0.6) == 3);
        CHECK(grid.layers_rounding(0.6) == 3);
        // 0.5 sits 0.1 above the 0.4 top and 0.3 below the 0.8 top, so it rounds down to two layers.
        CHECK(grid.layers_fitting(0.5) == 2);
        CHECK(grid.layers_covering(0.5) == 3);
        CHECK(grid.layers_rounding(0.5) == 2);
    }

    SECTION("The object material start of a layer only moves up, to the overhang face") {
        // Grid tops 0.2 .. 1.6, every layer starts at its own bottom.
        CHECK_THAT(grid.obj_bottom_z(4), Catch::Matchers::WithinAbs(1.2, 1e-9));
        // The slice of layer 5 reaches into an overhang whose face is at 1.5, inside layer 4: the
        // object inside layer 5 only starts there, so the clearance of the contact layer below has to
        // be measured against 1.5 instead of against the layer bottom 1.4.
        grid.set_obj_bottom_z(5, 1.5);
        CHECK_THAT(grid.obj_bottom_z(5), Catch::Matchers::WithinAbs(1.5, 1e-9));
        // Never downwards: a face measured below the layer bottom leaves the layer as it was.
        grid.set_obj_bottom_z(5, 1.1);
        CHECK_THAT(grid.obj_bottom_z(5), Catch::Matchers::WithinAbs(1.5, 1e-9));
        // Out of range indices are ignored instead of growing the grid.
        grid.set_obj_bottom_z(LayerIndex(grid.size()), 9.);
        grid.set_obj_bottom_z(-1, 9.);
        CHECK(grid.size() == 6);
        CHECK_THAT(grid.obj_bottom_z(5), Catch::Matchers::WithinAbs(1.5, 1e-9));
    }
}

// The two tiers give a uniform 0.2 mm grid with an overhang at z = 16, so every per-layer limit has
// to land exactly on the nominal value it replaced (contract §4.1/§5.1). getRadius_at() integrates the
// Z between a branch tip and the element, and the tips near the bed used to ask for a span below the
// first layer: clamping that Z shortened the span and changed the radius, which moved the branch.
TEST_CASE("A uniform layer grid keeps every per-layer growth limit at its nominal value", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;
    Slic3r::Print print;
    slice_with_tree_support(two_tier_mesh(), print, "organic", 30, 0, 0, {
        { "support_top_z_distance",      0.2 },
        { "support_interface_top_layers", 2 },
    });
    const PrintObject           &po = *print.objects().front();
    TreeSupportMeshGroupSettings ms(po);
    TreeSupportSettings          cfg(ms, po.slicing_parameters());

    REQUIRE(cfg.layer_grid.size() == size_t(po.layer_count()));
    REQUIRE(cfg.layer_grid.raft_layers == 0);

    for (size_t i = 0; i < cfg.layer_grid.size(); ++ i) {
        const LayerIndex layer_idx = LayerIndex(i);
        INFO("layer " << i);
        CHECK(cfg.maximum_move_distance_at(layer_idx) == cfg.maximum_move_distance);
        CHECK(cfg.maximum_move_distance_slow_at(layer_idx) == cfg.maximum_move_distance_slow);
        CHECK_THAT(cfg.branch_radius_increase_at(layer_idx), Catch::Matchers::WithinAbs(cfg.branch_radius_increase_per_layer, 1e-6));
        CHECK_THAT(cfg.bp_radius_increase_at(layer_idx), Catch::Matchers::WithinAbs(cfg.bp_radius_increase_per_layer, 1e-6));
    }

    // Tips anywhere in the grid, including the bottom ones whose growth span reaches below the first
    // layer and the top ones whose span runs past the object.
    const std::vector<size_t> tip_layers { 0, 1, 2, 5, 6, 7, 20, size_t(po.layer_count()) - 1 };
    for (const size_t tip : tip_layers)
        for (size_t distance_to_top = 0; distance_to_top <= 2 * cfg.tip_layers; ++ distance_to_top) {
            INFO("tip layer " << tip << ", distance_to_top " << distance_to_top);
            CHECK(cfg.getRadius_at(LayerIndex(tip), distance_to_top) == cfg.getRadius(distance_to_top));
        }

    // The radius the tree aims for at least has to stay the nominal layer count times the nominal
    // growth: on a uniform grid every layer grows the foot by exactly that much.
    for (size_t i = 0; i < cfg.layer_grid.size(); ++ i) {
        const LayerIndex layer_idx = LayerIndex(i);
        INFO("layer " << i);
        if (layer_idx < cfg.layer_start_bp_radius)
            CHECK(cfg.recommendedMinRadius(layer_idx) ==
                  coord_t(cfg.branch_radius + double(cfg.layer_start_bp_radius - layer_idx) * cfg.bp_radius_increase_per_layer));
        else
            CHECK(cfg.recommendedMinRadius(layer_idx) == 0);
    }
}

// The radius the tree aims for follows the growth of the elephant foot, which is integrated over the
// real layer heights by bp_radius_increase_at(). Multiplying a layer count by the nominal growth asks
// for a radius the branches below never reach as soon as the layers are thinner or thicker than the
// nominal one, which is what a variable layer height produces.
TEST_CASE("The recommended root radius accumulates the real layer heights", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;
    Slic3r::Print print;
    slice_with_tree_support(two_tier_mesh(), print, "organic", 30, 0, 0, {
        { "support_top_z_distance",       0.2 },
        { "support_interface_top_layers", 2 },
    });
    const PrintObject            &po = *print.objects().front();
    TreeSupportMeshGroupSettings  ms(po);

    // Same settings, but a grid of 0.1 mm layers: every layer grows the foot by half of what a nominal
    // 0.2 mm layer does, so the number of growth layers no longer says how much the foot grew.
    TreeSupportMeshGroupSettings mixed = ms;
    mixed.layer_grid.entries.clear();
    mixed.layer_grid.raft_layers = 0;
    double bottom                = 0.;
    for (size_t i = 0; i < 8; ++ i) {
        const double                height = 0.1;
        TreeSupportLayerGrid::Entry entry;
        entry.bottom_z     = bottom;
        entry.obj_bottom_z = bottom;
        entry.print_z      = bottom + height;
        entry.height       = height;
        bottom             = entry.print_z;
        mixed.layer_grid.entries.emplace_back(entry);
    }
    TreeSupportSettings cfg(mixed, po.slicing_parameters());

    const size_t widened = size_t(cfg.layer_start_bp_radius);
    // At least two growth layers, otherwise one increment would equal the nominal projection.
    REQUIRE(widened >= 2);
    double integrated = cfg.branch_radius;
    for (size_t idx = 1; idx <= widened; ++ idx)
        integrated += cfg.bp_radius_increase_at(LayerIndex(idx));
    const double nominal_projection = cfg.branch_radius + double(widened) * cfg.bp_radius_increase_per_layer;
    // Half as thick layers add up to less than the nominal projection ...
    CHECK(integrated < nominal_projection - 1.);
    // ... and the recommendation at the bottom layer follows the grid, not the layer count.
    CHECK_THAT(double(cfg.recommendedMinRadius(0)), Catch::Matchers::WithinAbs(integrated, 1.));
    CHECK(cfg.recommendedMinRadius(0) < coord_t(nominal_projection));
}

TEST_CASE("The top contact of an overhang never lands in the raft", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;
    // Two raft layers (tops 0.2 and 0.4) followed by four 0.2 mm object layers (tops 0.6 .. 1.2).
    TreeSupportLayerGrid grid;
    double               bottom = 0.;
    for (const double height : { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 }) {
        TreeSupportLayerGrid::Entry entry;
        entry.bottom_z     = bottom;
        entry.obj_bottom_z = bottom;
        entry.print_z      = bottom + height;
        entry.height       = height;
        bottom             = entry.print_z;
        grid.entries.emplace_back(entry);
    }
    grid.raft_layers = 2;

    SECTION("A contact requested inside the raft is refused instead of drawn there") {
        // Overhang at 0.9 with a 0.55 gap wants its top at 0.35, which is inside the first raft layer.
        CHECK(grid.place_top_contact(0.9, 0.55) == -1);
        CHECK_THAT(grid.z(0), Catch::Matchers::WithinAbs(0.2, 1e-9));
        CHECK_THAT(grid.z(1), Catch::Matchers::WithinAbs(0.4, 1e-9));
        CHECK_THAT(grid.z(2), Catch::Matchers::WithinAbs(0.6, 1e-9));
    }

    SECTION("A gap that does not fit into a whole layer takes the layer below") {
        // 1.4 - 0.395 asks for 1.005: layer 5's own top 1.0 is 0.4 away, so the contact takes layer 4
        // even though layer 5 would be closer. Refusing to overstep keeps the gap >= the request.
        CHECK(grid.place_top_contact(1.4, 0.395) == 4);
        CHECK_THAT(grid.z(4), Catch::Matchers::WithinAbs(1.0, 1e-9));
        CHECK_THAT(grid.h(4), Catch::Matchers::WithinAbs(0.2, 1e-9));
    }

    SECTION("A contact above the raft lands on a whole layer and leaves the raft alone") {
        // 1.4 - 0.35 asks for 1.05, so layer 4 (top 1.0) carries it with 0.4 of air.
        CHECK(grid.place_top_contact(1.4, 0.35) == 4);
        CHECK_THAT(grid.z(4), Catch::Matchers::WithinAbs(1.0, 1e-9));
        CHECK_THAT(grid.h(4), Catch::Matchers::WithinAbs(0.2, 1e-9));
        CHECK_THAT(grid.z(0), Catch::Matchers::WithinAbs(0.2, 1e-9));
        CHECK_THAT(grid.z(1), Catch::Matchers::WithinAbs(0.4, 1e-9));
    }
}

TEST_CASE("Organic support accepts a variable layer height profile", "[TreeSupportOrganicGap]")
{
    Slic3r::Model model;
    ModelObject *object = model.add_object();
    object->name        = "tshape.stl";
    object->add_volume(two_tier_mesh());
    object->add_instance();
    // 0.2 mm up to z=15, 0.4 mm above it up to the model top at z=16.
    object->layer_height_profile.set({ 0., 0.2, 15.0, 0.4, 16.0, 0.2 });
    object->ensure_on_bed();

    Slic3r::Print print;
    print.auto_assign_extruders(object);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_support", 1 },
        { "support_type",   "tree(auto)" },
        { "support_style",  "organic" },
        // Relative extruder addressing needs a reset at each layer, as validate() points out.
        { "before_layer_change_gcode", "G92 E0" },
    });
    print.apply(model, config);

    REQUIRE(print.objects().front()->model_object()->has_custom_layering());
    // Before the change this returned "Variable layer height is not supported with Organic supports."
    const StringObjectException error = print.validate();
    INFO("validate: " << error.string);
    CHECK(error.string.empty());
}

// The Cura derived engines (tree_slim / strong / hybrid) derive their clearance layer counts from
// settings.support_top_distance, which SlicingParameters rounds to whole layers unless support layers
// are independent of the object's. The organic generator needs the raw millimetres. Both have to be
// carried side by side: feeding the raw value to the quantised member shifts a tree_slim contact
// layer by one layer in that configuration.
TEST_CASE("The clearance is quantised for the layer synchronised engines and raw for the organic grid", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;
    constexpr double top_z_distance = 0.3; // 1.5 layer heights at 0.2, so the rounding decides the count

    Slic3r::Print print;
    slice_with_tree_support(two_tier_mesh(), print, "tree_slim", 30, 0, 0, {
        // 0 is the configuration in which SlicingParameters quantises the clearance, and the one where
        // feeding the raw value to the quantised member moved the contact layer.
        { "independent_support_layer_height", 0 },
        { "support_top_z_distance",          top_z_distance },
        { "support_interface_top_layers",    2 },
    });
    const PrintObject            &po = *print.objects().front();
    TreeSupportMeshGroupSettings mesh_settings(po);
    TreeSupportSettings          config(mesh_settings, po.slicing_parameters());

    // Quantised to whole layers for the layer synchronised engine ...
    CHECK_THAT(unscale<double>(mesh_settings.support_top_distance), Catch::Matchers::WithinAbs(0.4, 1e-9));
    // ... two of them at 0.2 mm; the raw 0.3 mm would round to one.
    CHECK(config.z_distance_top_layers == 2);
    // The raw millimetre value is the lower bound the organic grid places its contact layers from.
    CHECK_THAT(mesh_settings.support_top_z_distance_mm, Catch::Matchers::WithinAbs(top_z_distance, 1e-9));
}

// The layer heights the G-code reports, i.e. what the preview's layer-height legend shows: the
// `;HEIGHT:` comment printed right after a layer change is `print_z - m_last_layer_z`. It is a report,
// not the flow - the flow comes from `SupportLayer::height` and from the per-path height - so the layer
// heights are asserted on the support layers themselves and this comparison is the display cross-check.
std::multiset<double> gcode_layer_heights(const std::string &gcode)
{
    std::multiset<double> heights;
    std::istringstream    lines(gcode);
    std::string           line;
    bool                  after_layer_z = false;
    while (std::getline(lines, line)) {
        if (! line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.rfind(";Z:", 0) == 0 || line.rfind("; Z_HEIGHT:", 0) == 0) {
            after_layer_z = true;
            continue;
        }
        if (after_layer_z) {
            after_layer_z = false;
            if (line.rfind(";HEIGHT:", 0) == 0)
                heights.insert(std::round(std::stod(line.substr(8)) * 1000.) / 1000.);
        }
    }
    return heights;
}

// Heights of the extrusion paths a support layer prints. This is the number Flow is built from
// (`ExtrusionPath::height`), so a gap remainder that leaked into a layer height shows up here as well -
// asserting on `SupportLayer::height` alone would not prove what the extrusions actually lay down.
//
// The role comes along because the base of the support is printed as erSupportMaterial and the interface
// and roof stack as erSupportMaterialInterface with a flow of its own: a test that only looked at one of
// the two would not see a remainder that leaked into the other.
struct SupportPathRole
{
    ExtrusionRole role;
    double        height;
};

std::vector<SupportPathRole> support_path_roles(const SupportLayer &layer)
{
    std::vector<SupportPathRole> paths;
    const ExtrusionEntityCollection flat = layer.support_fills.flatten();
    for (const ExtrusionEntity *entity : flat.entities)
        if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity))
            paths.push_back({ path->role(), std::round(double(path->height) * 1000.) / 1000. });
    return paths;
}

// A layer height at the micron resolution support_path_roles() reports path heights with.
double micron_rounded(const double value) { return std::round(value * 1000.) / 1000.; }

// Every height a support path of `layer` may legitimately carry: the layer's own height (what it lays down
// where nothing else is in its slab), and - for a support layer whose print_z falls inside this layer's
// slab - the thinner height the over-extrusion mitigation reduces that overlap to
// (SupportCommon.cpp, modulate_extrusion_by_overlapping_layers(): this print_z - the overlapping print_z).
// Anything else is a contract violation: a gap remainder may not leak into another layer's paths, and a
// reduced height may not be invented either.
bool is_allowed_support_path_height(const PrintObject &po, const SupportLayer &layer, const double height)
{
    if (std::abs(micron_rounded(layer.height) - height) < 1e-6)
        return true;
    for (const SupportLayer *other : po.support_layers())
        if (other->print_z > layer.bottom_z() + 1e-6 && other->print_z < layer.print_z - 1e-6 &&
            std::abs(micron_rounded(layer.print_z - other->print_z) - height) < 1e-6)
            return true;
    return false;
}

// Every height the object's layer stack has, rounded to a micron: a support path may only be as thick as
// one of these. Anything else (0.1 / 0.14 / 0.23 on a 0.2 mm profile) is a gap remainder.
std::multiset<double> object_layer_heights(const PrintObject &po)
{
    std::multiset<double> heights;
    for (const Layer *layer : po.layers())
        heights.insert(std::round(layer->height * 1000.) / 1000.);
    return heights;
}

// The object layer these tests expect the top contact on, below `face_z`: the highest one whose top keeps
// at least `gap` of air under the face, or nullptr when every layer is too close to it. It applies the
// same rule as TreeSupportLayerGrid::place_top_contact() to the object layers a test can see, so an
// expectation never has to hard-code a layer Z - but only as this scenario's expected-layer lookup: these
// tests print no raft and their face-gap sits above the first layer's top. It does not restate the grid's
// raft exclusion (the grid's lowest candidate is above the raft layers) nor its -1 when no layer is left
// to carry the contact; the object layer list a test sees has neither of those cases.
const Layer *top_contact_layer(const PrintObject &po, double face_z, double gap)
{
    const Layer *contact = nullptr;
    for (const Layer *layer : po.layers())
        if (layer->print_z <= face_z - gap + TreeSupport3D::TreeSupportLayerGrid::z_roundoff)
            contact = layer;
    return contact;
}

// The layer directly above `layer` in the object's stack, or nullptr at the top of it.
const Layer *layer_above(const PrintObject &po, const Layer &layer)
{
    const Layer *above = nullptr;
    for (const Layer *candidate : po.layers())
        if (candidate->print_z > layer.print_z + 1e-9 && (above == nullptr || candidate->print_z < above->print_z))
            above = candidate;
    return above;
}

// The support layer that prints at `print_z`, or nullptr. A contact is looked up by Z instead of taken
// from the end of the stack: the object layers above it are in the grid too, they just print nothing.
const SupportLayer *support_layer_at(const PrintObject &po, double print_z)
{
    for (const SupportLayer *layer : po.support_layers())
        if (std::abs(layer->print_z - print_z) < 1e-9)
            return layer;
    return nullptr;
}

// Every support layer prints on an object layer at that layer's own height, except the one thin top
// contact layer that finishes an exact gap: that layer prints exactly the Z span it stands over, and every
// one of its paths carries that height (it is the number Flow is built from). Returns the number of paths
// checked, so a caller can tell that the assertions were not vacuous.
//
// Scoped to what these tests print: an organic stack with no raft, where every support layer Z is either an
// object layer Z or the exact contact Z above one of them. It is not a general layer grid check - a raft
// puts support layers below the object's first layer, which have no object layer to match.
size_t check_support_layers_on_object_grid(const PrintObject &po, const ContactExpectation &expected)
{
    size_t                      paths_checked   = 0;
    size_t                      exact_layers    = 0;
    for (const SupportLayer *layer : po.support_layers()) {
        INFO("support layer print_z " << layer->print_z << " height " << layer->height
                                      << " expected contact " << expected.top << " exact " << expected.exact);
        const bool exact_contact = expected.exact && std::abs(layer->print_z - expected.top) < 1e-9;
        if (exact_contact) {
            ++ exact_layers;
            CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(layer->print_z - expected.grid_top, 1e-6));
            for (const SupportPathRole &path : support_path_roles(*layer)) {
                INFO("exact contact path role " << int(path.role) << " height " << path.height);
                CHECK_THAT(path.height, Catch::Matchers::WithinAbs(micron_rounded(layer->height), 1e-6));
                ++ paths_checked;
            }
            continue;
        }
        const Layer *object_layer = object_layer_at(po, layer->print_z);
        REQUIRE(object_layer != nullptr);
        CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(object_layer->height, 1e-12));
        for (const SupportPathRole &path : support_path_roles(*layer)) {
            INFO("support path role " << int(path.role) << " height " << path.height);
            // Either the object layer's own height or - only where a support layer sits inside this
            // layer's slab - the thinner overlap height. A gap remainder shows up as neither.
            CHECK(is_allowed_support_path_height(po, *layer, path.height));
            ++ paths_checked;
        }
    }
    if (expected.exact)
        CHECK(exact_layers == 1);
    else
        CHECK(exact_layers == 0);
    return paths_checked;
}

// The G-code layer-height report of a print whose exact contact layer exists differs from the same print
// without support in exactly one place: the object layer above the contact is now reached in two advances,
// the contact layer's height and the rest of that object layer. Every other height has to be identical, so
// the report shows the split the machine really makes instead of a fabricated height.
void check_gcode_heights_with_exact_contact(const std::string &supported, const std::string &plain,
                                            const PrintObject &po, const ContactExpectation &expected)
{
    const std::multiset<double> heights       = gcode_layer_heights(supported);
    std::multiset<double>       expected_heights = gcode_layer_heights(plain);
    // Both sides parse the report; two empty sets would compare equal and prove nothing.
    REQUIRE(! heights.empty());
    REQUIRE(! expected_heights.empty());
    if (! expected.exact) {
        CHECK(heights == expected_heights);
        return;
    }
    const Layer *above = object_layer_above(po, expected.top);
    REQUIRE(above != nullptr);
    const double object_height = micron_rounded(above->height);
    const double contact_part  = micron_rounded(expected.top - expected.grid_top);
    const double rest_part     = micron_rounded(above->print_z - expected.top);
    const auto   it = expected_heights.find(object_height);
    REQUIRE(it != expected_heights.end());
    expected_heights.erase(it);
    expected_heights.insert(contact_part);
    expected_heights.insert(rest_part);
    CHECK(heights == expected_heights);
}

// A 0.2 mm profile with a gap the user did not round to it. The object's layers are the slicing result and
// are not touched. The contact stays on the highest object layer that keeps at least the requested air
// under the overhang; when the requested Z lies above that layer's top, one thin support layer on top of
// it finishes the exact gap. Only that thin layer is off the object grid and its height is the Z span it
// really prints, so its Flow is the real material thickness. A remainder below the profile's minimum layer
// height cannot be printed and the contact then stays on the whole layer (wider gap, never narrower).
TEST_CASE("An organic top gap keeps the object's layers and only the exact contact off its grid",
          "[TreeSupportOrganicGap]")
{
    const double gap = GENERATE(0.1, 0.2, 0.3);
    INFO("gap " << gap);

    const TriangleMesh mesh = two_tier_mesh();
    Slic3r::Print     supported;
    slice_with_tree_support(mesh, supported, "organic", 30, 0, 0, {
        { "support_top_z_distance",       gap },
        { "support_interface_top_layers", 2 },
    });
    // Same profile, no support: the reference for both the object layers and the layer height report.
    Slic3r::Print unsupported;
    {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({
            { "enable_support",          0 },
            { "layer_height",            0.2 },
            { "support_threshold_angle", 30 },
        });
        Slic3r::Test::init_and_process_print({ mesh }, unsupported, config);
    }

    const PrintObject &po       = *supported.objects().front();
    const PrintObject &po_plain = *unsupported.objects().front();
    REQUIRE(po.layer_count() == po_plain.layer_count());
    REQUIRE(po.layer_count() == 95);
    for (size_t i = 0; i < po.layer_count(); ++ i) {
        const Layer &layer       = *po.get_layer(i);
        const Layer &layer_plain = *po_plain.get_layer(i);
        INFO("object layer " << i);
        // Every object layer keeps exactly the print_z / height / slice_z the slicing produced.
        CHECK_THAT(layer.print_z, Catch::Matchers::WithinAbs(layer_plain.print_z, 1e-12));
        CHECK_THAT(layer.height, Catch::Matchers::WithinAbs(layer_plain.height, 1e-12));
        CHECK_THAT(layer.slice_z, Catch::Matchers::WithinAbs(layer_plain.slice_z, 1e-12));
        // ... and that is the 0.2 mm profile.
        CHECK_THAT(layer.height, Catch::Matchers::WithinAbs(0.2, 1e-9));
        CHECK_THAT(layer.print_z, Catch::Matchers::WithinAbs(0.2 * double(i + 1), 1e-9));
    }

    const ContactExpectation expected = expected_contact(po, 16.0, gap);

    // SupportLayer::height: every support layer is an object layer at that layer's height, except the one
    // thin contact layer that finishes the gap. That one prints the remainder it really covers, and its
    // paths - base and interface alike - are built with the same height.
    REQUIRE(! po.support_layers().empty());
    size_t                      base_paths      = 0;
    size_t                      interface_paths = 0;
    size_t                      thin_layers     = 0;
    for (const SupportLayer *layer : po.support_layers()) {
        INFO("support layer print_z " << layer->print_z << " height " << layer->height
                                      << " expected contact " << expected.top << " exact " << expected.exact);
        const bool exact_contact = expected.exact && std::abs(layer->print_z - expected.top) < 1e-9;
        if (exact_contact) {
            ++ thin_layers;
            CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(expected.top - expected.grid_top, 1e-6));
        } else {
            const Layer *object_layer = object_layer_at(po, layer->print_z);
            REQUIRE(object_layer != nullptr);
            CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(object_layer->height, 1e-12));
        }
        // Flow / path height: what the extrusions of this layer are actually built with, base and
        // interface stack alike.
        for (const SupportPathRole &path : support_path_roles(*layer)) {
            INFO("support path role " << int(path.role) << " height " << path.height);
            if (exact_contact)
                CHECK_THAT(path.height, Catch::Matchers::WithinAbs(micron_rounded(layer->height), 1e-6));
            else
                CHECK(is_allowed_support_path_height(po, *layer, path.height));
            (path.role == erSupportMaterialInterface ? interface_paths : base_paths) += 1;
        }
    }
    CHECK(thin_layers == (expected.exact ? 1u : 0u));
    // The interface stack has a flow of its own, so both kinds have to have been looked at.
    REQUIRE(base_paths > 0);
    REQUIRE(interface_paths > 0);

    // The top contact is the highest support layer, and its top is that layer's own print_z. The air gap
    // under the overhang at 16.0 is the requested one when the thin layer exists, and at least the request
    // (at most one layer wider) when the profile refuses the remainder.
    const SupportLayer &contact = *po.support_layers().back();
    INFO("contact print_z " << contact.print_z << " = gap " << 16.0 - contact.print_z);
    CHECK_THAT(contact.print_z, Catch::Matchers::WithinAbs(expected.top, 1e-9));
    CHECK(contact.print_z <= 16.0 - gap + 1e-9);
    CHECK(contact.print_z > 16.0 - gap - 0.2 - 1e-9);
    if (expected.exact)
        CHECK_THAT(16.0 - contact.print_z, Catch::Matchers::WithinAbs(gap, 1e-9));

    // The G-code legend reports the machine's real layer changes: the object stack, and where the thin
    // contact layer exists, the split of the object layer above it. No fabricated height appears.
    check_gcode_heights_with_exact_contact(Slic3r::Test::gcode(supported), Slic3r::Test::gcode(unsupported),
                                           po, expected);
}

// The zero the tests below assert is only meaningful if the capture reads the stream the generator
// writes and the counters match its format, so both are checked against the function that emits the
// record instead of against a string this test made up.
//
// What the count covers is exactly this one function: every support diagnostic goes through
// tree_supports_show_error(), which prints "Error: <message>, critical: <0|1>" to stdout. Diagnostics the
// generator writes through BOOST_LOG_TRIVIAL instead - the "detect_overhangs takes more than 30 secs"
// warning and the informational progress lines - are not part of the count and are not asserted on, so a
// zero here says nothing about them.
TEST_CASE("The support diagnostic count sees what the generator prints", "[TreeSupport]")
{
    ScopedStdoutCapture capture;
    TreeSupport3D::tree_supports_show_error(std::string_view("Uninitialized support element. A branch may be missing.\n"), true);
    TreeSupport3D::tree_supports_show_error(std::string_view("Not precalculated Collision requested."), false);
    const std::string text = capture.finish();
    CHECK(diagnostic_count(text) == 2);
    CHECK(critical_diagnostic_count(text) == 1);
}

// The same property on variable layer height profiles: 0.2 mm up to z = 15 and 0.4 mm above it, with the
// overhang face at z = 16 - once with that profile alone, once with a 0.1 mm section in the middle. The
// object's layers stay the profile's, every support layer takes the height of the object layer it sits
// on, and the gap is quantised to those layers.
TEST_CASE("An organic top gap keeps a variable layer height profile's support layers on its heights", "[TreeSupportOrganicGap]")
{
    const double gap = GENERATE(0.1, 0.3);
    INFO("gap " << gap);

    // (z, layer height) pairs; each pair is the height used from that z upwards.
    const std::vector<std::vector<double>> profiles {
        { 0., 0.2, 15.0, 0.4, 19.0, 0.4 },
        { 0., 0.2, 8.0, 0.4, 12.0, 0.1, 14.0, 0.2, 15.0, 0.4, 19.0, 0.4 },
    };
    for (const std::vector<double> &profile : profiles) {
        INFO("profile with " << profile.size() / 2 << " sections");

        auto slice = [&profile, gap](Slic3r::Print &print, bool with_support) {
            Slic3r::Model  model;
            ModelObject   *object = model.add_object();
            object->name          = "tshape.stl";
            object->add_volume(two_tier_mesh());
            object->add_instance();
            object->layer_height_profile.set(profile);
            object->ensure_on_bed();
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set_deserialize_strict({
                { "enable_support",               with_support ? 1 : 0 },
                { "support_type",                 "tree(auto)" },
                { "support_style",                "organic" },
                { "support_threshold_angle",      30 },
                { "support_top_z_distance",       gap },
                { "support_interface_top_layers", 2 },
                // Relative extruder addressing needs a reset at each layer, as validate() points out.
                { "before_layer_change_gcode",    "G92 E0" },
            });
            print.apply(model, config);
            REQUIRE(print.validate().string.empty());
            ScopedStdoutCapture capture;
            print.process();
            return capture.finish();
        };

        Slic3r::Print supported;
        Slic3r::Print unsupported;
        const std::string diagnostics = slice(supported, true);
        slice(unsupported, false);

        // A branch the generator had to drop is reported on stdout and leaves the geometry it returns
        // unchanged, so the assertions below would stay green while branches are missing. On a variable
        // layer height the generator must not have to drop one.
        //
        // Every support diagnostic goes through tree_supports_show_error(), which prints
        // "Error: <message>, critical: <0|1>" - the same stream this capture reads. Counting the records
        // covers every critical diagnostic of the path, including the ones this test cannot name.
        CHECK(diagnostic_count(diagnostics) == 0);
        CHECK(critical_diagnostic_count(diagnostics) == 0);
        // Named as well, so a failure points at the branch that was lost rather than only at a count.
        CHECK(occurrences(diagnostics, "Uninitialized support element") == 0);
        CHECK(occurrences(diagnostics, "Potentially lost branch") == 0);

        const PrintObject &po       = *supported.objects().front();
        const PrintObject &po_plain = *unsupported.objects().front();
        REQUIRE(po.layer_count() == po_plain.layer_count());
        REQUIRE(! po.support_layers().empty());

        for (size_t i = 0; i < po.layer_count(); ++ i) {
            const Layer &layer = *po.get_layer(i);
            const Layer &plain = *po_plain.get_layer(i);
            INFO("object layer " << i << " z " << layer.print_z << " h " << layer.height);
            // The profile's own layers, unchanged by the support.
            CHECK_THAT(layer.print_z, Catch::Matchers::WithinAbs(plain.print_z, 1e-12));
            CHECK_THAT(layer.height, Catch::Matchers::WithinAbs(plain.height, 1e-12));
            CHECK_THAT(layer.slice_z, Catch::Matchers::WithinAbs(plain.slice_z, 1e-12));
        }

        const ContactExpectation expected = expected_contact(po, 16.0, gap);

        // Every support layer sits on one of those Zs and prints *that* layer's height, except the one thin
        // contact layer that finishes the exact gap: it prints the remainder it really covers, at the same
        // height in every path. A run the overlap mitigation thinned is allowed as well - here it has nothing
        // to thin, so this stays the object's height.
        size_t                      base_paths      = 0;
        size_t                      interface_paths = 0;
        size_t                      thin_layers     = 0;
        double                      contact_print_z = 0.;
        for (const SupportLayer *layer : po.support_layers()) {
            INFO("support layer print_z " << layer->print_z << " height " << layer->height
                                          << " expected contact " << expected.top << " exact " << expected.exact);
            const bool exact_contact = expected.exact && std::abs(layer->print_z - expected.top) < 1e-9;
            if (exact_contact) {
                ++ thin_layers;
                CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(expected.top - expected.grid_top, 1e-6));
            } else {
                const Layer *object_layer = object_layer_at(po, layer->print_z);
                REQUIRE(object_layer != nullptr);
                CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(object_layer->height, 1e-12));
            }
            // Base and interface / roof paths both take their height from the layer, so both are checked.
            for (const SupportPathRole &path : support_path_roles(*layer)) {
                INFO("support path role " << int(path.role) << " height " << path.height);
                if (exact_contact)
                    CHECK_THAT(path.height, Catch::Matchers::WithinAbs(micron_rounded(layer->height), 1e-6));
                else
                    CHECK(is_allowed_support_path_height(po, *layer, path.height));
                (path.role == erSupportMaterialInterface ? interface_paths : base_paths) += 1;
            }
            contact_print_z = std::max(contact_print_z, layer->print_z);
        }
        CHECK(thin_layers == (expected.exact ? 1u : 0u));
        REQUIRE(base_paths > 0);
        REQUIRE(interface_paths > 0);

        // The gap is the requested one when the profile can print the remainder, and one whole layer wider
        // when it cannot - never narrower.
        INFO("contact print_z " << contact_print_z << " expected " << expected.top
                                << " = gap " << 16.0 - contact_print_z);
        CHECK_THAT(contact_print_z, Catch::Matchers::WithinAbs(expected.top, 1e-9));
        CHECK(contact_print_z <= 16.0 - gap + 1e-9);
        CHECK(contact_print_z > 16.0 - gap - 0.4 - 1e-9);
        if (expected.exact)
            CHECK_THAT(16.0 - contact_print_z, Catch::Matchers::WithinAbs(gap, 1e-9));

        // The G-code legend reports the real machine layer changes: the profile's stack plus the split of
        // the object layer above the thin contact layer, and nothing fabricated.
        check_gcode_heights_with_exact_contact(Slic3r::Test::gcode(supported), Slic3r::Test::gcode(unsupported),
                                               po, expected);
    }
}

// Slices two_tier_mesh() with `profile` as the object's layer height profile and the settings a tree print
// of a 0.2 mm profile carries. `with_support` false slices the same profile with support off: the reference
// for the object's own stack.
void slice_with_layer_height_profile(Slic3r::Print &print, const std::vector<double> &profile, double gap,
                                     bool with_support = true)
{
    Slic3r::Model  model;
    ModelObject   *object = model.add_object();
    object->name          = "tshape.stl";
    object->add_volume(two_tier_mesh());
    object->add_instance();
    object->layer_height_profile.set(profile);
    object->ensure_on_bed();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",                     0.2 },
        { "enable_support",                   with_support ? 1 : 0 },
        { "support_type",                     "tree(auto)" },
        // tree(auto) resolves the default style to organic, which is what an unset project gets.
        { "support_style",                    "default" },
        { "support_threshold_angle",          30 },
        // Carried in the config, as a project that had it on does. Organic's layer grid is the object's
        // layers either way; the 0.2 mm profile case below says what that means for the gap.
        { "independent_support_layer_height", 1 },
        { "support_top_z_distance",           gap },
        { "support_interface_top_layers",     2 },
        // Relative extruder addressing needs a reset at each layer, as validate() points out.
        { "before_layer_change_gcode",        "G92 E0" },
    });
    print.apply(model, config);
    REQUIRE(print.validate().string.empty());
    print.process();
}

// A gap narrower than the nominal layer height is reachable only because the layers there are thinner: the
// contact is the highest object layer whose top still leaves the requested distance under the face, and on a
// 0.1 mm section that layer is one 0.1 mm step down, so the air gap itself comes out below the nominal
// 0.2 mm. Quantising the gap to `layer_height` instead puts every request below 0.2 on the same 0.2 mm
// layer - the "0.12 comes out as 0.2" shape - and the assertions below fail for it.
//
// What this adds over "An organic top gap keeps a variable layer height profile's support layers on its
// heights": that case keeps the requested gap at or above the nominal layer height and asserts the layer
// heights are not bent to reach it. This one requests a gap below the nominal, so the contact has to be
// placed on a sub-nominal layer and the air gap has to come out thinner than `layer_height`.
TEST_CASE("An organic top gap follows a variable layer height below the nominal layer height", "[TreeSupportOrganicGap]")
{
    // 0.2 mm up to z = 15 and 0.1 mm above it, so the layers under the overhang are thinner than the
    // nominal layer height. The upper plate of two_tier_mesh() starts at z = 16.0, its underside being the
    // overhang face. The profile is resampled, so the layer Zs are not 15.1 / 15.2 / ...: every expectation
    // below is derived from the stack the slice actually produced.
    const std::vector<double> profile { 0., 0.2, 15.0, 0.1, 19.0, 0.1 };
    const double              gap     = GENERATE(0.05, 0.15);
    INFO("support_top_z_distance=" << gap);

    Slic3r::Print supported;
    Slic3r::Print unsupported;
    slice_with_layer_height_profile(supported, profile, gap);
    slice_with_layer_height_profile(unsupported, profile, gap, false);

    const PrintObject &po       = *supported.objects().front();
    const PrintObject &po_plain = *unsupported.objects().front();
    REQUIRE(po.layer_count() == po_plain.layer_count());

    // The support left the object's stack - Z, height and slice plane - exactly as the same print without
    // support had it.
    for (size_t i = 0; i < po.layer_count(); ++ i) {
        const Layer &layer = *po.get_layer(i);
        const Layer &plain = *po_plain.get_layer(i);
        INFO("object layer " << i);
        CHECK_THAT(layer.print_z, Catch::Matchers::WithinAbs(plain.print_z, 1e-12));
        CHECK_THAT(layer.height, Catch::Matchers::WithinAbs(plain.height, 1e-12));
        CHECK_THAT(layer.slice_z, Catch::Matchers::WithinAbs(plain.slice_z, 1e-12));
    }

    const ContactExpectation expected   = expected_contact(po, 16.0, gap);
    const Layer            *grid_layer = top_contact_layer(po, 16.0, gap);
    REQUIRE(grid_layer != nullptr);
    CHECK_THAT(grid_layer->print_z, Catch::Matchers::WithinAbs(expected.grid_top, 1e-9));

    // The premise this case rests on: the layer the gap has to clear is thinner than the nominal layer
    // height, so the contact can sit closer to the face than one nominal layer. If the resampling stopped
    // producing such a layer, the case would only prove that a 0.2 mm grid quantises to 0.2 - fail here
    // instead of passing on that.
    const Layer *above = layer_above(po, *grid_layer);
    REQUIRE(above != nullptr);
    REQUIRE(above->height < 0.2 - 1e-6);

    const SupportLayer *contact = support_layer_at(po, expected.top);
    REQUIRE(contact != nullptr);
    REQUIRE(contact->has_extrusions());
    // Nothing is printed above the contact: it is the top of the support stack under this face, not just
    // the last layer of a list.
    for (const SupportLayer *layer : po.support_layers())
        if (layer->has_extrusions()) {
            INFO("support layer print_z " << layer->print_z << " above the contact " << contact->print_z);
            CHECK(layer->print_z <= contact->print_z + 1e-9);
        }

    const double actual_gap = 16.0 - contact->print_z;
    INFO("contact print_z " << contact->print_z << " expected " << expected.top << " = gap " << actual_gap
         << " contact layer height " << contact->height << " grid top " << expected.grid_top
         << " exact " << expected.exact);
    CHECK_THAT(contact->print_z, Catch::Matchers::WithinAbs(expected.top, 1e-9));
    // At least the requested distance. When the profile can print the remainder it is exactly the request;
    // when it cannot, the overshoot is bounded by the step to the next object layer that is really there.
    CHECK(actual_gap >= gap - 1e-9);
    if (expected.exact) {
        CHECK_THAT(actual_gap, Catch::Matchers::WithinAbs(gap, 1e-9));
        CHECK_THAT(contact->height, Catch::Matchers::WithinAbs(expected.top - expected.grid_top, 1e-6));
    } else {
        CHECK(actual_gap < gap + (above->print_z - contact->print_z) + 1e-9);
    }
    // A request below 0.2 still gives less than 0.2 of air, on a grid layer thinner than 0.2 - quantising
    // to the nominal layer height would give 0.2 of air on a 0.2 mm layer for both.
    CHECK(actual_gap < 0.2 - 1e-6);
    CHECK(grid_layer->height < 0.2 - 1e-6);

    // The 0.1 mm grid layers are printed as 0.1; the one thin contact layer, if any, as the remainder it
    // covers - base paths and interface alike.
    const size_t paths_checked = check_support_layers_on_object_grid(po, expected);
    REQUIRE(paths_checked > 0);
}

// The settings of a reported project in the small: support_top_z_distance 0.12 on a 0.2 mm profile,
// tree(auto) with the default style - organic - and independent_support_layer_height on. The object's
// 0.2 mm layers are untouched; the contact takes the highest object layer that keeps at least 0.12 and one
// thin support layer finishes the remaining 0.08, so the printed air gap is the 0.12 the config asked for
// rather than a whole object layer. The remainder is printable on this profile (it is at least the minimum
// layer height); a smaller one would leave the contact on the whole layer and widen the gap.
//
// independent_support_layer_height is set here because a project that had it on carries it in its config.
// It does not give organic independent support layers: organic's grid is the object's layers regardless.
// Slicing's layer synchronized gap processing reads it (it decides whether the top, bottom and raft gaps
// are rounded to whole object layers, and the Cura derived support planner plans on those gaps), and none
// of that moves the organic grid; the GUI does not offer the setting for organic ("Independent support
// layer height is unavailable for organic tree supports"). What this case pins is that the setting reaches
// the config and does not move the grid - not that the support layers are independent.
//
// What this case covers: the raw millimetre in the config, the style routing to the organic generator, and
// that the gap stays exact below the nominal layer height. The two variable layer height cases ("... follows
// a variable layer height below the nominal layer height" and "... takes its contact from the layers a mixed
// profile produced") are the ones that separate the rules - a fix that scales the gap by the nominal layer
// height instead of following the actual stack lands on a different layer there.
//
// The numbers are this test's own mesh and config. A real project's slice depends on its geometry and its
// profile and is not reproducible from this repository.
TEST_CASE("A gap below the layer height is printed exactly on a 0.2 mm profile", "[TreeSupportOrganicGap]")
{
    Slic3r::Print print;
    slice_with_tree_support(two_tier_mesh(), print, "default", 30, 0, 0, {
        { "independent_support_layer_height", 1 },
        { "support_top_z_distance",           0.12 },
        { "support_interface_top_layers",     3 },
    });
    const PrintObject &po = *print.objects().front();

    // default + tree(auto) is the organic generator, the one that places the contact on the object's grid.
    Slic3r::SupportParameters support_params(po);
    CHECK(support_params.support_style == smsTreeOrganic);

    // The raw millimetre reaches the organic settings: it is not the round-to-whole-layers value the Cura
    // derived engines read from the same struct.
    Slic3r::TreeSupport3D::TreeSupportMeshGroupSettings mesh_settings(po);
    CHECK_THAT(mesh_settings.support_top_z_distance_mm, Catch::Matchers::WithinAbs(0.12, 1e-9));

    const ContactExpectation expected = expected_contact(po, 16.0, 0.12);
    INFO("expected contact " << expected.top << " grid top " << expected.grid_top << " exact " << expected.exact);
    REQUIRE(expected.exact);

    const SupportLayer *contact = support_layer_at(po, expected.top);
    REQUIRE(contact != nullptr);
    REQUIRE(contact->has_extrusions());
    INFO("contact print_z " << contact->print_z << " = gap " << 16.0 - contact->print_z
                            << " height " << contact->height);
    // The printed gap is the requested distance ...
    CHECK_THAT(16.0 - contact->print_z, Catch::Matchers::WithinAbs(0.12, 1e-9));
    // ... finished by a thin layer that prints the remainder it covers, not the object layer's 0.2 mm.
    CHECK_THAT(contact->height, Catch::Matchers::WithinAbs(expected.top - expected.grid_top, 1e-6));
    CHECK(contact->height < 0.2 - 1e-6);
}

// The same question on a profile that is not one uniform thin section under the face: 0.2 mm layers up to
// z = 15, a thin dip towards 0.1 mm, and back towards 0.2 mm at the face. The resampling spreads both
// transitions, so the grid under the face is a graded set of heights rather than a 0.1 or a 0.2 step, and
// the contact has to be the highest of those layers that keeps the gap - neither a copy of a nominal 0.2 mm
// grid nor a fixed 0.1 mm one.
TEST_CASE("An organic top gap takes its contact from the layers a mixed profile produced", "[TreeSupportOrganicGap]")
{
    const std::vector<double> profile { 0., 0.2, 15.0, 0.1, 15.9, 0.2, 19.0, 0.2 };
    const double              gap     = GENERATE(0.05, 0.15);
    INFO("support_top_z_distance=" << gap);

    Slic3r::Print print;
    slice_with_layer_height_profile(print, profile, gap);
    const PrintObject &po = *print.objects().front();

    const ContactExpectation expected   = expected_contact(po, 16.0, gap);
    const Layer            *grid_layer = top_contact_layer(po, 16.0, gap);
    REQUIRE(grid_layer != nullptr);
    CHECK_THAT(grid_layer->print_z, Catch::Matchers::WithinAbs(expected.grid_top, 1e-9));

    // The premise: the layer spanning the face is a nominal one, the contact layer under it is thinner, and
    // it is not the 0.1 mm of the dip either. Without all three this case could pass on a uniform grid.
    const Layer *below_face = top_contact_layer(po, 16.0, 0.);
    REQUIRE(below_face != nullptr);
    const Layer *face_layer = layer_above(po, *below_face);
    REQUIRE(face_layer != nullptr);
    CHECK(face_layer->height > 0.195);
    REQUIRE(grid_layer->height < 0.195);
    CHECK(grid_layer->height > 0.1 + 1e-6);

    const Layer *above = layer_above(po, *grid_layer);
    REQUIRE(above != nullptr);

    const SupportLayer *contact = support_layer_at(po, expected.top);
    REQUIRE(contact != nullptr);
    REQUIRE(contact->has_extrusions());
    for (const SupportLayer *layer : po.support_layers())
        if (layer->has_extrusions()) {
            INFO("support layer print_z " << layer->print_z << " above the contact " << contact->print_z);
            CHECK(layer->print_z <= contact->print_z + 1e-9);
        }

    const double actual_gap = 16.0 - contact->print_z;
    INFO("contact print_z " << contact->print_z << " expected " << expected.top << " = gap " << actual_gap
         << " contact layer height " << contact->height << " exact " << expected.exact);
    CHECK_THAT(contact->print_z, Catch::Matchers::WithinAbs(expected.top, 1e-9));
    // At least the requested distance; exactly it whenever the profile can print the remainder, and otherwise
    // bounded by the step to the next object layer that is really there.
    CHECK(actual_gap >= gap - 1e-9);
    if (expected.exact)
        CHECK_THAT(actual_gap, Catch::Matchers::WithinAbs(gap, 1e-9));
    else
        CHECK(actual_gap < gap + (above->print_z - contact->print_z) + 1e-9);

    const size_t paths_checked = check_support_layers_on_object_grid(po, expected);
    REQUIRE(paths_checked > 0);
}

// tree_slim does not go through the organic grid - generate_tree_support_3D() runs for smsTreeOrganic
// only - so the organic contact placement must leave it alone. With independent_support_layer_height
// off the Cura derived planner keeps its support layers on the object's grid, so the control is: the
// grid stays a verbatim copy of the object layers and every support layer prints one of those heights.
TEST_CASE("A tree_slim print keeps its own support layer heights", "[TreeSupportOrganicGap]")
{
    Slic3r::Print print;
    slice_with_tree_support(two_tier_mesh(), print, "tree_slim", 30, 0, 0, {
        // The layer synchronised mode: tree_slim then plans one support layer per object layer, so a
        // height that is not the object's could only come from the organic gap logic.
        { "independent_support_layer_height", 0 },
        { "support_top_z_distance",           0.3 },
        { "support_interface_top_layers",     2 },
    });
    const PrintObject &po = *print.objects().front();
    REQUIRE(! po.support_layers().empty());

    // The grid is the object's layer stack, whatever the gap: no organic contact logic writes into it.
    using namespace Slic3r::TreeSupport3D;
    TreeSupportMeshGroupSettings mesh_settings(po);
    REQUIRE(mesh_settings.layer_grid.size() == size_t(po.layer_count()));
    for (size_t i = 0; i < po.layer_count(); ++ i) {
        const Layer &layer = *po.get_layer(i);
        INFO("layer " << i);
        CHECK_THAT(mesh_settings.layer_grid.z(LayerIndex(i)), Catch::Matchers::WithinAbs(layer.print_z, 1e-9));
        CHECK_THAT(mesh_settings.layer_grid.h(LayerIndex(i)), Catch::Matchers::WithinAbs(layer.height, 1e-12));
        CHECK_THAT(mesh_settings.layer_grid.bottom_z(LayerIndex(i)),
                   Catch::Matchers::WithinAbs(layer.print_z - layer.height, 1e-12));
    }

    // Every support layer sits on an object layer Z and prints that layer's height - the same property
    // the organic generator has to keep, reached by its own planner rather than by the organic grid.
    for (const SupportLayer *layer : po.support_layers()) {
        INFO("support layer print_z " << layer->print_z << " height " << layer->height);
        const auto it = std::find_if(po.layers().begin(), po.layers().end(),
            [&layer](const Layer *object_layer) { return std::abs(object_layer->print_z - layer->print_z) < 1e-9; });
        CHECK(it != po.layers().end());
        if (it == po.layers().end())
            continue;
        CHECK_THAT(layer->height, Catch::Matchers::WithinAbs((*it)->height, 1e-12));
    }
}


// A layer's slices are taken at its mid-plane, so the overhang area of a layer is caused by a face that
// lies in the slab of the layer *below* it: with 0.4 mm layers and a plate starting at 15.9, only the
// layer 16.0 .. 16.4 sees the plate, and its face sits inside 15.6 .. 16.0. The face search therefore has
// to reach into that layer; a window narrowed to the area's own slab finds nothing and falls back to the
// layer bottom, which is not the face. A face further down belongs to a different overhang and must not
// be taken.
TEST_CASE("The overhang face is searched in the slab of the layer below the area's layer", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;

    // Two shelves sharing one XY footprint: the plate whose underside at 15.9 is the face of the area,
    // and a second shelf at 15.1, a whole layer further down, whose underside is a different overhang.
    TriangleMesh mesh = make_cube(10.f, 10.f, 0.1f);
    mesh.translate(0.f, 0.f, 15.1f);
    TriangleMesh face_plate = make_cube(10.f, 10.f, 0.1f);
    face_plate.translate(0.f, 0.f, 15.9f);
    mesh.merge(face_plate);

    // A 0.4 mm grid: layer 40 tops out at 16.4 and starts at 16.0, so the face at 15.9 lies in the slab
    // (15.6, 16.0] of layer 39.
    TreeSupportLayerGrid grid;
    double               bottom = 0.;
    for (int i = 0; i < 50; ++ i) {
        TreeSupportLayerGrid::Entry entry;
        entry.bottom_z      = bottom;
        entry.obj_bottom_z  = bottom;
        entry.print_z       = bottom + 0.4;
        entry.height        = 0.4;
        entry.obj_layer_idx = LayerIndex(i);
        bottom              = entry.print_z;
        grid.entries.emplace_back(entry);
    }

    OverhangFacetZIndex facets;
    facets.build(mesh);
    const Polygons area{ Polygon{ { Point::new_scale(0., 0.), Point::new_scale(10., 0.),
                                   Point::new_scale(10., 10.), Point::new_scale(0., 10.) } } };

    // The contact of the area is placed from the face inside the layer below, not from the layer bottom.
    const OverhangContact contact = place_overhang_contact(grid, 40, mesh, facets, area, 0.2);
    REQUIRE(contact.face_z.has_value());
    // The mesh is a float mesh, so the face carries its vertex precision.
    CHECK_THAT(*contact.face_z, Catch::Matchers::WithinAbs(15.9, 1e-4));
    // 15.9 - 0.2 asks for 15.7, which no layer top reaches, so the contact is layer 38 at 15.6.
    CHECK(contact.contact_layer == 38);
    CHECK_THAT(grid.z(contact.contact_layer), Catch::Matchers::WithinAbs(15.6, 1e-4));
    CHECK_THAT(15.9 - grid.z(contact.contact_layer), Catch::Matchers::WithinAbs(0.3, 1e-4));
    // The lower shelf sits below the window of the area, so it never becomes this area's face ...
    CHECK_THAT(*overhang_face_z(mesh, facets, area, grid.bottom_z(38), grid.z(39)), Catch::Matchers::WithinAbs(15.9, 1e-4));
    // ... while a window that starts at the area's own layer bottom does not see the real face at all.
    CHECK_FALSE(overhang_face_z(mesh, facets, area, grid.bottom_z(40), grid.z(40)).has_value());
}

// A sloped face can cross the whole search window while its lowest vertex lies below it. Such a facet is
// a candidate - its Z range overlaps the window - and what it contributes is the window bottom, the lowest
// Z its face inside the window can have: measuring the gap from the top of the layer below the area's
// instead closed it to 0.32 mm against a requested 0.4. The case that a *higher* face of the same area
// could stand in for the slope is the next test's.
TEST_CASE("A sloped overhang face below the search window still keeps the requested gap", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;

    // One down facing triangle: 16.2 at the origin, 15.0 at y = 10 and 16.0 at x = 10. Its lowest vertex
    // is below the slab the area's layer searches, the facet itself reaches up into it.
    TriangleMesh mesh(std::vector<Vec3f>{ Vec3f(0.f, 0.f, 16.2f), Vec3f(0.f, 10.f, 15.0f), Vec3f(10.f, 0.f, 16.f) },
                      std::vector<Vec3i32>{ Vec3i32(0, 1, 2) });

    // 0.4 mm layers: layer 40 tops out at 16.4, so the face of its area is searched in (15.6, 16.4].
    TreeSupportLayerGrid grid;
    double               bottom = 0.;
    for (int i = 0; i < 50; ++ i) {
        TreeSupportLayerGrid::Entry entry;
        entry.bottom_z      = bottom;
        entry.obj_bottom_z  = bottom;
        entry.print_z       = bottom + 0.4;
        entry.height        = 0.4;
        entry.obj_layer_idx = LayerIndex(i);
        bottom              = entry.print_z;
        grid.entries.emplace_back(entry);
    }
    const double window_lo = grid.bottom_z(39);
    REQUIRE_THAT(window_lo, Catch::Matchers::WithinAbs(15.6, 1e-9));
    // The premise of the case: the facet's lowest vertex is outside the window it spans.
    REQUIRE(15.0 < window_lo);

    OverhangFacetZIndex facets;
    facets.build(mesh);
    const Polygons area{ Polygon{ { Point::new_scale(0., 0.), Point::new_scale(2., 0.),
                                   Point::new_scale(2., 2.), Point::new_scale(0., 2.) } } };

    const std::vector<TreeSupportLayerGrid::Entry> before = grid.entries;
    constexpr double                               requested = 0.4;
    const OverhangContact contact = place_overhang_contact(grid, 40, mesh, facets, area, requested);
    REQUIRE(contact.contact_layer >= 1);

    // The contact is one of the grid's own layers, printed at that layer's height: no remainder of the
    // gap became a Z or a layer height.
    const TreeSupportLayerGrid::Entry *entry = grid_entry_at(grid, grid.z(contact.contact_layer));
    REQUIRE(entry != nullptr);
    CHECK_THAT(entry->height, Catch::Matchers::WithinAbs(0.4, 1e-12));
    CHECK_THAT(grid.z(contact.contact_layer) - grid.bottom_z(contact.contact_layer),
               Catch::Matchers::WithinAbs(0.4, 1e-12));
    REQUIRE(grid.entries.size() == before.size());
    for (size_t i = 0; i < before.size(); ++ i) {
        INFO("grid layer " << i);
        CHECK_THAT(grid.entries[i].print_z, Catch::Matchers::WithinAbs(before[i].print_z, 1e-12));
        CHECK_THAT(grid.entries[i].bottom_z, Catch::Matchers::WithinAbs(before[i].bottom_z, 1e-12));
        CHECK_THAT(grid.entries[i].height, Catch::Matchers::WithinAbs(before[i].height, 1e-12));
    }

    // The lowest point of the face inside the area is the far corner (2, 2): the triangle drops 0.12 per
    // millimetre in y and 0.02 in x. The air gap under it may come out wider than requested, never
    // narrower - 0.32 mm is what measuring from the layer top above the face produced.
    const double face_z = 16.2 - 0.02 * 2. - 0.12 * 2.;
    INFO("contact print_z " << grid.z(contact.contact_layer) << " lowest face " << face_z);
    CHECK(face_z - grid.z(contact.contact_layer) >= requested - 1e-9);
}

// A raft is not part of the layered object: its layers are thicker (0.3 on a 0.2 mm profile) and it is
// not a stack the branch grows along, so letting them into the layer counts would shorten every count
// and change the support of every existing raft print. The counts keep the fixed layer values, and the
// support layers still sit on the grid the raft and the object layers form together.
TEST_CASE("An organic raft keeps the layer counts of the object's own layers", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;
    const TriangleMesh mesh = two_tier_mesh();
    auto               slice = [&mesh](Slic3r::Print &print, int raft_layers) {
        slice_with_tree_support(mesh, print, "organic", 30, 0, raft_layers, {
            { "support_top_z_distance",       0.2 },
            { "support_interface_top_layers", 2 },
        });
    };

    Slic3r::Print rafted;
    Slic3r::Print plain;
    slice(rafted, 2);
    slice(plain, 0);

    const PrintObject            &po            = *rafted.objects().front();
    const PrintObject            &po_plain      = *plain.objects().front();
    TreeSupportMeshGroupSettings  mesh_settings(po);
    TreeSupportMeshGroupSettings  mesh_settings_plain(po_plain);
    REQUIRE(mesh_settings.layer_grid.raft_layers > 0);
    // The case is really exercised: a raft layer is thicker than the object's 0.2 mm layers, so a count
    // that walked through it would come out shorter.
    double thickest_raft_layer = 0.;
    for (size_t i = 0; i < mesh_settings.layer_grid.raft_layers; ++ i)
        thickest_raft_layer = std::max(thickest_raft_layer, mesh_settings.layer_grid.entries[i].height);
    CHECK(thickest_raft_layer > 0.2 + 1e-9);

    TreeSupportSettings config(mesh_settings, po.slicing_parameters());
    TreeSupportSettings config_plain(mesh_settings_plain, po_plain.slicing_parameters());

    // Machine readable control against the fixed layer height formulas the grid replaced.
    CHECK(config.layer_start_bp_radius ==
          LayerIndex((config.bp_radius - config.branch_radius) / config.bp_radius_increase_per_layer));
    check_counts_unchanged(config, config_plain);

    // Every support layer prints a layer of that grid, at that layer's own height - the raft layers
    // included, and never a gap remainder.
    REQUIRE(! po.support_layers().empty());
    for (const SupportLayer *layer : po.support_layers()) {
        INFO("support layer print_z " << layer->print_z << " height " << layer->height);
        const TreeSupportLayerGrid::Entry *entry = grid_entry_at(mesh_settings.layer_grid, layer->print_z);
        REQUIRE(entry != nullptr);
        CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(entry->height, 1e-12));
    }
}

// The first object layer is printed thicker for bed adhesion - a first layer setting, not a variable
// layer height - so it must not move the layer counts either. The object keeps its own 0.32 / 0.2
// layers, untouched by the support, and every support layer prints one of them.
TEST_CASE("A thicker first layer keeps the organic layer counts", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;
    const TriangleMesh mesh = two_tier_mesh();
    auto               slice = [&mesh](Slic3r::Print &print, double first_layer_height, int support) {
        slice_with_tree_support(mesh, print, "organic", 30, 0, 0, {
            { "enable_support",               support },
            { "initial_layer_print_height",   first_layer_height },
            { "support_top_z_distance",       0.2 },
            { "support_interface_top_layers", 2 },
        });
    };

    Slic3r::Print thick;
    Slic3r::Print thick_plain;
    Slic3r::Print uniform;
    slice(thick, 0.32, 1);
    slice(thick_plain, 0.32, 0);
    slice(uniform, 0.2, 1);

    const PrintObject            &po       = *thick.objects().front();
    const PrintObject            &po_plain = *thick_plain.objects().front();
    TreeSupportMeshGroupSettings  mesh_settings(po);
    TreeSupportMeshGroupSettings  mesh_settings_uniform(*uniform.objects().front());

    // The case is really exercised: the first layer is thicker than the regular ones.
    REQUIRE(po.layer_count() > 1);
    CHECK_THAT(po.get_layer(0)->height, Catch::Matchers::WithinAbs(0.32, 1e-9));
    CHECK(po.get_layer(1)->height < 0.32 - 1e-9);

    TreeSupportSettings config(mesh_settings, po.slicing_parameters());
    TreeSupportSettings config_uniform(mesh_settings_uniform, uniform.objects().front()->slicing_parameters());
    CHECK(config.layer_start_bp_radius ==
          LayerIndex((config.bp_radius - config.branch_radius) / config.bp_radius_increase_per_layer));
    check_counts_unchanged(config, config_uniform);

    // The support does not touch the object's layer stack ...
    REQUIRE(po.layer_count() == po_plain.layer_count());
    for (size_t i = 0; i < po.layer_count(); ++ i) {
        INFO("object layer " << i);
        CHECK_THAT(po.get_layer(i)->print_z, Catch::Matchers::WithinAbs(po_plain.get_layer(i)->print_z, 1e-12));
        CHECK_THAT(po.get_layer(i)->height, Catch::Matchers::WithinAbs(po_plain.get_layer(i)->height, 1e-12));
    }
    // ... and every support layer prints one of those layers at its height, except the one thin contact
    // layer that finishes the exact gap: the first layer's 0.32 mm shifts the whole stack, so 0.2 of air
    // does not land on a layer top and the remainder is printed by a layer of its own.
    REQUIRE(! po.support_layers().empty());
    const ContactExpectation expected = expected_contact(po, 16.0, 0.2);
    size_t                   thin_layers = 0;
    for (const SupportLayer *layer : po.support_layers()) {
        INFO("support layer print_z " << layer->print_z << " height " << layer->height
                                      << " expected contact " << expected.top << " exact " << expected.exact);
        if (expected.exact && std::abs(layer->print_z - expected.top) < 1e-9) {
            ++ thin_layers;
            CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(expected.top - expected.grid_top, 1e-6));
            continue;
        }
        const Layer *object_layer = object_layer_at(po, layer->print_z);
        REQUIRE(object_layer != nullptr);
        CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(object_layer->height, 1e-12));
    }
    CHECK(thin_layers == (expected.exact ? 1u : 0u));
}

// One overhang area can hold two downward faces at different heights: a slope whose lowest vertex sits
// below the window the layer searches, and a higher face of the very same area whose vertex sits inside
// it. Indexing the facets by their lowest vertex alone dropped the slope and kept the higher face, so the
// contact was placed from a face that is not the lowest one over the area - 0.23 mm of air under the
// slope where 0.4 was asked for. The candidates are now the facets whose Z range *overlaps* the window,
// so the lower face wins.
TEST_CASE("A higher face of the same overhang area does not lift the contact above the slope crossing the window",
          "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;

    // The slope reaches from 15.0 at y = 10 up to 16.2 at the origin, and over the area (0, 0) .. (2, 2)
    // its face drops to 16.2 - 0.02 * 2 - 0.12 * 2 = 15.92. The two triangles at 16.1 are a down facing
    // shelf over the same XY, well above the relevant part of the slope.
    const TriangleMesh mesh(
        std::vector<Vec3f>{ { 0.f, 0.f, 16.2f }, { 0.f, 10.f, 15.0f }, { 10.f, 0.f, 16.f },
                            { 0.f, 0.f, 16.1f }, { 0.f, 2.f, 16.1f }, { 2.f, 2.f, 16.1f },
                            { 0.f, 0.f, 16.1f }, { 2.f, 2.f, 16.1f }, { 2.f, 0.f, 16.1f } },
        std::vector<Vec3i32>{ { 0, 1, 2 }, { 3, 4, 5 }, { 6, 7, 8 } });

    // 0.4 mm layers: layer 40 tops out at 16.4 and starts at 16.0, so the window it searches is
    // [15.6, 16.4] and the slope's own lowest vertex at 15.0 lies below it.
    TreeSupportLayerGrid grid;
    double               bottom = 0.;
    for (int i = 0; i < 50; ++ i) {
        TreeSupportLayerGrid::Entry entry;
        entry.bottom_z      = bottom;
        entry.obj_bottom_z  = bottom;
        entry.print_z       = bottom + 0.4;
        entry.height        = 0.4;
        entry.obj_layer_idx = LayerIndex(i);
        bottom              = entry.print_z;
        grid.entries.emplace_back(entry);
    }
    const double window_lo = grid.bottom_z(39);
    REQUIRE_THAT(window_lo, Catch::Matchers::WithinAbs(15.6, 1e-9));
    // The premise of the case: the slope's lowest vertex is outside the window, the shelf's is inside it.
    REQUIRE(15.0 < window_lo);
    REQUIRE(16.1 > window_lo);

    OverhangFacetZIndex facets;
    facets.build(mesh);
    const Polygons area{ Polygon{ { Point::new_scale(0., 0.), Point::new_scale(2., 0.),
                                   Point::new_scale(2., 2.), Point::new_scale(0., 2.) } } };
    const double   slope_face_over_area = 16.2 - 0.02 * 2. - 0.12 * 2.;

    const std::vector<TreeSupportLayerGrid::Entry> before = grid.entries;
    constexpr double                               requested = 0.4;
    const OverhangContact contact = place_overhang_contact(grid, 40, mesh, facets, area, requested);
    REQUIRE(contact.face_z.has_value());
    INFO("reference " << *contact.face_z << " slope face over the area " << slope_face_over_area
                      << " contact " << grid.z(contact.contact_layer));

    // The reference is no higher than the face the slope holds over the area, and it is not the shelf's
    // 16.1: taking that one is what puts the contact a layer too high.
    CHECK(*contact.face_z <= slope_face_over_area + 1e-6);
    CHECK(*contact.face_z < 16.1 - 1e-6);
    // The contact is one of the grid's own layers, at that layer's height.
    REQUIRE(contact.contact_layer >= 1);
    REQUIRE(grid_entry_at(grid, grid.z(contact.contact_layer)) != nullptr);
    CHECK_THAT(grid.z(contact.contact_layer) - grid.bottom_z(contact.contact_layer),
               Catch::Matchers::WithinAbs(0.4, 1e-12));
    // The air gap under the slope is at least the requested distance - the candidate rule this replaced
    // put the contact at 15.6 and left 0.32 mm under it.
    CHECK(slope_face_over_area - grid.z(contact.contact_layer) >= requested - 1e-9);
    // No gap remainder became a Z or a layer height anywhere in the grid.
    REQUIRE(grid.entries.size() == before.size());
    for (size_t i = 0; i < before.size(); ++ i) {
        INFO("grid layer " << i);
        CHECK_THAT(grid.entries[i].print_z, Catch::Matchers::WithinAbs(before[i].print_z, 1e-12));
        CHECK_THAT(grid.entries[i].bottom_z, Catch::Matchers::WithinAbs(before[i].bottom_z, 1e-12));
        CHECK_THAT(grid.entries[i].height, Catch::Matchers::WithinAbs(before[i].height, 1e-12));
    }
}

// The same property on a real model and through the real pipeline: a prism whose underside is a shallow
// slope (5.7 degrees below horizontal) that runs into a horizontal shelf 0.35 mm above where the slope
// leaves the layer below. With 0.4 mm layers the overhang area of one layer covers both faces - the
// slope's portion and the whole shelf - so the contact must be placed from the slope, not from the shelf.
//
// The numbers are derived from the settings, not measured: the layer below is expanded by
// layer_height / tan(31 deg) = 0.666 before the slices are differenced, so the area of the layer whose
// slice is at 3.4 starts at x = 10 + 0.666, where the slope (z = 2 + 0.1 * x) is at 3.067; the shelf is at
// 3.35. A 0.4 mm gap under the slope asks for a top at or below 2.667, i.e. layer top 2.4, while measuring
// from the shelf asks for 2.95, i.e. layer top 2.8 - which leaves 0.267 mm under the slope.
TEST_CASE("A shallow slope sharing its overhang area with a shelf keeps the requested top gap", "[TreeSupportOrganicGap]")
{
    const TriangleMesh mesh = sloped_shelf_mesh();
    constexpr double   gap  = 0.4;

    Slic3r::Model         model;
    Slic3r::ModelObject  *object = model.add_object();
    object->name = "sloped_shelf.stl";
    object->add_volume(mesh);
    // No instance offset: the Zs this test asserts are the mesh's own.
    object->add_instance();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_support",               1 },
        { "support_type",                 "tree(auto)" },
        { "support_style",                "organic" },
        { "support_threshold_angle",      30 },
        { "support_top_z_distance",       gap },
        { "support_interface_top_layers", 2 },
        { "layer_height",                 0.4 },
        { "initial_layer_print_height",   0.4 },
        // A 0.4 mm layer has to be thinner than every extrusion width, as validate() points out, so the
        // print runs a 0.6 mm nozzle instead of the default one.
        { "nozzle_diameter",              "0.6" },
        // Relative extruder addressing needs a reset at each layer, as validate() points out.
        { "before_layer_change_gcode",    "G92 E0" },
    });
    Slic3r::Print print;
    print.apply(model, config);
    const std::string validation = print.validate().string;
    INFO("validate: " << validation);
    REQUIRE(validation.empty());
    print.process();

    const PrintObject &po = *print.objects().front();
    // The top contact of the highest overhang is the highest support layer that carries fill.
    const SupportLayer *contact = nullptr;
    for (const SupportLayer *layer : po.support_layers()) {
        Points points;
        layer->support_fills.collect_points(points);
        if (! points.empty() && (contact == nullptr || layer->print_z > contact->print_z))
            contact = layer;
    }
    REQUIRE(contact != nullptr);
    INFO("contact print_z " << contact->print_z << " height " << contact->height);

    // The contact is one of the object's own layers, never a Z of its own and never a gap remainder.
    CHECK_THAT(contact->print_z - 0.4,
               Catch::Matchers::WithinAbs(std::round((contact->print_z - 0.4) / 0.4) * 0.4, 1e-9));
    CHECK_THAT(contact->height, Catch::Matchers::WithinAbs(0.4, 1e-9));

    // The reference the contact was placed from: the slope at the lower edge of the area, derived above.
    const double area_edge_x        = 10. + 0.4 / std::tan(M_PI * 31. / 180.);
    const double slope_face_at_edge = 2. + 0.1 * area_edge_x;
    INFO("slope face at the area edge " << slope_face_at_edge);
    CHECK(contact->print_z <= slope_face_at_edge - gap + 1e-6);

    // ... and the same statement per support point, measured against the mesh itself: no point of the
    // contact may be closer to the surface above it than the requested gap. The slices live in the print
    // frame, which is the mesh frame moved onto the bed, so the shift between the two is taken from the
    // slices of a layer that spans the whole footprint instead of being assumed.
    const BoundingBox  slices_bb  = get_extents(po.get_layer(8)->lslices);
    const BoundingBoxf3 mesh_bb   = mesh.bounding_box();
    const double       shift_x    = unscaled(slices_bb.min.x()) - double(mesh_bb.min.x());
    const double       shift_y    = unscaled(slices_bb.min.y()) - double(mesh_bb.min.y());
    INFO("print frame shift " << shift_x << ", " << shift_y);
    Points points;
    contact->support_fills.collect_points(points);
    REQUIRE(! points.empty());
    size_t                checked = 0;
    std::optional<double> lowest_face;
    for (const Point &p : points) {
        const double                x    = unscaled(p.x()) - shift_x;
        const double                y    = unscaled(p.y()) - shift_y;
        const std::optional<double> face = face_above(mesh, x, y, contact->print_z + 1e-6);
        if (! face)
            continue;
        ++ checked;
        if (! lowest_face || *face < *lowest_face)
            lowest_face = *face;
        INFO("support point " << x << ", " << y << " at " << contact->print_z << " under " << *face);
        CHECK(*face - contact->print_z >= gap - 1e-9);
    }
    // The loop has to have looked at points under the slope, not only under the shelf, or the assertion
    // above says nothing about this case.
    REQUIRE(checked > 10);
    REQUIRE(lowest_face.has_value());
    INFO("lowest face above the contact " << *lowest_face);
    CHECK(*lowest_face < 3.35 - 1e-6);
}

// A belt print slices its mesh in the frame `trafo_sliced()` describes - belt rotation and min-Z lift
// included - and adds `belt_global_z_offset()` to `Layer::print_z` only afterwards (PrintObject::slice()).
// The organic layer grid is built from those layers with that offset removed (TreeSupportMeshGroupSettings
// ::build_layer_grid()), so the transformed mesh and the grid are in one and the same frame and the
// overhang search must not convert between them. This runs the whole pipeline on a belt printer, where the
// offset is not zero, and measures the gap under every point of the contact against the mesh.
// The belt fixture: two_tier_mesh() on a 45 degree belt printer, with and without organic support.
// Returns what the generator printed to stdout, i.e. the support diagnostics.
std::string slice_belt_two_tier(Slic3r::Print &print, const TriangleMesh &mesh, const bool with_support, const double gap)
{

    Slic3r::Model        model;
    Slic3r::ModelObject *object = model.add_object();
    object->name = "two_tier.stl";
    object->add_volume(mesh);
    object->add_instance();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "belt_printer",                 1 },
        { "belt_slice_rotation",          "x" },
        { "belt_slice_rotation_angle",    45 },
        { "gcode_remap_x",                "rev_x" },
        { "gcode_remap_y",                "pos_z" },
        { "gcode_remap_z",                "pos_y" },
        { "enable_support",               with_support ? 1 : 0 },
        { "support_type",                 "tree(auto)" },
        { "support_style",                "organic" },
        { "support_threshold_angle",      30 },
        { "support_top_z_distance",       gap },
        { "support_interface_top_layers", 2 },
        { "layer_height",                 0.2 },
        { "initial_layer_print_height",   0.2 },
        { "skirt_loops",                  0 },
        { "z_hop",                        0 },
        { "machine_start_gcode",          "T[initial_tool]\n" },
        { "before_layer_change_gcode",    "G92 E0" },
    });
    print.apply(model, config);
    const std::string validation = print.validate().string;
    INFO("validate: " << validation);
    REQUIRE(validation.empty());
    ScopedStdoutCapture capture;
    print.process();
    return capture.finish();
}

TEST_CASE("A belt print keeps the organic top gap it asked for", "[TreeSupportOrganicGap][belt]")
{
    const TriangleMesh mesh = two_tier_mesh();
    constexpr double  gap   = 0.25;


    Slic3r::Print     supported;
    Slic3r::Print     unsupported;
    const std::string diagnostics = slice_belt_two_tier(supported, mesh, true, gap);
    slice_belt_two_tier(unsupported, mesh, false, gap);
    CHECK(diagnostic_count(diagnostics) == 0);
    CHECK(critical_diagnostic_count(diagnostics) == 0);

    const PrintObject &po       = *supported.objects().front();
    const PrintObject &po_plain = *unsupported.objects().front();
    // The offset this regression is about: it must be there, and the frame the organic generator works in
    // must not carry it a second time.
    const double z_offset = po.belt_global_z_offset();
    INFO("belt global z offset " << z_offset);
    REQUIRE(std::abs(z_offset) > 0.1);

    // The object's layer stack is the slicing result, whatever the gap asks for.
    REQUIRE(po.layer_count() == po_plain.layer_count());
    for (size_t i = 0; i < po.layer_count(); ++ i) {
        const Layer &layer = *po.get_layer(i);
        const Layer &plain = *po_plain.get_layer(i);
        INFO("object layer " << i << " z " << layer.print_z << " h " << layer.height
                             << " slice_z " << layer.slice_z);
        CHECK_THAT(layer.print_z, Catch::Matchers::WithinAbs(plain.print_z, 1e-12));
        CHECK_THAT(layer.height, Catch::Matchers::WithinAbs(plain.height, 1e-12));
        CHECK_THAT(layer.slice_z, Catch::Matchers::WithinAbs(plain.slice_z, 1e-12));
    }

    // The top contact: the highest support layer that carries fill. On a belt print the requested gap
    // does not have to land on a layer top either, and then this layer is the exact contact layer, whose
    // Z is the requested gap rather than an object layer Z. It is checked against the mesh below; every
    // other support layer must still sit on an object layer Z.
    const SupportLayer *contact = nullptr;
    for (const SupportLayer *layer : po.support_layers()) {
        Points points;
        layer->support_fills.collect_points(points);
        if (! points.empty() && (contact == nullptr || layer->print_z > contact->print_z))
            contact = layer;
    }
    REQUIRE(contact != nullptr);
    INFO("contact print_z " << contact->print_z << " height " << contact->height);

    REQUIRE(! po.support_layers().empty());
    const std::multiset<double> profile_heights = object_layer_heights(po);
    size_t                      base_paths      = 0;
    for (const SupportLayer *layer : po.support_layers()) {
        const auto it = std::find_if(po.layers().begin(), po.layers().end(),
            [&layer](const Layer *object_layer) { return std::abs(object_layer->print_z - layer->print_z) < 1e-9; });
        INFO("support layer print_z " << layer->print_z << " height " << layer->height);
        if (it == po.layers().end()) {
            // Not a grid layer: this is the exact contact layer that finishes a gap which does not end on an
            // object layer top. It must stand on a real layer top and stay thinner than any profile layer,
            // so the gap remainder cannot sneak into the object's own layer stack.
            const auto below = std::find_if(po.layers().begin(), po.layers().end(),
                [&layer](const Layer *object_layer) {
                    return std::abs(object_layer->print_z - (layer->print_z - layer->height)) < 1e-9; });
            INFO("exact contact layer bottom " << (layer->print_z - layer->height));
            CHECK(below != po.layers().end());
            CHECK(layer->height > 0.);
            CHECK(layer->height < *profile_heights.begin());
            continue;
        }
        CHECK_THAT(layer->height, Catch::Matchers::WithinAbs((*it)->height, 1e-12));
        for (const SupportPathRole &path : support_path_roles(*layer)) {
            INFO("support path role " << int(path.role) << " height " << path.height);
            CHECK(is_allowed_support_path_height(po, *layer, path.height));
            ++base_paths;
        }
    }
    REQUIRE(base_paths > 0);


    // The mesh in the frame it was sliced in. Applying `trafo_sliced()` puts the vertices in the frame of
    // `Layer::lslices`, and that frame carries the belt rotation but not the global offset the layers got
    // afterwards, so a support point's XY is a point of this mesh and the contact's local Z is its
    // print_z minus the offset.
    TriangleMesh sliced_mesh = mesh;
    sliced_mesh.transform(po.trafo_sliced());
    const double contact_local_z = contact->print_z - z_offset;
    INFO("contact local z " << contact_local_z);

    Points points;
    contact->support_fills.collect_points(points);
    REQUIRE(! points.empty());
    size_t                checked = 0;
    std::optional<double> lowest_face;
    for (const Point &p : points) {
        const double                x    = unscaled(p.x());
        const double                y    = unscaled(p.y());
        const std::optional<double> face = face_above(sliced_mesh, x, y, contact_local_z + 1e-6);
        if (! face)
            continue;
        ++ checked;
        if (! lowest_face || *face < *lowest_face)
            lowest_face = *face;
        INFO("support point " << x << ", " << y << " at " << contact_local_z << " under " << *face
                              << " gap " << *face - contact_local_z);
        CHECK(*face - contact_local_z >= gap - 1e-9);
    }
    // The mapping has to have found the mesh under the contact, or the assertion above says nothing.
    REQUIRE(checked > 10);
    REQUIRE(lowest_face.has_value());
    // The contact sits on a whole layer of the object, not on a Z of its own...
    const auto contact_layer = std::find_if(po.layers().begin(), po.layers().end(),
        [contact](const Layer *object_layer) { return std::abs(object_layer->print_z - contact->print_z) < 1e-9; });
    REQUIRE(contact_layer != po.layers().end());
    CHECK_THAT(contact->height, Catch::Matchers::WithinAbs((*contact_layer)->height, 1e-12));
    // ... and, measured against the lowest face above it, the gap is quantised upwards only: at least the
    // requested distance, and not more than one layer plus the slope the face gains over one layer's worth
    // of area (the belt rotation tilts every face, so a strip of one layer is not at a single Z).
    INFO("lowest face above the contact " << *lowest_face << " gap " << *lowest_face - contact_local_z);
    CHECK(*lowest_face - contact_local_z > gap - 0.2 - 0.5 - 1e-9);
    CHECK(*lowest_face - contact_local_z < gap + 0.2 + 0.5 + 1e-9);

    // The G-code is produced and carries the support: the print is not empty and its output is not
    // dropped. Its layer-height legend is not compared on a belt - the belt axis is machine Z, so the
    // legend carries machine-frame entries that are not layer heights - the heights are asserted above,
    // on the object layers and on every support path.
    const std::string gcode = Slic3r::Test::gcode(supported);
    REQUIRE(! gcode.empty());
    CHECK(gcode.find("; support") != std::string::npos);
    CHECK(! Slic3r::Test::gcode(unsupported).empty());
}

// The grid and the mesh the organic generator searches are in one frame: the grid stores each object
// layer's Z with `belt_global_z_offset()` removed (TreeSupportMeshGroupSettings::build_layer_grid()), and
// `trafo_sliced()` - the frame the layers were sliced in - does not carry that offset either, it is added
// to `Layer::print_z` only after slicing. This case is the one a frame slip breaks first: the face sits
// exactly on the window bottom, as a uniform grid puts it, so a search window or a reference moved by the
// belt offset (negative on a lifted belt mesh) lifts the contact onto the layer above the face, i.e. into
// the object. The grid here is built the way the object layers are, so a reference that leaves the
// window shows up as a contact above the face.
TEST_CASE("A belt Z offset does not shift the overhang search frame", "[TreeSupportOrganicGap][belt]")
{
    using namespace Slic3r::TreeSupport3D;

    // One down facing face at 15.6, over the area the layer's overhang covers.
    const TriangleMesh mesh(std::vector<Vec3f>{ { 0.f, 0.f, 15.6f }, { 0.f, 2.f, 15.6f }, { 2.f, 2.f, 15.6f },
                                                { 0.f, 0.f, 15.6f }, { 2.f, 2.f, 15.6f }, { 2.f, 0.f, 15.6f } },
                            std::vector<Vec3i32>{ { 0, 1, 2 }, { 3, 4, 5 } });

    // 0.4 mm layers: layer 40 tops out at 16.4 and starts at 16.0, so the face at 15.6 sits on the bottom
    // of the window [15.6, 16.4] the layer searches.
    TreeSupportLayerGrid grid;
    double               bottom = 0.;
    for (int i = 0; i < 50; ++ i) {
        TreeSupportLayerGrid::Entry entry;
        entry.bottom_z      = bottom;
        entry.obj_bottom_z  = bottom;
        entry.print_z       = bottom + 0.4;
        entry.height        = 0.4;
        entry.obj_layer_idx = LayerIndex(i);
        bottom              = entry.print_z;
        grid.entries.emplace_back(entry);
    }
    REQUIRE_THAT(grid.bottom_z(39), Catch::Matchers::WithinAbs(15.6, 1e-9));

    OverhangFacetZIndex facets;
    facets.build(mesh);
    const Polygons area{ Polygon{ { Point::new_scale(0., 0.), Point::new_scale(2., 0.),
                                   Point::new_scale(2., 2.), Point::new_scale(0., 2.) } } };

    const std::vector<TreeSupportLayerGrid::Entry> before = grid.entries;
    constexpr double                               requested = 0.1;
    // No belt offset is handed to the placement: the grid already holds the frame the mesh was sliced in.
    const OverhangContact contact = place_overhang_contact(grid, 40, mesh, facets, area, requested);
    REQUIRE(contact.face_z.has_value());
    REQUIRE(contact.contact_layer >= 1);
    INFO("reference " << *contact.face_z << " contact " << grid.z(contact.contact_layer));
    // The reference is the face itself, not the face moved by an offset.
    CHECK_THAT(*contact.face_z, Catch::Matchers::WithinAbs(15.6, 1e-6));
    // The contact stays under the face with the requested air: 15.6 - 0.1 asks for a top at or below
    // 15.5, which is layer 38 at 15.2. A reference moved up by a belt offset of -0.5 could ask for 16.0
    // and put the contact 0.4 above the face, inside the object.
    CHECK_THAT(grid.z(contact.contact_layer), Catch::Matchers::WithinAbs(15.2, 1e-6));
    CHECK(15.6 - grid.z(contact.contact_layer) >= requested - 1e-9);
    // No layer height or object Z was rewritten to absorb anything.
    REQUIRE(grid.entries.size() == before.size());
    for (size_t i = 0; i < before.size(); ++ i) {
        INFO("grid layer " << i);
        CHECK_THAT(grid.entries[i].print_z, Catch::Matchers::WithinAbs(before[i].print_z, 1e-12));
        CHECK_THAT(grid.entries[i].bottom_z, Catch::Matchers::WithinAbs(before[i].bottom_z, 1e-12));
        CHECK_THAT(grid.entries[i].height, Catch::Matchers::WithinAbs(before[i].height, 1e-12));
    }
}

// An overhang area with no down facing facet behind it - a support enforcer adds areas that no mesh face
// backs - has no face to measure from. The fallback is the bottom of the window that was searched, which
// keeps the gap a lower bound, and it must not move the layer's material start.
TEST_CASE("An overhang area with no face behind it falls back to the searched window's bottom", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;

    TreeSupportLayerGrid grid;
    double               bottom = 0.;
    for (int i = 0; i < 50; ++ i) {
        TreeSupportLayerGrid::Entry entry;
        entry.bottom_z      = bottom;
        entry.obj_bottom_z  = bottom;
        entry.print_z       = bottom + 0.4;
        entry.height        = 0.4;
        entry.obj_layer_idx = LayerIndex(i);
        bottom              = entry.print_z;
        grid.entries.emplace_back(entry);
    }

    // One up facing facet over the area: no down facing face anywhere, so the search finds nothing even
    // though the facet's Z range overlaps the window.
    const TriangleMesh  mesh(std::vector<Vec3f>{ { 0.f, 0.f, 16.2f }, { 10.f, 0.f, 16.2f }, { 0.f, 10.f, 16.2f } },
                             std::vector<Vec3i32>{ { 0, 1, 2 } });
    OverhangFacetZIndex facets;
    facets.build(mesh);
    const Polygons area{ Polygon{ { Point::new_scale(0., 0.), Point::new_scale(2., 0.),
                                   Point::new_scale(2., 2.), Point::new_scale(0., 2.) } } };

    constexpr double     requested = 0.4;
    const OverhangContact contact  = place_overhang_contact(grid, 40, mesh, facets, area, requested);
    CHECK_FALSE(contact.face_z.has_value());
    // 15.6 - 0.4 asks for a top at or below 15.2, which is layer 37.
    CHECK(contact.contact_layer == 37);
    CHECK_THAT(grid.z(contact.contact_layer), Catch::Matchers::WithinAbs(15.2, 1e-9));
    // The material over that area starts at the facet above it, so the gap stays a lower bound ...
    const double material_start = 16.2;
    CHECK(material_start - grid.z(contact.contact_layer) >= requested - 1e-9);
    // ... and without a face the layer's material start is left at its bottom.
    CHECK_THAT(grid.obj_bottom_z(40), Catch::Matchers::WithinAbs(16.0, 1e-12));
    // No layer height was touched.
    for (size_t i = 0; i < grid.entries.size(); ++ i) {
        INFO("grid layer " << i);
        CHECK_THAT(grid.entries[i].height, Catch::Matchers::WithinAbs(0.4, 1e-12));
    }
}

// The contact is the highest layer whose top keeps the whole gap, so the tolerance of that comparison may
// only cover the round-off of the Zs themselves. The geometric EPSILON used to be applied there, which let
// a contact sit 1e-4 mm too high; the other half of the test is that a layer top that *is* the requested Z
// up to round-off is still taken, so the tolerance cannot be dropped either.
TEST_CASE("The top contact comparison only tolerates round-off, never a shorter gap", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;

    auto grid_with_heights = [](const std::vector<double> &heights) {
        TreeSupportLayerGrid grid;
        double               bottom = 0.;
        for (const double height : heights) {
            TreeSupportLayerGrid::Entry entry;
            entry.bottom_z     = bottom;
            entry.obj_bottom_z = bottom;
            entry.print_z      = bottom + height;
            entry.height       = height;
            bottom             = entry.print_z;
            grid.entries.emplace_back(entry);
        }
        return grid;
    };

    SECTION("A layer top that is the requested Z up to round-off is taken") {
        // A top one ULP above the requested Z is that Z: refusing it would drop the contact a whole layer
        // for no reason, and the double round-off of a sum of layer heights is that order of magnitude.
        std::vector<double> tops { 0.2, 0.4, 0.6, std::nextafter(0.8, 1.0), 1.0 };
        TreeSupportLayerGrid grid;
        double               bottom = 0.;
        for (const double top : tops) {
            TreeSupportLayerGrid::Entry entry;
            entry.bottom_z     = bottom;
            entry.obj_bottom_z = bottom;
            entry.print_z      = top;
            entry.height       = top - bottom;
            bottom             = top;
            grid.entries.emplace_back(entry);
        }
        REQUIRE(grid.z(3) > 0.8);
        REQUIRE(grid.z(3) - 0.8 < 1e-12);
        CHECK(grid.place_top_contact(1.0, 0.2) == 3);
        CHECK(1.0 - grid.z(3) >= 0.2 - 1e-9);
    }

    SECTION("A layer top that is a ten-thousandth too high is refused") {
        // Tops 0.2, 0.4, 0.6, 0.80005, 1.0. A face at 1.0 with a 0.2 gap asks for a top at or below 0.8:
        // taking 0.80005 would leave 0.19995 mm of air, which is what the geometric EPSILON allowed.
        TreeSupportLayerGrid grid = grid_with_heights({ 0.2, 0.2, 0.2, 0.20005, 0.19995 });
        REQUIRE_THAT(grid.z(3), Catch::Matchers::WithinAbs(0.80005, 1e-9));
        CHECK(grid.place_top_contact(1.0, 0.2) == 2);
        CHECK(1.0 - grid.z(2) >= 0.2 - 1e-9);
        // A face one ten-thousandth higher does land on it, and then the gap is exactly the requested one.
        CHECK(grid.place_top_contact(1.00005, 0.2) == 3);
        CHECK_THAT(1.00005 - grid.z(3), Catch::Matchers::WithinAbs(0.2, 1e-9));
    }
}

// A raft layer is thicker so it can be peeled off and a thicker first layer is a first layer setting;
// neither says how the object stacks, which is why the layer *counts* stay the historical scalars on such
// a print. The per-layer growth has to stay the scalar with them: deriving it from the raft's 0.3 mm or the
// first layer's 0.32 mm would move the branches while the counts that sized them still assume nominal
// layers.
TEST_CASE("A raft and a thicker first layer keep the per-layer growth at its nominal value", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;
    const TriangleMesh mesh = two_tier_mesh();

    const bool   with_raft   = GENERATE(false, true);
    const double first_layer = GENERATE(0.2, 0.32);
    INFO("raft " << with_raft << ", first layer height " << first_layer);

    auto slice = [&mesh](Slic3r::Print &print, int raft_layers, double first_layer_height) {
        slice_with_tree_support(mesh, print, "organic", 30, 0, raft_layers, {
            { "initial_layer_print_height",   first_layer_height },
            { "support_top_z_distance",       0.2 },
            { "support_interface_top_layers", 2 },
        });
    };

    Slic3r::Print print;
    Slic3r::Print uniform;
    slice(print, with_raft ? 2 : 0, first_layer);
    slice(uniform, 0, 0.2);

    const PrintObject &po = *print.objects().front();
    TreeSupportMeshGroupSettings ms(po);
    TreeSupportMeshGroupSettings ms_uniform(*uniform.objects().front());
    TreeSupportSettings          config(ms, po.slicing_parameters());
    TreeSupportSettings          config_uniform(ms_uniform, uniform.objects().front()->slicing_parameters());

    if (with_raft)
        REQUIRE(config.layer_grid.raft_layers > 0);
    else
        REQUIRE(config.layer_grid.raft_layers == 0);
    // The case is really exercised: the grid holds layers thicker than the nominal 0.2 mm - the raft's own
    // layers and/or the first object layer.
    size_t thicker = 0;
    for (const TreeSupportLayerGrid::Entry &entry : config.layer_grid.entries)
        if (entry.height > 0.2 + 1e-9)
            ++ thicker;
    if (with_raft || first_layer > 0.2)
        // Everything but the plain 0.2 mm profile has a layer thicker than nominal ...
        REQUIRE(thicker > 0);
    else
        // ... and the plain one has none, which is the baseline the other cases are compared against.
        REQUIRE(thicker == 0);
    if (! with_raft && first_layer > 0.2)
        // No raft: the only thicker layer is the object's first one.
        CHECK(thicker == 1);
    // ... and the counts are the historical scalars anyway.
    check_counts_unchanged(config, config_uniform);

    // Every layer therefore reports the nominal growth, the raft layers and the first one included.
    for (size_t i = 0; i < config.layer_grid.size(); ++ i) {
        const LayerIndex layer_idx = LayerIndex(i);
        INFO("layer " << i << " height " << config.layer_grid.h(layer_idx));
        CHECK(config.maximum_move_distance_at(layer_idx) == config.maximum_move_distance);
        CHECK(config.maximum_move_distance_slow_at(layer_idx) == config.maximum_move_distance_slow);
        CHECK_THAT(config.branch_radius_increase_at(layer_idx),
                   Catch::Matchers::WithinAbs(config.branch_radius_increase_per_layer, 1e-6));
        CHECK_THAT(config.bp_radius_increase_at(layer_idx),
                   Catch::Matchers::WithinAbs(config.bp_radius_increase_per_layer, 1e-6));
    }
}

// ---------------------------------------------------------------------------------------------------
// Exact top gap: what organic and tree_slim actually print on a fixed 0.2 mm profile.
//
// The cases above pin the layer synchronised semantics: the contact takes a whole existing layer, so a
// requested 0.1 mm comes out as 0.2. The measurement below records what each engine really produces -
// the support Z and height of every printed support layer, the air gap measured point by point against
// the mesh, and the Z the G-code actually prints the support at - so a change that makes the gap exact
// is judged on the printed result rather than on the layer grid.
// ---------------------------------------------------------------------------------------------------

namespace {

constexpr double exact_gap_overhang_face_z = 16.0; // underside of two_tier_mesh()'s upper plate

struct SupportPathSample
{
    ExtrusionRole role;
    double        height;
    double        width;
    double        mm3_per_mm;
};

std::vector<SupportPathSample> support_path_samples(const SupportLayer &layer)
{
    std::vector<SupportPathSample> paths;
    for (const ExtrusionEntity *entity : layer.support_fills.flatten().entities)
        if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity))
            paths.push_back({ path->role(), double(path->height), double(path->width), double(path->mm3_per_mm) });
    return paths;
}

// Shift from the print frame (scaled Points of the slices and support fills) to the mesh frame, taken
// from a layer whose slice spans the whole model footprint instead of being assumed.
std::optional<std::pair<double, double>> print_to_mesh_shift(const PrintObject &po, const TriangleMesh &mesh)
{
    const BoundingBoxf3 mesh_bb = mesh.bounding_box();
    for (const Layer *layer : po.layers()) {
        if (layer->lslices.empty())
            continue;
        const BoundingBox bb = get_extents(layer->lslices);
        if (std::abs(unscaled(bb.max.x() - bb.min.x()) - double(mesh_bb.max.x() - mesh_bb.min.x())) > 1e-3)
            continue;
        if (std::abs(unscaled(bb.max.y() - bb.min.y()) - double(mesh_bb.max.y() - mesh_bb.min.y())) > 1e-3)
            continue;
        return std::make_pair(unscaled(bb.min.x()) - double(mesh_bb.min.x()),
                              unscaled(bb.min.y()) - double(mesh_bb.min.y()));
    }
    return std::nullopt;
}

struct OverhangSupportSample
{
    double face_z;   // lowest mesh face above this support point
    double print_z;  // support layer Z the point is printed at
    double height;   // SupportLayer::height of that layer
};

// Every support point printed under the upper plate's underside (the face at z = 16.0), with the face
// above it. Points over the column, the lower plate or open air are dropped, so the sample is the top
// contact under the overhang and not some unrelated highest support Z.
std::vector<OverhangSupportSample> overhang_support_samples(const PrintObject &po, const TriangleMesh &mesh,
                                                            double shift_x, double shift_y)
{
    std::vector<OverhangSupportSample> samples;
    for (const SupportLayer *layer : po.support_layers()) {
        Points points;
        layer->support_fills.collect_points(points);
        for (const Point &p : points) {
            const double                x    = unscaled(p.x()) - shift_x;
            const double                y    = unscaled(p.y()) - shift_y;
            const std::optional<double> face = face_above(mesh, x, y, layer->print_z + 1e-6);
            if (face && std::abs(*face - exact_gap_overhang_face_z) < 1e-3)
                samples.push_back({ *face, layer->print_z, layer->height });
        }
    }
    return samples;
}

// One extruding move of the emitted G-code: the machine Z it is printed at and the extruded length. The Z
// is the modal machine state (G90/G91 for XYZ, G92 for the current position, G0/G1/G2/G3 for the move),
// not the ;Z: / ;HEIGHT: comments, so a layer model that never reaches the machine cannot pass.
struct SupportMotionSample
{
    double z;
    double e_length;
};

struct GcodeMotion
{
    std::vector<SupportMotionSample> support_moves;
    std::set<double>                 support_z;
    // Extruded length at a Z, within `tolerance`.
    double extrusion_at(const double z, const double tolerance = 1e-3) const
    {
        double total = 0.;
        for (const SupportMotionSample &move : support_moves)
            if (std::abs(move.z - z) <= tolerance)
                total += move.e_length;
        return total;
    }
};

// Axis words of one G-code line, without the command number. A command letter (G/M/T) is skipped with
// its number; every other letter carries the value that follows it.
std::vector<std::pair<char, double>> gcode_axis_words(const std::string &line)
{
    std::vector<std::pair<char, double>> words;
    for (size_t i = 0; i < line.size(); ++ i) {
        const char c = line[i];
        if (c == ';')
            break;
        if (! std::isalpha(static_cast<unsigned char>(c)))
            continue;
        size_t j = i + 1;
        while (j < line.size() && (line[j] == ' ' || line[j] == '='))
            ++ j;
        if (c == 'G' || c == 'M' || c == 'T') {
            while (j < line.size() && std::isdigit(static_cast<unsigned char>(line[j])))
                ++ j;
            i = j - 1;
            continue;
        }
        char        *end   = nullptr;
        const double value = std::strtod(line.c_str() + j, &end);
        if (end == line.c_str() + j)
            continue;
        words.emplace_back(c, value);
        i = size_t(end - line.c_str()) - 1;
    }
    return words;
}

bool gcode_is_command(const std::string &line, const char *command)
{
    const size_t first = line.find_first_not_of(" \t");
    if (first == std::string::npos || line.compare(first, strlen(command), command) != 0)
        return false;
    const size_t after = first + strlen(command);
    return after >= line.size() || line[after] == ' ' || line[after] == '\t' || line[after] == ';';
}

// Every extruding move of the G-code that prints support, with its machine Z. G90/G91 switch all axes
// between absolute and relative, G92 defines the current position, M82/M83 switch the extruder axis. The
// emitted G-code starts in the absolute XYZ / absolute E default the machine is reset to.
GcodeMotion support_gcode_motion(const std::string &gcode)
{
    GcodeMotion motion;
    double      x = 0., y = 0., z = 0., e = 0.;
    bool        xyz_relative = false;
    bool        e_relative   = false;
    bool        support      = false;
    std::istringstream lines(gcode);
    std::string        line;
    while (std::getline(lines, line)) {
        if (! line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.rfind(";TYPE:", 0) == 0) {
            support = line.rfind(";TYPE:Support", 0) == 0;
            continue;
        }
        if (line.rfind(";", 0) == 0)
            continue;
        if (gcode_is_command(line, "G90")) { xyz_relative = false; continue; }
        if (gcode_is_command(line, "G91")) { xyz_relative = true;  continue; }
        if (gcode_is_command(line, "M82")) { e_relative   = false; continue; }
        if (gcode_is_command(line, "M83")) { e_relative   = true;  continue; }
        const std::vector<std::pair<char, double>> words = gcode_axis_words(line);
        if (gcode_is_command(line, "G92")) {
            for (const std::pair<char, double> &word : words) {
                if (word.first == 'X') x = word.second;
                else if (word.first == 'Y') y = word.second;
                else if (word.first == 'Z') z = word.second;
                else if (word.first == 'E') e = word.second;
            }
            continue;
        }
        if (! gcode_is_command(line, "G0") && ! gcode_is_command(line, "G1") &&
            ! gcode_is_command(line, "G2") && ! gcode_is_command(line, "G3"))
            continue;
        const double old_e = e;
        bool         has_xy = false;
        for (const std::pair<char, double> &word : words) {
            switch (word.first) {
            case 'X': x = xyz_relative ? x + word.second : word.second; has_xy = true; break;
            case 'Y': y = xyz_relative ? y + word.second : word.second; has_xy = true; break;
            case 'Z': z = xyz_relative ? z + word.second : word.second; break;
            case 'E': e = e_relative ? e + word.second : word.second;   break;
            default: break;
            }
        }
        const double extruded = e - old_e;
        if (support && has_xy && extruded > 0.) {
            motion.support_moves.push_back({ z, extruded });
            motion.support_z.insert(std::round(z * 1e6) / 1e6);
        }
    }
    return motion;
}

// The support area a layer covers including the spacing between its lines: what it really lays under the
// layer above, as opposed to the individual line polygons.
Polygons support_covered_area(const SupportLayer &layer)
{
    return layer.support_fills.polygons_covered_by_spacing();
}

// The object layer face the upper plate starts at, derived from the sliced stack instead of assumed: the
// first layer whose slice is the plate footprint (the column alone is far smaller) begins at the face.
double plate_underside_z(const PrintObject &po)
{
    for (const Layer *layer : po.layers()) {
        const Polygons polygons = to_polygons(layer->lslices);
        if (! polygons.empty() && area(polygons) * SCALING_FACTOR * SCALING_FACTOR > 1000.)
            return layer->print_z - layer->height;
    }
    return 0.;
}

// Every printed path of `layer` must be as thick as the material this layer really lays down at that XY:
// its own height where nothing else is in its slab, and - where a support layer sits inside the slab - the
// thinner height the over-extrusion mitigation leaves above it (is_allowed_support_path_height()). The
// height reaches Flow::with_height(), so a wrong one is a wrong material thickness, not a cosmetic value;
// and a reduced height has to carry the flow that height implies, because the mitigation rebuilds the
// fragment with Flow(width, height, -1).
size_t check_layer_paths_thickness(const SupportLayer &layer, const PrintObject &po)
{
    size_t checked = 0;
    for (const SupportPathSample &path : support_path_samples(layer)) {
        INFO("path role " << int(path.role) << " height " << path.height << " width " << path.width
                          << " mm3_per_mm " << path.mm3_per_mm << " layer height " << layer.height
                          << " layer print_z " << layer.print_z);
        CHECK(path.width > 0.);
        CHECK(path.mm3_per_mm > 0.);
        CHECK(is_allowed_support_path_height(po, layer, path.height));
        if (std::abs(path.height - layer.height) > 1e-6)
            CHECK_THAT(path.mm3_per_mm, Catch::Matchers::WithinRel(Flow(path.width, float(path.height), -1.f).mm3_per_mm(), 1e-6));
        ++ checked;
    }
    return checked;
}

// `upper` stands on `lower`: no covered part of the upper layer lies outside the lower one, allowing one
// line width for the pattern/spacing difference between two interface layers.
void check_supported_from_below(const SupportLayer &upper, const SupportLayer &lower)
{
    const Polygons upper_area = support_covered_area(upper);
    const Polygons lower_area = support_covered_area(lower);
    REQUIRE(! upper_area.empty());
    REQUIRE(! lower_area.empty());
    const Polygons uncovered = diff(upper_area, offset(lower_area, scale_(0.05)));
    INFO("upper print_z " << upper.print_z << " lower print_z " << lower.print_z
                          << " uncovered area " << area(uncovered) << " of " << area(upper_area));
    CHECK(area(uncovered) < 0.01 * area(upper_area));
    CHECK_THAT(upper.bottom_z(), Catch::Matchers::WithinAbs(lower.print_z, 1e-6));
}

// ---------------------------------------------------------------------------------------------------
// The over-extrusion mitigation: where a support layer overlaps another support layer inside its own slab,
// modulate_extrusion_by_overlapping_layers() re-fragments its toolpaths so the overlap is printed thinner.
// A support material layer is not a flat list of paths - a large island is wrapped in an
// ExtrusionEntityCollection (no_sort) and plain ExtrusionPath objects are appended next to it - so the
// mitigation has to dispatch per entity instead of skipping the whole list.
// ---------------------------------------------------------------------------------------------------
struct ModulatedPathSample
{
    ExtrusionRole role;
    double        height;
    float         width;
    double        mm3_per_mm;
    double        min_x;
    double        max_x;
    bool          nested;   // the path came out of an ExtrusionEntityCollection of the list
};

void collect_modulated_paths(const ExtrusionEntity &entity, const bool nested, std::vector<ModulatedPathSample> &out)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(&entity); collection != nullptr) {
        for (const ExtrusionEntity *child : collection->entities)
            collect_modulated_paths(*child, true, out);
        return;
    }
    if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath*>(&entity); multipath != nullptr) {
        for (const ExtrusionPath &path : multipath->paths)
            collect_modulated_paths(path, nested, out);
        return;
    }
    if (const auto *path = dynamic_cast<const ExtrusionPath*>(&entity); path != nullptr) {
        double min_x = 0.;
        double max_x = 0.;
        bool   first = true;
        for (const Point3 &point : path->polyline.points) {
            const double x = unscaled(point.x());
            min_x = first ? x : std::min(min_x, x);
            max_x = first ? x : std::max(max_x, x);
            first = false;
        }
        out.push_back({ path->role(), double(path->height), path->width, double(path->mm3_per_mm), min_x, max_x, nested });
    }
}

// The runs of a support layer that the mitigation reduced below the layer's own height, and the ones that
// still carry it.
struct OverlapThinning
{
    std::vector<ModulatedPathSample> reduced;
    std::vector<ModulatedPathSample> full;
    size_t                           reduced_nested { 0 };
    size_t                           reduced_plain  { 0 };
};

OverlapThinning overlap_thinning(const SupportLayer &layer)
{
    OverlapThinning                  result;
    std::vector<ModulatedPathSample> paths;
    for (const ExtrusionEntity *entity : layer.support_fills.entities)
        collect_modulated_paths(*entity, false, paths);
    for (const ModulatedPathSample &path : paths) {
        if (path.height < layer.height - 1e-6) {
            result.reduced.push_back(path);
            (path.nested ? result.reduced_nested : result.reduced_plain) += 1;
        } else
            result.full.push_back(path);
    }
    return result;
}

void delete_toolpath_entities(ExtrusionEntitiesPtr &entities)
{
    for (ExtrusionEntity *entity : entities)
        delete entity;
    entities.clear();
}

Polygons axis_rectangle(const double x0, const double y0, const double x1, const double y1)
{
    Polygon polygon;
    polygon.points = { Point(coord_t(scale_(x0)), coord_t(scale_(y0))),
                       Point(coord_t(scale_(x1)), coord_t(scale_(y0))),
                       Point(coord_t(scale_(x1)), coord_t(scale_(y1))),
                       Point(coord_t(scale_(x0)), coord_t(scale_(y1))) };
    return { polygon };
}

// One straight support line of a synthetic layer, built exactly as the generator builds them.
void append_test_support_line(ExtrusionEntitiesPtr &dst, const double y, const double x0, const double x1,
                              const double mm3_per_mm, const float width, const float height)
{
    Polylines lines;
    lines.push_back(Polyline(Points{ Point(coord_t(scale_(x0)), coord_t(scale_(y))),
                                     Point(coord_t(scale_(x1)), coord_t(scale_(y))) }));
    extrusion_entities_append_paths(dst, std::move(lines), ExtrusionRole::erSupportMaterial, mm3_per_mm, width, height);
}

} // namespace

TEST_CASE("The overlap modulation thins the runs of a support layer that holds island collections",
          "[TreeSupportOrganicGap]")
{
    constexpr float  width     = 0.4f;
    constexpr float  height    = 0.2f;
    const double     mm3_full  = Flow(width, height, -1.f).mm3_per_mm();
    constexpr double overlap_x = 5.;   // the layer above covers x in [0, 5] of this layer's x in [0, 10]

    SupportGeneratorLayer layer;
    layer.print_z  = 0.4;
    layer.bottom_z = 0.2;
    layer.height   = 0.2;

    SupportGeneratorLayer overlapping_layer;
    overlapping_layer.print_z  = 0.3;
    overlapping_layer.bottom_z = 0.1;
    overlapping_layer.height   = 0.2;
    overlapping_layer.polygons = axis_rectangle(0., 0., overlap_x, 10.);
    SupportGeneratorLayersPtr overlapping{ &overlapping_layer };

    // [plain path][island collection: 2 paths, no_sort][plain path], the shape a support material layer of
    // tree_supports_generate_paths() has when it holds a large island next to small ones.
    ExtrusionEntitiesPtr entities;
    append_test_support_line(entities, 1., 0., 10., mm3_full, width, height);
    auto *island = new ExtrusionEntityCollection();
    island->no_sort = true;
    append_test_support_line(island->entities, 3., 0., 10., mm3_full, width, height);
    append_test_support_line(island->entities, 7., 0., 10., mm3_full, width, height);
    entities.push_back(island);
    append_test_support_line(entities, 9., 0., 10., mm3_full, width, height);
    REQUIRE(entities.size() == 3);
    REQUIRE(island->entities.size() == 2);

    modulate_extrusion_by_overlapping_layers(entities, layer, overlapping);

    // The list keeps its structure: the island is the same collection object, still no_sort, still one entry
    // per generated path, in the same position.
    REQUIRE(entities.size() == 3);
    CHECK(entities[1] == island);
    CHECK(island->no_sort);
    REQUIRE(island->entities.size() == 2);

    std::vector<ModulatedPathSample> paths;
    for (const ExtrusionEntity *entity : entities)
        collect_modulated_paths(*entity, false, paths);

    const double thin_height = layer.print_z - overlapping_layer.print_z;   // 0.1
    const double thin_mm3    = Flow(width, float(thin_height), -1.f).mm3_per_mm();
    size_t thin = 0, full = 0, thin_nested = 0, thin_plain = 0, full_nested = 0, full_plain = 0;
    double thin_length = 0., full_length = 0., total_length = 0.;
    for (const ModulatedPathSample &path : paths) {
        INFO("path role " << int(path.role) << " height " << path.height << " x " << path.min_x << ".." << path.max_x
                          << " nested " << path.nested << " mm3_per_mm " << path.mm3_per_mm);
        CHECK_THAT(double(path.width), Catch::Matchers::WithinAbs(double(width), 1e-9));
        CHECK(path.role == erSupportMaterial);
        const double length = path.max_x - path.min_x;
        total_length += length;
        CHECK(length > 0.);
        if (std::abs(path.height - thin_height) < 1e-6) {
            ++ thin;
            (path.nested ? thin_nested : thin_plain) += 1;
            // The reduced run is the part inside the overlapping area and it carries the flow its own
            // thickness implies - not the full layer's flow spread over a thinner layer.
            CHECK(path.min_x >= -1e-6);
            CHECK(path.max_x <= overlap_x + 0.5 * double(width) + 1e-3);
            CHECK_THAT(path.mm3_per_mm, Catch::Matchers::WithinRel(thin_mm3, 1e-6));
            thin_length += length;
        } else {
            ++ full;
            (path.nested ? full_nested : full_plain) += 1;
            // The rest of that path lies outside the overlap and keeps the layer's full thickness.
            CHECK_THAT(path.height, Catch::Matchers::WithinAbs(double(height), 1e-6));
            CHECK_THAT(path.mm3_per_mm, Catch::Matchers::WithinRel(mm3_full, 1e-6));
            CHECK(path.min_x >= overlap_x - 1e-3);
            CHECK(path.max_x <= 10. + 1e-6);
            full_length += length;
        }
    }
    // Four generated paths (two plain, two inside the island), each split into a thin and a full run.
    CHECK(thin == 4);
    CHECK(full == 4);
    CHECK(thin_nested == 2);
    CHECK(thin_plain == 2);
    CHECK(full_nested == 2);
    CHECK(full_plain == 2);
    CHECK(thin_length > 0.);
    CHECK(full_length > 0.);
    // Nothing is lost: the four 10 mm lines still total 40 mm of extrusion.
    CHECK_THAT(total_length, Catch::Matchers::WithinAbs(40., 0.05));

    delete_toolpath_entities(entities);

    // With nothing inside the slab the list is left exactly as generated: same entities, same numbers.
    ExtrusionEntitiesPtr untouched;
    append_test_support_line(untouched, 1., 0., 10., mm3_full, width, height);
    auto *untouched_island = new ExtrusionEntityCollection();
    untouched_island->no_sort = true;
    append_test_support_line(untouched_island->entities, 3., 0., 10., mm3_full, width, height);
    untouched.push_back(untouched_island);
    ExtrusionEntity *const plain_before  = untouched[0];
    ExtrusionEntity *const nested_before = untouched_island->entities[0];
    modulate_extrusion_by_overlapping_layers(untouched, layer, SupportGeneratorLayersPtr{});
    REQUIRE(untouched.size() == 2);
    CHECK(untouched[0] == plain_before);
    CHECK(untouched[1] == untouched_island);
    REQUIRE(untouched_island->entities.size() == 1);
    CHECK(untouched_island->entities[0] == nested_before);
    CHECK_THAT(dynamic_cast<ExtrusionPath*>(untouched[0])->height, Catch::Matchers::WithinAbs(double(height), 1e-9));
    delete_toolpath_entities(untouched);
}

// The user-visible requirement: support_top_z_distance is a distance in millimetres, so 0.1 mm on a
// 0.2 mm profile has to leave 0.1 mm of air under the overhang, not one whole 0.2 mm layer. The contact
// layer of the object grid stays where it is and keeps its height; one thin support layer on top of it
// finishes the exact gap.
//
// The object's own stack is untouched - that constraint is not negotiable - and the assertions refuse the
// shortcuts: the thin layer's height is the Z span it really prints over, every path of it carries that
// height (it reaches Flow::with_height()), the layer stands on the one below at the same XY, and the
// G-code moves the machine to that Z and extrudes support there. The G-code Z comes from the modal motion
// state, not from the ;Z: / ;HEIGHT: comments.
TEST_CASE("An organic top gap of 0.1 mm leaves 0.1 mm of air under the overhang on a 0.2 mm profile",
          "[TreeSupportOrganicGapExact]")
{
    constexpr double   gap  = 0.1;
    const TriangleMesh mesh = two_tier_mesh();

    Slic3r::Print print;
    slice_with_tree_support(mesh, print, "organic", 30, 0, 0, {
        { "support_top_z_distance",       gap },
        { "support_interface_top_layers", 2 },
    });
    const PrintObject &po = *print.objects().front();

    // The object's real layers are the slicing result and stay 0.2 mm; a fix may not buy the exact gap
    // by rewriting them.
    REQUIRE(po.layer_count() == 95);
    for (size_t i = 0; i < po.layer_count(); ++ i) {
        const Layer &layer = *po.get_layer(i);
        INFO("object layer " << i << " print_z " << layer.print_z << " height " << layer.height
                             << " slice_z " << layer.slice_z);
        CHECK_THAT(layer.height, Catch::Matchers::WithinAbs(0.2, 1e-9));
        CHECK_THAT(layer.print_z, Catch::Matchers::WithinAbs(0.2 * double(i + 1), 1e-9));
        CHECK_THAT(layer.slice_z, Catch::Matchers::WithinAbs(0.2 * double(i + 1) - 0.1, 1e-9));
    }

    const std::optional<std::pair<double, double>> shift = print_to_mesh_shift(po, mesh);
    REQUIRE(shift.has_value());
    const std::vector<OverhangSupportSample> samples = overhang_support_samples(po, mesh, shift->first, shift->second);
    REQUIRE(samples.size() > 10);
    REQUIRE(! po.support_layers().empty());

    double contact_print_z = 0.;
    for (const OverhangSupportSample &sample : samples)
        contact_print_z = std::max(contact_print_z, sample.print_z);
    REQUIRE(contact_print_z > 15.);
    INFO("contact print_z " << contact_print_z << " measured air gap " << exact_gap_overhang_face_z - contact_print_z
                            << " samples " << samples.size());

    // No printed support point under the face may sit closer than the requested gap ...
    for (const OverhangSupportSample &sample : samples) {
        INFO("support point at " << sample.print_z << " under face " << sample.face_z);
        CHECK(sample.face_z - sample.print_z >= gap - 1e-6);
    }
    // ... and the gap itself has to be the requested distance, not an upward quantisation of it.
    CHECK_THAT(exact_gap_overhang_face_z - contact_print_z, Catch::Matchers::WithinAbs(gap, 1e-3));

    std::vector<const SupportLayer *> printed_layers;
    for (const SupportLayer *layer : po.support_layers())
        if (layer->has_extrusions())
            printed_layers.emplace_back(layer);
    REQUIRE(printed_layers.size() > 1);
    std::sort(printed_layers.begin(), printed_layers.end(),
              [](const SupportLayer *a, const SupportLayer *b) { return a->print_z < b->print_z; });

    // Every printed support layer covers exactly the Z span it reports, and its material is as thick as
    // that height - base and interface paths alike. The stack is the object's grid plus one thin layer on
    // top; the fix may not move or retime any other layer.
    size_t off_grid_layers = 0;
    for (const SupportLayer *layer : printed_layers) {
        INFO("support layer print_z " << layer->print_z << " height " << layer->height
                                      << " bottom " << layer->bottom_z());
        CHECK(layer->height > 0.);
        CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(layer->print_z - layer->bottom_z(), 1e-6));
        CHECK(check_layer_paths_thickness(*layer, po) > 0);
        if (std::abs(layer->print_z / 0.2 - std::round(layer->print_z / 0.2)) > 1e-6)
            ++ off_grid_layers;
    }
    CHECK(off_grid_layers <= 1);
    CHECK_THAT(printed_layers.front()->bottom_z(), Catch::Matchers::WithinAbs(0., 1e-6));

    // The thin layer is the top of the stack and it stands on the one below at the same XY: no part of it
    // hangs in the air and no other layer is inserted under it.
    const SupportLayer &contact = *printed_layers.back();
    const SupportLayer &below   = *printed_layers[printed_layers.size() - 2];
    CHECK_THAT(contact.print_z, Catch::Matchers::WithinAbs(contact_print_z, 1e-9));
    CHECK_THAT(contact.print_z - contact.height, Catch::Matchers::WithinAbs(below.print_z, 1e-6));
    check_supported_from_below(contact, below);

    // The G-code moves to the contact Z to print support there; a Z that only exists in the layer model
    // would mean the fix never reached the machine. Every printed support layer is really printed at its
    // own Z, and no support is extruded at a Z the layer model does not know.
    const std::string         gcode  = Slic3r::Test::gcode(print);
    const GcodeMotion         motion = support_gcode_motion(gcode);
    REQUIRE(! motion.support_moves.empty());
    for (const SupportLayer *layer : printed_layers) {
        INFO("support layer print_z " << layer->print_z);
        CHECK(motion.extrusion_at(layer->print_z) > 0.);
    }
    INFO("contact z " << contact.print_z << " extrusion " << motion.extrusion_at(contact.print_z));
    CHECK(motion.extrusion_at(contact.print_z) > 0.);
    for (const double z : motion.support_z) {
        const auto it = std::find_if(printed_layers.begin(), printed_layers.end(),
            [z](const SupportLayer *layer) { return std::abs(layer->print_z - z) < 1e-3; });
        INFO("support G-code Z " << z);
        CHECK(it != printed_layers.end());
    }

    // Same input, same stack and same support Z set.
    Slic3r::Print again;
    slice_with_tree_support(mesh, again, "organic", 30, 0, 0, {
        { "support_top_z_distance",       gap },
        { "support_interface_top_layers", 2 },
    });
    const PrintObject &repeated = *again.objects().front();
    REQUIRE(repeated.support_layers().size() == po.support_layers().size());
    for (size_t i = 0; i < po.support_layers().size(); ++ i) {
        INFO("support layer " << i << " print_z " << po.support_layers()[i]->print_z);
        CHECK_THAT(repeated.support_layers()[i]->print_z, Catch::Matchers::WithinAbs(po.support_layers()[i]->print_z, 1e-12));
        CHECK_THAT(repeated.support_layers()[i]->height,  Catch::Matchers::WithinAbs(po.support_layers()[i]->height, 1e-12));
    }
    CHECK(support_gcode_motion(Slic3r::Test::gcode(again)).support_z == motion.support_z);
}

// The requested gap is a millimetre distance, so it is printed exactly whenever the remainder above the
// object layer can be printed at all: whole multiples of the layer height need no extra layer, a fraction
// gets one thin layer. A remainder below the profile's minimum layer height cannot be printed; the contact
// then stays on the whole layer below and the gap comes out wider than requested - never narrower.
TEST_CASE("An organic top gap ends at the requested Z unless its remainder is below the minimum layer height",
          "[TreeSupportOrganicGapExact]")
{
    constexpr double   face = 16.0;
    const TriangleMesh mesh = two_tier_mesh();
    const double       gap  = GENERATE(0.0, 0.1, 0.12, 0.19, 0.199999, 0.2, 0.3, 0.399999);
    INFO("support_top_z_distance " << gap);

    Slic3r::Print print;
    slice_with_tree_support(mesh, print, "organic", 30, 0, 0, {
        { "support_top_z_distance",       gap },
        { "support_interface_top_layers", 2 },
    });
    const PrintObject &po = *print.objects().front();

    // The object layers are the 0.2 mm stack, whatever the gap asked for.
    REQUIRE(po.layer_count() == 95);
    for (const Layer *layer : po.layers()) {
        INFO("object layer " << layer->print_z);
        CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(0.2, 1e-9));
    }

    std::vector<const SupportLayer *> printed_layers;
    for (const SupportLayer *layer : po.support_layers())
        if (layer->has_extrusions())
            printed_layers.emplace_back(layer);
    REQUIRE(! printed_layers.empty());
    std::sort(printed_layers.begin(), printed_layers.end(),
              [](const SupportLayer *a, const SupportLayer *b) { return a->print_z < b->print_z; });
    const SupportLayer &contact = *printed_layers.back();

    // The gap split: the contact layer of the object grid, the remainder above it, and whether that
    // remainder is printable on this profile. The printable minimum is the profile's own
    // min_layer_height, the same number add_exact_contact_area() refuses on.
    const double min_layer_height = po.slicing_parameters().min_layer_height;
    const double target     = face - gap;
    const double grid_top   = std::floor((target + 1e-9) / 0.2) * 0.2;
    const double remainder  = target - grid_top;
    const bool   printable  = remainder > EPSILON && remainder + EPSILON >= min_layer_height;
    const double expected   = printable ? target : grid_top;
    INFO("contact print_z " << contact.print_z << " height " << contact.height << " expected " << expected
                            << " remainder " << remainder << " min layer height " << min_layer_height);
    CHECK_THAT(contact.print_z, Catch::Matchers::WithinAbs(expected, 1e-6));
    // The air gap never gets narrower than requested: a rounded-up remainder could lift the contact onto a
    // grid top that sits above the requested Z.
    CHECK(contact.print_z <= target + 1e-9);
    // No two support layers share a print_z. If a remainder were lifted onto a grid layer top, the two top
    // contacts at that Z would merge into one support layer and one of the two areas would lose its
    // toolpaths (generate_support_layers() keeps at most one top contact per print_z).
    std::set<double> support_print_zs;
    for (const SupportLayer *layer : po.support_layers())
        CHECK(support_print_zs.insert(micron_rounded(layer->print_z)).second);
    if (printable)
        CHECK_THAT(face - contact.print_z, Catch::Matchers::WithinAbs(gap, 1e-6));
    else {
        CHECK(face - contact.print_z > gap);
        CHECK(face - contact.print_z < gap + 0.2);
    }
    // An exact request is a layer of its own; anything else leaves the top on the object grid.
    for (const SupportLayer *layer : printed_layers) {
        INFO("support layer print_z " << layer->print_z << " height " << layer->height);
        CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(layer->print_z - layer->bottom_z(), 1e-6));
        CHECK(check_layer_paths_thickness(*layer, po) > 0);
    }
    if (std::abs(contact.print_z - target) < 1e-6 && printable) {
        REQUIRE(printed_layers.size() > 1);
        check_supported_from_below(contact, *printed_layers[printed_layers.size() - 2]);
    }

    const GcodeMotion motion = support_gcode_motion(Slic3r::Test::gcode(print));
    REQUIRE(! motion.support_moves.empty());
    CHECK(motion.extrusion_at(contact.print_z) > 0.);
}

// A raft moves the object's whole stack up, including the overhang face; the gap is still measured from
// that face and ends exactly. The tree starts on the raft contact layer, not on the bed and not above it.
TEST_CASE("An organic top gap is exact above a raft and the tree starts on the raft contact", "[TreeSupportOrganicGapExact]")
{
    constexpr double   gap  = 0.1;
    const TriangleMesh mesh = two_tier_mesh();

    Slic3r::Print print;
    slice_with_tree_support(mesh, print, "organic", 30, 0, 2, {
        { "support_top_z_distance",       gap },
        { "support_interface_top_layers", 2 },
    });
    const PrintObject &po = *print.objects().front();
    const double       face = plate_underside_z(po);
    INFO("raft top " << po.slicing_parameters().raft_contact_top_z << " face " << face);
    REQUIRE(face > 16.0);

    std::vector<const SupportLayer *> printed_layers;
    for (const SupportLayer *layer : po.support_layers())
        if (layer->has_extrusions())
            printed_layers.emplace_back(layer);
    REQUIRE(printed_layers.size() > 3);
    std::sort(printed_layers.begin(), printed_layers.end(),
              [](const SupportLayer *a, const SupportLayer *b) { return a->print_z < b->print_z; });
    const SupportLayer &contact = *printed_layers.back();
    INFO("contact print_z " << contact.print_z);
    CHECK_THAT(face - contact.print_z, Catch::Matchers::WithinAbs(gap, 1e-6));
    CHECK_THAT(contact.height, Catch::Matchers::WithinAbs(contact.print_z - contact.bottom_z(), 1e-6));
    check_supported_from_below(contact, *printed_layers[printed_layers.size() - 2]);

    // The raft contact layer is the floor of the tree: the first layer above it starts exactly on its top.
    const double raft_top = po.slicing_parameters().raft_contact_top_z;
    const auto   tree_first = std::find_if(printed_layers.begin(), printed_layers.end(),
        [raft_top](const SupportLayer *layer) { return layer->print_z > raft_top + 1e-6; });
    REQUIRE(tree_first != printed_layers.end());
    INFO("first tree layer bottom " << (*tree_first)->bottom_z() << " raft top " << raft_top);
    CHECK_THAT((*tree_first)->bottom_z(), Catch::Matchers::WithinAbs(raft_top, 1e-6));
}

// The pipeline state F1 is about: a thin exact contact layer can end inside the slab of the support layer
// above it (the belt tilt gives every layer its own overhang, so support stacks over support). That list is
// what the mitigation now walks: with the old all-or-nothing guard the toolpath stage crashed on it.
// In this fixture the contact area and the material of that layer do not intersect in XY - the layer above a
// contact stands next to the object, not over it - so a correct modulation leaves every run at the layer's
// full thickness. Inventing a thinner run here would be a wrong material thickness, not a fix.
TEST_CASE("An exact contact layer inside another support layer's slab does not thin material that does not overlap",
          "[TreeSupportOrganicGap][belt]")
{
    const TriangleMesh mesh = two_tier_mesh();
    Slic3r::Print     print;
    slice_belt_two_tier(print, mesh, true, 0.25);
    const PrintObject &po = *print.objects().front();

    size_t interleaved_layers = 0, paths_checked = 0, reduced_total = 0;
    for (const SupportLayer *layer : po.support_layers()) {
        const bool interleaved = std::any_of(po.support_layers().begin(), po.support_layers().end(),
            [layer](const SupportLayer *other) {
                return other->print_z > layer->bottom_z() + 1e-6 && other->print_z < layer->print_z - 1e-6;
            });
        if (! interleaved)
            continue;
        const OverlapThinning thinning = overlap_thinning(*layer);
        INFO("interleaved support layer print_z " << layer->print_z << " height " << layer->height
                                                  << " bottom " << layer->bottom_z()
                                                  << " reduced " << thinning.reduced.size()
                                                  << " full " << thinning.full.size());
        ++ interleaved_layers;
        reduced_total += thinning.reduced.size();
        for (const ModulatedPathSample &path : thinning.full) {
            INFO("path height " << path.height << " width " << path.width << " mm3_per_mm " << path.mm3_per_mm);
            CHECK_THAT(path.height, Catch::Matchers::WithinAbs(layer->height, 1e-6));
            CHECK(path.width > 0.f);
            CHECK(path.mm3_per_mm > 0.);
            ++ paths_checked;
        }
    }
    INFO("interleaved layers " << interleaved_layers << ", checked paths " << paths_checked
                               << ", reduced runs " << reduced_total);
    // The Z-interleaving exists in a real print ...
    REQUIRE(interleaved_layers > 0);
    REQUIRE(paths_checked > 0);
    // ... and it is a Z-only overlap here: no material may be thinned where nothing shares the slab in XY.
    CHECK(reduced_total == 0);
}
