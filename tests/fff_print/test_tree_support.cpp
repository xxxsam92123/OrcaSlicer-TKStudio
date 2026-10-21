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
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Support/TreeSupportCommon.hpp"

#include <algorithm>

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

// The organic generator used to turn support_top_z_distance into a whole number of layers, so 0.3
// came out as one skipped layer plus a full one (0.4) and 0.1 as 0.2. The top contact is now placed
// at the requested distance below the overhang, whatever the layer height.
TEST_CASE("Organic support places its top contact the configured distance below the overhang", "[TreeSupportOrganicGap]")
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

    // "support" matches both the base and the interface role comments.
    const std::string      g         = gcode(print);
    const std::set<double> support_z = layers_with_role(g, "support");
    INFO("support layers=" << print.objects().front()->support_layers().size() << " gcode=" << g.size() << " zs=" << support_z.size());
    REQUIRE(! support_z.empty());
    REQUIRE_THAT(*support_z.rbegin(), Catch::Matchers::WithinAbs(16.0 - gap, 1e-6));
}

TEST_CASE("Support layer grid resolves Z and gap layers on a grid with mixed layer heights", "[TreeSupportOrganicGap]")
{
    using namespace Slic3r::TreeSupport3D;
    // Tops 0.2, 0.4, 0.8, 1.2, 1.4, 1.6: two 0.2 layers, two 0.4 layers, two 0.2 layers.
    const std::vector<double> heights { 0.2, 0.2, 0.4, 0.4, 0.2, 0.2 };
    TreeSupportLayerGrid grid;
    double               bottom = 0.;
    for (const double h : heights) {
        TreeSupportLayerGrid::Entry entry;
        entry.bottom_z = bottom;
        entry.print_z  = bottom + h;
        entry.height   = h;
        bottom         = entry.print_z;
        grid.entries.emplace_back(entry);
    }

    CHECK_THAT(grid.z(2), Catch::Matchers::WithinAbs(0.8, 1e-9));
    CHECK_THAT(grid.h(2), Catch::Matchers::WithinAbs(0.4, 1e-9));
    CHECK(grid.idx_ceil(0.7) == 2);
    CHECK(grid.idx_floor(0.7) == 1);
    CHECK_THAT(grid.z_span(1, 4), Catch::Matchers::WithinAbs(1.0, 1e-9));

    SECTION("A gap that lands on a layer top leaves that layer unchanged") {
        // Overhang bottom 1.6 and a 0.2 gap give 1.4, which is layer 4's own top.
        REQUIRE(grid.lower_top(4, 1.4));
        CHECK(! grid.entries[4].is_gap_layer);
        CHECK_THAT(grid.z(4), Catch::Matchers::WithinAbs(1.4, 1e-9));
        CHECK_THAT(grid.h(4), Catch::Matchers::WithinAbs(0.2, 1e-9));
    }

    SECTION("A gap between two layer tops lowers the upper one and flags it") {
        // Overhang bottom 1.6 and a 0.3 gap give 1.3, which is inside layer 4.
        REQUIRE(grid.lower_top(4, 1.3));
        CHECK(grid.entries[4].is_gap_layer);
        CHECK_THAT(grid.z(4), Catch::Matchers::WithinAbs(1.3, 1e-9));
        CHECK_THAT(grid.h(4), Catch::Matchers::WithinAbs(0.1, 1e-9));
        // The grid stays tiled: the layer above now starts at the lowered top, so the two neighbours
        // leave no vertical hole and the heights still add up to the last top (contract §4.1 rule 4).
        CHECK_THAT(grid.entries[5].bottom_z, Catch::Matchers::WithinAbs(1.3, 1e-9));
        CHECK_THAT(grid.entries[5].height, Catch::Matchers::WithinAbs(0.3, 1e-9));
        CHECK_THAT(grid.z_span(0, 5), Catch::Matchers::WithinAbs(1.6, 1e-9));
        // Exactly one contact layer is flagged, not the layers above it.
        CHECK(std::count_if(grid.entries.begin(), grid.entries.end(),
                            [](const TreeSupportLayerGrid::Entry &entry) { return entry.is_gap_layer; }) == 1);
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
