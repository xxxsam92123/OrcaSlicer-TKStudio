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
#include <set>
#include <string>
#include <vector>
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
        // Tops still end at 1.6, so the heights of the grid add up to the same total as before the gap.
        CHECK_THAT(grid.z(5), Catch::Matchers::WithinAbs(1.6, 1e-9));
        CHECK_THAT(grid.bottom_z(5), Catch::Matchers::WithinAbs(1.3, 1e-9));
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
        CHECK(std::count_if(grid.entries.begin(), grid.entries.end(),
                            [](const TreeSupportLayerGrid::Entry &entry) { return entry.is_gap_layer; }) == 0);
    }

    SECTION("A gap thinner than a layer steps down but stays above the raft") {
        // 1.01 leaves less than MIN_SUPPORT_LAYER_HEIGHT of layer 5, so the contact stays one layer lower.
        CHECK(grid.place_top_contact(1.4, 0.39) == 4);
        CHECK_THAT(grid.z(5), Catch::Matchers::WithinAbs(1.2, 1e-9));
        CHECK(! grid.entries[5].is_gap_layer);
    }

    SECTION("A contact that fits lowers its own layer, leaving the raft alone") {
        CHECK(grid.place_top_contact(1.4, 0.35) == 5);
        CHECK_THAT(grid.z(5), Catch::Matchers::WithinAbs(1.05, 1e-9));
        CHECK(grid.entries[5].is_gap_layer);
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
// carried side by side: feeding the raw value to the quantised member shifted a tree_slim contact
// layer by one layer in that configuration (review round 1, issue 2).
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
    // The raw millimetre value is what the organic grid places its contact layers from.
    CHECK_THAT(mesh_settings.support_top_z_distance_mm, Catch::Matchers::WithinAbs(top_z_distance, 1e-9));
}
