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
#include "libslic3r/Support/TreeSupport3D.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Support/TreeSupportCommon.hpp"

#include <algorithm>
#include <cstdio>
#include <set>
#include <string>
#include <vector>
#include <sstream>
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
    // ends there, so the air gap under the overhang is 16.0 - print_z.
    const SupportLayer &contact = *po.support_layers().back();
    INFO("contact layer print_z=" << contact.print_z << " height=" << contact.height);
    // support_top_z_distance is a lower bound, not a layer height: the contact takes the highest layer
    // that is at least `gap` below the face, so the gap is never narrower than requested ...
    CHECK(contact.print_z <= 16.0 - gap + 1e-9);
    // ... and at most one layer wider.
    CHECK(contact.print_z > 16.0 - gap - 0.2 - 1e-9);
    // The layer prints the object layer's own 0.2 mm: the gap remainder never becomes its height.
    CHECK_THAT(contact.height, Catch::Matchers::WithinAbs(0.2, 1e-9));
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

// Every height the object's layer stack has, rounded to a micron: a support path may only be as thick as
// one of these. Anything else (0.1 / 0.14 / 0.23 on a 0.2 mm profile) is a gap remainder.
std::multiset<double> object_layer_heights(const PrintObject &po)
{
    std::multiset<double> heights;
    for (const Layer *layer : po.layers())
        heights.insert(std::round(layer->height * 1000.) / 1000.);
    return heights;
}

// A 0.2 mm profile with a gap the user did not round to it. support_top_z_distance is a distance in
// millimetres, but the support has no layer stack of its own: the contact goes onto the highest support
// layer that keeps at least that much air under the overhang, so the gap is quantised upwards and every
// layer keeps the object's Z and height. The previous implementation pushed the remainder into the
// contact layer's height (0.1 on this profile, 0.14 on a 0.4 mm one), which reached Flow and the layer
// height report. The assertions below are the ones that fail for that implementation.
TEST_CASE("An organic top gap keeps every support layer on the object's own layer heights", "[TreeSupportOrganicGap]")
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

    // SupportLayer::height: every support layer sits on an object layer Z and prints that object layer's
    // own height. Not one of them may be the gap remainder.
    REQUIRE(! po.support_layers().empty());
    const std::multiset<double> profile_heights = object_layer_heights(po);
    size_t                      base_paths      = 0;
    size_t                      interface_paths = 0;
    for (const SupportLayer *layer : po.support_layers()) {
        INFO("support layer print_z " << layer->print_z << " height " << layer->height);
        CHECK_THAT(layer->print_z, Catch::Matchers::WithinAbs(std::round(layer->print_z / 0.2) * 0.2, 1e-9));
        CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(0.2, 1e-12));
        // Flow / path height: what the extrusions of this layer are actually built with, base and
        // interface stack alike.
        for (const SupportPathRole &path : support_path_roles(*layer)) {
            INFO("support path role " << int(path.role) << " height " << path.height);
            CHECK(profile_heights.find(path.height) != profile_heights.end());
            (path.role == erSupportMaterialInterface ? interface_paths : base_paths) += 1;
        }
    }
    // The interface stack has a flow of its own, so both kinds have to have been looked at.
    REQUIRE(base_paths > 0);
    REQUIRE(interface_paths > 0);

    // The top contact is the highest support layer, and its top is that layer's own print_z. The air gap
    // under the overhang at 16.0 is therefore never narrower than requested and at most one layer wider;
    // 0.2 is a whole layer of this profile and comes out exact.
    const SupportLayer &contact = *po.support_layers().back();
    INFO("contact print_z " << contact.print_z << " = gap " << 16.0 - contact.print_z);
    CHECK(contact.print_z <= 16.0 - gap + 1e-9);
    CHECK(contact.print_z > 16.0 - gap - 0.2 - 1e-9);
    if (std::abs(gap - 0.2) < 1e-12)
        CHECK_THAT(16.0 - contact.print_z, Catch::Matchers::WithinAbs(gap, 1e-9));

    // The G-code reports the same set of layer heights as the same print without support: no 0.1, 0.14
    // or 0.23 sneaks into the legend through a support layer.
    const std::multiset<double> heights       = gcode_layer_heights(Slic3r::Test::gcode(supported));
    const std::multiset<double> heights_plain = gcode_layer_heights(Slic3r::Test::gcode(unsupported));
    // Both sides parse the report; two empty sets would compare equal and prove nothing.
    REQUIRE(! heights.empty());
    REQUIRE(! heights_plain.empty());
    CHECK(heights == heights_plain);
}

// The zero the tests below assert is only meaningful if the capture reads the stream the generator
// writes and the counters match its format, so both are checked against the function that emits the
// record instead of against a string this test made up.
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
        // layer height the generator must not have to drop one (review round 3, issue 3).
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

        std::vector<const Layer *> object_layers;
        for (size_t i = 0; i < po.layer_count(); ++ i) {
            const Layer &layer = *po.get_layer(i);
            const Layer &plain = *po_plain.get_layer(i);
            INFO("object layer " << i << " z " << layer.print_z << " h " << layer.height);
            // The profile's own layers, unchanged by the support.
            CHECK_THAT(layer.print_z, Catch::Matchers::WithinAbs(plain.print_z, 1e-12));
            CHECK_THAT(layer.height, Catch::Matchers::WithinAbs(plain.height, 1e-12));
            CHECK_THAT(layer.slice_z, Catch::Matchers::WithinAbs(plain.slice_z, 1e-12));
            object_layers.emplace_back(&layer);
        }

        // Every support layer sits on one of those Zs and prints *that* layer's height - the 0.4 mm
        // section is printed as 0.4, never as the 0.14 a gap remainder would leave of it.
        const std::multiset<double> profile_heights = object_layer_heights(po);
        size_t                      base_paths      = 0;
        size_t                      interface_paths = 0;
        double contact_print_z = 0.;
        for (const SupportLayer *layer : po.support_layers()) {
            INFO("support layer print_z " << layer->print_z << " height " << layer->height);
            const auto it = std::find_if(object_layers.begin(), object_layers.end(),
                [&layer](const Layer *object_layer) { return std::abs(object_layer->print_z - layer->print_z) < 1e-9; });
            CHECK(it != object_layers.end());
            if (it == object_layers.end())
                continue;
            CHECK_THAT(layer->height, Catch::Matchers::WithinAbs((*it)->height, 1e-12));
            // Base and interface / roof paths both take their height from the layer, so both are checked.
            for (const SupportPathRole &path : support_path_roles(*layer)) {
                INFO("support path role " << int(path.role) << " height " << path.height);
                CHECK(profile_heights.find(path.height) != profile_heights.end());
                (path.role == erSupportMaterialInterface ? interface_paths : base_paths) += 1;
            }
            contact_print_z = std::max(contact_print_z, layer->print_z);
        }
        REQUIRE(base_paths > 0);
        REQUIRE(interface_paths > 0);

        // The gap is at least as wide as requested, and less than one layer wider. On a variable layer
        // height a whole layer is what it is at that Z, so one 0.4 mm layer is the bound here.
        INFO("contact print_z " << contact_print_z << " = gap " << 16.0 - contact_print_z);
        CHECK(contact_print_z <= 16.0 - gap + 1e-9);
        CHECK(contact_print_z > 16.0 - gap - 0.4 - 1e-9);

        // The G-code reports the same layer heights as the same print without support: the support adds
        // no 0.14 / 0.23 to the legend.
        const std::multiset<double> heights       = gcode_layer_heights(Slic3r::Test::gcode(supported));
        const std::multiset<double> heights_plain = gcode_layer_heights(Slic3r::Test::gcode(unsupported));
        // Both sides parse the report; two empty sets would compare equal and prove nothing.
        REQUIRE(! heights.empty());
        REQUIRE(! heights_plain.empty());
        CHECK(heights == heights_plain);
    }
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
// be taken (review round 3, issue 2).
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
    const OverhangContact contact = place_overhang_contact(grid, 40, mesh, facets, area, 0., 0.2);
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

// A sloped face can cross the whole search slab while its lowest vertex lies below it. Indexing the
// facets by their lowest vertex alone dropped such a facet, and the placement then measured the gap from
// the top of the layer below the area's - a Z the face is not on - which closed the gap to 0.32 mm
// against a requested 0.4. The reference has to be bounded by the slab that was searched instead: for a
// facet that only reaches into the slab from below, the bottom of this slab is the lowest Z the face can
// have, so a contact placed against it can only widen the gap.
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
    const OverhangContact contact = place_overhang_contact(grid, 40, mesh, facets, area, 0., requested);
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
    // ... and every support layer prints one of those layers, at that layer's height.
    REQUIRE(! po.support_layers().empty());
    for (const SupportLayer *layer : po.support_layers()) {
        INFO("support layer print_z " << layer->print_z << " height " << layer->height);
        const Layer *object_layer = nullptr;
        for (const Layer *candidate : po.layers())
            if (std::abs(candidate->print_z - layer->print_z) < 1e-9)
                object_layer = candidate;
        REQUIRE(object_layer != nullptr);
        CHECK_THAT(layer->height, Catch::Matchers::WithinAbs(object_layer->height, 1e-12));
    }
}
