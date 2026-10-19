#include <catch2/catch_all.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExternalBridgeGrid.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Surface.hpp"

#include "test_helpers.hpp"

using namespace Slic3r;

static ExPolygon make_square(double width, double height)
{
    Points points {
        Point::new_scale(0, 0),
        Point::new_scale(width, 0),
        Point::new_scale(width, height),
        Point::new_scale(0, height)
    };
    return ExPolygon(std::move(points));
}

static double total_area(const Surfaces &surfaces)
{
    double result = 0.;
    for (const Surface &surface : surfaces)
        result += surface.area();
    return result;
}

static double total_area(const ExPolygons &expolygons)
{
    double result = 0.;
    for (const ExPolygon &expolygon : expolygons)
        result += expolygon.area();
    return result;
}

TEST_CASE("External bridge grid falls back to the complete surface", "[ExternalBridgeGrid]")
{
    Surface bridge(stBottomBridge, make_square(20, 20));

    ExternalBridgeGridSettings settings;
    REQUIRE(split_external_bridge_surface(bridge, settings).size() == 1);

    settings.enabled = true;
    settings.cells_x = 2;
    settings.cells_y = 2;
    settings.angle_step_deg = 0.;
    REQUIRE(split_external_bridge_surface(bridge, settings).size() == 1);

    settings.angle_step_deg = 15.;
    Surface small_bridge(stBottomBridge, make_square(5, 5));
    REQUIRE(split_external_bridge_surface(small_bridge, settings).size() == 1);
}

TEST_CASE("External bridge grid preserves surface area", "[ExternalBridgeGrid]")
{
    Surface bridge(stBottomBridge, make_square(64, 64));
    bridge.bridge_angle = 0.;

    ExternalBridgeGridSettings settings;
    settings.enabled = true;
    settings.cells_x = 4;
    settings.cells_y = 4;
    settings.angle_step_deg = 15.;
    const Surfaces split = split_external_bridge_surface(bridge, settings);

    REQUIRE(split.size() == 16);
    REQUIRE(total_area(split) == Catch::Approx(bridge.area()));
}

TEST_CASE("External bridge grid honors requested resolution", "[ExternalBridgeGrid]")
{
    for (const int cells : { 5, 8, 32 }) {
        Surface bridge(stBottomBridge, make_square(cells * 8., cells * 8.));
        ExternalBridgeGridSettings settings;
        settings.enabled = true;
        settings.cells_x = cells;
        settings.cells_y = cells;
        settings.angle_step_deg = 15.;

        const Surfaces split = split_external_bridge_surface(bridge, settings);
        REQUIRE(split.size() == size_t(cells * cells));
        REQUIRE(total_area(split) == Catch::Approx(bridge.area()));
    }
}

// A CAD or mesh derived outline reaches the slicer as a quantized polyline: its vertices
// carry a unit scale wobble that no integer clipping can reproduce exactly. Splitting such
// an outline into grid cells therefore loses slivers far below printer resolution, which
// must not make the splitter silently fall back to a coarser grid than the requested one.
// The wobble alternates with the vertex index, so the fixture's vertex sequence is fixed.
static void quantize_outline(Polygon &outline)
{
    for (size_t i = 0; i < outline.points.size(); ++i)
        outline.points[i].y() += (i % 2 == 1) ? 1 : -1;
}

static ExPolygon make_quantized_bridge()
{
    ExPolygon bridge = make_square(64., 64.);
    quantize_outline(bridge.contour);
    return bridge;
}

// A quantized hole is outline as well, and the production budget counts it alongside the
// contour, because the same cuts re-snap its boundary. Nothing here proves that term is needed:
// see the hole-only case below, whose residual the contour length alone would already cover.
// The outline directions are the valid ones for the non-zero fill rule the clipping uses -
// counter-clockwise contour, clockwise hole - and the hole edges are deliberately off the 8 mm
// grid lines, so every cut crosses them instead of running along them.
static ExPolygon make_quantized_bridge_with_hole(bool quantize_contour)
{
    ExPolygon bridge = make_square(64., 64.);
    if (quantize_contour)
        quantize_outline(bridge.contour);

    // Clockwise 25 x 26 mm hole from (13, 11) to (38, 37).
    Polygon hole { Points {
        Point::new_scale(13., 11.),
        Point::new_scale(13., 37.),
        Point::new_scale(38., 37.),
        Point::new_scale(38., 11.)
    } };
    quantize_outline(hole);
    bridge.holes.emplace_back(std::move(hole));
    return bridge;
}

// The outline length the production budget sums: the contour plus every hole.
static double outline_length(const ExPolygon &expolygon)
{
    double length = expolygon.contour.length();
    for (const Polygon &hole : expolygon.holes)
        length += hole.length();
    return length;
}

// Stand-in for AREA_TOLERANCE_PER_OUTLINE_UNIT in src/libslic3r/ExternalBridgeGrid.cpp. That
// constant is internal to its translation unit, so the tests restate its value here rather than
// reading it, and the two have to be kept in step by hand.
static constexpr double area_tolerance_per_outline_unit = 4.;

TEST_CASE("External bridge grid keeps the requested grid on a quantized bridge outline", "[ExternalBridgeGrid]")
{
    Surface bridge(stBottomBridge, make_quantized_bridge());
    bridge.bridge_angle = 0.;

    const Surfaces cells = split_external_bridge_surface(bridge, ExternalBridgeGridSettings { true, 8, 8, 15. });
    REQUIRE(cells.size() == 64);

    const Polylines walls = external_bridge_grid_walls(cells);
    REQUIRE(walls.size() == 14);
    REQUIRE(std::count_if(walls.begin(), walls.end(), [](const Polyline &wall) {
        return wall.points.front().x() == wall.points.back().x();
    }) == 7);
    REQUIRE(std::count_if(walls.begin(), walls.end(), [](const Polyline &wall) {
        return wall.points.front().y() == wall.points.back().y();
    }) == 7);

    // The partition may lose or gain integer clipping slivers along the outline, never a cell.
    const double residual = std::abs(total_area(cells) - bridge.area());
    // The rule this replaced measured the residual against max(1, |area| * 1e-9), smaller than
    // the clipping noise of a quantized outline: this fixture is already above it, so the old
    // rule could only degrade the requested grid.
    REQUIRE(residual > std::max(1., std::abs(bridge.area()) * 1e-9));
    REQUIRE(residual <= outline_length(bridge.expolygon) * area_tolerance_per_outline_unit);
    REQUIRE(residual < scale_(8.) * scale_(8.));
}

TEST_CASE("External bridge grid keeps the requested grid on a quantized bridge with a hole", "[ExternalBridgeGrid]")
{
    Surface bridge(stBottomBridge, make_quantized_bridge_with_hole(true));
    bridge.bridge_angle = 0.;

    REQUIRE(bridge.expolygon.contour.is_counter_clockwise());
    REQUIRE(bridge.expolygon.holes.size() == 1);
    REQUIRE(bridge.expolygon.holes.front().is_clockwise());

    const Surfaces cells = split_external_bridge_surface(bridge, ExternalBridgeGridSettings { true, 8, 8, 15. });
    // The four cells that lie entirely inside the hole carry no material and drop out.
    REQUIRE(cells.size() == 60);
    REQUIRE(external_bridge_grid_walls(cells).size() == 20);

    const double residual = std::abs(total_area(cells) - bridge.area());
    REQUIRE(residual > std::max(1., std::abs(bridge.area()) * 1e-9));
    REQUIRE(residual <= outline_length(bridge.expolygon) * area_tolerance_per_outline_unit);
    REQUIRE(residual < scale_(8.) * scale_(8.));
}

TEST_CASE("External bridge grid keeps the requested grid when only the hole outline is quantized", "[ExternalBridgeGrid]")
{
    // The same hole under an exact outer contour, so the hole boundary is the only quantized
    // outline. This covers that the request still holds and that the residual stays inside the
    // budget; it does not pin the hole term down. The residual here is 3500000 units2, about
    // 290x below the 1024000000 units2 the contour length alone would allow (256e6 units of
    // contour * 4), so the fixture is accepted with or without the hole length in the sum and
    // would keep passing if that term were removed. Making the hole term load-bearing needs a
    // fixture whose residual has to exceed the contour-only budget, which this is not.
    //
    // This case also leaves out the rule the change replaced, residual > max(1, |area| * 1e-9):
    // here the residual is 3500000 units2 against that rule's 3446000 units2 bound, a margin of
    // only ~1.6%, which one integer unit of difference in a single clipping intersection is
    // enough to flip. A guard that shallow would only fire while the fixture and the clipping
    // path are both byte-identical, so it is not asserted as a regression. The old rule still
    // degrades this fixture to 52 cells instead of 60, which is what the cells.size() assertion
    // below holds; the two other quantized fixtures keep the replaced rule in full and clear its
    // bound by ~5.9x and ~8.0x, and they are the ones that show the old rule failing. The
    // remaining assertions here are the new budget and the one-cell bound.
    Surface bridge(stBottomBridge, make_quantized_bridge_with_hole(false));
    bridge.bridge_angle = 0.;

    const Surfaces cells = split_external_bridge_surface(bridge, ExternalBridgeGridSettings { true, 8, 8, 15. });
    REQUIRE(cells.size() == 60);

    const double residual = std::abs(total_area(cells) - bridge.area());
    REQUIRE(residual <= outline_length(bridge.expolygon) * area_tolerance_per_outline_unit);
    REQUIRE(residual < scale_(8.) * scale_(8.));
}

TEST_CASE("External bridge grid still caps the grid at the cells the bridge can hold", "[ExternalBridgeGrid]")
{
    // A 64 x 40 mm bridge cannot hold eight rows of the 8 mm minimum cell, so the request
    // is capped at 8 x 5 rather than split into cells below the minimum size.
    Surface bridge(stBottomBridge, make_square(64., 40.));
    bridge.bridge_angle = 0.;

    const Surfaces cells = split_external_bridge_surface(bridge, ExternalBridgeGridSettings { true, 8, 8, 15. });
    REQUIRE(cells.size() == 40);
    REQUIRE(external_bridge_grid_walls(cells).size() == 11);
}

TEST_CASE("External bridge grid configuration allows 32 cells per axis", "[ExternalBridgeGrid]")
{
    REQUIRE(print_config_def.get("external_bridge_grid_cells_x")->min == 1);
    REQUIRE(print_config_def.get("external_bridge_grid_cells_x")->max == 32);
    REQUIRE(print_config_def.get("external_bridge_grid_cells_y")->min == 1);
    REQUIRE(print_config_def.get("external_bridge_grid_cells_y")->max == 32);
}

TEST_CASE("External bridge grid infill/wall overlap stays within the original bridge", "[ExternalBridgeGrid]")
{
    const DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    REQUIRE_THAT(config.opt<ConfigOptionPercent>("external_bridge_infill_wall_overlap")->value, Catch::Matchers::WithinAbs(15., 1e-6));
    REQUIRE_THAT(config.opt<ConfigOptionPercent>("external_bridge_grid_infill_wall_overlap")->value, Catch::Matchers::WithinAbs(7.5, 1e-6));
    REQUIRE(print_config_def.get("external_bridge_infill_wall_overlap")->min == 0);
    REQUIRE(print_config_def.get("external_bridge_infill_wall_overlap")->max == 100);
    REQUIRE(print_config_def.get("external_bridge_grid_infill_wall_overlap")->min == 0);
    REQUIRE(print_config_def.get("external_bridge_grid_infill_wall_overlap")->max == 100);

    Surface bridge(stBottomBridge, make_square(64, 32));
    bridge.bridge_angle = 0.;
    const Surfaces cells = split_external_bridge_surface(bridge, { true, 2, 1, 15. });
    REQUIRE(cells.size() == 2);

    const Polylines shared_walls = external_bridge_grid_walls(cells);
    REQUIRE(shared_walls.size() == 1);

    const ExPolygons default_infill = external_bridge_grid_infill_area(cells.front(), 0., 0.4);
    const ExPolygons overlapping_infill = external_bridge_grid_infill_area(cells.front(), 0.5, 0.4);
    REQUIRE_THAT(total_area(default_infill), Catch::Matchers::WithinAbs(cells.front().area(), 1e-6));
    REQUIRE(total_area(overlapping_infill) > total_area(default_infill));
    const BoundingBox original_bounds = get_extents(bridge.expolygon);
    const BoundingBox infill_bounds = get_extents(overlapping_infill);
    REQUIRE(infill_bounds.min.x() >= original_bounds.min.x());
    REQUIRE(infill_bounds.min.y() >= original_bounds.min.y());
    REQUIRE(infill_bounds.max.x() <= original_bounds.max.x());
    REQUIRE(infill_bounds.max.y() <= original_bounds.max.y());
}


TEST_CASE("External bridge grid caps the bounded grid", "[ExternalBridgeGrid]")
{
    Surface bridge(stBottomBridge, make_square(256, 256));
    bridge.bridge_angle = 0.;

    ExternalBridgeGridSettings settings;
    settings.enabled = true;
    settings.cells_x = 64;
    settings.cells_y = 64;
    settings.angle_step_deg = 15.;
    const Surfaces split = split_external_bridge_surface(bridge, settings);

    REQUIRE(split.size() == 1024);
}

TEST_CASE("External bridge grid alternates bridge angles", "[ExternalBridgeGrid]")
{
    Surface bridge(stBottomBridge, make_square(32, 32));
    bridge.bridge_angle = Geometry::deg2rad(90.);

    ExternalBridgeGridSettings settings;
    settings.enabled = true;
    settings.cells_x = 2;
    settings.cells_y = 2;
    settings.angle_step_deg = 15.;
    const Surfaces split = split_external_bridge_surface(bridge, settings);

    REQUIRE(split.size() == 4);
    size_t negative_offset = 0;
    size_t positive_offset = 0;
    for (const Surface &surface : split) {
        if (surface.bridge_angle == Catch::Approx(Geometry::deg2rad(75.)))
            ++negative_offset;
        if (surface.bridge_angle == Catch::Approx(Geometry::deg2rad(105.)))
            ++positive_offset;
    }
    REQUIRE(negative_offset == 2);
    REQUIRE(positive_offset == 2);
}

TEST_CASE("External bridge grid emits bridge walls only for split cells", "[ExternalBridgeGrid]")
{
    struct BridgeGridEntities {
        std::set<int> surface_angles;
        std::set<int> path_angles;
        size_t        cell_count = 0;
        size_t        bridge_wall_loops = 0;
        size_t        first_wall_entity = size_t(-1);
        size_t        first_bridge_fill_entity = size_t(-1);
        size_t        wall_entities_before_bridge = 0;
        double        bridge_fill_length = 0.;
        double        bridge_surface_area = 0.;
        BoundingBox   bridge_bbox;
    };

    const auto collect_entities = [](bool enabled, double angle_step, const char *overlap = "0%", const char *grid_overlap = "7.5%") {
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "external_bridge_grid_enable", enabled ? "1" : "0" },
        { "external_bridge_grid_cells_x", "2" },
        { "external_bridge_grid_cells_y", "2" },
        { "external_bridge_grid_angle_step", std::to_string(angle_step) },
        { "external_bridge_infill_wall_overlap", overlap },
        { "external_bridge_grid_infill_wall_overlap", grid_overlap },
        { "wall_loops", "0" }
    });

    Print print;
    TriangleMesh model_mesh = Test::mesh(Test::TestMesh::bridge_with_hole);
    model_mesh.scale(Vec3f(1.f, 2.f, 1.f));
    Test::init_and_process_print({ std::move(model_mesh) }, print, config);

    BridgeGridEntities result;
    for (const Layer *layer : print.objects().front()->layers()) {
        for (const LayerRegion *region : layer->regions()) {
            for (const Surface &surface : region->fill_surfaces.surfaces)
                if (surface.surface_type == stBottomBridge && surface.bridge_angle >= 0.) {
                    result.surface_angles.insert(int(std::lround(Geometry::rad2deg(surface.bridge_angle))) % 180);
                    result.cell_count += surface.external_bridge_grid;
                    result.bridge_surface_area += surface.expolygon.area();
                    result.bridge_bbox.merge(get_extents(surface.expolygon));
                }
            for (size_t collection_index = 0; collection_index < region->fills.entities.size(); ++collection_index) {
                const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(region->fills.entities[collection_index]);
                REQUIRE(collection != nullptr);
                bool has_wall = false;
                bool has_bridge_fill = false;
                for (const ExtrusionEntity *entity : collection->entities) {
                    has_wall |= entity->role() == erOverhangPerimeter;
                    has_bridge_fill |= entity->role() == erBridgeInfill;
                }
                if (has_wall)
                    result.first_wall_entity = std::min(result.first_wall_entity, collection_index);
                if (has_bridge_fill)
                    result.first_bridge_fill_entity = std::min(result.first_bridge_fill_entity, collection_index);
            }
            for (const ExtrusionEntity *entity : region->fills.flatten().entities) {
                const auto account = [&result](const ExtrusionPath &path) {
                    if (path.role() != erBridgeInfill)
                        return;
                    result.bridge_fill_length += path.length();
                    const Points3 &points = path.polyline.points;
                    for (size_t i = 1; i < points.size(); ++i) {
                        const double dx = double(points[i].x() - points[i - 1].x());
                        const double dy = double(points[i].y() - points[i - 1].y());
                        if (std::hypot(dx, dy) > 0.)
                            result.path_angles.insert(int(std::lround(Geometry::rad2deg(std::atan2(dy, dx)))) % 180);
                    }
                };
                if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity)) {
                    if (path->role() == erOverhangPerimeter)
                        ++result.bridge_wall_loops;
                    else
                        account(*path);
                }
                else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(entity))
                    for (const ExtrusionPath &path : multi->paths)
                        account(path);
                else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity))
                    for (const ExtrusionPath &path : loop->paths)
                        account(path);
            }
        }
    }

    REQUIRE(result.bridge_bbox.defined);
    // The bridge surface is the unsupported part around the hole, not the
    // full model envelope.  Its width depends on the fixture geometry and is
    // smaller than the 32 mm model dimension on the X axis.
    REQUIRE(unscaled<double>(result.bridge_bbox.size().x()) > 0.);
    REQUIRE(unscaled<double>(result.bridge_bbox.size().y()) > 0.);
    return result;
    };

    const auto disabled = collect_entities(false, 15.);
    REQUIRE(disabled.surface_angles.size() == 1);
    REQUIRE(disabled.cell_count == 0);
    REQUIRE(disabled.bridge_wall_loops == 0);

    const auto fallback = collect_entities(true, 0.);
    REQUIRE(fallback.cell_count == 0);
    REQUIRE(fallback.bridge_wall_loops == 0);

    const auto enabled = collect_entities(true, 15.);
    // Boundary classification can differ at the model's quantized edge
    // between platforms.  The stable contract is that splitting produces
    // multiple valid cells and at least one bridge angle.
    REQUIRE(enabled.cell_count >= 2);
    REQUIRE(enabled.surface_angles.size() >= 1);
    REQUIRE(enabled.path_angles.size() >= 2);
    REQUIRE(enabled.first_wall_entity != size_t(-1));
    REQUIRE(enabled.first_bridge_fill_entity != size_t(-1));
    REQUIRE(enabled.first_wall_entity < enabled.first_bridge_fill_entity);
    // The shared grid boundaries are merged into continuous straight paths.
    // A valid split must therefore emit at least one overhang-perimeter path.
    REQUIRE(enabled.bridge_wall_loops >= 1);

    const auto grid_zero = collect_entities(true, 15., "15%", "0%");
    const auto grid_full = collect_entities(true, 15., "15%", "100%");
    REQUIRE(grid_full.bridge_surface_area != grid_zero.bridge_surface_area);

}

TEST_CASE("External bridge overlap changes the bridge-to-overhang-wall seam", "[ExternalBridgeOverlap]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    CAPTURE(wall_generator);
    struct BridgeMeasure {
        double seam_area = 0.;
        double bridge_length = 0.;
        double surface_area = 0.;
        double angle_sum = 0.;
        double seed_area = 0.;
    };
    const auto bridge_measure = [wall_generator](const char *external_overlap, const char *infill_overlap) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({
            { "external_bridge_grid_enable", false },
            { "external_bridge_infill_wall_overlap", external_overlap },
            { "infill_wall_overlap", infill_overlap },
            { "wall_generator", wall_generator },

            // The dedicated overlap is defined at the bridge-to-overhang-wall
            // seam, so this fixture must generate an actual perimeter target.
            { "wall_loops", 3 },
        });

        Print print;
        Test::init_and_process_print({ Test::TestMesh::bridge_with_hole }, print, config);

        BridgeMeasure result;
        for (const Layer *layer : print.objects().front()->layers())
            for (const LayerRegion *region : layer->regions()) {
                for (const Surface &surface : region->slices.surfaces)
                    if (surface.surface_type == stBottomBridge)
                        result.seed_area += area(intersection_ex(ExPolygons{surface.expolygon}, region->external_bridge_fill_expolygons));
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    if (surface.surface_type == stBottomBridge) {
                        result.surface_area += surface.expolygon.area();
                        result.angle_sum += surface.bridge_angle;
                        result.seam_area += area(intersection_ex(
                            ExPolygons{ surface.expolygon },
                            offset(region->external_bridge_wall_boundary, float(scale_(1.0)))));
                    }
                for (const ExtrusionEntity *entity : region->fills.flatten().entities) {
                    const auto account = [&result](const ExtrusionPath &path) {
                        if (path.role() == erBridgeInfill)
                            result.bridge_length += path.length();
                    };
                    if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity))
                        account(*path);
                    else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(entity))
                        for (const ExtrusionPath &path : multi->paths)
                            account(path);
                    else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity))
                        for (const ExtrusionPath &path : loop->paths)
                            account(path);
                }
            }
        return result;
    };

    const auto external_zero = bridge_measure("0%", "0%");
    const auto external_full = bridge_measure("100%", "0%");
    const auto infill_only_zero = bridge_measure("0%", "100%");
    const auto both_full = bridge_measure("100%", "100%");
    CAPTURE(external_zero.surface_area, infill_only_zero.surface_area, external_zero.angle_sum, infill_only_zero.angle_sum);
    CAPTURE(external_zero.seed_area, infill_only_zero.seed_area);
    REQUIRE(external_zero.seam_area > 0.);
    CHECK(external_full.seam_area > external_zero.seam_area);
    CHECK(infill_only_zero.seam_area == Catch::Approx(external_zero.seam_area).margin(SCALED_EPSILON));
    CHECK(both_full.seam_area == Catch::Approx(external_full.seam_area).margin(SCALED_EPSILON));
    REQUIRE(external_zero.bridge_length > 0.);
    CHECK(infill_only_zero.bridge_length == Catch::Approx(external_zero.bridge_length).margin(SCALED_EPSILON));
    CHECK(both_full.bridge_length == Catch::Approx(external_full.bridge_length).margin(SCALED_EPSILON));

    DynamicPrintConfig defaults = DynamicPrintConfig::full_print_config();
    CHECK(defaults.opt<ConfigOptionPercent>("external_bridge_infill_wall_overlap")->value == Catch::Approx(15.));
}

TEST_CASE("Internal bridge overlap is isolated from external bridge overlap", "[InternalBridgeOverlap]")
{
    struct Measure {
        size_t count = 0;
        double length = 0.;
        double outside_clean_area = 0.;
        double outside_internal_boundary_area = 0.;
        double internal_bridge_boundary_area = 0.;
    };

    const auto measure = [](Test::TestMesh mesh, const char *wall_generator, const char *external_overlap, const char *internal_overlap) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({
            { "dont_filter_internal_bridges", "nofilter" },
            { "wall_generator", wall_generator },
            { "external_bridge_infill_wall_overlap", external_overlap },
            { "internal_bridge_infill_wall_overlap", internal_overlap },
            { "wall_loops", 2 },
        });

        Print print;
        Test::init_and_process_print({ mesh }, print, config);

        Measure result;
        for (const Layer *layer : print.objects().front()->layers())
            for (const LayerRegion *region : layer->regions())
                for (const ExtrusionEntity *entity : region->fills.flatten().entities)
                    if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity))
                        if (path->role() == erInternalBridgeInfill) {
                            ++result.count;
                            result.length += path->length();
                        }
        for (const Layer *layer : print.objects().front()->layers())
            for (const LayerRegion *region : layer->regions()) {
                if (region->fill_no_overlap_expolygons.empty())
                    continue;
                result.internal_bridge_boundary_area += area(region->fill_internal_bridge_expolygons);
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    if (surface.surface_type == stInternalBridge)
                        result.outside_clean_area += area(diff_ex(
                            ExPolygons{ surface.expolygon }, region->fill_no_overlap_expolygons));
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    if (surface.surface_type == stInternalBridge)
                        result.outside_internal_boundary_area += area(diff_ex(
                            ExPolygons{ surface.expolygon }, region->fill_internal_bridge_expolygons));
            }
        return result;
    };

    const std::array<Test::TestMesh, 4> candidates {
        Test::TestMesh::slopy_cube,
        Test::TestMesh::sloping_hole,
        Test::TestMesh::cube_with_concave_hole,
        Test::TestMesh::cube_with_hole,
    };
    for (const char *wall_generator : { "classic", "arachne" }) {
        bool found_internal_bridge = false;
        for (const Test::TestMesh mesh : candidates) {
            const Measure internal_zero = measure(mesh, wall_generator, "0%", "0%");
            const Measure internal_full = measure(mesh, wall_generator, "0%", "100%");
            const Measure external_changed = measure(mesh, wall_generator, "100%", "0%");
            if (internal_zero.count == 0)
                continue;
            found_internal_bridge = true;
            CHECK(internal_full.internal_bridge_boundary_area != internal_zero.internal_bridge_boundary_area);
            CHECK(external_changed.internal_bridge_boundary_area == Catch::Approx(internal_zero.internal_bridge_boundary_area));
            CHECK(internal_zero.outside_clean_area == Catch::Approx(0.).margin(SCALED_EPSILON));
            CHECK(internal_zero.outside_internal_boundary_area == Catch::Approx(0.).margin(SCALED_EPSILON));
            CHECK(internal_full.outside_internal_boundary_area == Catch::Approx(0.).margin(SCALED_EPSILON));
            break;
        }
        REQUIRE(found_internal_bridge);
    }
}
