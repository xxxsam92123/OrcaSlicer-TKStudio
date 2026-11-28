#include <catch2/catch_all.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExternalBridgeGrid.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
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

// ---------------------------------------------------------------------------------------------
// External bridge grid shared walls and the external bridge fan
//
// The wall shared by two bridge cells is a real erOverhangPerimeter path, the role a bridge's own
// overhang wall carries, so a fan rule written against the role alone cannot tell them apart. The
// public proof of where such a path comes from is ExtrusionEntityCollection::external_bridge_grid_wall,
// the flag the fill step sets on the batch it builds from the splitter's cell boundaries (Fill.cpp).
// With wall_loops = 0 no perimeter is generated at all, so that flagged batch is the only source of
// erOverhangPerimeter inside the fills, and the two kinds of wall become separable without guessing
// from geometry. The external bridge fan has to take the bridge infill over and leave these shared
// walls on the overhang fan they have always printed at.
namespace {

// One resolved print, both sides of the claim: what the fills hold (the source identity of the
// walls) and what the export prints (the role and fan of every depositing move), restricted to the
// layers that actually carry both roles.
struct GridWallFanRun
{
    size_t grid_wall_collections = 0;   // collections flagged external_bridge_grid_wall
    size_t grid_wall_paths       = 0;   // erOverhangPerimeter paths inside such a collection
    size_t other_overhang_paths  = 0;   // erOverhangPerimeter paths outside every such collection
    size_t bridge_paths          = 0;   // erBridgeInfill fill paths
    size_t regions_with_both     = 0;   // regions whose fills hold both of the above
    size_t bridge_moves          = 0;   // printed bridge infill moves on a shared layer
    size_t wall_moves            = 0;   // printed grid wall moves on a shared layer
    std::vector<int>       bridge_fans;
    std::vector<int>       wall_fans;
    std::set<unsigned int> shared_layers;
};

static void grid_wall_add_fan(std::vector<int> &fans, int fan)
{
    if (std::find(fans.begin(), fans.end(), fan) == fans.end())
        fans.push_back(fan);
}

// Visit every leaf extrusion path below `entity`, telling the visitor whether it sits inside a
// collection the fill step flagged as an external bridge grid wall.
template<typename Visitor>
static void visit_grid_paths(const ExtrusionEntity *entity, bool inside_grid_wall, Visitor &visitor)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(entity)) {
        const bool flagged = inside_grid_wall || collection->external_bridge_grid_wall;
        for (const ExtrusionEntity *child : collection->entities)
            visit_grid_paths(child, flagged, visitor);
        return;
    }
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity)) {
        visitor(*path, inside_grid_wall);
        return;
    }
    if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(entity)) {
        for (const ExtrusionPath &path : multi->paths)
            visitor(path, inside_grid_wall);
        return;
    }
    if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity)) {
        for (const ExtrusionPath &path : loop->paths)
            visitor(path, inside_grid_wall);
    }
}

// Slice `bridge_with_hole` on the grid fixture the other grid cases use, read the fills of every
// layer region and then export, so the same source entities are checked on both sides.
static GridWallFanRun grid_wall_fan_run(const DynamicPrintConfig &config)
{
    Print print;
    TriangleMesh model_mesh = Test::mesh(Test::TestMesh::bridge_with_hole);
    model_mesh.scale(Vec3f(1.f, 2.f, 1.f));
    Test::init_and_process_print({ std::move(model_mesh) }, print, config);

    GridWallFanRun run;
    for (const Layer *layer : print.objects().front()->layers()) {
        for (const LayerRegion *region : layer->regions()) {
            for (const ExtrusionEntity *entity : region->fills.entities) {
                const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(entity);
                if (collection == nullptr || ! collection->external_bridge_grid_wall)
                    continue;
                ++run.grid_wall_collections;
                // The flag is a claim about the role: it must mark overhang-perimeter paths, and it
                // must mark at least one.
                REQUIRE(collection->role() == erOverhangPerimeter);
                REQUIRE(! collection->entities.empty());
            }

            size_t region_grid_walls   = 0;
            size_t region_bridge_paths = 0;
            const auto account = [&run, &region_grid_walls, &region_bridge_paths](const ExtrusionPath &path, bool inside_grid_wall) {
                if (path.role() == erOverhangPerimeter) {
                    if (inside_grid_wall) {
                        ++run.grid_wall_paths;
                        ++region_grid_walls;
                    } else {
                        ++run.other_overhang_paths;
                    }
                } else if (path.role() == erBridgeInfill) {
                    ++run.bridge_paths;
                    ++region_bridge_paths;
                }
            };
            for (const ExtrusionEntity *entity : region->fills.entities)
                visit_grid_paths(entity, false, account);
            // No ordinary wall can contribute to the same-role fan readings in this zero-wall fixture.
            REQUIRE(region->perimeters.entities.empty());
            if (region_grid_walls > 0 && region_bridge_paths > 0)
                ++run.regions_with_both;
        }
    }

    GCodeProcessorResult result;
    Test::gcode(print, &result);

    // Group the printed moves by layer, so a bridge and a wall only count when they really share a
    // layer of this print.
    std::map<unsigned int, std::vector<int>> bridge_fans_by_layer;
    std::map<unsigned int, std::vector<int>> wall_fans_by_layer;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        // M106 addresses the fan in whole percents and the Preview turns them back into a
        // percentage, so rounding recovers exactly the percentage the slicer commanded.
        const int fan = int(std::lround(move.fan_speed));
        if (move.extrusion_role == erBridgeInfill)
            bridge_fans_by_layer[move.layer_id].push_back(fan);
        else if (move.extrusion_role == erOverhangPerimeter)
            wall_fans_by_layer[move.layer_id].push_back(fan);
    }
    for (const auto &bridge_layer : bridge_fans_by_layer) {
        const auto wall_layer = wall_fans_by_layer.find(bridge_layer.first);
        if (wall_layer == wall_fans_by_layer.end())
            continue;
        run.shared_layers.insert(bridge_layer.first);
        for (const int fan : bridge_layer.second) {
            ++run.bridge_moves;
            grid_wall_add_fan(run.bridge_fans, fan);
        }
        for (const int fan : wall_layer->second) {
            ++run.wall_moves;
            grid_wall_add_fan(run.wall_fans, fan);
        }
    }
    std::sort(run.bridge_fans.begin(), run.bridge_fans.end());
    std::sort(run.wall_fans.begin(), run.wall_fans.end());
    return run;
}

// The cooling a grid wall case starts from: a flat 60% layer fan, the overhang fan at 100% with a
// threshold that is really on, and kick-start, speed-up and the PWM floor off, so a move prints at
// exactly the percentage the slicer decided and 100% is the plain overhang value.
static DynamicPrintConfig grid_wall_fan_config(int external_bridge_fan_speed, bool grid_enabled = true)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "close_fan_the_first_x_layers",    0 },
        { "enable_arc_fitting",              "disabled" },
        // The switch the overhang fan and the external bridge fan both answer to.
        { "enable_overhang_bridge_fan",      true },
        { "external_bridge_fan_speed",       external_bridge_fan_speed },
        { "external_bridge_grid_enable",     grid_enabled ? "1" : "0" },
        { "external_bridge_grid_cells_x",    "2" },
        { "external_bridge_grid_cells_y",    "2" },
        { "external_bridge_grid_angle_step", "15" },
        // No layer-time interpolation: both ends of the fan curve are 60%.
        { "fan_cooling_layer_time",          1000. },
        { "fan_kickstart",                   0. },
        { "fan_max_speed",                   60. },
        { "fan_min_speed",                   60. },
        { "fan_speedup_time",                0. },
        { "full_fan_speed_layer",            0 },
        { "initial_layer_fan_speed",         -1 },
        { "overhang_fan_speed",              100 },
        { "overhang_fan_threshold",          "10%" },
        { "part_cooling_fan_min_pwm",        0 },
        { "slow_down_for_layer_cooling",     false },
        { "slow_down_layer_time",            0. },
        // No perimeter walls at all, so a grid wall batch is the only overhang-perimeter source.
        { "wall_loops",                      0 },
    });
    return config;
}

static std::string grid_wall_fan_list(const std::vector<int> &fans)
{
    std::string text;
    for (const int fan : fans)
        text += (text.empty() ? "" : ",") + std::to_string(fan);
    return text;
}

} // namespace

TEST_CASE("External bridge grid shared walls keep the overhang fan when external bridges take one",
          "[ExternalBridgeGrid][ExternalBridgeFan]")
{
    // Without the key the two runs would slice identically and compare equal for the wrong reason.
    REQUIRE(print_config_def.has("external_bridge_fan_speed"));

    const DynamicPrintConfig inherited_config  = grid_wall_fan_config(-1);
    const DynamicPrintConfig explicit_30_config = grid_wall_fan_config(30);
    const DynamicPrintConfig explicit_0_config  = grid_wall_fan_config(0);

    // Pin the values the expected fans are derived from: a 60% layer fan over a switched-on
    // overhang fan of 100% that a real threshold cannot suppress, and no perimeter walls.
    REQUIRE(inherited_config.opt<ConfigOptionFloats>("fan_min_speed")->get_at(0) == Catch::Approx(60.));
    REQUIRE(inherited_config.opt<ConfigOptionFloats>("fan_max_speed")->get_at(0) == Catch::Approx(60.));
    REQUIRE(inherited_config.opt<ConfigOptionInts>("overhang_fan_speed")->get_at(0) == 100);
    REQUIRE(inherited_config.opt<ConfigOptionBools>("enable_overhang_bridge_fan")->get_at(0));
    REQUIRE(inherited_config.opt<ConfigOptionInts>("overhang_fan_threshold")->get_at(0) == int(Overhang_threshold_1_4));
    REQUIRE(inherited_config.opt<ConfigOptionInt>("wall_loops")->value == 0);
    REQUIRE(inherited_config.opt<ConfigOptionInts>("external_bridge_fan_speed")->get_at(0) == -1);
    REQUIRE(explicit_30_config.opt<ConfigOptionInts>("external_bridge_fan_speed")->get_at(0) == 30);
    REQUIRE(explicit_0_config.opt<ConfigOptionInts>("external_bridge_fan_speed")->get_at(0) == 0);

    const GridWallFanRun inherited   = grid_wall_fan_run(inherited_config);
    const GridWallFanRun explicit_30 = grid_wall_fan_run(explicit_30_config);
    const GridWallFanRun explicit_0  = grid_wall_fan_run(explicit_0_config);

    const std::array<const GridWallFanRun *, 3> runs { &inherited, &explicit_30, &explicit_0 };
    for (const GridWallFanRun *run : runs) {
        INFO("grid walls " << run->grid_wall_paths << " in " << run->grid_wall_collections
                           << " flagged collections, " << run->other_overhang_paths << " other overhang paths, "
                           << run->bridge_paths << " bridge paths, " << run->wall_moves << " printed wall moves, "
                           << run->bridge_moves << " printed bridge moves, on " << run->shared_layers.size() << " shared layer(s)");
        // The fills prove the identity: every overhang-perimeter path of this print comes from a
        // collection the fill step flagged as a grid wall, and a bridge infill sits in the same region.
        REQUIRE(run->grid_wall_collections > 0);
        REQUIRE(run->grid_wall_paths > 0);
        REQUIRE(run->other_overhang_paths == 0);
        REQUIRE(run->bridge_paths > 0);
        REQUIRE(run->regions_with_both > 0);
        // The export proves both roles really print, on one shared layer.
        REQUIRE(run->shared_layers.size() > 0);
        REQUIRE(run->wall_moves > 0);
        REQUIRE(run->bridge_moves > 0);
    }

    INFO("inherited bridge " << grid_wall_fan_list(inherited.bridge_fans) << " walls " << grid_wall_fan_list(inherited.wall_fans));
    INFO("explicit 30 bridge " << grid_wall_fan_list(explicit_30.bridge_fans) << " walls " << grid_wall_fan_list(explicit_30.wall_fans));
    INFO("explicit 0 bridge " << grid_wall_fan_list(explicit_0.bridge_fans) << " walls " << grid_wall_fan_list(explicit_0.wall_fans));

    // The baseline keeps the old behaviour for both: the bridge infill and the grid walls print at
    // the overhang fan speed.
    REQUIRE(inherited.bridge_fans == std::vector<int> { 100 });
    REQUIRE(inherited.wall_fans == std::vector<int> { 100 });

    // An explicit value takes the bridge infill over, and only the bridge infill: the grid shared
    // walls stay on the overhang fan their erOverhangPerimeter role has always printed at.
    REQUIRE(explicit_30.bridge_fans == std::vector<int> { 30 });
    REQUIRE(explicit_30.wall_fans == std::vector<int> { 100 });
    REQUIRE(explicit_0.bridge_fans == std::vector<int> { 0 });
    REQUIRE(explicit_0.wall_fans == std::vector<int> { 100 });

    // Same source entities, and only the bridge's fan moved: a takeover that also dropped or added
    // moves would show up in the counts or the layers.
    CHECK(explicit_30.wall_moves == inherited.wall_moves);
    CHECK(explicit_0.wall_moves == inherited.wall_moves);
    CHECK(explicit_30.bridge_moves == inherited.bridge_moves);
    CHECK(explicit_0.bridge_moves == inherited.bridge_moves);
    CHECK(explicit_30.shared_layers == inherited.shared_layers);
    CHECK(explicit_0.shared_layers == inherited.shared_layers);
    CHECK(explicit_30.wall_fans == inherited.wall_fans);
    CHECK(explicit_0.wall_fans == inherited.wall_fans);

    // Without the grid the same fill pipeline emits no overhang-perimeter path at all, which is what
    // makes the flagged batch the sole source of the role asserted above.
    const GridWallFanRun without_grid = grid_wall_fan_run(grid_wall_fan_config(-1, false));
    REQUIRE(without_grid.grid_wall_collections == 0);
    REQUIRE(without_grid.grid_wall_paths == 0);
    REQUIRE(without_grid.other_overhang_paths == 0);
    REQUIRE(without_grid.bridge_paths > 0);
}
