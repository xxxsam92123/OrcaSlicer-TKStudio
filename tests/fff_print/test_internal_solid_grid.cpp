#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExternalBridgeGrid.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Fill/FillInternalSolidGrid.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/InternalSolidGrid.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Surface.hpp"

#include "test_helpers.hpp"

using namespace Slic3r;

namespace {

ExPolygon make_rectangle(double width, double height)
{
    return ExPolygon(Points {
        Point::new_scale(0., 0.),
        Point::new_scale(width, 0.),
        Point::new_scale(width, height),
        Point::new_scale(0., height)
    });
}

double total_area(const Surfaces &surfaces)
{
    double result = 0.;
    for (const Surface &surface : surfaces)
        result += surface.area();
    return result;
}

void require_partition(const Surface &source, const Surfaces &parts, size_t expected_parts)
{
    REQUIRE(parts.size() == expected_parts);
    REQUIRE_THAT(total_area(parts), Catch::Matchers::WithinAbs(source.area(), 1e-6));
    REQUIRE_THAT(area(union_ex(to_expolygons(parts))), Catch::Matchers::WithinAbs(source.area(), 1e-6));
    for (size_t lhs = 0; lhs < parts.size(); ++lhs)
        for (size_t rhs = lhs + 1; rhs < parts.size(); ++rhs)
            REQUIRE_THAT(area(intersection_ex(parts[lhs].expolygon, parts[rhs].expolygon)),
                         Catch::Matchers::WithinAbs(0., 1e-6));
}

ExPolygon make_concave_l()
{
    return ExPolygon(Points {
        Point::new_scale(0., 0.),
        Point::new_scale(64., 0.),
        Point::new_scale(64., 16.),
        Point::new_scale(16., 16.),
        Point::new_scale(16., 64.),
        Point::new_scale(0., 64.)
    });
}

void require_grid_paths_and_walls(TriangleMesh mesh)
{
    Print print;
    Test::init_and_process_print({ std::move(mesh) }, print,
                                  {{ "wall_loops", "0" },
                                   { "sparse_infill_density", "15%" },
                                   { "internal_solid_infill_pattern", "internal_solid_grid" },
                                   { "internal_solid_grid_cells_x", "2" },
                                   { "internal_solid_grid_cells_y", "2" },
                                   { "internal_solid_grid_angle_step", "15" },
                                   // Compatibility data must not act as a runtime wall switch.
                                   { "internal_solid_grid_walls", "0" },
                                   { "inner_wall_line_width", "0.31" },
                                   { "internal_solid_infill_line_width", "0.42" },
                                   { "top_surface_pattern", "rectilinear" },
                                   { "bottom_surface_pattern", "rectilinear" },
                                   { "layer_height", 0.2 }});

    size_t wall_paths = 0;
    size_t solid_paths = 0;
    std::vector<const ExtrusionPath *> grid_walls;
    for (const Layer *layer : print.objects().front()->layers()) {
        for (const LayerRegion *region : layer->regions()) {
            for (const ExtrusionEntity *entity : region->fills.entities) {
                const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(entity);
                REQUIRE(collection != nullptr);
                if (collection->internal_solid_infill_wall) {
                    REQUIRE(collection->no_sort);
                    for (const ExtrusionEntity *wall : collection->entities) {
                        const auto *path = dynamic_cast<const ExtrusionPath *>(wall);
                        REQUIRE(path != nullptr);
                        REQUIRE(path->role() == erPerimeter);
                        REQUIRE_THAT(path->width, Catch::Matchers::WithinAbs(0.31f, 1e-4f));
                        REQUIRE(path->width < 1.f);
                        grid_walls.push_back(path);
                        ++wall_paths;
                    }
                }
            }
            for (const ExtrusionEntity *entity : region->fills.flatten().entities)
                if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity);
                    path != nullptr && path->role() == erSolidInfill) {
                    REQUIRE(path->width < 1.f);
                    ++solid_paths;
                }
        }
    }

    REQUIRE(wall_paths > 0);
    REQUIRE_FALSE(grid_walls.empty());
    REQUIRE(solid_paths > 0);
}

} // namespace

TEST_CASE("Internal solid grid cell angle advances one step per cell and folds at 180 degrees", "[InternalSolidGrid]")
{
    using Catch::Matchers::WithinAbs;
    const float step = float(Geometry::deg2rad(15.));
    const float base = float(Geometry::deg2rad(0.));

    // step == 0 keeps every cell on the base angle.
    REQUIRE_THAT(detail::cell_angle(base, 0, 4, 0.f), WithinAbs(base, 1e-6));
    REQUIRE_THAT(detail::cell_angle(base, 7, 4, 0.f), WithinAbs(base, 1e-6));

    // 4x4 cells: index = y * cells_x + x, offset = (x + y) % 12 for a 15 degree step.
    // The checkerboard +/-step implementation returns base - step for index 1 and index 6,
    // so these assertions fail if it is reverted.
    REQUIRE_THAT(detail::cell_angle(base, 0, 4, step), WithinAbs(base + 0.f * step, 1e-5));  // (0,0)
    REQUIRE_THAT(detail::cell_angle(base, 1, 4, step), WithinAbs(base + 1.f * step, 1e-5));  // (1,0)
    REQUIRE_THAT(detail::cell_angle(base, 5, 4, step), WithinAbs(base + 2.f * step, 1e-5));  // (1,1)
    REQUIRE_THAT(detail::cell_angle(base, 6, 4, step), WithinAbs(base + 3.f * step, 1e-5));  // (2,1)

    // Fold-back: a 15 degree step gives 12 slots, so (x + y) == 12 returns the base angle again.
    REQUIRE_THAT(detail::cell_angle(base, 12, 16, step), WithinAbs(base, 1e-5));             // (12,0)
    // A 30 degree step gives 6 slots, so (x + y) == 6 folds back as well.
    REQUIRE_THAT(detail::cell_angle(base, 6, 16, float(Geometry::deg2rad(30.))), WithinAbs(base, 1e-5));
}

TEST_CASE("Internal solid grid partitions regular, concave, and holed surfaces", "[InternalSolidGrid]")
{
    InternalSolidGridSettings settings { 4, 2 };
    Surface rectangle(stInternalSolid, make_rectangle(64., 32.));
    const Surfaces rectangle_cells = split_internal_solid_grid_surface(rectangle, settings);
    require_partition(rectangle, rectangle_cells, 8);
    REQUIRE(std::all_of(rectangle_cells.begin(), rectangle_cells.end(), [](const Surface &cell) {
        return cell.internal_solid_grid;
    }));

    Surface concave(stInternalSolid, make_concave_l());
    const Surfaces concave_cells = split_internal_solid_grid_surface(concave, { 4, 4 });
    require_partition(concave, concave_cells, 7);

    ExPolygon holed = make_rectangle(64., 32.);
    const ExPolygon hole = make_rectangle(4., 4.);
    Polygon translated_hole = hole.contour;
    translated_hole.translate(scale_(6.), scale_(6.));
    translated_hole.reverse();
    const ExPolygon hole_region(translated_hole);
    holed.holes.emplace_back(std::move(translated_hole));
    Surface holed_surface(stInternalSolid, std::move(holed));
    const Surfaces holed_cells = split_internal_solid_grid_surface(holed_surface, settings);
    require_partition(holed_surface, holed_cells, 8);
    REQUIRE(std::any_of(holed_cells.begin(), holed_cells.end(), [](const Surface &cell) {
        return !cell.expolygon.holes.empty();
    }));
    for (const Surface &cell : holed_cells)
        REQUIRE_THAT(area(intersection_ex(cell.expolygon, hole_region)), Catch::Matchers::WithinAbs(0., 1e-6));
}

TEST_CASE("Internal solid grid keeps complex cell fragments instead of falling back", "[InternalSolidGrid]")
{
    constexpr double body_width = 64.;
    constexpr double body_height = 16.;
    constexpr double tooth_depth = 16.;
    constexpr double bridge_height = 0.05;
    constexpr double tooth_pitch = 0.4;
    constexpr double tooth_width = 0.2;
    constexpr int tooth_count = 40;
    // Build a valid simple ExPolygon through the production boolean union:
    // a rectangular upper body plus 40 separated teeth below its baseline.
    Polygons components;
    components.emplace_back(Points {
        Point::new_scale(0., tooth_depth),
        Point::new_scale(body_width, tooth_depth),
        Point::new_scale(body_width, body_height + tooth_depth),
        Point::new_scale(0., body_height + tooth_depth)
    });
    for (int tooth = 0; tooth < tooth_count; ++tooth) {
        const double left = tooth * tooth_pitch;
        const double right = left + tooth_width;
        components.emplace_back(Points {
            Point::new_scale(left, 0.),
            Point::new_scale(right, 0.),
            Point::new_scale(right, tooth_depth + bridge_height),
            Point::new_scale(left, tooth_depth + bridge_height)
        });
    }
    const ExPolygons comb_regions = union_ex(components);
    REQUIRE(comb_regions.size() == 1);
    Surface comb_surface(stInternalSolid, comb_regions.front());
    const Surfaces cells = split_internal_solid_grid_surface(comb_surface, { 4, 2 });

    // The lower-left cell contains 40 disconnected teeth; the remaining four
    // occupied cells are ordinary one-fragment partitions. This used to trigger
    // an atomic fallback for the entire surface.
    require_partition(comb_surface, cells, 44);
    REQUIRE(std::count_if(cells.begin(), cells.end(), [](const Surface &cell) {
        return cell.internal_solid_grid_index == 0;
    }) == tooth_count);
}

TEST_CASE("Internal solid grid configuration and end-to-end walls remain active", "[InternalSolidGrid]")
{
    const DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    REQUIRE(print_config_def.get("internal_solid_infill_pattern")->has_enum_value("internal_solid_grid"));
    REQUIRE(config.opt_int("internal_solid_grid_cells_x") == 2);
    REQUIRE(config.opt_int("internal_solid_grid_cells_y") == 2);
    REQUIRE_THAT(config.opt_float("internal_solid_grid_angle_step"), Catch::Matchers::WithinAbs(15., 1e-6));
    REQUIRE(config.opt_bool("internal_solid_grid_walls"));
    REQUIRE_FALSE(config.opt_bool("external_bridge_grid_enable"));
    REQUIRE(config.opt_int("external_bridge_grid_cells_x") == 2);
    REQUIRE(config.opt_int("external_bridge_grid_cells_y") == 2);
    REQUIRE_THAT(config.opt_float("external_bridge_grid_angle_step"), Catch::Matchers::WithinAbs(15., 1e-6));
    REQUIRE(std::unique_ptr<Fill>(Fill::new_from_type(ipInternalSolidGrid)) != nullptr);

    require_grid_paths_and_walls(Test::cube(30));
    require_grid_paths_and_walls(Test::mesh(Test::TestMesh::cube_with_concave_hole));
}

TEST_CASE("External bridge grid preserves geometry and alternates bridge directions", "[ExternalBridgeGrid]")
{
    Surface bridge(stBottomBridge, make_rectangle(64., 32.));
    bridge.bridge_angle = Geometry::deg2rad(45.);

    ExternalBridgeGridSettings enabled { true, 4, 2, 15. };
    const Surfaces cells = split_external_bridge_surface(bridge, enabled);
    require_partition(bridge, cells, 8);
    const Polylines grid_walls = external_bridge_grid_walls(cells);
    double grid_wall_length = 0.;
    for (const Polyline &wall : grid_walls)
        grid_wall_length += unscaled<double>(wall.length());
    // 4x2 cells have three vertical and one horizontal internal grid lines.
    REQUIRE(grid_walls.size() == 4);
    REQUIRE(std::count_if(grid_walls.begin(), grid_walls.end(), [](const Polyline &wall) {
        return std::abs(unscaled<double>(wall.length()) - 32.) < 1e-6;
    }) == 3);
    REQUIRE(std::count_if(grid_walls.begin(), grid_walls.end(), [](const Polyline &wall) {
        return std::abs(unscaled<double>(wall.length()) - 64.) < 1e-6;
    }) == 1);
    REQUIRE_THAT(grid_wall_length, Catch::Matchers::WithinAbs(160., 1e-6));

    Surface dense_bridge(stBottomBridge, make_rectangle(128., 128.));
    dense_bridge.bridge_angle = Geometry::deg2rad(45.);
    const Surfaces dense_cells = split_external_bridge_surface(dense_bridge, { true, 16, 16, 15. });
    require_partition(dense_bridge, dense_cells, 256);
    double dense_wall_length = 0.;
    const Polylines dense_walls = external_bridge_grid_walls(dense_cells);
    for (const Polyline &wall : dense_walls)
        dense_wall_length += unscaled<double>(wall.length());
    // A 16x16 grid has 15 full-height and 15 full-width shared boundaries.
    REQUIRE(dense_walls.size() == 30);
    REQUIRE(std::all_of(dense_walls.begin(), dense_walls.end(), [](const Polyline &wall) {
        return std::abs(unscaled<double>(wall.length()) - 128.) < 1e-6;
    }));
    REQUIRE_THAT(dense_wall_length, Catch::Matchers::WithinAbs(3840., 1e-6));
    REQUIRE(std::all_of(cells.begin(), cells.end(), [](const Surface &cell) {
        return cell.external_bridge_grid;
    }));
    REQUIRE_THAT(cells.front().bridge_angle, Catch::Matchers::WithinAbs(Geometry::deg2rad(30.), 1e-6));
    REQUIRE_THAT(cells[1].bridge_angle, Catch::Matchers::WithinAbs(Geometry::deg2rad(60.), 1e-6));

    ExternalBridgeGridSettings disabled { false, 4, 2, 15. };
    const Surfaces unsplit = split_external_bridge_surface(bridge, disabled);
    require_partition(bridge, unsplit, 1);
    REQUIRE_FALSE(unsplit.front().external_bridge_grid);
}

TEST_CASE("External bridge grid emits overhang walls before bridge infill", "[ExternalBridgeGrid]")
{
    Print print;
    Test::init_and_process_print({ Test::mesh(Test::TestMesh::bridge) }, print,
                                 {{ "wall_loops", "0" },
                                  { "sparse_infill_density", "15%" },
                                  { "external_bridge_grid_enable", "1" },
                                  { "external_bridge_grid_cells_x", "2" },
                                  { "external_bridge_grid_cells_y", "2" },
                                  { "external_bridge_grid_angle_step", "15" },
                                  { "bridge_line_width", "0.37" },
                                  { "bridge_flow", "0.73" },
                                  { "layer_height", 0.2 }});

    size_t first_wall = SIZE_MAX;
    size_t first_bridge = SIZE_MAX;
    const Flow expected_wall_flow = print.objects().front()->get_layer(0)->regions().front()->bridging_flow(frPerimeter, false);
    for (const Layer *layer : print.objects().front()->layers())
        for (const LayerRegion *region : layer->regions())
            for (size_t index = 0; index < region->fills.entities.size(); ++index) {
                const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(region->fills.entities[index]);
                REQUIRE(collection != nullptr);
                for (const ExtrusionEntity *entity : collection->entities) {
                    const auto *path = dynamic_cast<const ExtrusionPath *>(entity);
                    if (path == nullptr)
                        continue;
                    if (path->role() == erOverhangPerimeter)
                    {
                        first_wall = std::min(first_wall, index);
                        REQUIRE_THAT(path->width, Catch::Matchers::WithinAbs(expected_wall_flow.width(), 1e-4f));
                        REQUIRE_THAT(path->mm3_per_mm, Catch::Matchers::WithinAbs(expected_wall_flow.mm3_per_mm(), 1e-4f));
                    }
                    else if (path->role() == erBridgeInfill)
                        first_bridge = std::min(first_bridge, index);
                }
            }

    REQUIRE(first_wall != SIZE_MAX);
    REQUIRE(first_bridge != SIZE_MAX);
    REQUIRE(first_wall < first_bridge);
}
