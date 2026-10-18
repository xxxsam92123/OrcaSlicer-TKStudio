#include "ExternalBridgeGrid.hpp"

#include "BoundingBox.hpp"
#include "ClipperUtils.hpp"
#include "Geometry.hpp"
#include "libslic3r.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace Slic3r {

static Polylines make_external_bridge_grid_walls(const Surfaces &cells);

namespace {

constexpr int    MAX_GRID_AXIS        = 32;
constexpr int    MAX_GRID_CELLS       = 1024;
constexpr size_t MAX_GRID_FRAGMENTS   = 32;
constexpr double MIN_CELL_MM          = 8.;
constexpr double MIN_SURFACE_AREA_MM2 = 100.;

// Empirical budget for the area a grid partition may lose: the tolerance is the outline length
// times this value. The cells tile the bounding box exactly, so a residual can only come from
// the outline being re-snapped inside each cell by Clipper2's Clipper64, which rounds the
// intersection coordinates of every cut with nearbyint() - at most half an integer unit per
// axis. That bound is per coordinate, not on the area: the rounding has no inward bias, so a cut
// can as well lose area as gain it (on the 39099 mm2 bridge below the individual cut losses run
// from -69552 to +145204936 units2), and the resulting error tracks the outline length, not the
// area. The linear relation is measured, not derived - which is why this is a budget rather than
// a geometric upper bound, and why the check around it uses abs(). Measured on that reported
// bridge (790 mm outline, no holes): 0.1475 units2 of residual per outline unit at the accepted
// 8x8 grid, and 0.1700 at 7x8, the highest of the seven candidates tried for the model
// (E:/orca/ebg-evidence/followup-diag.log). This factor of 4 therefore sits about 23.5x above
// the worst candidate measured on it: a margin over the candidates actually measured, not a
// guarantee for every surface. A partition above the budget is rejected and the caller tries the
// next coarser candidate, falling back to the whole surface when they all fail, so an over-tight
// budget costs resolution, never geometry.
constexpr double AREA_TOLERANCE_PER_OUTLINE_UNIT = 4.;

struct GridCandidate
{
    int cells_x;
    int cells_y;
};

Surfaces full_surface(const Surface &surface)
{
    return Surfaces { surface };
}

std::vector<GridCandidate> make_grid_candidates(const BoundingBox &bbox,
                                                const ExternalBridgeGridSettings &settings)
{
    const int requested_x = std::clamp(settings.cells_x, 1, MAX_GRID_AXIS);
    const int requested_y = std::clamp(settings.cells_y, 1, MAX_GRID_AXIS);
    const double width_mm = unscaled<double>(bbox.size().x());
    const double height_mm = unscaled<double>(bbox.size().y());
    const int max_x = std::min(requested_x, int(std::floor(width_mm / MIN_CELL_MM + 1e-9)));
    const int max_y = std::min(requested_y, int(std::floor(height_mm / MIN_CELL_MM + 1e-9)));

    std::vector<GridCandidate> candidates;
    for (int cells_y = 1; cells_y <= max_y; ++cells_y)
        for (int cells_x = 1; cells_x <= max_x; ++cells_x)
            if (cells_x * cells_y <= MAX_GRID_CELLS)
                candidates.push_back({ cells_x, cells_y });

    std::sort(candidates.begin(), candidates.end(), [width_mm, height_mm](const GridCandidate &lhs, const GridCandidate &rhs) {
        const int lhs_count = lhs.cells_x * lhs.cells_y;
        const int rhs_count = rhs.cells_x * rhs.cells_y;
        if (lhs_count != rhs_count)
            return lhs_count > rhs_count;

        const double lhs_cell_aspect = width_mm * lhs.cells_y / (height_mm * lhs.cells_x);
        const double rhs_cell_aspect = width_mm * rhs.cells_y / (height_mm * rhs.cells_x);
        const double lhs_aspect_error = std::abs(std::log(lhs_cell_aspect));
        const double rhs_aspect_error = std::abs(std::log(rhs_cell_aspect));
        if (lhs_aspect_error != rhs_aspect_error)
            return lhs_aspect_error < rhs_aspect_error;

        return lhs.cells_x > rhs.cells_x;
    });
    return candidates;
}

std::optional<Surfaces> split_for_grid(const Surface &surface, const BoundingBox &bbox,
                                       int cells_x, int cells_y, double angle_step)
{
    const coord_t bbox_width = bbox.size().x();
    const coord_t bbox_height = bbox.size().y();

    Surfaces result;
    size_t fragment_count = 0;
    for (int y = 0; y < cells_y; ++y) {
        for (int x = 0; x < cells_x; ++x) {
            const coord_t x0 = bbox.min.x() + coord_t(std::floor(coordf_t(bbox_width) * x / cells_x));
            const coord_t x1 = bbox.min.x() + coord_t(std::floor(coordf_t(bbox_width) * (x + 1) / cells_x));
            const coord_t y0 = bbox.min.y() + coord_t(std::floor(coordf_t(bbox_height) * y / cells_y));
            const coord_t y1 = bbox.min.y() + coord_t(std::floor(coordf_t(bbox_height) * (y + 1) / cells_y));
            if (x1 <= x0 || y1 <= y0 ||
                unscaled<double>(x1 - x0) < MIN_CELL_MM || unscaled<double>(y1 - y0) < MIN_CELL_MM)
                return std::nullopt;

            const Polygon cell { Points {
                Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1)
            } };
            ExPolygons clipped = intersection_ex(surface.expolygon, Polygons { cell });
            if (clipped.size() > 1)
                fragment_count += clipped.size() - 1;
            if (fragment_count > MAX_GRID_FRAGMENTS)
                return std::nullopt;

            const double cell_angle = surface.bridge_angle + (((x + y) & 1) ? angle_step : -angle_step);
            for (ExPolygon &expolygon : clipped)
                if (!expolygon.empty()) {
                    Surface split_surface(surface, std::move(expolygon));
                    split_surface.bridge_angle = cell_angle;
                    split_surface.external_bridge_grid = true;
                    result.emplace_back(std::move(split_surface));
                }
        }
    }

    if (result.size() <= 1)
        return std::nullopt;

    double result_area = 0.;
    for (const Surface &split_surface : result)
        result_area += split_surface.area();
    // The budget covers the whole outline - the outer contour and every hole - because every cut
    // re-snaps whichever boundary it crosses.
    double outline_length = surface.expolygon.contour.length();
    for (const Polygon &hole : surface.expolygon.holes)
        outline_length += hole.length();
    const double area_tolerance = std::max(1., outline_length * AREA_TOLERANCE_PER_OUTLINE_UNIT);
    if (std::abs(result_area - surface.area()) > area_tolerance)
        return std::nullopt;

    // Keep the exact shared boundaries from the pre-grouping cell geometry.
    // group_fills() later applies safety offsets and overlap clipping, which can
    // otherwise erase or shorten these boundaries.
    auto walls = std::make_shared<Polylines>(make_external_bridge_grid_walls(result));
    auto area = std::make_shared<ExPolygon>(surface.expolygon);
    for (Surface &cell : result)
        cell.external_bridge_grid_walls = walls;
    for (Surface &cell : result)
        cell.external_bridge_grid_area = area;

    return result;
}

} // namespace

Surfaces split_external_bridge_surface(const Surface &surface,
                                       const ExternalBridgeGridSettings &settings)
{
    if (!settings.enabled || surface.empty() || surface.surface_type != stBottomBridge ||
        !std::isfinite(settings.angle_step_deg) || settings.angle_step_deg == 0. ||
        surface.area() * SCALING_FACTOR * SCALING_FACTOR < MIN_SURFACE_AREA_MM2)
        return full_surface(surface);

    const BoundingBox bbox = get_extents(surface.expolygon);
    if (!bbox.defined || bbox.size().x() <= 0 || bbox.size().y() <= 0)
        return full_surface(surface);

    const double angle_step = Geometry::deg2rad(settings.angle_step_deg);
    for (const GridCandidate &candidate : make_grid_candidates(bbox, settings)) {
        if (candidate.cells_x == 1 && candidate.cells_y == 1)
            continue;
        if (std::optional<Surfaces> result = split_for_grid(
                surface, bbox, candidate.cells_x, candidate.cells_y, angle_step))
            return std::move(*result);
    }
    return full_surface(surface);
}

static Polylines make_external_bridge_grid_walls(const Surfaces &cells)
{
    struct Edge {
        Point first;
        Point second;
        size_t cell_index;
    };
    std::vector<Edge> edges;
    for (size_t cell_index = 0; cell_index < cells.size(); ++cell_index) {
        const Surface &cell = cells[cell_index];
        if (!cell.external_bridge_grid)
            continue;
        const Points &points = cell.expolygon.contour.points;
        for (size_t i = 0; i < points.size(); ++i) {
            edges.push_back({ points[i], points[(i + 1) % points.size()], cell_index });
        }
    }

    constexpr long double PARALLEL_EPSILON = 1e-12L;
    constexpr coord_t    ENDPOINT_TOLERANCE = 2;
    constexpr coord_t    MIN_SHARED_LENGTH = 2;

    const auto length = [](const Point &first, const Point &second) {
        const long double dx = static_cast<long double>(second.x()) - static_cast<long double>(first.x());
        const long double dy = static_cast<long double>(second.y()) - static_cast<long double>(first.y());
        return std::hypotl(dx, dy);
    };
    const auto cross = [](long double ax, long double ay, long double bx, long double by) {
        return ax * by - ay * bx;
    };
    const auto same_line = [&](const Point &first, const Point &second,
                               const Point &candidate_first, const Point &candidate_second) {
        const long double dx = static_cast<long double>(second.x()) - static_cast<long double>(first.x());
        const long double dy = static_cast<long double>(second.y()) - static_cast<long double>(first.y());
        const long double candidate_dx = static_cast<long double>(candidate_second.x()) - static_cast<long double>(candidate_first.x());
        const long double candidate_dy = static_cast<long double>(candidate_second.y()) - static_cast<long double>(candidate_first.y());
        const long double first_length = std::hypotl(dx, dy);
        const long double candidate_length = std::hypotl(candidate_dx, candidate_dy);
        if (first_length == 0. || candidate_length == 0. ||
            std::abs(cross(dx, dy, candidate_dx, candidate_dy)) >
                PARALLEL_EPSILON * first_length * candidate_length)
            return false;
        return std::abs(cross(dx, dy,
                              static_cast<long double>(candidate_first.x()) - static_cast<long double>(first.x()),
                              static_cast<long double>(candidate_first.y()) - static_cast<long double>(first.y()))) /
                   first_length <= static_cast<long double>(ENDPOINT_TOLERANCE) &&
               std::abs(cross(dx, dy,
                              static_cast<long double>(candidate_second.x()) - static_cast<long double>(first.x()),
                              static_cast<long double>(candidate_second.y()) - static_cast<long double>(first.y()))) /
                   first_length <= static_cast<long double>(ENDPOINT_TOLERANCE);
    };
    const auto projection = [](const Point &origin, const Point &direction, const Point &point) {
        const long double dx = static_cast<long double>(direction.x()) - static_cast<long double>(origin.x());
        const long double dy = static_cast<long double>(direction.y()) - static_cast<long double>(origin.y());
        const long double length = std::hypotl(dx, dy);
        return ((static_cast<long double>(point.x()) - static_cast<long double>(origin.x())) * dx +
                (static_cast<long double>(point.y()) - static_cast<long double>(origin.y())) * dy) / length;
    };
    const auto point_at = [](const Point &first, const Point &second, long double distance) {
        const long double dx = static_cast<long double>(second.x()) - static_cast<long double>(first.x());
        const long double dy = static_cast<long double>(second.y()) - static_cast<long double>(first.y());
        const long double segment_length = std::hypotl(dx, dy);
        return Point(coord_t(std::llround(static_cast<long double>(first.x()) + distance * dx / segment_length)),
                     coord_t(std::llround(static_cast<long double>(first.y()) + distance * dy / segment_length)));
    };

    // Compare every pair of edges from different cells. Clipping can split one
    // side of a shared grid boundary while leaving the neighboring side whole,
    // so exact endpoint equality is insufficient here.
    std::vector<Edge> shared_segments;
    for (size_t lhs = 0; lhs < edges.size(); ++lhs) {
        const Edge &first = edges[lhs];
        const long double first_length = length(first.first, first.second);
        if (first_length < static_cast<long double>(MIN_SHARED_LENGTH))
            continue;
        for (size_t rhs = lhs + 1; rhs < edges.size(); ++rhs) {
            const Edge &second = edges[rhs];
            if (first.cell_index == second.cell_index ||
                !same_line(first.first, first.second, second.first, second.second))
                continue;
            const long double first_begin = 0.;
            const long double first_end = first_length;
            const long double second_begin = projection(first.first, first.second, second.first);
            const long double second_end = projection(first.first, first.second, second.second);
            const long double overlap_begin = std::max(first_begin, std::min(second_begin, second_end));
            const long double overlap_end = std::min(first_end, std::max(second_begin, second_end));
            if (overlap_end - overlap_begin < static_cast<long double>(MIN_SHARED_LENGTH))
                continue;
            shared_segments.push_back({ point_at(first.first, first.second, overlap_begin),
                                        point_at(first.first, first.second, overlap_end),
                                        first.cell_index });
        }
    }

    // Merge overlapping or endpoint-adjacent fragments on the same straight
    // boundary using a fixed line origin and scalar interval. Crossings of
    // horizontal and vertical boundaries remain separate.
    struct MergedSegment {
        Point origin;
        Point direction;
        long double begin;
        long double end;
    };
    std::vector<MergedSegment> merged;
    for (const Edge &segment : shared_segments) {
        const long double segment_length = length(segment.first, segment.second);
        if (segment_length < static_cast<long double>(MIN_SHARED_LENGTH))
            continue;

        bool merged_segment = false;
        for (MergedSegment &existing : merged) {
            if (!same_line(existing.origin, existing.direction, segment.first, segment.second))
                continue;
            const long double candidate_begin = projection(existing.origin, existing.direction, segment.first);
            const long double candidate_end = projection(existing.origin, existing.direction, segment.second);
            const long double begin = std::min(candidate_begin, candidate_end);
            const long double end = std::max(candidate_begin, candidate_end);
            if (begin > existing.end + static_cast<long double>(ENDPOINT_TOLERANCE) ||
                end < existing.begin - static_cast<long double>(ENDPOINT_TOLERANCE))
                continue;
            existing.begin = std::min(existing.begin, begin);
            existing.end = std::max(existing.end, end);
            merged_segment = true;
            break;
        }
        if (!merged_segment)
            merged.push_back({ segment.first, segment.second, 0., segment_length });
    }

    Polylines result;
    result.reserve(merged.size());
    for (const MergedSegment &segment : merged) {
        const Point first = point_at(segment.origin, segment.direction, segment.begin);
        const Point second = point_at(segment.origin, segment.direction, segment.end);
        if (first != second)
            result.emplace_back(Polyline(Points { first, second }));
    }

    return result;
}

Polylines external_bridge_grid_walls(const Surfaces &cells)
{
    for (const Surface &cell : cells)
        if (cell.external_bridge_grid_walls)
            return *cell.external_bridge_grid_walls;
    return {};
}

ExPolygons external_bridge_grid_infill_area(
    const Surface &cell, double wall_overlap, double line_width)
{
    wall_overlap = std::clamp(wall_overlap, 0., 1.);
    if (!cell.external_bridge_grid || !cell.external_bridge_grid_area ||
        wall_overlap == 0. || line_width <= 0.)
        return ExPolygons { cell.expolygon };

    return intersection_ex(
        offset_ex(cell.expolygon, scale_(wall_overlap * line_width)),
        ExPolygons { *cell.external_bridge_grid_area });
}

} // namespace Slic3r
