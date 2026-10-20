#include "FillInternalSolidGrid.hpp"

#include "../InternalSolidGrid.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <math.h>

namespace Slic3r {
namespace detail {

float cell_angle(float angle, unsigned short cell_index, int cells_x, float angle_step)
{
    cells_x = std::max(cells_x, 1);
    if (angle_step <= 0.f)
        return angle;
    const int cell_x = cell_index % cells_x;
    const int cell_y = cell_index / cells_x;
    // Adjacent cells along either axis differ by exactly one step. The sequence folds back
    // every 180 degrees (the same direction), so a 4x4 grid spans 7 directions instead of the
    // two a checkerboard +/-step produces. A zero step keeps every cell aligned.
    // angle and angle_step are radians here (Fill.cpp converts the user-facing degrees with
    // Geometry::deg2rad), so the 180-degree fold-back must use M_PI, not 180.0.
    const int slots  = std::max(1, int(std::lround(M_PI / double(angle_step))));
    const int offset = (cell_x + cell_y) % slots;
    return angle + angle_step * float(offset);
}

} // namespace

Polylines FillInternalSolidGrid::fill_surface(const Surface *surface, const FillParams &params)
{
    if (surface == nullptr || surface->surface_type != stInternalSolid ||
        params.pattern != ipInternalSolidGrid || params.extrusion_role != erSolidInfill)
        return FillRectilinear::fill_surface(surface, params);

    struct RestoreAngle {
        float &angle;
        float original_angle;
        ~RestoreAngle() { angle = original_angle; }
    } restore_angle { angle, angle };

    const InternalSolidGridSettings settings { params.internal_solid_grid_cells_x, params.internal_solid_grid_cells_y };
    if (surface->internal_solid_grid) {
        angle = detail::cell_angle(restore_angle.original_angle, surface->internal_solid_grid_index,
                           settings.cells_x, params.internal_solid_grid_angle_step);
        return FillRectilinear::fill_surface(surface, params);
    }

    const Surfaces cells = split_internal_solid_grid_surface(*surface, settings);
    if (cells.size() == 1)
        return FillRectilinear::fill_surface(surface, params);

    Polylines result;
    result.reserve(cells.size());
    for (const Surface &cell : cells) {
        angle = detail::cell_angle(restore_angle.original_angle, cell.internal_solid_grid_index,
                           settings.cells_x, params.internal_solid_grid_angle_step);
        Polylines cell_lines = FillRectilinear::fill_surface(&cell, params);
        result.insert(result.end(), std::make_move_iterator(cell_lines.begin()), std::make_move_iterator(cell_lines.end()));
    }
    return result;
}

} // namespace Slic3r
