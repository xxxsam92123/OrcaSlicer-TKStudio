#pragma once

#include "FillRectilinear.hpp"

namespace Slic3r {

namespace detail {

// Angle (radians) for one grid cell. Neighbouring cells along either axis differ by exactly one
// angle_step; the offset folds back every 180 degrees because line directions are equivalent mod pi.
// angle_step == 0 keeps every cell on the base angle.
float cell_angle(float angle, unsigned short cell_index, int cells_x, float angle_step);

} // namespace detail

class FillInternalSolidGrid : public FillRectilinear {
public:
    Fill *clone() const override { return new FillInternalSolidGrid(*this); }
    Polylines fill_surface(const Surface *surface, const FillParams &params) override;
};

} // namespace Slic3r
