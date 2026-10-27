#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/PerimeterGenerator.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"

using namespace Slic3r;

TEST_CASE("a void in the lower layer does not support a point", "[OverhangWallOverlap]")
{
    const coord_t mm = scaled<coord_t>(1.);
    ExPolygon     plate;
    plate.contour = Polygon({ Point(0, 0), Point(20 * mm, 0), Point(20 * mm, 20 * mm), Point(0, 20 * mm) });
    plate.holes.emplace_back(
        Polygon({ Point(5 * mm, 5 * mm), Point(15 * mm, 5 * mm), Point(15 * mm, 15 * mm), Point(5 * mm, 15 * mm) }));

    // The generator grows the lower layer by half a nozzle before asking whether it supports a sample.
    const Polygons supported = offset(ExPolygons{ plate }, float(scale_(0.2)));
    REQUIRE(supported.size() == 2);

    CHECK(point_is_supported(supported, Point(10 * mm, 10 * mm)) == false); // over the void
    CHECK(point_is_supported(supported, Point(1 * mm, 10 * mm)) == true);   // on the material
}

TEST_CASE("overhang wall overlap derives a bounded center spacing", "[OverhangWallOverlap]")
{
    const coord_t layer_height = scaled<coord_t>(0.20);
    const coord_t nominal_spacing = scaled<coord_t>(0.45);
    const coord_t overhang_width = scaled<coord_t>(0.50);

    CHECK(overhang_wall_spacing(nominal_spacing, overhang_width, layer_height, 0.) == nominal_spacing);
    CHECK(overhang_wall_spacing(nominal_spacing, overhang_width, layer_height, 20.) == scaled<coord_t>(0.35));
    CHECK(overhang_wall_spacing(nominal_spacing, overhang_width, layer_height, 100.) == layer_height);
    CHECK(overhang_wall_spacing(nominal_spacing, overhang_width, layer_height, 150.) == layer_height);
}