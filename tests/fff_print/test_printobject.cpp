#include <algorithm>
#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/AABBTreeLines.hpp"

#include "test_helpers.hpp"

#include <cmath>
#include <cstddef>
#include <iterator>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/BoundingBox.hpp"
#include <functional>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/SurfaceCollection.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

SCENARIO("Object layer heights", "[PrintObject]") {
    GIVEN("A 20mm cube") {
        WHEN("sliced with a 2mm layer height and a 3mm nozzle") {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, {
                { "initial_layer_print_height", 2 },
                { "layer_height",               2 },
                { "nozzle_diameter",            3 }
	        });
            ConstLayerPtrsAdaptor layers = print.objects().front()->layers();
            THEN("The output vector has 10 entries") {
                REQUIRE(layers.size() == 10);
            }
            AND_THEN("Each layer is approximately 2mm above the previous Z") {
                coordf_t last = 0.0;
                for (size_t i = 0; i < layers.size(); ++ i) {
                    REQUIRE_THAT(layers[i]->print_z - last, Catch::Matchers::WithinAbs(2.0, 1e-4));
                    last = layers[i]->print_z;
                }
            }
        }
        WHEN("sliced with a 10mm layer height and an 11mm nozzle") {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, {
                { "initial_layer_print_height", 2 },
                { "layer_height",               10 },
                { "nozzle_diameter",            11 }
	        });
            ConstLayerPtrsAdaptor layers = print.objects().front()->layers();
			THEN("The output vector has 3 entries") {
                REQUIRE(layers.size() == 3);
            }
            AND_THEN("Layer 0 is at 2mm") {
                REQUIRE_THAT(layers.front()->print_z, Catch::Matchers::WithinAbs(2.0, 1e-4));
            }
            AND_THEN("Layer 1 is at 12mm") {
                REQUIRE_THAT(layers[1]->print_z, Catch::Matchers::WithinAbs(12.0, 1e-4));
            }
        }
        WHEN("sliced with a 15mm layer height and a 16mm nozzle") {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, {
                { "initial_layer_print_height", 2 },
                { "layer_height",               15 },
                { "nozzle_diameter",            16 }
	        });
            ConstLayerPtrsAdaptor layers = print.objects().front()->layers();
			THEN("The output vector has 2 entries") {
                REQUIRE(layers.size() == 2);
            }
            AND_THEN("Layer 0 is at 2mm") {
                REQUIRE_THAT(layers[0]->print_z, Catch::Matchers::WithinAbs(2.0, 1e-4));
            }
            AND_THEN("Layer 1 is at 17mm") {
                REQUIRE_THAT(layers[1]->print_z, Catch::Matchers::WithinAbs(17.0, 1e-4));
            }
        }
        WHEN("layer height exceeds the nozzle diameter") {
            // Orca does not clamp an over-large layer height to the nozzle; it
            // rejects the slice during flow computation. Pin that behavior.
            THEN("Slicing is rejected") {
                Slic3r::Print print;
                REQUIRE_THROWS(Slic3r::Test::init_and_process_print({cube(20)}, print, {
                    { "initial_layer_print_height", 0.3 },
                    { "layer_height",               0.5 },
                    { "nozzle_diameter",            0.4 }
                }));
            }
        }
    }
}

SCENARIO("Perimeter generation", "[PrintObject]") {
    GIVEN("20mm cube and default config") {
        WHEN("make_perimeters() is called")  {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, { { "sparse_infill_density", 0 } });
			const PrintObject &object = *print.objects().front();
            THEN("Every layer in region 0 has 1 island of perimeters") {
                for (const Layer *layer : object.layers())
                    REQUIRE(layer->regions().front()->perimeters.entities.size() == 1);
            }
        }
        WHEN("wall_loops is set to 3")  {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, {
                { "sparse_infill_density", 0 },
                { "wall_loops",            3 }
            });
            const PrintObject &object = *print.objects().front();
            THEN("Every layer in region 0 has 3 perimeter loops") {
                for (const Layer *layer : object.layers())
                    REQUIRE(layer->regions().front()->perimeters.items_count() == 3);
            }
        }
    }
}

TEST_CASE("Initial layer height is honored", "[PrintObject]")
{
    const std::string gcode = Slic3r::Test::slice({cube(20)}, {
        { "initial_layer_print_height", 0.3 },
        { "layer_height",               0.2 },
        { "z_hop",                      0 } // keep recorded Z equal to the printed layer height
    });

    std::set<double> layer_zs;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&layer_zs] (GCodeReader& self, const GCodeReader::GCodeLine& line) {
        if (line.extruding(self) && line.dist_XY(self) > 0)
            layer_zs.insert(self.z());
    });

    REQUIRE(layer_zs.size() > 1);
    REQUIRE_THAT(*layer_zs.begin(),            Catch::Matchers::WithinAbs(0.3, 1e-4));
    REQUIRE_THAT(*std::next(layer_zs.begin()), Catch::Matchers::WithinAbs(0.5, 1e-4));
}

TEST_CASE("An object too thin for any layer fails with a slicing error", "[PrintObject][Regression]")
{
    const int  raft_layers      = GENERATE(0, 2);
    const bool precise_z_height = GENERATE(false, true);
    DYNAMIC_SECTION("raft_layers " << raft_layers << ", precise_z_height " << precise_z_height) {
        Slic3r::Print print;
        Slic3r::Model model;
        Slic3r::Test::init_print({make_cube(20, 20, 0.002)}, print, model, {
            { "layer_height",               0.2 },
            { "initial_layer_print_height", 0.2 },
            { "min_layer_height",           0.07 },
            { "raft_layers",                raft_layers },
            { "precise_z_height",           precise_z_height }
        });
        REQUIRE_THROWS_WITH(print.process(), Catch::Matchers::ContainsSubstring("No layers were detected"));
    }
}

static TriangleMesh internal_bridge_step()
{
    // Orca: The smaller tower leaves a shoulder whose solid skin needs internal bridges
    // over the sparse infill in the base, without relying on an external model file.
    TriangleMesh mesh = make_cube(30, 24, 3);
    TriangleMesh tower = make_cube(14, 10, 1);
    tower.translate(8, 7, 3);
    mesh.merge(tower);
    return mesh;
}

static DynamicPrintConfig internal_bridge_config(const std::string &pattern, int multiline)
{
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"sparse_infill_pattern", pattern},
                                   {"fill_multiline", multiline},
                                   {"sparse_infill_density", "15%"},
                                   {"sparse_infill_smooth_factor", "100%"},
                                   {"infill_direction", 45},
                                   {"internal_bridge_angle", 0},
                                   {"thick_internal_bridges", true},
                                   {"top_shell_layers", 3},
                                   {"bottom_shell_layers", 2},
                                   {"top_shell_thickness", 0},
                                   {"bottom_shell_thickness", 0},
                                   {"layer_height", 0.2},
                                   {"initial_layer_print_height", 0.2}});
    return config;
}

TEST_CASE("Internal bridge angles follow the lower infill layer and model rotation", "[PrintObject][InternalBridge][Regression]")
{
    const std::string pattern = GENERATE("hilbertcurve", "octagramspiral");
    // Orca: Cover both a central line (odd counts) and offset pairs (even counts).
    const int multiline = GENERATE(1, 2, 3);
    CAPTURE(multiline);
    const double rotation = GENERATE(23., -123.);
    const std::vector<double> cycle{10., 30., 70.};
    auto config = internal_bridge_config(pattern, multiline);
    config.set_deserialize_strict({{"sparse_infill_rotate_template", "10,30,70"},
                                   {"align_infill_direction_to_model", true},
                                   {"separated_infills", false}});
    Print print;
    Model model;
    init_print({internal_bridge_step()}, print, model, config, nullptr, false);
    model.objects.front()->instances.front()->set_rotation(Vec3d(0., 0., Geometry::deg2rad(rotation)));
    print.apply(model, config);
    print.process();
    const PrintObject &object = *print.objects().front();
    size_t bridges = 0;
    for (size_t i = 1; i < object.layer_count(); ++i) {
        // Orca: The support is one layer below the bridge. Check the template and model
        // rotation together, including normalization when the resulting angle is negative.
        double expected = std::fmod(cycle[(i - 1) % cycle.size()] + 90. + rotation, 180.);
        if (expected < 0.) expected += 180.;
        for (const LayerRegion *region : object.get_layer(i)->regions())
            for (const Surface *surface : region->fill_surfaces.filter_by_type(stInternalBridge)) {
                CAPTURE(pattern, rotation, i);
                CHECK_THAT(Geometry::rad2deg(surface->bridge_angle), Catch::Matchers::WithinAbs(expected, 0.001));
                ++bridges;
            }
    }
    REQUIRE(bridges > 0);
}

TEST_CASE("Internal bridge angles follow a ranged rotation template of the infill below", "[PrintObject][InternalBridge][RotateTemplate]")
{
    auto config = internal_bridge_config("hilbertcurve", 1);
    // A range uses the template parser, which a plain list skips, and the infill direction of 30 must not apply.
    config.set_deserialize_strict({{"sparse_infill_rotate_template", "+90/10"}, {"infill_direction", 30}});
    Print print;
    init_and_process_print({internal_bridge_step()}, print, config);
    const PrintObject &object = *print.objects().front();
    size_t bridges = 0;
    for (size_t i = 1; i < object.layer_count(); ++i) {
        // "+90/10" puts layer i - 1 at 9 * i degrees, and the bridge crosses it.
        const double expected = 9. * double(i) + 90.;
        for (const LayerRegion *region : object.get_layer(i)->regions())
            for (const Surface *surface : region->fill_surfaces.filter_by_type(stInternalBridge)) {
                const double bridge_angle = Geometry::rad2deg(surface->bridge_angle);
                CAPTURE(i, bridge_angle, expected);
                CHECK_THAT(std::remainder(bridge_angle - expected, 180.), Catch::Matchers::WithinAbs(0., 0.001));
                ++bridges;
            }
    }
    REQUIRE(bridges > 0);
}

TEST_CASE("Turning infill does not replace the anchors of another region", "[PrintObject][InternalBridge][Regression]")
{
    // Orca: Keep the right-hand region fixed while changing the left-hand pattern in the
    // same object. Its bridge areas must be independent of a previous candidate's anchors.
    const int multiline = GENERATE(1, 2, 3);
    CAPTURE(multiline);
    auto right_bridges = [multiline](const std::string &left_pattern) {
        auto config = internal_bridge_config(left_pattern, multiline);
        Print print;
        Model model;
        init_print({internal_bridge_step()}, print, model, config, nullptr, false);
        TriangleMesh right = internal_bridge_step();
        right.translate(50, 0, 0);
        ModelVolume *volume = model.objects.front()->add_volume(std::move(right));
        volume->config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipRectilinear));
        volume->config.set_key_value("infill_direction", new ConfigOptionFloat(17.));
        print.apply(model, config);
        print.process();
        std::map<size_t, Polygons> result;
        const PrintObject &object = *print.objects().front();
        for (size_t i = 0; i < object.layer_count(); ++i)
            for (const LayerRegion *region : object.get_layer(i)->regions())
                if (region->region().config().infill_direction == 17.)
                    polygons_append(result[i], to_polygons(region->fill_surfaces.filter_by_type(stInternalBridge)));
        return result;
    };
    const auto baseline = right_bridges("rectilinear");
    const auto actual = right_bridges(GENERATE("hilbertcurve", "octagramspiral"));
    REQUIRE(actual.size() == baseline.size());
    double total_area = 0.;
    for (const auto &[layer, expected] : baseline) {
        CAPTURE(layer);
        const auto &polys = actual.at(layer);
        CHECK(area(diff(expected, polys)) < scaled<double>(1.) * scaled<double>(1.) * 1e-6);
        CHECK(area(diff(polys, expected)) < scaled<double>(1.) * scaled<double>(1.) * 1e-6);
        total_area += area(expected);
    }
    REQUIRE(total_area > 0.);
}

TEST_CASE("Rounded internal bridges end on printed support", "[PrintObject][InternalBridge][Regression]")
{
    const std::string pattern = GENERATE("hilbertcurve", "octagramspiral");
    const bool separated = GENERATE(false, true);
    CAPTURE(pattern, separated);
    auto config = internal_bridge_config(pattern, 1);
    config.set_deserialize_strict({{"infill_wall_overlap", "0%"}, {"separated_infills", separated}});
    TriangleMesh mesh = internal_bridge_step();
    if (separated) {
        TriangleMesh second = internal_bridge_step();
        second.translate(50, 0, 0);
        mesh.merge(second);
    }
    Print print;
    Model model;
    init_print({mesh}, print, model, config, nullptr, false);
    print.process();

    // Orca: Check final extrusion endpoints after polygon cleanup and fill generation.
    // A correct bridge angle and correct sparse anchors alone do not guarantee contact.
    const PrintObject &object = *print.objects().front();
    size_t checked = 0;
    for (size_t i = 1; i < object.layer_count(); ++i) {
        Polygons support;
        Polylines walls;
        for (const LayerRegion *region : object.get_layer(i - 1)->regions()) {
            region->perimeters.polygons_covered_by_width(support, 0.f);
            region->fills.polygons_covered_by_width(support, 0.f);
            region->perimeters.collect_polylines(walls);
        }
        REQUIRE_FALSE(support.empty());
        const AABBTreeLines::LinesDistancer<Line> support_tree(to_lines(union_(support)));
        const AABBTreeLines::LinesDistancer<Line> wall_tree(to_lines(walls));
        for (const LayerRegion *region : object.get_layer(i)->regions())
            for (const ExtrusionEntity *entity : region->fills.flatten().entities) {
                if (entity->role() != erInternalBridgeInfill)
                    continue;
                const auto *path = dynamic_cast<const ExtrusionPath *>(entity);
                REQUIRE(path != nullptr);
                for (const Line &line : path->polyline.to_polyline().lines()) {
                    // Orca: Sample span ends, excluding short connectors and wall overlap.
                    if (line.length() < scale_(std::max(0.7, 3. * path->width)))
                        continue;
                    for (const Point &point : {line.a, line.b}) {
                        if (wall_tree.distance_from_lines<false>(point) <= scale_(0.5))
                            continue;
                        CAPTURE(i, point.x(), point.y());
                        const double gap = unscale<double>(support_tree.distance_from_lines<true>(point)) - 0.5 * path->width;
                        CHECK(gap <= 0.1);
                        ++checked;
                    }
                }
            }
    }
    REQUIRE(checked > 0);
}

TEST_CASE("Enabling separated infill recomputes body origins", "[PrintObject][InternalBridge][Regression]")
{
    const std::string pattern = GENERATE("hilbertcurve", "octagramspiral", "archimedeanchords");
    CAPTURE(pattern);
    auto footprint = [&](bool reslice) {
        auto config = internal_bridge_config(pattern, 2);
        config.set_deserialize_strict({{"separated_infills", !reslice}});
        TriangleMesh mesh = internal_bridge_step();
        TriangleMesh second = internal_bridge_step();
        second.translate(50, 0, 0);
        mesh.merge(second);
        Print print;
        Model model;
        init_print({mesh}, print, model, config, nullptr, false);
        print.process();
        if (reslice) {
            // Orca: Enabling centering after a completed slice must rebuild the body
            // origins now shared by bridge preparation and printed infill.
            config.set_deserialize_strict({{"separated_infills", true}});
            print.apply(model, config);
            print.process();
        }
        Polygons result;
        for (const LayerRegion *region : print.objects().front()->get_layer(4)->regions())
            region->fills.polygons_covered_by_width(result, 0.f);
        return union_(result);
    };
    const Polygons fresh = footprint(false);
    const Polygons resliced = footprint(true);
    REQUIRE_FALSE(fresh.empty());
    CHECK(area(diff(fresh, resliced)) < scaled<double>(1.) * scaled<double>(1.) * 1e-6);
    CHECK(area(diff(resliced, fresh)) < scaled<double>(1.) * scaled<double>(1.) * 1e-6);
}

TEST_CASE("Surface centering survives changes to separated infill settings", "[PrintObject][SurfaceInfill][Regression]")
{
    const std::string pattern = GENERATE("archimedeanchords", "octagramspiral");
    const std::string initial_center = GENERATE("each_surface", "each_model", "each_assembly");
    const std::string final_center = GENERATE("each_surface", "each_model", "each_assembly");
    const bool separated = GENERATE(false, true);
    const std::string top_order = GENERATE("default", "outward", "inward");
    const std::string bottom_order = top_order == "outward" ? "inward" : top_order == "inward" ? "outward" : "default";
    const std::string density = GENERATE("80%", "100%");
    const bool change_center = initial_center != final_center;
    CAPTURE(pattern, initial_center, final_center, separated, top_order, bottom_order, density);

    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"top_surface_pattern", pattern},
                                   {"bottom_surface_pattern", pattern},
                                   {"top_surface_fill_order", top_order},
                                   {"bottom_surface_fill_order", bottom_order},
                                   {"top_surface_density", density},
                                   {"bottom_surface_density", density},
                                   {"center_of_surface_pattern", initial_center},
                                   {"separated_infills", change_center ? separated : !separated},
                                   {"sparse_infill_pattern", "rectilinear"},
                                   {"sparse_infill_density", "15%"},
                                   {"top_shell_layers", 2},
                                   {"bottom_shell_layers", 2},
                                   {"top_shell_thickness", 0},
                                   {"bottom_shell_thickness", 0},
                                   {"layer_height", 0.2},
                                   {"initial_layer_print_height", 0.2}});

    // Orca: Two disconnected bodies exercise per-body centering. The offset tower also
    // makes each-surface and each-model centering differ on the top surfaces.
    TriangleMesh mesh = make_cube(30, 24, 2);
    TriangleMesh tower = make_cube(12, 10, 1);
    tower.translate(4, 3, 2);
    mesh.merge(tower);
    TriangleMesh second = mesh;
    second.translate(50, 0, 0);
    mesh.merge(second);

    // Orca: Equal footprints can hide reordered or reversed paths. Retain their point
    // sequences and ordering protection to cover the directional surface behavior too.
    struct SurfaceFillSnapshot {
        std::map<bool, std::vector<Points>> paths;
        bool protected_order = true;
    };
    auto surface_fills = [](const Print &print) {
        std::map<std::pair<size_t, ExtrusionRole>, SurfaceFillSnapshot> result;
        const PrintObject &object = *print.objects().front();
        for (size_t i = 0; i < object.layer_count(); ++i) {
            auto collect = [&](const auto &self, const ExtrusionEntity &entity, bool no_sort) -> void {
                if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
                    for (const ExtrusionEntity *child : collection->entities)
                        self(self, *child, no_sort || collection->no_sort);
                } else if (entity.role() == erTopSolidInfill || entity.role() == erBottomSurface) {
                    const auto *path = dynamic_cast<const ExtrusionPath *>(&entity);
                    REQUIRE(path != nullptr);
                    auto &snapshot = result[{i, entity.role()}];
                    // Orca: The centered test model has one body on either side of X=0.
                    // Their traversal order may vary; preserve path order within each body.
                    Points points = path->polyline.to_polyline().points;
                    REQUIRE_FALSE(points.empty());
                    snapshot.paths[points.front().x() > 0].push_back(std::move(points));
                    snapshot.protected_order &= no_sort && !path->can_reverse();
                }
            };
            for (const LayerRegion *region : object.get_layer(i)->regions())
                collect(collect, region->fills, false);
        }
        return result;
    };

    Print print;
    Model model;
    init_print({mesh}, print, model, config, nullptr, false);
    print.process();
    const auto initial = surface_fills(print);
    config.set_deserialize_strict({{"center_of_surface_pattern", final_center}, {"separated_infills", separated}});
    print.apply(model, config);
    // Orca: Preparation owns the body origins, and its invalidation must also force
    // regeneration of top/bottom extrusion paths, even when sparse infill is unchanged.
    CHECK_FALSE(print.objects().front()->is_step_done(posPrepareInfill));
    CHECK_FALSE(print.objects().front()->is_step_done(posInfill));
    print.process();
    const auto resliced = surface_fills(print);

    // The fresh slice depends only on the final settings, which several combinations share,
    // so slice each set once. The resliced paths must match it exactly, so slicing is deterministic.
    using FreshKey = std::tuple<std::string, std::string, bool, std::string, std::string>;
    static std::map<FreshKey, decltype(surface_fills(print))> fresh_slices;
    const FreshKey fresh_key{pattern, final_center, separated, top_order, density};
    auto fresh_it = fresh_slices.find(fresh_key);
    if (fresh_it == fresh_slices.end()) {
        Print fresh_print;
        Model fresh_model;
        init_print({mesh}, fresh_print, fresh_model, config, nullptr, false);
        fresh_print.process();
        fresh_it = fresh_slices.emplace(fresh_key, surface_fills(fresh_print)).first;
    }
    const auto &fresh = fresh_it->second;
    REQUIRE_FALSE(fresh.empty());
    REQUIRE(resliced.size() == fresh.size());
    std::set<ExtrusionRole> roles;
    bool changed_paths = false;
    for (const auto &entry : fresh) {
        CAPTURE(entry.first.first, entry.first.second);
        REQUIRE_FALSE(entry.second.paths.empty());
        roles.insert(entry.first.second);
        REQUIRE(resliced.count(entry.first) == 1);
        REQUIRE(initial.count(entry.first) == 1);
        const auto &actual = resliced.at(entry.first);
        const auto &expected = entry.second;
        const auto &before = initial.at(entry.first);
        CHECK((actual.paths == expected.paths));
        if (!change_center)
            CHECK((actual.paths == before.paths));
        if (top_order != "default") {
            CHECK(expected.protected_order);
            CHECK(actual.protected_order);
            CHECK(before.protected_order);
        }
        changed_paths |= expected.paths != before.paths;
    }
    CHECK(roles.count(erTopSolidInfill) == 1);
    CHECK(roles.count(erBottomSurface) == 1);
    // Orca: Guard against a vacuous comparison: changing surface centering must change
    // the printed pattern, while toggling separated sparse infill must leave it alone.
    CHECK(changed_paths == change_center);
}

TEST_CASE("Separated infill keeps fragmented and nested bodies independent", "[PrintObject][SurfaceInfill][Regression]")
{
    constexpr size_t grid_size = 8;
    TriangleMesh mesh;
    auto add_box = [&](double x, double y, double width, double depth) {
        TriangleMesh box = make_cube(width, depth, 0.6);
        box.translate(x, y, 0);
        mesh.merge(box);
    };
    // Orca: Many small islands exercise spatial pruning and the tree's original
    // island indices. A pillar inside a frame also overlaps its bounding box,
    // but must remain a separate body because it lies entirely inside the hole.
    for (size_t x = 0; x < grid_size; ++ x)
        for (size_t y = 0; y < grid_size; ++ y)
            add_box(6 * x, 6 * y, 3, 3);
    add_box(54, 0, 20, 4);
    add_box(54, 16, 20, 4);
    add_box(54, 0, 4, 20);
    add_box(70, 0, 4, 20);
    add_box(62, 8, 4, 4);

    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"separated_infills", true},
                                   {"center_of_surface_pattern", "each_surface"},
                                   {"layer_height", 0.2},
                                   {"initial_layer_print_height", 0.2},
                                   {"elefant_foot_compensation", 0},
                                   {"wall_loops", 1}});
    Print print;
    Model model;
    init_print({mesh}, print, model, config, nullptr, false);
    // Orca: Prepare body bounds through the public pipeline, then inspect the object read-only.
    print.process();
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.layer_count() > 1);
    CHECK(object.separated_body_bboxes().size() == grid_size * grid_size + 2);
    for (const Layer *layer : object.layers()) {
        REQUIRE(layer->lslices.size() == grid_size * grid_size + 2);
        REQUIRE(layer->lslices_separated_component_ids.size() == layer->lslices.size());
        size_t holes = 0;
        for (size_t i = 0; i < layer->lslices.size(); ++ i) {
            const BoundingBox &body = object.separated_body_bboxes()[layer->lslices_separated_component_ids[i]];
            const BoundingBox &island = layer->lslices_bboxes[i];
            CHECK(body.min == island.min);
            CHECK(body.max == island.max);
            holes += layer->lslices[i].holes.size();
        }
        CHECK(holes == 1);
    }
}

TEST_CASE("Body centering survives islands merging and splitting between layers", "[PrintObject][SurfaceInfill][Regression]")
{
    const bool separated = GENERATE(false, true);
    CAPTURE(separated);
    // Orca: Four posts join through horizontal then vertical rails, creating a
    // cycle of overlaps before splitting into four islands again. This exercises
    // redundant connections and indexing either adjacent layer. A fifth post
    // stays separate at every height.
    TriangleMesh mesh;
    for (int x : {0, 8})
        for (int y : {0, 8}) {
            TriangleMesh post = make_cube(4, 4, 1);
            post.translate(x, y, 0);
            mesh.merge(post);
        }
    for (int y : {0, 8}) {
        TriangleMesh rail = make_cube(12, 4, 0.2);
        rail.translate(0, y, 0.2);
        mesh.merge(rail);
    }
    for (int x : {0, 8}) {
        TriangleMesh rail = make_cube(4, 12, 0.2);
        rail.translate(x, 0, 0.4);
        mesh.merge(rail);
    }
    TriangleMesh isolated = make_cube(4, 4, 1);
    isolated.translate(20, 0, 0);
    mesh.merge(isolated);

    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"separated_infills", separated},
                                   {"center_of_surface_pattern", separated ? "each_surface" : "each_model"},
                                   {"layer_height", 0.2},
                                   {"initial_layer_print_height", 0.2},
                                   {"elefant_foot_compensation", 0},
                                   {"wall_loops", 1}});
    Print print;
    Model model;
    init_print({mesh}, print, model, config, nullptr, false);
    // Orca: Prepare body bounds through the public pipeline, then inspect the object read-only.
    print.process();
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.layer_count() == 5);
    REQUIRE(object.get_layer(0)->lslices.size() == 5);
    REQUIRE(object.get_layer(1)->lslices.size() == 3);
    REQUIRE(object.get_layer(2)->lslices.size() == 3);
    REQUIRE(object.get_layer(4)->lslices.size() == 5);
    CHECK(object.separated_body_bboxes().size() == 2);

    BoundingBox isolated_bbox = object.get_layer(0)->lslices_bboxes.front();
    for (const BoundingBox &bbox : object.get_layer(0)->lslices_bboxes)
        if (bbox.min.x() > isolated_bbox.min.x())
            isolated_bbox = bbox;
    BoundingBox connected_bbox;
    for (const Layer *layer : object.layers())
        for (const BoundingBox &bbox : layer->lslices_bboxes)
            if (bbox.min.x() < isolated_bbox.min.x())
                connected_bbox.merge(bbox);
    for (const Layer *layer : object.layers()) {
        REQUIRE(layer->lslices_separated_component_ids.size() == layer->lslices.size());
        for (size_t i = 0; i < layer->lslices.size(); ++ i) {
            const BoundingBox &expected = layer->lslices_bboxes[i].min.x() < isolated_bbox.min.x() ? connected_bbox : isolated_bbox;
            const BoundingBox &actual = object.separated_body_bboxes()[layer->lslices_separated_component_ids[i]];
            CHECK(actual.min == expected.min);
            CHECK(actual.max == expected.max);
        }
    }
}

TEST_CASE("Z contouring slices an object that has several instances", "[PrintObject]")
{
    Print print;
    Model model;
    place_two_cube_instances_apart(10., { { "zaa_enabled", "1" } }, print, model);
    REQUIRE(print.objects().size() == 1);
    REQUIRE(print.objects().front()->instances().size() == 2);

    CHECK_NOTHROW(print.process());
    CHECK(print.objects().front()->is_step_done(posContouring));
}

TEST_CASE("Z contouring slices instances of one object that are turned differently", "[PrintObject]")
{
    // An instance turned about Z is sliced as a PrintObject of its own, with the same ModelObject behind it.
    Print print;
    Model model;
    place_two_cube_instances_apart(10., { { "zaa_enabled", "1" } }, print, model);
    model.objects.front()->instances.back()->set_rotation(Z, PI / 4.);
    print.apply(model, DynamicPrintConfig(print.full_print_config()));
    REQUIRE(print.objects().size() == 2);

    CHECK_NOTHROW(print.process());
    for (const PrintObject *object : print.objects())
        CHECK(object->is_step_done(posContouring));
}

// A box tilted about X, so that its top slopes along Y and Z contouring has a surface to follow, with
// one instance for each Z rotation given, laid out in a row.
static void slice_tilted_box_instances(const std::vector<double> &z_rotations, Print &print, Model &model)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "zaa_enabled", "1" } });

    TriangleMesh box = make_cube(20, 20, 10);
    box.rotate_x(float(PI / 12.));
    ModelObject *object = model.add_object();
    object->add_volume(std::move(box));
    for (size_t idx = 0; idx < z_rotations.size(); ++idx) {
        ModelInstance *instance = object->add_instance();
        instance->set_offset(Vec3d(50. + 50. * double(idx), 100., 0.));
        instance->set_rotation(Z, z_rotations[idx]);
    }
    object->ensure_on_bed();
    print.auto_assign_extruders(object);
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
    print.process();
}

// The Z that contouring gave each point of the object's extrusions, by layer and by XY in the
// PrintObject's own frame, which does not depend on where its instances stand on the bed.
static std::vector<std::map<std::pair<coord_t, coord_t>, coord_t>> contoured_heights(const PrintObject &object)
{
    std::vector<std::map<std::pair<coord_t, coord_t>, coord_t>> heights(object.layer_count());
    for (size_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
        auto &layer_heights = heights[layer_idx];
        std::function<void(const ExtrusionEntity &)> visit = [&](const ExtrusionEntity &entity) {
            if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) {
                if (path->z_contoured)
                    for (const Point3 &pt : path->polyline.points)
                        layer_heights[{ pt.x(), pt.y() }] = pt.z();
            } else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath *>(&entity)) {
                for (const ExtrusionPath &path : multipath->paths)
                    visit(path);
            } else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity)) {
                for (const ExtrusionPath &path : loop->paths)
                    visit(path);
            } else if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
                for (const ExtrusionEntity *child : collection->entities)
                    visit(*child);
            }
        };
        for (const LayerRegion *region : object.get_layer(int(layer_idx))->regions()) {
            visit(region->perimeters);
            visit(region->fills);
        }
    }
    return heights;
}

TEST_CASE("Z contouring follows the surface of each instance as it is turned", "[PrintObject]")
{
    // Two instances of a box with a sloped top, the second turned a quarter turn about Z: its top slopes
    // along X instead of Y. Each is contoured against the box turned as that instance is, so it has to
    // match the same box placed alone with that turn.
    Print print;
    Model model;
    slice_tilted_box_instances({ 0., PI / 2. }, print, model);
    REQUIRE(print.objects().size() == 2);

    for (const PrintObject *object : print.objects()) {
        const double z_rotation = object->instances().front().model_instance->get_rotation().z();
        Print reference;
        Model reference_model;
        slice_tilted_box_instances({ z_rotation }, reference, reference_model);
        REQUIRE(reference.objects().size() == 1);

        const auto heights           = contoured_heights(*object);
        const auto reference_heights = contoured_heights(*reference.objects().front());
        REQUIRE(heights.size() == reference_heights.size());

        size_t matched = 0, lifted = 0;
        for (size_t layer_idx = 0; layer_idx < heights.size(); ++layer_idx) {
            const coord_t print_z = scaled(object->get_layer(int(layer_idx))->print_z);
            for (const auto &[xy, z] : heights[layer_idx]) {
                const auto it = reference_heights[layer_idx].find(xy);
                if (it == reference_heights[layer_idx].end())
                    continue;
                INFO("layer " << layer_idx << ", rotation " << z_rotation);
                CHECK(std::abs(z - it->second) <= scaled(0.001));
                ++matched;
                if (std::abs(z - print_z) > scaled(0.01))
                    ++lifted;
            }
        }
        INFO("rotation " << z_rotation);
        // Enough points in common to mean something, and contouring did move some of them.
        CHECK(matched > 100);
        CHECK(lifted > 0);
    }
}

// The per-volume first-layer slices are the input of the volume grouping; the groups it produces
// are what brim generation reads afterwards. A sliced object therefore has to keep the groups and
// must not keep a second copy of the same first layer.
TEST_CASE("A sliced object keeps its first-layer groups and drops the per-volume slices", "[PrintObject]")
{
    Print print;
    init_and_process_print({ cube(20.) }, print, DynamicPrintConfig::full_print_config());
    REQUIRE(print.objects().size() == 1);
    const PrintObject *object = print.objects().front();
    REQUIRE(!object->layers().empty());

    CHECK(object->firstLayerObjSlice().empty());
    CHECK(!object->firstLayerObjGroups().empty());

    // Dropping the volume slices did not cut the slice short: the export that consumes it still
    // produces G-code.
    CHECK(!gcode(print).empty());
}

// Orca: With the extra bridge layer pass disabled, nothing else may print a second bridging
// layer over the same footprint. A layer that sits on the internal bridge of the layer directly
// below it is supported solid infill, not a new span over sparse infill.
//
// thick_internal_bridges defaults to true in production, so every check below runs for the thin
// and the thick bridging flow. The surface geometry is computed from the thick bridging flow
// either way (PrintObject::bridge_over_infill uses bridging_flow(frSolidInfill, true)); only the
// deposited line bands differ, which is why the face areas are shared and the band checks are not.
static DynamicPrintConfig internal_bridge_no_extra_layer_config(bool thick_internal_bridges)
{
    auto config = internal_bridge_config("gyroid", 1);
    config.set_deserialize_strict({{"sparse_infill_density", "35%"},
                                   {"internal_bridge_density", "65%"},
                                   {"internal_bridge_infill_wall_overlap", "10%"},
                                   {"enable_extra_bridge_layer", "disabled"},
                                   {"thick_internal_bridges", thick_internal_bridges},
                                   {"thick_bridges", false},
                                   {"wall_generator", "classic"},
                                   {"wall_loops", 4},
                                   {"top_shell_layers", 4},
                                   {"bottom_shell_layers", 4},
                                   {"infill_direction", 45}});
    return config;
}

static double internal_bridge_area_mm2(const Polygons &polys)
{
    // Orca: Polygons are in scaled coordinates, so the shoelace area needs SCALING_FACTOR^2 applied.
    return double(area(polys)) * SCALING_FACTOR * SCALING_FACTOR;
}

// Orca: Every target layer is addressed by the print Z it is printed at and validated against the
// object before use. Indexing a fixed 12 or 16 assumed a layer count the fixture does not promise,
// so a shorter object would have read past the end.
static size_t internal_bridge_layer_at_z(const PrintObject &object, double print_z)
{
    for (size_t i = 0; i < object.layer_count(); ++i)
        if (std::abs(object.get_layer(i)->print_z - print_z) <= 0.5 * object.get_layer(i)->height)
            return i;
    return object.layer_count(); // Sentinel: callers REQUIRE the mapping before indexing.
}

static const Layer &internal_bridge_layer_checked(const PrintObject &object, size_t layer_id)
{
    REQUIRE(layer_id < object.layer_count());
    return *object.get_layer(layer_id);
}

static Polygons internal_bridge_surfaces(const PrintObject &object, size_t layer_id, SurfaceType type)
{
    Polygons out;
    for (const LayerRegion *region : internal_bridge_layer_checked(object, layer_id).regions())
        polygons_append(out, to_polygons(region->fill_surfaces.filter_by_type(type)));
    return union_(out);
}

static double internal_bridge_role_length(const PrintObject &object, size_t layer_id, ExtrusionRole role)
{
    double length = 0.;
    for (const LayerRegion *region : internal_bridge_layer_checked(object, layer_id).regions())
        for (const ExtrusionEntity *entity : region->fills.flatten().entities)
            if (entity->role() == role)
                length += unscale<double>(entity->length());
    return length;
}

// Orca: Polygon covered by the internal bridge line bands of one layer, as deposited.
static Polygons internal_bridge_band(const PrintObject &object, size_t layer_id)
{
    Polygons band;
    for (const LayerRegion *region : internal_bridge_layer_checked(object, layer_id).regions()) {
        ExtrusionEntityCollection collection;
        for (const ExtrusionEntity *entity : region->fills.flatten().entities)
            if (entity->role() == erInternalBridgeInfill)
                collection.append(*entity);
        polygons_append(band, collection.polygons_covered_by_width(0.f));
    }
    return union_(band);
}

// Orca: One deposited internal bridge line of a layer, as Fill actually produced it.
static std::optional<ExtrusionPath> first_internal_bridge_path(const PrintObject &object, size_t layer_id)
{
    if (layer_id >= object.layer_count())
        return std::nullopt;
    for (const LayerRegion *region : object.get_layer(layer_id)->regions())
        for (const ExtrusionEntity *entity : region->fills.flatten().entities)
            if (entity->role() == erInternalBridgeInfill)
                if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity); path != nullptr)
                    return *path;
    return std::nullopt;
}

// Width the internal bridge lines of a layer are deposited with, from the G-code producing
// entities themselves rather than from the nominal config. Absent when the layer prints no
// internal bridge line at all, so the caller has to decide instead of fabricating a width.
static std::optional<double> current_bridge_width(const PrintObject &object, size_t layer_id)
{
    if (layer_id >= object.layer_count())
        return std::nullopt;
    for (const LayerRegion *region : object.get_layer(layer_id)->regions())
        for (const ExtrusionEntity *entity : region->fills.flatten().entities)
            if (entity->role() == erInternalBridgeInfill)
                if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity); path != nullptr)
                    // ExtrusionPath::width is already in mm, the unit the config uses.
                    return double(path->width);
    return std::nullopt;
}

TEST_CASE("Internal bridges do not repeat the bridge of the layer below", "[PrintObject][InternalBridge][InternalBridgeRepeat][Regression]")
{
    // Orca: A shoulder whose solid skin lands on the sparse infill of the base bridges once.
    // The next layer prints the same solid footprint but sits on that bridge, so it must not
    // bridge again there. Only a real gap may still span, and no bridge may be printed twice.
    const bool thick = GENERATE(false, true);
    CAPTURE(thick);
    Print print;
    init_and_process_print({internal_bridge_step()}, print, internal_bridge_no_extra_layer_config(thick));
    const PrintObject &object = *print.objects().front();

    // Orca: The fixture is sliced at 0.2 mm, so layer i is printed at 0.2 * (i + 1) mm. Pin that
    // mapping before addressing any layer by Z, and REQUIRE it instead of assuming it.
    REQUIRE(object.layer_count() > 0);
    for (size_t i = 0; i < object.layer_count(); ++i) {
        CAPTURE(i, object.get_layer(i)->print_z);
        REQUIRE(std::abs(object.get_layer(i)->print_z - 0.2 * double(i + 1)) <= 0.5 * object.get_layer(i)->height);
    }
    const size_t bleed_layer    = internal_bridge_layer_at_z(object, 2.6); // layer 12
    const size_t spanning_layer = internal_bridge_layer_at_z(object, 3.4); // layer 16
    REQUIRE(bleed_layer < object.layer_count());
    REQUIRE(spanning_layer < object.layer_count());
    // Orca: Assert the index the target checks below rely on, rather than assuming 12 and 16.
    CHECK(bleed_layer == 12);
    CHECK(spanning_layer == 16);
    REQUIRE(bleed_layer >= 2);

    // Orca: The thick/thin choice has to reach Fill, not just the config object: a bridge line
    // deposited on the bleed layer must be the flow LayerRegion::bridging_flow(frSolidInfill, thick)
    // yields. A thick bridge is a rounded line one nozzle diameter high; a thin one keeps the layer
    // height. This is what decides the extrusion width and height behind every band check below,
    // so an area-only comparison could not tell which flow ran.
    {
        const std::optional<ExtrusionPath> path = first_internal_bridge_path(object, bleed_layer);
        REQUIRE(path.has_value());
        const Layer &layer = internal_bridge_layer_checked(object, bleed_layer);
        REQUIRE_FALSE(layer.regions().empty());
        const Flow expected = layer.regions().front()->bridging_flow(frSolidInfill, thick);
        CAPTURE(path->width, path->height, path->mm3_per_mm, expected.width(), expected.height(), expected.mm3_per_mm());
        CHECK(path->width > 0.f);
        CHECK_THAT(path->width, Catch::Matchers::WithinAbs(double(expected.width()), 1e-5));
        CHECK_THAT(path->height, Catch::Matchers::WithinAbs(double(expected.height()), 1e-5));
        CHECK_THAT(path->mm3_per_mm, Catch::Matchers::WithinAbs(expected.mm3_per_mm(), 1e-8));
        // Orca: And the two runs must differ, otherwise the flag would be wired to nothing.
        CHECK((path->height > 1.5f * float(object.config().layer_height.value)) == thick);
    }

    double total_bridge_area = 0.;
    double repeated_area     = 0.;
    size_t bridged_layers    = 0;
    for (size_t i = 1; i < object.layer_count(); ++i) {
        const Polygons current      = internal_bridge_surfaces(object, i, stInternalBridge);
        if (current.empty())
            continue;
        const Polygons lower_bridge = internal_bridge_surfaces(object, i - 1, stInternalBridge);
        const double   current_area = internal_bridge_area_mm2(current);
        const double   repeat_area  = internal_bridge_area_mm2(intersection(current, lower_bridge));
        CAPTURE(i, current_area, repeat_area);
        // Orca: Contact along the shared boundary is expected where a real span starts on the
        // lower bridge. Repeating it over the same footprint is not.
        CHECK(repeat_area <= 0.02 * current_area);
        repeated_area += repeat_area;
        total_bridge_area += current_area;
        if (current_area > 20.)
            ++bridged_layers;
    }
    REQUIRE(total_bridge_area > 0.);
    REQUIRE(bridged_layers > 0);

    // Orca: The exact fixture used to bleed on its 12th layer: its bridge covered the bridge
    // below it over 32.3 mm2 of an 82.5 mm2 face. That footprint must be removed, and the same
    // area must come back as the internal solid infill it is, not disappear. These are face
    // areas, independent of thick_internal_bridges, which changes only the deposited bands.
    CHECK(internal_bridge_area_mm2(internal_bridge_surfaces(object, bleed_layer, stInternalBridge)) <= 50.5);
    CHECK(internal_bridge_area_mm2(internal_bridge_surfaces(object, bleed_layer, stInternalSolid)) >= 475.6);
    CHECK(repeated_area <= 0.02 * total_bridge_area);

    // Orca: A bridge that spans a real unsupported area has no bridge below it, so it must
    // survive untouched. Without this the checks above would also pass by not bridging at all.
    const double unsupported_layer_bridge = internal_bridge_area_mm2(internal_bridge_surfaces(object, spanning_layer, stInternalBridge));
    CHECK(unsupported_layer_bridge >= 71.6);

    // Orca: The G-code an operator actually gets must not lay a second bridging line band over
    // the first one either. Extrusion widths differ per role and per thick/thin bridging flow,
    // so this is measured on the deposited bands, not on the surface faces above.
    for (size_t i = 1; i < object.layer_count(); ++i) {
        const Polygons current_band = internal_bridge_band(object, i);
        if (current_band.empty())
            continue;
        const Polygons lower_band = internal_bridge_band(object, i - 1);
        if (lower_band.empty())
            continue;
        const double band_area   = internal_bridge_area_mm2(current_band);
        const double band_repeat = internal_bridge_area_mm2(intersection(current_band, lower_band));
        // Orca: Where a surviving span starts on the lower bridge, its first line overlaps that
        // bridge for one extrusion width. That is contact, not a repeat: eroding the lower band
        // by one deposited bridge line width must leave essentially nothing underneath. The width
        // is read off the deposited path; a layer that prints no bridge line had an empty band and
        // was skipped above, never handed a made-up width.
        const std::optional<double> bridge_width = current_bridge_width(object, i);
        REQUIRE(bridge_width.has_value());
        const double band_interior = internal_bridge_area_mm2(
            intersection(current_band, offset(lower_band, float(-scale_(*bridge_width)))));
        CAPTURE(i, band_area, band_repeat, band_interior, *bridge_width);
        // Orca: All three are mm2 and passed band_area is the covered area, not a length. The
        // 3% / 0.5% limits are set from the measured anchor rim, not from a guess: on this fixture
        // (layer 12, 0.4 mm bridge line) the rim is 0.10 mm2 / 0.32% for thick_internal_bridges
        // and 0.75 mm2 / 1.90% for the thin flow, and band_interior is 0.0 in both cases. A
        // repeat of the removed footprint would be ~14.5 mm2 / ~28% here, an order of magnitude
        // past the first limit, so the check still separates contact from a real repeat.
        CHECK(band_repeat <= 0.03 * band_area);
        CHECK(band_interior <= 0.005 * band_area);
    }

    // Orca: The removed repeat must not vanish and must not be printed a second time under a
    // different name: the same footprint has to come back exactly once, as internal solid infill.
    const double layer_bridge = internal_bridge_area_mm2(internal_bridge_surfaces(object, bleed_layer, stInternalBridge));
    const double layer_solid  = internal_bridge_area_mm2(internal_bridge_surfaces(object, bleed_layer, stInternalSolid));
    CAPTURE(layer_bridge, layer_solid);
    // Orca: 50.2417 + 475.6315 = 525.8732 mm2, against 82.5469 + 443.3267 = 525.8736 mm2 before
    // the repeat was removed, so the total is conserved to 0.0004 mm2. The 0.01 mm2 tolerance is
    // ~25 times that Clipper integer-offset residue and 3200 times smaller than the 32.3 mm2 the
    // fix moves, so it can only fail on a real loss or duplication of area.
    CHECK(std::abs(layer_bridge + layer_solid - 525.874) <= 0.01);

    // Orca: Removing a repeat must not move the surviving bridges off the region they belong to,
    // nor across the region boundary of the layer below, and no bridge may end up inside a hole
    // of its layer.
    for (size_t i = 0; i < object.layer_count(); ++i) {
        for (const LayerRegion *region : object.get_layer(i)->regions()) {
            Polygons holes;
            for (const ExPolygon &expoly : to_expolygons(region->slices.surfaces))
                for (const Polygon &hole : expoly.holes)
                    holes.emplace_back(hole);
            const Polygons boundary = to_polygons(region->fill_internal_bridge_expolygons);
            Polygons bridges;
            polygons_append(bridges, to_polygons(region->fill_surfaces.filter_by_type(stInternalBridge)));
            if (bridges.empty())
                continue;
            CAPTURE(i);
            if (!boundary.empty()) {
                // Orca: The bridge may sit one extrusion width inside the perimeter-derived
                // boundary it was reconstructed against; nothing may leave that domain. The
                // tolerance is a mm2 area, sized as four line-width squares from the deposited
                // (not nominal) bridge width, i.e. 0.64 mm2 for the 0.4 mm lines of this fixture.
                // Measured escape is exactly 0.0 mm2 on every layer, so this only bounds rounding.
                const std::optional<double> width = current_bridge_width(object, i);
                REQUIRE(width.has_value());
                CHECK(internal_bridge_area_mm2(diff(bridges, boundary)) <= 4. * *width * *width);
            }
            if (!holes.empty())
                CHECK(internal_bridge_area_mm2(intersection(bridges, holes)) <= 0.01);
        }
    }

    // Orca: The footprint handed back to solid infill must be *filled* as solid infill, with the
    // solid flow, not left to a leftover bridge path. This is where a thick bridge (a rounded line
    // one nozzle diameter high, mm3/mm ~0.126) would print differently from a thin one (~0.071),
    // so the check is repeated for both flows by the GENERATE above. A thick bridge line left over
    // the converted area would show up as bridge band on top of solid infill here.
    {
        const Polygons bridge_band = internal_bridge_band(object, bleed_layer);
        REQUIRE_FALSE(bridge_band.empty());
        Polygons solid_faces;
        for (const LayerRegion *region : object.get_layer(bleed_layer)->regions())
            polygons_append(solid_faces, to_polygons(region->fill_surfaces.filter_by_type(stInternalSolid)));
        const Polygons solid_union = union_(solid_faces);
        REQUIRE_FALSE(solid_union.empty());
        const double on_solid = internal_bridge_area_mm2(intersection(bridge_band, solid_union));
        CAPTURE(on_solid, thick);
        // Orca: Areas are mm2. Only the anchor rim where a surviving span starts may touch the
        // solid face; measured 0.132 mm2 (thick) and 0.556 mm2 (thin) against the 0.64 mm2 bound,
        // while the repeat itself would be 32.3 mm2. This is the thick-bridge-specific guard: a
        // leftover rounded bridge line over the converted face would land here.
        CHECK(on_solid <= 4. * 0.4 * 0.4);

        // Orca: And the solid replacement must be deposited with the solid flow of its layer, i.e.
        // one layer height high, never with the bridge flow this code removed.
        bool saw_solid = false;
        for (const LayerRegion *region : object.get_layer(bleed_layer)->regions())
            for (const ExtrusionEntity *entity : region->fills.flatten().entities) {
                if (entity->role() != erSolidInfill)
                    continue;
                const auto *path = dynamic_cast<const ExtrusionPath *>(entity);
                REQUIRE(path != nullptr);
                CAPTURE(path->height, path->width, path->mm3_per_mm, thick);
                CHECK_THAT(path->height,
                           Catch::Matchers::WithinAbs(double(object.get_layer(bleed_layer)->height), 1e-5));
                saw_solid = true;
            }
        CHECK(saw_solid);
    }

    // Orca: Sparse infill below the removed repeat must still be printed normally, not left
    // empty because the bridge that used to cover it is gone.
    CHECK(internal_bridge_role_length(object, bleed_layer - 2, erInternalInfill) > 400.);
}
// Orca: The repeat removal is scoped by what a mode actually does, checked against the enum
// (PrintConfig.hpp: EnableExtraBridgeLayer) and the pass gates in PrintObject::bridge_over_infill.
//   eblDisabled          - no extra pass at all.  -> nothing may double an internal bridge.
//   eblExternalBridgeOnly- only detect_surfaces_type() adds an external bridge layer
//                          (gated on ExternalBridgeOnly || ApplyToAll at the two
//                          stInternalAfterExternalBridge sites). It adds NO internal bridge
//                          layer, so it must dedup exactly like disabled.
//   eblInternalBridgeOnly- runs the "second internal bridge layer" pass (gated on
//   eblApplyToAll          InternalBridgeOnly || ApplyToAll), raising stSecondInternalBridge and
//                          re-classifying it to stInternalBridge on the layer above. That second
//                          layer is the feature, so the overlap there is deliberate.
// So the intent is NOT "every non-disabled mode keeps the overlap": external_bridge_only is a
// dedup mode, and the check below fails if a future refactor moves it back to the doubling group.
TEST_CASE("Extra bridge layer modes keep their deliberate second bridge", "[PrintObject][InternalBridge][ExtraBridgeLayer][Regression]")
{
    const std::string mode = GENERATE("disabled", "internal_bridge_only", "external_bridge_only", "apply_to_all");
    CAPTURE(mode);

    // Orca: These are the modes whose double internal bridge is intentional, spelled out here so the
    // test cannot drift into "all non-disabled modes double" by accident.
    const bool doubles_internal_bridge = (mode == "internal_bridge_only" || mode == "apply_to_all");

    auto config = internal_bridge_no_extra_layer_config(false);
    config.set_deserialize_strict({{"enable_extra_bridge_layer", mode}});
    Print print;
    init_and_process_print({internal_bridge_step()}, print, config);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.layer_count() > 1);

    // Orca: Same fixture and same layer mapping as the repeat test, so the same bleed layer is the
    // one whose repeat the two dedup modes must remove.
    const size_t bleed_layer = internal_bridge_layer_at_z(object, 2.6);
    CHECK(bleed_layer == 12);
    REQUIRE(bleed_layer < object.layer_count());

    // Orca: On the layers that carry a bridge below them, ask how much of the upper bridge sits on
    // the lower one. Both dedup modes must give that footprint back; only the two explicit
    // second-internal-bridge-layer modes may keep it.
    double overlap_area = 0.;
    double total_area   = 0.;
    double worst_share  = 0.;
    for (size_t i = 1; i < object.layer_count(); ++i) {
        const Polygons current = internal_bridge_surfaces(object, i, stInternalBridge);
        if (current.empty())
            continue;
        const double current_area = internal_bridge_area_mm2(current);
        const double overlap      = internal_bridge_area_mm2(
            intersection(current, internal_bridge_surfaces(object, i - 1, stInternalBridge)));
        CAPTURE(i, current_area, overlap);
        total_area   += current_area;
        overlap_area += overlap;
        worst_share   = std::max(worst_share, overlap / current_area);
    }
    REQUIRE(total_area > 0.);
    CAPTURE(total_area, overlap_area, worst_share, doubles_internal_bridge);

    if (!doubles_internal_bridge) {
        // Orca: disabled and external_bridge_only. No pass in this mode may produce a second
        // internal bridge layer, so anything above contact-level overlap is the bleed bug.
        CHECK(worst_share <= 0.02);
        CHECK(overlap_area <= 0.02 * total_area);
    } else {
        // Orca: The second internal bridge layer must actually be there, otherwise the feature this
        // mode exists for was silently disabled by the dedup. Measured on this fixture: 618.712 mm2
        // of overlap in total (1214.28 mm2 of bridge above 0.3 mm2 of contact-level rounding), i.e.
        // the second layer covers the first almost completely, so 500 mm2 is a floor on a real
        // second layer and 10x above anything contact could produce.
        CHECK(overlap_area >= 500.);
    }

    // Orca: The bleed layer itself, in the two dedup modes: the repeated 32.3 mm2 footprint must be
    // gone from the bridge faces and present as internal solid infill instead, exactly as in the
    // disabled-mode test. This is the strong assertion for external_bridge_only.
    if (!doubles_internal_bridge) {
        const double bleed_bridge = internal_bridge_area_mm2(internal_bridge_surfaces(object, bleed_layer, stInternalBridge));
        const double bleed_solid  = internal_bridge_area_mm2(internal_bridge_surfaces(object, bleed_layer, stInternalSolid));
        CAPTURE(bleed_bridge, bleed_solid);
        CHECK(bleed_bridge <= 50.5);
        CHECK(bleed_solid >= 475.6);
        CHECK(std::abs(bleed_bridge + bleed_solid - 525.874) <= 0.01);
    } else {
        // Orca: The deliberate second layer must still be deposited, on the layer above the bridge.
        CHECK(internal_bridge_area_mm2(internal_bridge_surfaces(object, bleed_layer, stInternalBridge)) >= 71.6);
    }

    // Orca: A bridge that spans a real unsupported area must survive in every mode: the dedup may
    // not eat the genuine span, and the doubling modes may not be replaced by it either.
    const size_t spanning_layer = internal_bridge_layer_at_z(object, 3.4);
    CHECK(spanning_layer == 16);
    REQUIRE(spanning_layer < object.layer_count());
    CHECK(internal_bridge_area_mm2(internal_bridge_surfaces(object, spanning_layer, stInternalBridge)) >= 71.6);

    // Orca: In every mode the bridges must still stay inside the domain they belong to.
    for (size_t i = 0; i < object.layer_count(); ++i)
        for (const LayerRegion *region : object.get_layer(i)->regions()) {
            const Polygons boundary = to_polygons(region->fill_internal_bridge_expolygons);
            Polygons bridges;
            polygons_append(bridges, to_polygons(region->fill_surfaces.filter_by_type(stInternalBridge)));
            if (bridges.empty() || boundary.empty())
                continue;
            CAPTURE(i, mode);
            CHECK(internal_bridge_area_mm2(diff(bridges, boundary)) <= 0.64);
        }
}

// Orca: The repeat removal compares bridge FACES by XY, not by region index. A modifier volume can
// push the same footprint into a different printing region on the layer above (a modifier starting
// between the two layers shifts one of them to region 1), and the user's contract is about the same
// XY, no matter which region collects it: two bridges on one footprint are still two bridges.
//
// This matters because the pass unions the bridges of ALL lower regions before intersecting, so a
// cross-region pair is deduped exactly like a same-region pair. The test asserts that on a fixture
// where the overlap really is cross-region, so a future "compare region r to region r" rewrite
// cannot silently pass.
TEST_CASE("Internal bridge repeat is removed across printing regions", "[PrintObject][InternalBridge][ExtraBridgeLayer][Regression]")
{
    auto config = internal_bridge_no_extra_layer_config(false);
    DynamicPrintConfig modifier_config = DynamicPrintConfig::full_print_config();
    modifier_config.apply(config);
    modifier_config.set_deserialize_strict({{"sparse_infill_density", "60%"},
                                            {"sparse_infill_pattern", "grid"},
                                            {"wall_loops", 2}});
    // Orca: external_bridge_only, because the user requirement covers that mode too and it is the
    // mode that must dedup without any extra internal bridge layer.
    config.set_deserialize_strict({{"enable_extra_bridge_layer", "external_bridge_only"}});

    Print print;
    Model model;
    Slic3r::Test::init_print({internal_bridge_step()}, print, model, config, nullptr, false);
    // Orca: 0.2 mm tall, starting exactly one layer below the bleed layer, so layer 11 is in the
    // modifier's region and layer 12 is not. That is the index-shifted case this test is about.
    TriangleMesh modifier = make_cube(30, 24, 0.2);
    modifier.translate(0, 0, 2.2);
    model.objects.front()->add_volume(TriangleMesh(modifier), ModelVolumeType::PARAMETER_MODIFIER, false)->config.apply(modifier_config);
    print.apply(model, config);
    print.process();

    const PrintObject &object = *print.objects().front();
    REQUIRE(object.num_printing_regions() == 2);

    const size_t bleed_layer = internal_bridge_layer_at_z(object, 2.6);
    CHECK(bleed_layer == 12);
    REQUIRE(bleed_layer >= 1);

    // Orca: REQUIRE the fixture to be the case it claims: the lower bridge and the upper one must
    // sit in DIFFERENT region indices, otherwise this test would pass without testing anything.
    auto bridge_region = [&object](size_t layer_id, double &area) -> long {
        long found = -1;
        for (size_t r = 0; r < object.get_layer(layer_id)->regions().size(); ++r) {
            const Polygons polys = union_(to_polygons(
                object.get_layer(layer_id)->regions()[r]->fill_surfaces.filter_by_type(stInternalBridge)));
            if (polys.empty())
                continue;
            CAPTURE(layer_id, r);
            REQUIRE(found < 0); // exactly one region carries the bridge on these layers
            found = long(r);
            area  = internal_bridge_area_mm2(polys);
        }
        return found;
    };
    double lower_area = 0., upper_area = 0.;
    const long lower_region = bridge_region(bleed_layer - 1, lower_area);
    const long upper_region = bridge_region(bleed_layer, upper_area);
    REQUIRE(lower_region >= 0);
    REQUIRE(upper_region >= 0);
    CAPTURE(lower_region, upper_region, lower_area, upper_area);
    CHECK(lower_region != upper_region);

    // Orca: And the overlap must be gone even though it was cross-region. Measured on this fixture:
    // with the pass off, layer 12 keeps 77.2251 mm2 of bridge of which 32.3142 mm2 lies exactly on
    // the 541.396 mm2 bridge of layer 11 (a different region); with the pass on, layer 12 keeps
    // 44.911 mm2 and 0.0 mm2 of it overlaps any lower bridge. The remainder is the real span.
    const Polygons upper_bridge = union_(to_polygons(
        object.get_layer(bleed_layer)->regions()[size_t(upper_region)]->fill_surfaces.filter_by_type(stInternalBridge)));
    Polygons lower_all;
    for (const LayerRegion *region : object.get_layer(bleed_layer - 1)->regions())
        polygons_append(lower_all, to_polygons(region->fill_surfaces.filter_by_type(stInternalBridge)));
    const double cross_overlap = internal_bridge_area_mm2(intersection(upper_bridge, union_(lower_all)));
    CAPTURE(cross_overlap);
    CHECK(cross_overlap <= 0.02 * internal_bridge_area_mm2(upper_bridge));
    CHECK(internal_bridge_area_mm2(upper_bridge) <= 50.5);

    // Orca: Area conservation, so "removed" cannot mean "printed as bridge elsewhere". On this
    // layer the bridge + internal solid of the region that carries the bridge must still be the
    // 520.6080 mm2 measured with the pass off (77.2251 + 443.3830): the footprint moved from
    // bridge to solid, nothing appeared and nothing vanished.
    const double solid_area = internal_bridge_area_mm2(union_(to_polygons(
        object.get_layer(bleed_layer)->regions()[size_t(upper_region)]->fill_surfaces.filter_by_type(stInternalSolid))));
    CAPTURE(solid_area);
    CHECK(internal_bridge_area_mm2(upper_bridge) + solid_area >= 520.5);
    CHECK(internal_bridge_area_mm2(upper_bridge) + solid_area <= 520.7);
}
