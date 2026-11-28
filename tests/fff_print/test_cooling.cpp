#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/CoolingBuffer.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Surface.hpp"
#include <map>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

// The fan is held off for the first close_fan_the_first_x_layers layers, so an explicit
// fan-off command is emitted.
TEST_CASE("Fan is held off for the initial layers", "[Cooling]")
{
    const std::string gcode = slice({ cube(20) }, {
        { "cooling",                      true },
        { "close_fan_the_first_x_layers", 5 },
    });
    CHECK(gcode.find("M106 S0") != std::string::npos);
}

// The cooling pass resolves and strips its internal speed placeholders; none leak into
// the final G-code.
TEST_CASE("Cooling consumes its internal speed markers", "[Cooling]")
{
    const std::string gcode = slice({ cube(20) }, { { "layer_height", 0.2 } });
    CHECK(gcode.find(";_EXTRUDE_SET_SPEED") == std::string::npos);
}

TEST_CASE("Arc fitting remains active with overhang fan control", "[Cooling][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_arc_fitting",             "enabled" },
        { "enable_overhang_bridge_fan",     true },
        { "enable_overhang_speed",          true },
        { "initial_layer_print_height",     0.2 },
        { "layer_height",                   0.2 },
        { "slow_down_for_layer_cooling",    false },
    });
    config.set_key_value("fan_max_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("fan_min_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{100});
    config.set_key_value("overhang_fan_threshold", new ConfigOptionEnumsGeneric{Overhang_threshold_2_4});
    config.set_key_value("layer_change_gcode", new ConfigOptionString{";TEST_LAYER=[layer_num]"});

    const auto has_arc_after_first_layer = [](const std::string &gcode) {
        std::istringstream input(gcode);
        std::string line;
        size_t layers_seen = 0;
        while (std::getline(input, line)) {
            if (line.rfind(";TEST_LAYER=", 0) == 0)
                ++layers_seen;
            else if (layers_seen > 1 && (line.rfind("G2 ", 0) == 0 || line.rfind("G3 ", 0) == 0))
                return true;
        }
        return false;
    };
    const auto has_arc_with_overhang_fan = [](const std::string &gcode) {
        std::istringstream input(gcode);
        std::string line;
        bool overhang_fan_active = false;
        bool outer_wall = false;
        while (std::getline(input, line)) {
            if (line.rfind("M106 S255", 0) == 0)
                overhang_fan_active = true;
            else if (line.rfind("M106 S0", 0) == 0 || line.rfind("M107", 0) == 0)
                overhang_fan_active = false;
            else if (line.rfind(";TYPE:", 0) == 0)
                outer_wall = line == ";TYPE:Outer wall";
            else if (overhang_fan_active && outer_wall && (line.rfind("G2 ", 0) == 0 || line.rfind("G3 ", 0) == 0))
                return true;
        }
        return false;
    };

    const std::string with_overhang_speed = slice({ make_cylinder(25.0, 10.0, 2.0 * PI / 72.0) }, config);
    CHECK(has_arc_after_first_layer(with_overhang_speed));

    config.set_deserialize_strict({ { "enable_overhang_speed", false } });
    const std::string without_overhang_speed = slice({ make_cylinder(25.0, 10.0, 2.0 * PI / 72.0) }, config);
    CHECK(has_arc_after_first_layer(without_overhang_speed));

    const std::string uniform_overhang_gcode = slice({ make_sphere(50.0, 2.0 * PI / 72.0) }, config);
    CHECK(has_arc_with_overhang_fan(uniform_overhang_gcode));
}

TEST_CASE("Overhang fan transitions do not depend on overhang speed", "[Cooling][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "bridge_speed",                   2.0 },
        { "enable_arc_fitting",             "disabled" },
        { "enable_overhang_bridge_fan",     true },
        { "enable_overhang_speed",          false },
        { "initial_layer_print_height",     0.3 },
        { "inner_wall_speed",              30.0 },
        { "layer_height",                   0.3 },
        { "outer_wall_speed",              30.0 },
        { "overhang_1_4_speed",            "30" },
        { "overhang_2_4_speed",            "29" },
        { "overhang_3_4_speed",             "6" },
        { "overhang_4_4_speed",             "3" },
        { "slow_down_for_layer_cooling",    false },
        { "slowdown_for_curled_perimeters", false },
    });
    config.set_key_value("fan_max_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("fan_min_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{100});
    config.set_key_value("overhang_fan_threshold", new ConfigOptionEnumsGeneric{Overhang_threshold_2_4});
    config.set_key_value("layer_change_gcode", new ConfigOptionString{";TEST_LAYER_Z=[layer_z]"});

    const auto fan_commands = [](const std::string &gcode) {
        std::vector<std::pair<std::string, std::string>> commands;
        std::istringstream input(gcode);
        std::string layer;
        std::string line;
        while (std::getline(input, line)) {
            if (line.rfind(";TEST_LAYER_Z=", 0) == 0)
                layer = line;
            else if (!layer.empty() && (line.rfind("M106", 0) == 0 || line.rfind("M107", 0) == 0))
                commands.emplace_back(layer, line);
        }
        return commands;
    };
    const auto feedrates = [](const std::string &gcode) {
        std::vector<std::string> values;
        std::istringstream input(gcode);
        std::string word;
        while (input >> word)
            if (!word.empty() && word.front() == 'F')
                values.push_back(word);
        return values;
    };

    constexpr double sphere_radius = 50.0; // 100 mm diameter.
    const std::string without_speed_gcode = slice({make_sphere(sphere_radius, PI / 24.0)}, config);
    config.set_deserialize_strict({{"enable_overhang_speed", true}});
    const std::string with_speed_gcode = slice({make_sphere(sphere_radius, PI / 24.0)}, config);

    const auto without_speed_fan = fan_commands(without_speed_gcode);
    const auto with_speed_fan = fan_commands(with_speed_gcode);
    const auto without_speed_feedrates = feedrates(without_speed_gcode);
    const auto with_speed_feedrates = feedrates(with_speed_gcode);

    REQUIRE_FALSE(without_speed_fan.empty());
    REQUIRE(std::any_of(without_speed_fan.begin(), without_speed_fan.end(),
                        [](const auto &command) { return command.second.find("S255") != std::string::npos; }));
    REQUIRE(with_speed_feedrates != without_speed_feedrates);
    CHECK(with_speed_fan == without_speed_fan);
}

// Speeds of the extruding moves, in mm/s, in print order.
static std::vector<double> extrusion_speeds(const std::string &gcode, const DynamicPrintConfig &config)
{
    std::vector<double> speeds;
    GCodeReader parser;
    parser.apply_config(config);
    parser.parse_buffer(gcode, [&speeds](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self) && line.dist_XY(self) > 0.)
            speeds.push_back(line.new_F(self) / MM_PER_MIN);
    });
    return speeds;
}

// A line that only sets the feed rate takes no time, however the extrusion distances are written.
TEST_CASE("Layer cooling slowdown does not depend on relative extrusion distances", "[Cooling][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        // High enough that the defaults print at their own speeds.
        { "filament_max_volumetric_speed", 100 },
        { "initial_layer_print_height",    0.3 },
        { "layer_height",                  0.3 },
        { "slow_down_for_layer_cooling",   true },
        // Longer than the cube's layers take, so they are slowed down.
        { "slow_down_layer_time",          10 },
        { "slow_down_min_speed",           1 },
    });
    config.set_deserialize_strict({ { "use_relative_e_distances", false } });
    const std::vector<double> absolute = extrusion_speeds(slice({ cube(20) }, config), config);
    config.set_deserialize_strict({ { "use_relative_e_distances", true } });
    const std::vector<double> relative = extrusion_speeds(slice({ cube(20) }, config), config);

    REQUIRE(relative.size() == absolute.size());
    double largest_difference = 0.;
    for (size_t i = 0; i < relative.size(); ++i)
        largest_difference = std::max(largest_difference, std::abs(relative[i] - absolute[i]));
    // Feed rates are written in whole mm/min.
    CHECK_THAT(largest_difference, Catch::Matchers::WithinAbs(0., 1. / MM_PER_MIN));
}

// Per layer: its time as the G-code preview estimates it, with acceleration, and the part cooling fan speed of its first
// extrusion, in percent.
struct EstimatedLayer
{
    double time = 0.;
    float  fan  = -1.f;
};

static std::vector<EstimatedLayer> estimated_layers(const TriangleMesh &mesh, const DynamicPrintConfig &config,
                                                    std::string *gcode_text = nullptr)
{
    Print print;
    Model model;
    init_print({ mesh }, print, model, config);
    GCodeProcessorResult result;
    std::string text = gcode(print, &result);
    if (gcode_text)
        *gcode_text = std::move(text);
    std::vector<EstimatedLayer> layers;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        if (move.layer_id >= layers.size())
            layers.resize(move.layer_id + 1);
        EstimatedLayer &layer = layers[move.layer_id];
        layer.time += move.time[static_cast<size_t>(PrintEstimatedStatistics::ETimeMode::Normal)];
        if (layer.fan < 0.f && move.type == EMoveType::Extrude)
            layer.fan = move.fan_speed;
    }
    return layers;
}

// Print speeds that a low acceleration never reaches on a small part, so its layers take several times their time at the feed rates.
static DynamicPrintConfig slowly_accelerating_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "filament_max_volumetric_speed",      100 },
        { "initial_layer_print_height",         0.3 },
        { "inner_wall_speed",                   200 },
        { "internal_solid_infill_speed",        200 },
        { "layer_height",                       0.3 },
        { "machine_max_acceleration_extruding", "300,300" },
        { "outer_wall_speed",                   200 },
        { "sparse_infill_speed",                200 },
        { "top_surface_speed",                  200 },
    });
    return config;
}

TEST_CASE("Layer cooling slowdown reaches the minimum layer time with acceleration", "[Cooling]")
{
    // A little longer than most layers of the cube take, so they slow down just a little and still accelerate a lot.
    constexpr double minimum_layer_time = 8.;
    DynamicPrintConfig config = slowly_accelerating_config();
    config.set_deserialize_strict({
        { "slow_down_for_layer_cooling", false },
        { "slow_down_layer_time",        minimum_layer_time },
        { "slow_down_min_speed",         1 },
    });
    const std::vector<EstimatedLayer> unslowed = estimated_layers(cube(10), config);
    config.set_deserialize_strict({ { "slow_down_for_layer_cooling", true } });
    const std::vector<EstimatedLayer> slowed = estimated_layers(cube(10), config);
    REQUIRE(slowed.size() == unslowed.size());

    // Layer 0 holds the start G-code, and the last layer the end G-code.
    const size_t quick = std::count_if(unslowed.begin() + 1, unslowed.end() - 1,
                                       [](const EstimatedLayer &layer) { return layer.time < minimum_layer_time; });
    REQUIRE(quick > 10);
    for (size_t i = 1; i + 1 < slowed.size(); ++i) {
        if (unslowed[i].time < minimum_layer_time) {
            CHECK(slowed[i].time > 0.99 * minimum_layer_time);
            CHECK(slowed[i].time < 1.03 * minimum_layer_time);
        } else
            CHECK_THAT(slowed[i].time, Catch::Matchers::WithinRel(unslowed[i].time, 0.001));
    }
}

TEST_CASE("Fan speed follows the layer time with acceleration", "[Cooling]")
{
    constexpr double full_fan_time    = 2.;
    constexpr double minimum_fan_time = 100.;
    constexpr double full_fan         = 100.;
    constexpr double minimum_fan      = 20.;
    DynamicPrintConfig config = slowly_accelerating_config();
    config.set_deserialize_strict({
        { "close_fan_the_first_x_layers", 0 },
        { "fan_cooling_layer_time",       minimum_fan_time },
        { "fan_max_speed",                full_fan },
        { "fan_min_speed",                minimum_fan },
        { "full_fan_speed_layer",         0 },
        // A pause before the first extrusion of each layer, which counts as in the Preview.
        { "layer_change_gcode",           "G4 S3" },
        { "slow_down_for_layer_cooling",  false },
        { "slow_down_layer_time",         full_fan_time },
    });
    // The low acceleration is the machine limit, or the one the start G-code sets when the layers set none of their own.
    if (GENERATE(false, true))
        config.set_deserialize_strict({
            { "default_acceleration",               0 },
            { "machine_max_acceleration_extruding", "20000,20000" },
            { "machine_start_gcode",                "G90\n  M204 S300 ; indented\nG92 E0" },
        });
    const std::vector<EstimatedLayer> layers = estimated_layers(cube(20), config);
    REQUIRE(layers.size() > 4);
    for (size_t i = 1; i + 1 < layers.size(); ++i) {
        REQUIRE(layers[i].time > full_fan_time);
        REQUIRE(layers[i].time < minimum_fan_time);
        const double t = (layers[i].time - full_fan_time) / (minimum_fan_time - full_fan_time);
        // The cooling rounds the speed down to a whole percent, and M106 takes it in 1/255 steps.
        CHECK_THAT(layers[i].fan, Catch::Matchers::WithinAbs(t * minimum_fan + (1. - t) * full_fan, 1.5));
    }
}

// The speeds of the outer wall extrusions, in mm/s.
static std::vector<double> outer_wall_speeds(const std::string &gcode, const DynamicPrintConfig &config)
{
    std::vector<double> speeds;
    bool outer_wall = false;
    GCodeReader parser;
    parser.apply_config(config);
    parser.parse_buffer(gcode, [&speeds, &outer_wall](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.rfind("TYPE:", 0) == 0)
            outer_wall = comment == "TYPE:Outer wall";
        if (outer_wall && line.extruding(self) && line.dist_XY(self) > 0.)
            speeds.push_back(line.new_F(self) / MM_PER_MIN);
    });
    return speeds;
}

TEST_CASE("Layer cooling slowdown keeps outer walls out of the resonance avoidance range", "[Cooling]")
{
    constexpr double min_speed = 40.;
    constexpr double max_speed = 90.;
    // With inner walls and infill to slow down instead, the outer walls stay at the top of the range. A lone outer
    // wall cannot pass its time on, so it drops to the bottom; its layers are quick, hence the shorter layer time.
    const bool   lone_wall          = GENERATE(false, true);
    const double held_speed         = lone_wall ? min_speed : max_speed;
    const double minimum_layer_time = lone_wall ? 2. : 10.;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "bottom_shell_layers",           lone_wall ? 0 : 3 },
        { "dont_slow_down_outer_wall",     false },
        { "enable_arc_fitting",            "disabled" },
        { "enable_overhang_speed",         false },
        { "filament_max_volumetric_speed", 100 },
        { "initial_layer_print_height",    0.3 },
        // Below the range, so resonance avoidance leaves it alone.
        { "initial_layer_speed",           30 },
        { "layer_height",                  0.3 },
        { "max_resonance_avoidance_speed", max_speed },
        { "min_resonance_avoidance_speed", min_speed },
        { "outer_wall_speed",              200 },
        { "slow_down_for_layer_cooling",   true },
        { "slow_down_layer_time",          minimum_layer_time },
        { "slow_down_layers",              0 },
        { "slow_down_min_speed",           10 },
        { "sparse_infill_density",         lone_wall ? "0%" : "15%" },
        { "top_shell_layers",              lone_wall ? 0 : 3 },
        { "wall_loops",                    lone_wall ? 1 : 3 },
    });

    // A cone's layers shrink upwards, so the slowdown deepens through the range.
    std::string without_gcode;
    std::string with_gcode;
    config.set_deserialize_strict({ { "resonance_avoidance", false } });
    const std::vector<EstimatedLayer> without = estimated_layers(make_cone(15., 15.), config, &without_gcode);
    config.set_deserialize_strict({ { "resonance_avoidance", true } });
    const std::vector<EstimatedLayer> with = estimated_layers(make_cone(15., 15.), config, &with_gcode);

    std::vector<double> without_in_range;
    std::vector<double> with_in_range;
    bool                held = false;
    for (double speed : outer_wall_speeds(without_gcode, config))
        if (speed > min_speed + 0.5 && speed < max_speed - 0.5)
            without_in_range.push_back(speed);
    for (double speed : outer_wall_speeds(with_gcode, config)) {
        if (speed > min_speed + 0.5 && speed < max_speed - 0.5)
            with_in_range.push_back(speed);
        held |= std::abs(speed - held_speed) < 0.1;
    }
    REQUIRE_FALSE(without_in_range.empty());
    CHECK(with_in_range.empty());
    CHECK(held);

    // No layer gets shorter than the slowdown made it without resonance avoidance, within the slowdown's tolerance.
    REQUIRE(with.size() == without.size());
    for (size_t i = 0; i < with.size(); ++i)
        CHECK(with[i].time > 0.99 * std::min(without[i].time, minimum_layer_time));
}

// ---------------------------------------------------------------------------------------------
// External bridge fan
//
// `external_bridge_fan_speed` is a per-filament fan speed for external bridge infill (erBridgeInfill)
// alone. -1, the default, keeps the old behaviour: the bridge inherits whatever the overhang fan
// resolves to. An explicit 0..100 takes the bridge over on its own, below the layer fan included and
// whatever the overhang threshold says, while every other role and every layer gate (the first-layer
// override, the first-N-layers blackout, the "force cooling for overhangs and bridges" switch) keeps
// working as it did.

// One extrusion of the final G-code, as the Preview reads it: its role, its layer, and the part
// cooling fan speed in whole percent it prints at.
struct Extrusion
{
    ExtrusionRole role     = erNone;
    unsigned int  layer_id = 0;
    int           fan      = 0;
};

struct SlicedPrint
{
    std::string            gcode;
    std::vector<Extrusion> extrusions;
};

// Slice `test_mesh` and return the final G-code with the extrusions the Preview reads out of it.
static SlicedPrint slice_extrusions(TestMesh test_mesh, const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    init_print({ mesh(test_mesh) }, print, model, config);
    GCodeProcessorResult result;
    SlicedPrint          sliced;
    sliced.gcode = gcode(print, &result);
    for (const GCodeProcessorResult::MoveVertex &move : result.moves)
        if (move.type == EMoveType::Extrude)
            // M106 addresses the fan in whole percents and the Preview turns them back into a
            // percentage, so rounding recovers exactly the percentage the slicer commanded.
            sliced.extrusions.push_back({ move.extrusion_role, move.layer_id, int(std::lround(move.fan_speed)) });
    return sliced;
}

// How many extrusions of `role` the print has, and the distinct fan speeds they run at, ascending.
struct RoleFans
{
    size_t           moves = 0;
    std::vector<int> fans;
};

static RoleFans fans_of_role(const std::vector<Extrusion> &extrusions, ExtrusionRole role)
{
    RoleFans out;
    for (const Extrusion &extrusion : extrusions) {
        if (extrusion.role != role)
            continue;
        ++ out.moves;
        if (std::find(out.fans.begin(), out.fans.end(), extrusion.fan) == out.fans.end())
            out.fans.push_back(extrusion.fan);
    }
    std::sort(out.fans.begin(), out.fans.end());
    return out;
}

// Every role but `excluded`, with each fan speed its extrusions run at and how many run there: a
// compact fingerprint of the fan behaviour an external bridge fan must leave alone.
static std::vector<std::string> fan_histogram(const std::vector<Extrusion> &extrusions, ExtrusionRole excluded)
{
    std::map<std::pair<int, int>, size_t> counts;
    for (const Extrusion &extrusion : extrusions)
        if (extrusion.role != excluded)
            ++ counts[{ int(extrusion.role), extrusion.fan }];
    std::vector<std::string> histogram;
    histogram.reserve(counts.size());
    for (const auto &entry : counts)
        histogram.push_back(ExtrusionEntity::role_to_string(ExtrusionRole(entry.first.first)) + " at " +
                            std::to_string(entry.first.second) + "% x " + std::to_string(entry.second));
    return histogram;
}

// The part cooling fan and tool change commands of a print, in the order the G-code prints them. This is
// the output a filament that forces no new cooling has to leave alone, command for command.
static std::vector<std::string> fan_and_tool_commands(const std::string &gcode)
{
    std::vector<std::string> commands;
    std::istringstream       lines(gcode);
    for (std::string line; std::getline(lines, line);) {
        if (line.rfind("M106", 0) == 0 || line.rfind("M107", 0) == 0 || line.rfind("T", 0) == 0)
            commands.push_back(line);
    }
    return commands;
}

// The part cooling fan speeds `gcode` asks for, in order, as whole percents. An auxiliary fan command is
// left out: it addresses a different fan.
static std::vector<int> fan_percents(const std::string &gcode)
{
    std::vector<int> percents;
    std::istringstream lines(gcode);
    for (std::string line; std::getline(lines, line);) {
        if ((line.rfind("M106", 0) != 0 && line.rfind("M107", 0) != 0) || line.find("P2") != std::string::npos)
            continue;
        const size_t s = line.find('S');
        percents.push_back(s == std::string::npos ? 0 : int(std::lround(std::atof(line.c_str() + s + 1) * 100. / 255.)));
    }
    return percents;
}

// A G-code generator set up by hand, so the cooling buffer can be handed a written-out layer instead of
// sliced geometry. Without a Print the filament config index of a filament is the filament id itself,
// which is the column the per-filament fan options are stored at.
class CoolingTestGCode : public GCode
{
public:
    CoolingTestGCode(const DynamicPrintConfig &config, unsigned int filaments)
    {
        m_config.apply(config);
        std::vector<unsigned int> extruders;
        for (unsigned int filament = 0; filament < filaments; ++filament)
            extruders.emplace_back(filament);
        m_writer.apply_print_config(m_config);
        m_writer.set_extruders(extruders);
        m_writer.init_extruder(0);
    }
};

// The bridges of a sliced print, per filament: the speeds they run at, ascending and de-duplicated.
static std::map<unsigned int, std::vector<int>> bridge_fans_by_filament(const GCodeProcessorResult &result)
{
    std::map<unsigned int, std::vector<int>> fans;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erBridgeInfill)
            continue;
        std::vector<int> &filament_fans = fans[move.extruder_id];
        const int         fan           = int(std::lround(move.fan_speed));
        if (std::find(filament_fans.begin(), filament_fans.end(), fan) == filament_fans.end())
            filament_fans.push_back(fan);
    }
    for (auto &filament : fans)
        std::sort(filament.second.begin(), filament.second.end());
    return fans;
}

// Run `layer_gcode` through the cooling buffer as one layer and return what it emits.
static std::string cool_layer(const DynamicPrintConfig &config, const std::string &layer_gcode, size_t layer_id,
                             unsigned int filaments)
{
    CoolingTestGCode gcodegen(config, filaments);
    CoolingBuffer     buffer(gcodegen);
    buffer.set_current_extruder(0, 0);
    return buffer.process_layer(std::string(layer_gcode), layer_id, true);
}

// Two filaments whose layer fan is flat at 60%. `external_bridge`, `bridge_cooling` and `overhang` are the
// per-filament lists the case under test varies.
static DynamicPrintConfig two_filament_fan_config(const char *external_bridge, const char *bridge_cooling = "1,1",
                                                  const char *overhang = "80,80")
{
    return multifilament_config(2, {
        { "close_fan_the_first_x_layers", "0,0" },
        { "enable_arc_fitting",           "disabled" },
        { "enable_overhang_bridge_fan",   bridge_cooling },
        { "external_bridge_fan_speed",    external_bridge },
        // No cooling threshold to reach, so the buffer never has to time the layer: the fan sits at
        // the minimum speed, which is the 60% this case starts from.
        { "fan_cooling_layer_time",       "0,0" },
        { "reduce_fan_stop_start_freq",   "1,1" },
        { "fan_kickstart",                0. },
        { "fan_max_speed",                "60,60" },
        { "fan_min_speed",                "60,60" },
        { "fan_speedup_time",             0. },
        { "full_fan_speed_layer",         "0,0" },
        { "initial_layer_fan_speed",      "-1,-1" },
        { "overhang_fan_speed",           overhang },
        { "part_cooling_fan_min_pwm",     0 },
        { "slow_down_for_layer_cooling",  false },
        { "slow_down_layer_time",         "0,0" },
    });
}

// The cooling a bridge fan case starts from: a flat 60% layer fan (both ends of the layer-time
// interpolation at 60%, over a window every layer falls in), the overhang fan at `overhang_fan`, and
// kick-start, speed-up and the PWM floor off, so a commanded percentage is exactly what the slicer
// decided. Layer-cooling slowdowns are off so the layer times cannot bend the speeds.
static DynamicPrintConfig bridge_fan_config(int overhang_fan)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "close_fan_the_first_x_layers", 0 },
        { "enable_arc_fitting",           "disabled" },
        { "enable_overhang_bridge_fan",   true },
        { "fan_cooling_layer_time",       1000. },
        { "fan_kickstart",                0. },
        { "fan_max_speed",                60. },
        { "fan_min_speed",                60. },
        { "fan_speedup_time",             0. },
        { "full_fan_speed_layer",         0 },
        { "initial_layer_fan_speed",      -1 },
        { "overhang_fan_speed",           overhang_fan },
        { "part_cooling_fan_min_pwm",     0 },
        { "slow_down_for_layer_cooling",  false },
        { "slow_down_layer_time",         0. },
    });
    return config;
}

// Set the per-filament external bridge fan speed. The option has to be part of this build: without it
// the case would slice the same mesh as its reference and compare two runs that are equal because
// neither of them wrote the key.
static void set_external_bridge_fan_speed(DynamicPrintConfig &config, int percent)
{
    REQUIRE(print_config_def.has("external_bridge_fan_speed"));
    config.set_deserialize_strict({ { "external_bridge_fan_speed", percent } });
}

// The contract in one case: 30% takes the bridge off the layer fan even though it is below it, which
// the old "the overhang fan only wins above the layer fan" rule could never do.
TEST_CASE("An explicit external bridge fan speed below the layer fan still reaches the bridge",
          "[Cooling][ExternalBridgeFan]")
{
    const bool overhang_speed = GENERATE(false, true);
    INFO("enable_overhang_speed " << overhang_speed);

    // 40% is below the 60% layer fan, so the old rule leaves a bridge at 60%.
    DynamicPrintConfig reference_config = bridge_fan_config(40);
    reference_config.set_deserialize_strict({ { "enable_overhang_speed", overhang_speed } });
    const SlicedPrint reference = slice_extrusions(TestMesh::bridge, reference_config);

    DynamicPrintConfig config = reference_config;
    set_external_bridge_fan_speed(config, 30);
    const SlicedPrint external_30 = slice_extrusions(TestMesh::bridge, config);

    const RoleFans reference_bridges = fans_of_role(reference.extrusions, erBridgeInfill);
    const RoleFans explicit_bridges  = fans_of_role(external_30.extrusions, erBridgeInfill);
    // A real external bridge has to be printed for any of this to mean anything.
    REQUIRE(reference_bridges.moves > 0);
    REQUIRE(explicit_bridges.moves == reference_bridges.moves);
    CHECK(reference_bridges.fans == std::vector<int>{ 60 });
    INFO("bridge fan " << explicit_bridges.fans.front() << "% over " << explicit_bridges.moves << " extrusions");
    CHECK(explicit_bridges.fans == std::vector<int>{ 30 });
    // The marker is for the cooling buffer only: none of it survives into the G-code.
    CHECK(external_30.gcode.find("_EXTERNAL_BRIDGE") == std::string::npos);
    // The bridge moves in and out of the layer fan's 60% without disturbing anything else.
    CHECK(fan_histogram(external_30.extrusions, erBridgeInfill) == fan_histogram(reference.extrusions, erBridgeInfill));
}

// An explicit zero is a real value, not "unset": the bridge prints with the fan off while the rest of
// the layer keeps its 60%.
TEST_CASE("An explicit external bridge fan speed of zero turns the fan off on external bridges",
          "[Cooling][ExternalBridgeFan]")
{
    DynamicPrintConfig reference_config = bridge_fan_config(80);
    const SlicedPrint reference = slice_extrusions(TestMesh::bridge, reference_config);

    DynamicPrintConfig config = reference_config;
    set_external_bridge_fan_speed(config, 0);
    const SlicedPrint external_0 = slice_extrusions(TestMesh::bridge, config);

    const RoleFans reference_bridges = fans_of_role(reference.extrusions, erBridgeInfill);
    const RoleFans explicit_bridges  = fans_of_role(external_0.extrusions, erBridgeInfill);
    REQUIRE(reference_bridges.moves > 0);
    REQUIRE(explicit_bridges.moves == reference_bridges.moves);
    // Without the option the bridge takes the 80% overhang fan.
    CHECK(reference_bridges.fans == std::vector<int>{ 80 });
    INFO("bridge fan " << explicit_bridges.fans.front() << "% over " << explicit_bridges.moves << " extrusions");
    CHECK(explicit_bridges.fans == std::vector<int>{ 0 });
    CHECK(fan_histogram(external_0.extrusions, erBridgeInfill) == fan_histogram(reference.extrusions, erBridgeInfill));
}

// The default keeps the old rule: the overhang fan wins on the bridge when it is above the layer fan,
// and the layer fan wins when it is below.
TEST_CASE("The default external bridge fan speed keeps following the overhang fan", "[Cooling][ExternalBridgeFan]")
{
    const int overhang_fan = GENERATE(80, 40);
    INFO("overhang fan " << overhang_fan);

    DynamicPrintConfig reference_config = bridge_fan_config(overhang_fan);
    const SlicedPrint reference = slice_extrusions(TestMesh::bridge, reference_config);

    DynamicPrintConfig config = reference_config;
    set_external_bridge_fan_speed(config, -1);
    const SlicedPrint inherited = slice_extrusions(TestMesh::bridge, config);

    const RoleFans reference_bridges = fans_of_role(reference.extrusions, erBridgeInfill);
    const RoleFans inherited_bridges = fans_of_role(inherited.extrusions, erBridgeInfill);
    REQUIRE(reference_bridges.moves > 0);
    REQUIRE(inherited_bridges.moves == reference_bridges.moves);
    CHECK(reference_bridges.fans == std::vector<int>{ std::max(overhang_fan, 60) });
    CHECK(inherited_bridges.fans == reference_bridges.fans);
    CHECK(fan_histogram(inherited.extrusions, erBridgeInfill) == fan_histogram(reference.extrusions, erBridgeInfill));
    // A guard on this fixture only: the two runs differ in nothing but the key being written, so equal
    // fan and tool change commands say the default value adds no command here. They do not prove the
    // print is unchanged against an older build of the slicer, and neither run is a reference artifact.
    CHECK(fan_and_tool_commands(inherited.gcode) == fan_and_tool_commands(reference.gcode));
}

// The overhang threshold decides which walls are cooled, not what an explicit external bridge fan
// does to the bridge: 0% and 95% both leave the bridge at its own value.
TEST_CASE("The overhang threshold does not gate an explicit external bridge fan speed", "[Cooling][ExternalBridgeFan]")
{
    const std::string threshold = GENERATE(std::string("0%"), std::string("95%"));
    INFO("overhang_fan_threshold " << threshold);

    DynamicPrintConfig reference_config = bridge_fan_config(80);
    reference_config.set_deserialize_strict({ { "overhang_fan_threshold", threshold } });
    const SlicedPrint reference = slice_extrusions(TestMesh::bridge, reference_config);

    DynamicPrintConfig config = reference_config;
    set_external_bridge_fan_speed(config, 30);
    const SlicedPrint external_30 = slice_extrusions(TestMesh::bridge, config);

    const RoleFans reference_bridges = fans_of_role(reference.extrusions, erBridgeInfill);
    const RoleFans explicit_bridges  = fans_of_role(external_30.extrusions, erBridgeInfill);
    REQUIRE(reference_bridges.moves > 0);
    REQUIRE(explicit_bridges.moves == reference_bridges.moves);
    CHECK(reference_bridges.fans == std::vector<int>{ 80 });
    INFO("bridge fan " << explicit_bridges.fans.front() << "% over " << explicit_bridges.moves << " extrusions");
    CHECK(explicit_bridges.fans == std::vector<int>{ 30 });
}

// Overhang walls and internal bridges keep the overhang fan: only the external bridge role is split
// off. The deck mesh prints both roles in one slice.
TEST_CASE("An explicit external bridge fan leaves internal bridges and overhang walls alone",
          "[Cooling][ExternalBridgeFan]")
{
    DynamicPrintConfig reference_config = bridge_fan_config(80);
    // 0% cools every outer and overhang wall, so the overhang fan is visible on more than bridges.
    reference_config.set_deserialize_strict({ { "overhang_fan_threshold", "0%" } });
    const SlicedPrint reference = slice_extrusions(TestMesh::bridge, reference_config);

    DynamicPrintConfig config = reference_config;
    set_external_bridge_fan_speed(config, 30);
    const SlicedPrint external_30 = slice_extrusions(TestMesh::bridge, config);

    const RoleFans internal = fans_of_role(external_30.extrusions, erInternalBridgeInfill);
    INFO("internal bridge fan " << (internal.fans.empty() ? -1 : internal.fans.front()) << "% over " << internal.moves
                                << " extrusions");
    REQUIRE(internal.moves > 0);
    CHECK(internal.fans == std::vector<int>{ 80 });
    CHECK(fans_of_role(reference.extrusions, erInternalBridgeInfill).fans == internal.fans);
    CHECK(fans_of_role(external_30.extrusions, erOverhangPerimeter).fans ==
          fans_of_role(reference.extrusions, erOverhangPerimeter).fans);
    CHECK(fan_histogram(external_30.extrusions, erBridgeInfill) == fan_histogram(reference.extrusions, erBridgeInfill));
}

// A sphere's overhang walls show the same: the wall that the deck only touches is cooled by the
// overhang fan, whatever the external bridge fan says.
TEST_CASE("An explicit external bridge fan leaves a sphere's overhang walls on the overhang fan",
          "[Cooling][ExternalBridgeFan]")
{
    DynamicPrintConfig reference_config = bridge_fan_config(80);
    // 0% cools every outer and overhang wall, so the overhang fan shows on more than the bridges;
    // the thicker layers keep the 100 mm sphere cheap to slice.
    reference_config.set_deserialize_strict({
        { "initial_layer_print_height", 0.4 },
        { "layer_height",               0.4 },
        { "overhang_fan_threshold",     "0%" },
    });
    const SlicedPrint reference = slice_extrusions(TestMesh::sphere_50mm, reference_config);

    DynamicPrintConfig config = reference_config;
    set_external_bridge_fan_speed(config, 30);
    const SlicedPrint external_30 = slice_extrusions(TestMesh::sphere_50mm, config);

    const RoleFans overhang = fans_of_role(external_30.extrusions, erOverhangPerimeter);
    REQUIRE(overhang.moves > 0);
    CHECK(overhang.fans == std::vector<int>{ 80 });
    CHECK(fans_of_role(reference.extrusions, erOverhangPerimeter).fans == overhang.fans);
    CHECK(fan_histogram(external_30.extrusions, erBridgeInfill) == fan_histogram(reference.extrusions, erBridgeInfill));
}

// The switch on the cooling page is the gate: with forced overhang and bridge cooling off, nothing
// overrides the layer fan, explicit external bridge value or not.
TEST_CASE("Turning off forced bridge cooling leaves the layer fan on external bridges", "[Cooling][ExternalBridgeFan]")
{
    DynamicPrintConfig config = bridge_fan_config(80);
    config.set_deserialize_strict({ { "enable_overhang_bridge_fan", false } });
    set_external_bridge_fan_speed(config, 30);
    const SlicedPrint sliced = slice_extrusions(TestMesh::bridge, config);

    const RoleFans bridges = fans_of_role(sliced.extrusions, erBridgeInfill);
    REQUIRE(bridges.moves > 0);
    INFO("bridge fan " << bridges.fans.front() << "% over " << bridges.moves << " extrusions");
    CHECK(bridges.fans == std::vector<int>{ 60 });
}

// A blackout layer keeps the fan off: the deck bridges from layer 25 up, so every bridge of the print
// sits inside the first 45 layers.
TEST_CASE("A bridge inside the no-cooling first layers keeps the fan off", "[Cooling][ExternalBridgeFan]")
{
    DynamicPrintConfig config = bridge_fan_config(80);
    config.set_deserialize_strict({ { "close_fan_the_first_x_layers", 45 } });
    set_external_bridge_fan_speed(config, 30);
    const SlicedPrint sliced = slice_extrusions(TestMesh::bridge, config);

    const RoleFans bridges = fans_of_role(sliced.extrusions, erBridgeInfill);
    REQUIRE(bridges.moves > 0);
    INFO("bridge fan " << bridges.fans.front() << "% over " << bridges.moves << " extrusions");
    CHECK(bridges.fans == std::vector<int>{ 0 });
}

// "Full fan speed at layer" ramps the fan up layer by layer, and an explicit external bridge value
// rides the same ramp as the overhang fan. The deck of the test mesh bridges in a single layer, so the
// ramp is read on that layer, before and after it is done.
TEST_CASE("An explicit external bridge fan speed ramps up with the layer fan", "[Cooling][ExternalBridgeFan]")
{
    constexpr int full_fan_layer = 40;
    auto bridge_fans_by_layer = [](int full_fan_speed_layer) {
        DynamicPrintConfig config = bridge_fan_config(100);
        config.set_deserialize_strict({ { "full_fan_speed_layer", full_fan_speed_layer } });
        set_external_bridge_fan_speed(config, 30);
        const SlicedPrint sliced = slice_extrusions(TestMesh::bridge, config);
        // One fan speed per bridged layer: the whole bridge of a layer runs at one speed.
        std::map<unsigned int, int> fans_by_layer;
        for (const Extrusion &extrusion : sliced.extrusions) {
            if (extrusion.role != erBridgeInfill)
                continue;
            const auto inserted = fans_by_layer.emplace(extrusion.layer_id, extrusion.fan);
            INFO("layer " << extrusion.layer_id << " bridges at " << extrusion.fan << "%");
            CHECK(inserted.first->second == extrusion.fan);
        }
        return fans_by_layer;
    };

    // Ramp still climbing at the bridge: the bridge runs at the ramped share of 30%.
    const std::map<unsigned int, int> ramped = bridge_fans_by_layer(full_fan_layer);
    REQUIRE(ramped.size() == 1);
    const std::pair<const unsigned int, int> bridge_layer = *ramped.begin();
    const double                             factor       = std::min(1., double(bridge_layer.first + 1) / double(full_fan_layer));
    INFO("bridge on layer " << bridge_layer.first << " with ramp factor " << factor << " at " << bridge_layer.second << "%");
    REQUIRE(factor < 1.);
    CHECK_THAT(double(bridge_layer.second), Catch::Matchers::WithinAbs(std::floor(30. * factor + 0.5), 1.));

    // The same print with the ramp finished before the bridge: the value itself, not a share of it.
    const std::map<unsigned int, int> unramped = bridge_fans_by_layer(int(bridge_layer.first));
    REQUIRE(unramped.size() == 1);
    INFO("bridge on layer " << unramped.begin()->first << " after the ramp, at " << unramped.begin()->second << "%");
    CHECK(unramped.begin()->first == bridge_layer.first);
    CHECK(unramped.begin()->second == 30);
}

// The first-layer override anchors the ramp, and an anchor pulls every ramped speed up from it. An
// explicit 0 asks for no cooling on the bridge, so it has to come out of the anchored ramp at 0.
TEST_CASE("An explicit external bridge fan speed of zero stays off while the fan ramps up",
          "[Cooling][ExternalBridgeFan]")
{
    DynamicPrintConfig config = bridge_fan_config(100);
    config.set_deserialize_strict({
        { "initial_layer_fan_speed", 50 },
        { "full_fan_speed_layer",    30 },
    });

    // The control shows the anchor is on for the bridge: 30% comes out at the anchor-narrowed curve,
    // which stays above 30 all the way, where the plain ramp alone would start below it.
    DynamicPrintConfig control_config = config;
    set_external_bridge_fan_speed(control_config, 30);
    const SlicedPrint control        = slice_extrusions(TestMesh::bridge, control_config);
    const RoleFans    control_bridge = fans_of_role(control.extrusions, erBridgeInfill);
    REQUIRE(control_bridge.moves > 0);
    INFO("lowest control bridge fan " << control_bridge.fans.front() << "% over " << control_bridge.moves << " extrusions");
    CHECK(control_bridge.fans.front() >= 30);

    DynamicPrintConfig zero_config = config;
    set_external_bridge_fan_speed(zero_config, 0);
    const SlicedPrint zero        = slice_extrusions(TestMesh::bridge, zero_config);
    const RoleFans    zero_bridge = fans_of_role(zero.extrusions, erBridgeInfill);
    REQUIRE(zero_bridge.moves == control_bridge.moves);
    INFO("bridge fan " << zero_bridge.fans.front() << "% over " << zero_bridge.moves << " extrusions");
    CHECK(zero_bridge.fans == std::vector<int>{ 0 });
}

// The top of the range is 100%, above both the layer fan and the maximum fan speed threshold: the
// bridge runs at exactly the value that was set.
TEST_CASE("An explicit external bridge fan speed of 100 reaches the bridge at full speed",
          "[Cooling][ExternalBridgeFan]")
{
    DynamicPrintConfig config = bridge_fan_config(40);
    set_external_bridge_fan_speed(config, 100);
    const SlicedPrint sliced = slice_extrusions(TestMesh::bridge, config);

    const RoleFans bridges = fans_of_role(sliced.extrusions, erBridgeInfill);
    REQUIRE(bridges.moves > 0);
    INFO("bridge fan " << bridges.fans.front() << "% over " << bridges.moves << " extrusions");
    CHECK(bridges.fans == std::vector<int>{ 100 });
}

// The internal bridges keep the setting they had: an external bridge fan speed splits the external
// bridge off and leaves the internal one where it was set.
TEST_CASE("Internal bridges keep their own fan speed when external bridges take one",
          "[Cooling][ExternalBridgeFan]")
{
    DynamicPrintConfig config = bridge_fan_config(80);
    config.set_deserialize_strict({ { "internal_bridge_fan_speed", 50 } });
    set_external_bridge_fan_speed(config, 30);
    const SlicedPrint sliced = slice_extrusions(TestMesh::bridge, config);

    const RoleFans internal = fans_of_role(sliced.extrusions, erInternalBridgeInfill);
    const RoleFans external = fans_of_role(sliced.extrusions, erBridgeInfill);
    REQUIRE(internal.moves > 0);
    REQUIRE(external.moves > 0);
    INFO("internal bridge " << internal.fans.front() << "%, external bridge " << external.fans.front() << "%");
    CHECK(internal.fans == std::vector<int>{ 50 });
    CHECK(external.fans == std::vector<int>{ 30 });
}

// Each filament carries its own value, and the tool change between them keeps the two apart: the
// filament that took its bridges over keeps them, the -1 one hands them back to its overhang fan.
TEST_CASE("Each filament's external bridge fan speed reaches only its own bridges",
          "[Cooling][ExternalBridgeFan]")
{
    DynamicPrintConfig config = multifilament_config(2, {
        { "close_fan_the_first_x_layers",  "0,0" },
        { "enable_arc_fitting",            "disabled" },
        { "enable_overhang_bridge_fan",    "1,1" },
        { "external_bridge_fan_speed",     "30,-1" },
        { "fan_cooling_layer_time",        "1000,1000" },
        { "fan_kickstart",                 0. },
        { "fan_max_speed",                 "60,60" },
        { "fan_min_speed",                 "60,60" },
        { "fan_speedup_time",              0. },
        { "full_fan_speed_layer",          "0,0" },
        { "initial_layer_fan_speed",       "-1,-1" },
        { "overhang_fan_speed",            "80,80" },
        { "part_cooling_fan_min_pwm",      0 },
        { "slow_down_for_layer_cooling",   false },
        { "slow_down_layer_time",          "0,0" },
    });

    Print print;
    Model model;
    // One bridge per filament, so the bridges of a layer are printed on both and the tool changes
    // between them land inside a run of external bridges.
    std::vector<std::vector<ConfigBase::SetDeserializeItem>> extruders = {
        { { "extruder", 1 } },
        { { "extruder", 2 } },
    };
    init_print({ mesh(TestMesh::bridge), mesh(TestMesh::bridge) }, print, model, config, &extruders);
    GCodeProcessorResult result;
    const std::string     sliced_gcode = gcode(print, &result);
    REQUIRE(!sliced_gcode.empty());
    // The markers are for the cooling buffer only: neither filament leaves one in the G-code.
    CHECK(sliced_gcode.find("_EXTERNAL_BRIDGE") == std::string::npos);

    const std::map<unsigned int, std::vector<int>> fans_by_filament = bridge_fans_by_filament(result);

    // Both filaments have to bridge, or one of the two expectations below is vacuous.
    REQUIRE(fans_by_filament.count(0) == 1);
    REQUIRE(fans_by_filament.count(1) == 1);
    INFO("filament 1 bridges at " << fans_by_filament.at(0).front() << "%, filament 2 at " << fans_by_filament.at(1).front() << "%");
    CHECK(fans_by_filament.at(0) == std::vector<int>{ 30 });
    CHECK(fans_by_filament.at(1) == std::vector<int>{ 80 });
}

// ---------------------------------------------------------------------------------------------
// The seam between an ordinary overhang wall and an external bridge. The bridge infill and the
// innermost overhang wall of the same layer overlap (external_bridge_infill_wall_overlap), and on
// some walls of that layer the bridge material runs along the whole wall, while on the ones beside
// them it only comes near. The question is per whole motion: one native G1 runs at one fan speed,
// so a wall is joined only when the bridge material covers that command end to end.
//
// The reading below comes from the G-code the slicer actually wrote, not from the Preview: the
// Preview subdivides one G1 into speed-interpolated vertices that all carry the command's
// gcode_id, so reading it asks a single command for several fan states at once. Reading whole
// commands also keeps the rule clear of the production classifier: the only facts used here are
// what the writer emitted.
//
// The footprint the frozen material analysis used, and the one this fixture's own widths support: a
// bridge command is as wide as its own ;WIDTH (0.4 mm on this layer), square ended, and it joins a
// wall only where it covers that wall's centre line inward half band [0, w/2] along the whole
// command. That 0.4 is the width of the seam walls and of the bridge commands themselves, never the
// 0.45 of the layer's inner, outer and infill commands: at 0.45 with round ends the same geometry
// would cover the partial walls end to end (their gap closes at 0.449784 mm), so the widths below
// are pinned as well as the verdicts.
//
// Every command of the layer is pinned, in emission order, against the frozen capture of this very
// fixture: role, printed ;WIDTH, both endpoints, modal delta E and the fan the contract wants. The
// windows alone would leave the walls that no window names free to drift, and a split, reordered
// or re-extruded command could move a fan reading onto a command that is not the one analysed.

// G-code prints three decimals, so a coordinate is within half of one micrometre of the value the
// writer used. The endpoint tolerance covers that printed resolution plus one rounding step, and
// stays far below the 0.407 mm that separates the closest distinct endpoints of this layer.
constexpr double gcode_coordinate_quantum  = 0.001; // mm, three printed decimals
constexpr double native_endpoint_tolerance = 1.5 * gcode_coordinate_quantum;
constexpr double native_e_tolerance        = 1.1e-5; // mm, E prints at most five decimals
constexpr double native_width_tolerance    = 5e-4;   // mm, ;WIDTH prints two decimals
// The fixture's own fans: fan_min_speed 60 for the layer, overhang_fan_speed 100 for overhang
// walls, and the case's explicit value for the bridge.
constexpr int layer_fan_percent    = 60;
constexpr int overhang_fan_percent = 100;

// The ;TYPE: block a native command was written in.
enum class NativeRole { Other, InnerWall, OuterWall, OverhangWall, Bridge, SparseInfill };

// The fan a command is expected to run at, by the role it plays in this case.
enum class NativeFan { Bridge, Overhang, Layer };

// One whole emitted command: both endpoints of the native G1, the material it deposits, its role,
// the ;WIDTH: in force and the part cooling PWM it starts at, as whole percent.
struct NativeMove
{
    Vec2d      from;
    Vec2d      to;
    double     e     = 0.;              // mm of filament; positive on every command kept here
    NativeRole role  = NativeRole::Other;
    double     width = 0.;              // mm, the ;WIDTH: in force at the command
    int        fan   = 0;               // percent of full PWM
};

// The native commands of one layer, delimited by the ;Z: marker the writer prints on every change.
struct NativeLayer
{
    double                  z     = 0.;
    std::vector<NativeMove> moves;      // positive-XY, positive-E G1 commands only
};

// What the whole emitted file says, in layer order.
struct NativeGCode
{
    std::vector<NativeLayer> layers;
    size_t                   arcs = 0;  // G2/G3 commands; this fixture must emit none
};

static NativeRole native_role_of(std::string_view type)
{
    if (type.rfind("Overhang wall", 0) == 0) return NativeRole::OverhangWall;
    if (type.rfind("Inner wall", 0)    == 0) return NativeRole::InnerWall;
    if (type.rfind("Outer wall", 0)    == 0) return NativeRole::OuterWall;
    if (type.rfind("Sparse infill", 0) == 0) return NativeRole::SparseInfill;
    if (type.rfind("Bridge", 0)        == 0) return NativeRole::Bridge;
    return NativeRole::Other;
}

static const char *native_role_name(NativeRole role)
{
    switch (role) {
    case NativeRole::InnerWall:    return "Inner wall";
    case NativeRole::OuterWall:    return "Outer wall";
    case NativeRole::OverhangWall: return "Overhang wall";
    case NativeRole::Bridge:       return "Bridge";
    case NativeRole::SparseInfill: return "Sparse infill";
    case NativeRole::Other:        break;
    }
    return "other";
}

// Split the emitted G-code into layers of whole native commands. Only a command that both moves in
// XY and deposits material is kept: a positive XY displacement and a positive modal delta E are
// what make a command a wall or a bridge line rather than a travel, a retract or a Z move.
//
// E is tracked here rather than through GCodeReader's own E state: GCodeReader::update_coordinates
// only advances its position on G commands, so its delta E is only the modal one while the file is
// in absolute E mode. This one declares M83 before the first layer, so M82/M83 are followed.
static NativeGCode read_native_gcode(const std::string &text)
{
    NativeGCode native;
    NativeRole  role       = NativeRole::Other;
    int         fan        = 0;
    double      width      = 0.;    // ;WIDTH: is only printed when it changes, so it is carried
    bool        relative_e = false; // G90/M82 is the default; this file switches to M83
    Vec2d       position   = Vec2d::Zero();
    double      last_e     = 0.;
    GCodeReader reader;
    reader.parse_buffer(text, [&](GCodeReader &, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.rfind("Z:", 0) == 0) {
            NativeLayer layer;
            layer.z = std::atof(std::string(comment.substr(2)).c_str());
            native.layers.push_back(std::move(layer));
            return;
        }
        if (comment.rfind("TYPE:", 0) == 0) {
            role = native_role_of(comment.substr(5));
            return;
        }
        if (comment.rfind("WIDTH:", 0) == 0) {
            width = std::atof(std::string(comment.substr(6)).c_str());
            return;
        }
        if (line.cmd_is("G2") || line.cmd_is("G3")) {
            ++ native.arcs;
            return;
        }
        if (line.cmd_is("M82") || line.cmd_is("M83")) {
            relative_e = line.cmd_is("M83");
            return;
        }
        if (line.cmd_is("M106") || line.cmd_is("M107")) {
            float pwm = 0.f;
            fan = (! line.cmd_is("M107") && line.has_value('S', pwm)) ?
                      int(std::lround(double(pwm) * 100. / 255.)) : 0;
            return;
        }
        if (line.cmd_is("G92")) {
            float value = 0.f;
            if (line.has_value('X', value)) position.x() = value;
            if (line.has_value('Y', value)) position.y() = value;
            if (line.has_value('E', value)) last_e = value;
            return;
        }
        if (! line.cmd_is("G1"))
            return;
        // Modal coordinates: the endpoints this command actually commands.
        Vec2d to    = position;
        float value = 0.f;
        if (line.has_value('X', value)) to.x() = value;
        if (line.has_value('Y', value)) to.y() = value;
        double deposit = 0.;
        if (line.has_value('E', value)) {
            deposit = relative_e ? double(value) : double(value) - last_e;
            last_e  = value;
        }
        if (deposit > 0. && (to - position).norm() > 0. && ! native.layers.empty()) {
            NativeMove move;
            move.from  = position;
            move.to    = to;
            move.e     = deposit;
            move.role  = role;
            move.width = width;
            move.fan   = fan;
            native.layers.back().moves.push_back(std::move(move));
        }
        position = to;
    });
    return native;
}

template<typename T, size_t N>
constexpr size_t array_count(const T (&)[N])
{
    return N;
}

// One command of the bridged layer, frozen: what the writer emitted and the fan it must run at.
// `fan` is the contract's expectation, not the observation: the four complete seams are expected at
// the case's explicit value while this frozen output still runs them at the overhang fan.
struct NativeCommandKey
{
    NativeRole role;
    double     width;
    Vec2d      from;
    Vec2d      to;
    double     e;
    NativeFan  fan;
};

// Short names, so the table below stays one command per line.
using Role = NativeRole;
using Fan  = NativeFan;

// Layer Z=1.2 of this fixture, command by command: the sixth layer, the only one that bridges and
// writes overhang walls. Read from the frozen capture (`provenance-plan/stair/stair-30.gcode`,
// identical in `stair-0.gcode` and `stair--1.gcode`), in emission order.
const NativeCommandKey layer5_commands[] = {
    { Role::InnerWall, 0.45, Vec2d(9.907, -8.918), Vec2d(10.2, -8.918), 0.00993, Fan::Layer },
    { Role::OverhangWall, 0.40, Vec2d(10.2, -8.918), Vec2d(13.918, -8.918), 0.11039, Fan::Overhang },
    { Role::OverhangWall, 0.40, Vec2d(13.918, -8.918), Vec2d(13.918, 8.918), 0.52957, Fan::Bridge },
    { Role::OverhangWall, 0.40, Vec2d(13.918, 8.918), Vec2d(10.2, 8.918), 0.11039, Fan::Bridge },
    { Role::InnerWall, 0.45, Vec2d(10.2, 8.918), Vec2d(-10.2, 8.918), 0.69052, Fan::Layer },
    { Role::OverhangWall, 0.40, Vec2d(-10.2, 8.918), Vec2d(-13.918, 8.918), 0.11039, Fan::Overhang },
    { Role::OverhangWall, 0.40, Vec2d(-13.918, 8.918), Vec2d(-13.918, -8.918), 0.52957, Fan::Bridge },
    { Role::OverhangWall, 0.40, Vec2d(-13.918, -8.918), Vec2d(-10.2, -8.918), 0.11039, Fan::Bridge },
    { Role::InnerWall, 0.45, Vec2d(-10.2, -8.918), Vec2d(9.867, -8.918), 0.67923, Fan::Layer },
    { Role::InnerWall, 0.45, Vec2d(9.907, -9.325), Vec2d(10.2, -9.325), 0.00993, Fan::Layer },
    { Role::OverhangWall, 0.40, Vec2d(10.2, -9.325), Vec2d(14.325, -9.325), 0.12248, Fan::Overhang },
    { Role::OverhangWall, 0.40, Vec2d(14.325, -9.325), Vec2d(14.325, 9.325), 0.55374, Fan::Overhang },
    { Role::OverhangWall, 0.40, Vec2d(14.325, 9.325), Vec2d(10.2, 9.325), 0.12248, Fan::Overhang },
    { Role::InnerWall, 0.45, Vec2d(10.2, 9.325), Vec2d(-10.2, 9.325), 0.69052, Fan::Layer },
    { Role::OverhangWall, 0.40, Vec2d(-10.2, 9.325), Vec2d(-14.325, 9.325), 0.12248, Fan::Overhang },
    { Role::OverhangWall, 0.40, Vec2d(-14.325, 9.325), Vec2d(-14.325, -9.325), 0.55374, Fan::Overhang },
    { Role::OverhangWall, 0.40, Vec2d(-14.325, -9.325), Vec2d(-10.2, -9.325), 0.12248, Fan::Overhang },
    { Role::InnerWall, 0.45, Vec2d(-10.2, -9.325), Vec2d(9.867, -9.325), 0.67923, Fan::Layer },
    { Role::OuterWall, 0.45, Vec2d(9.907, -9.775), Vec2d(10.2, -9.775), 0.00993, Fan::Layer },
    { Role::OverhangWall, 0.40, Vec2d(10.2, -9.775), Vec2d(14.775, -9.775), 0.13584, Fan::Overhang },
    { Role::OverhangWall, 0.40, Vec2d(14.775, -9.775), Vec2d(14.775, 9.775), 0.58046, Fan::Overhang },
    { Role::OverhangWall, 0.40, Vec2d(14.775, 9.775), Vec2d(10.2, 9.775), 0.13584, Fan::Overhang },
    { Role::OuterWall, 0.45, Vec2d(10.2, 9.775), Vec2d(-10.2, 9.775), 0.69052, Fan::Layer },
    { Role::OverhangWall, 0.40, Vec2d(-10.2, 9.775), Vec2d(-14.775, 9.775), 0.13584, Fan::Overhang },
    { Role::OverhangWall, 0.40, Vec2d(-14.775, 9.775), Vec2d(-14.775, -9.775), 0.58046, Fan::Overhang },
    { Role::OverhangWall, 0.40, Vec2d(-14.775, -9.775), Vec2d(-10.2, -9.775), 0.13584, Fan::Overhang },
    { Role::OuterWall, 0.45, Vec2d(-10.2, -9.775), Vec2d(9.867, -9.775), 0.67923, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(5.344, -8.572), Vec2d(6.972, -8.572), 0.05512, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(6.972, -8.572), Vec2d(8.425, -7.118), 0.06957, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(8.425, -7.118), Vec2d(8.425, -4.01), 0.10523, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(8.425, -4.01), Vec2d(3.863, -8.572), 0.21839, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(3.863, -8.572), Vec2d(0.754, -8.572), 0.10523, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(0.754, -8.572), Vec2d(8.425, -0.901), 0.36720, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(8.425, -0.901), Vec2d(8.425, 2.208), 0.10523, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(8.425, 2.208), Vec2d(-2.354, -8.572), 0.51602, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(-2.354, -8.572), Vec2d(-5.463, -8.572), 0.10523, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(-5.463, -8.572), Vec2d(8.425, 5.317), 0.66483, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(8.425, 5.317), Vec2d(8.425, 8.425), 0.10523, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(8.425, 8.425), Vec2d(-8.425, -8.425), 0.80663, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(-8.425, -8.425), Vec2d(-8.425, -5.317), 0.10523, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(-8.425, -5.317), Vec2d(5.463, 8.572), 0.66483, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(5.463, 8.572), Vec2d(2.354, 8.572), 0.10523, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(2.354, 8.572), Vec2d(-8.425, -2.208), 0.51602, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(-8.425, -2.208), Vec2d(-8.425, 0.901), 0.10523, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(-8.425, 0.901), Vec2d(-0.754, 8.572), 0.36720, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(-0.754, 8.572), Vec2d(-3.863, 8.572), 0.10523, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(-3.863, 8.572), Vec2d(-8.425, 4.01), 0.21839, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(-8.425, 4.01), Vec2d(-8.425, 7.118), 0.10523, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(-8.425, 7.118), Vec2d(-6.972, 8.572), 0.06957, Fan::Layer },
    { Role::SparseInfill, 0.45, Vec2d(-6.972, 8.572), Vec2d(-5.344, 8.572), 0.05512, Fan::Layer },
    { Role::Bridge, 0.40, Vec2d(13.747, 8.964), Vec2d(13.747, -8.804), 0.52755, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(13.747, -8.804), Vec2d(13.39, -8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(13.39, -8.804), Vec2d(13.39, 8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(13.39, 8.804), Vec2d(13.033, 8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(13.033, 8.804), Vec2d(13.033, -8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(13.033, -8.804), Vec2d(12.676, -8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(12.676, -8.804), Vec2d(12.676, 8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(12.676, 8.804), Vec2d(12.319, 8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(12.319, 8.804), Vec2d(12.319, -8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(12.319, -8.804), Vec2d(11.962, -8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(11.962, -8.804), Vec2d(11.962, 8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(11.962, 8.804), Vec2d(11.605, 8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(11.605, 8.804), Vec2d(11.605, -8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(11.605, -8.804), Vec2d(11.248, -8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(11.248, -8.804), Vec2d(11.248, 8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(11.248, 8.804), Vec2d(10.891, 8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(10.891, 8.804), Vec2d(10.891, -8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(10.891, -8.804), Vec2d(10.534, -8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(10.534, -8.804), Vec2d(10.534, 8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(10.534, 8.804), Vec2d(10.379, 8.804), 0.00461, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(10.379, 8.804), Vec2d(10.379, 8.536), 0.00795, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(10.379, 8.536), Vec2d(10.177, 8.536), 0.00599, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(10.177, 8.536), Vec2d(10.177, -8.536), 0.50688, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(10.177, -8.536), Vec2d(9.82, -8.536), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(9.82, -8.536), Vec2d(9.82, 8.536), 0.50688, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(9.82, 8.536), Vec2d(9.463, 8.536), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(9.463, 8.536), Vec2d(9.463, -8.536), 0.50688, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(9.463, -8.536), Vec2d(9.105, -8.536), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(9.105, -8.536), Vec2d(9.105, 8.697), 0.51165, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-13.747, -8.964), Vec2d(-13.747, 8.804), 0.52755, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-13.747, 8.804), Vec2d(-13.39, 8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-13.39, 8.804), Vec2d(-13.39, -8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-13.39, -8.804), Vec2d(-13.033, -8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-13.033, -8.804), Vec2d(-13.033, 8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-13.033, 8.804), Vec2d(-12.676, 8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-12.676, 8.804), Vec2d(-12.676, -8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-12.676, -8.804), Vec2d(-12.319, -8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-12.319, -8.804), Vec2d(-12.319, 8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-12.319, 8.804), Vec2d(-11.962, 8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-11.962, 8.804), Vec2d(-11.962, -8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-11.962, -8.804), Vec2d(-11.605, -8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-11.605, -8.804), Vec2d(-11.605, 8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-11.605, 8.804), Vec2d(-11.248, 8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-11.248, 8.804), Vec2d(-11.248, -8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-11.248, -8.804), Vec2d(-10.891, -8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-10.891, -8.804), Vec2d(-10.891, 8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-10.891, 8.804), Vec2d(-10.534, 8.804), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-10.534, 8.804), Vec2d(-10.534, -8.804), 0.52278, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-10.534, -8.804), Vec2d(-10.379, -8.804), 0.00461, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-10.379, -8.804), Vec2d(-10.379, -8.536), 0.00795, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-10.379, -8.536), Vec2d(-10.177, -8.536), 0.00599, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-10.177, -8.536), Vec2d(-10.177, 8.536), 0.50688, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-10.177, 8.536), Vec2d(-9.82, 8.536), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-9.82, 8.536), Vec2d(-9.82, -8.536), 0.50688, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-9.82, -8.536), Vec2d(-9.463, -8.536), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-9.463, -8.536), Vec2d(-9.463, 8.536), 0.50688, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-9.463, 8.536), Vec2d(-9.105, 8.536), 0.01060, Fan::Bridge },
    { Role::Bridge, 0.40, Vec2d(-9.105, 8.536), Vec2d(-9.105, -8.697), 0.51165, Fan::Bridge },
};

// How an overhang wall of that layer relates to the bridge material, as the frozen material
// analysis assigned it: `Complete` walls are covered end to end on their material side, `Partial`
// walls of the same ring keep a real gap, `Adjacent` walls sit on the next ring out and `Detached`
// ones are the rest (the second ring's short wall and the whole third ring). A `Complete` wall must
// take the bridge's fan; every other one keeps the overhang fan.
enum class SeamKind { Complete, Partial, Adjacent, Detached };

struct SeamWindow
{
    const char *name;
    SeamKind    kind;
    Vec2d       from;
    Vec2d       to;
};

// The 18 overhang walls of the layer, in fixture-centred millimetres, as directed segments: the
// direction of the move is part of the key, so a reversed command is a different key. Taken from
// the same frozen capture; the coordinates are printed values, not line numbers.
//
// The two `short` walls of the complete group are thin positives and that is intended: the bridge
// covers them for 3.718 mm of 3.718 mm, but by only 0.086 mm of half-band depth over the body and
// 0.018 mm at the far end, through the bridge's own end connection. A matcher that asks for more
// than footprint-into-half-band coverage would keep them red, which is why the model is written
// down here.
const SeamWindow seam_windows[] = {
    { "right partial", SeamKind::Partial, Vec2d(10.2, -8.918), Vec2d(13.918, -8.918) },
    { "right long", SeamKind::Complete, Vec2d(13.918, -8.918), Vec2d(13.918, 8.918) },
    { "right short", SeamKind::Complete, Vec2d(13.918, 8.918), Vec2d(10.2, 8.918) },
    { "left partial", SeamKind::Partial, Vec2d(-10.2, 8.918), Vec2d(-13.918, 8.918) },
    { "left long", SeamKind::Complete, Vec2d(-13.918, 8.918), Vec2d(-13.918, -8.918) },
    { "left short", SeamKind::Complete, Vec2d(-13.918, -8.918), Vec2d(-10.2, -8.918) },
    { "right adjacent low", SeamKind::Detached, Vec2d(10.2, -9.325), Vec2d(14.325, -9.325) },
    { "right adjacent long", SeamKind::Adjacent, Vec2d(14.325, -9.325), Vec2d(14.325, 9.325) },
    { "right adjacent short", SeamKind::Adjacent, Vec2d(14.325, 9.325), Vec2d(10.2, 9.325) },
    { "left adjacent low", SeamKind::Detached, Vec2d(-10.2, 9.325), Vec2d(-14.325, 9.325) },
    { "left adjacent long", SeamKind::Adjacent, Vec2d(-14.325, 9.325), Vec2d(-14.325, -9.325) },
    { "left adjacent short", SeamKind::Adjacent, Vec2d(-14.325, -9.325), Vec2d(-10.2, -9.325) },
    { "right outer low", SeamKind::Detached, Vec2d(10.2, -9.775), Vec2d(14.775, -9.775) },
    { "right outer long", SeamKind::Detached, Vec2d(14.775, -9.775), Vec2d(14.775, 9.775) },
    { "right outer short", SeamKind::Detached, Vec2d(14.775, 9.775), Vec2d(10.2, 9.775) },
    { "left outer low", SeamKind::Detached, Vec2d(-10.2, 9.775), Vec2d(-14.775, 9.775) },
    { "left outer long", SeamKind::Detached, Vec2d(-14.775, 9.775), Vec2d(-14.775, -9.775) },
    { "left outer short", SeamKind::Detached, Vec2d(-14.775, -9.775), Vec2d(-10.2, -9.775) },
};

static const SeamWindow *seam_window_named(const char *name)
{
    for (const SeamWindow &window : seam_windows)
        if (std::string_view(window.name) == name)
            return &window;
    return nullptr;
}

// The centre of the layer's own emitted geometry: the widest ring is symmetric about it, so the
// midpoint of the emitted extremes is the frame the frozen coordinates are measured in. Nothing
// here is taken from the slicer's internal layer coordinates or from a fixed plate origin.
static Vec2d emitted_centre(const NativeLayer &layer)
{
    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (const NativeMove &move : layer.moves)
        for (const Vec2d &point : { move.from, move.to }) {
            min_x = std::min(min_x, point.x());
            max_x = std::max(max_x, point.x());
            min_y = std::min(min_y, point.y());
            max_y = std::max(max_y, point.y());
        }
    return Vec2d(0.5 * (min_x + max_x), 0.5 * (min_y + max_y));
}

// Index of the one native overhang-wall command that is `window` verbatim. Two such commands mean
// the writer split the window and none means this output cannot address it: both are reported
// instead of being taken as one.
static size_t find_window_move(const NativeLayer &layer, const Vec2d &centre, const SeamWindow &window)
{
    size_t found = size_t(-1), matches = 0;
    for (size_t i = 0; i < layer.moves.size(); ++ i) {
        const NativeMove &move = layer.moves[i];
        if (move.role != NativeRole::OverhangWall)
            continue;
        if ((move.from - centre - window.from).norm() > native_endpoint_tolerance ||
            (move.to   - centre - window.to).norm()   > native_endpoint_tolerance)
            continue;
        found = i;
        ++ matches;
    }
    return matches == 1 ? found : size_t(-1);
}

static void add_fan(std::vector<int> &fans, int fan)
{
    if (std::find(fans.begin(), fans.end(), fan) == fans.end())
        fans.push_back(fan);
}

static void add_width(std::vector<double> &widths, double width)
{
    for (const double known : widths)
        if (std::abs(known - width) <= native_width_tolerance)
            return;
    widths.push_back(width);
}

// The one fan of a group, for the INFO lines; -1 when the group is empty.
static int only_fan(const std::vector<int> &fans)
{
    return fans.empty() ? -1 : fans.front();
}

struct SeamWindowFans
{
    // The fan of every whole command in each group of windows, ascending and de-duplicated.
    std::vector<int> complete_fans, partial_fans, adjacent_fans, detached_fans;
    std::vector<int> bridge_fans, bridge_lane_fans;
    // The ;WIDTH: the seam walls and the bridge commands run at: the footprint's own width.
    std::vector<double> seam_widths, bridge_widths;
    // Windows this output does not address with exactly one native command.
    std::vector<std::string> unaddressed;
    size_t complete = 0, partial = 0, adjacent = 0, detached = 0;
    size_t bridges = 0, bridge_lane = 0;
};

// |x| of the bridge's outermost line on each side: the material the seam is made of, written as
// whole G1 lines like the walls.
constexpr double bridge_lane_x = 13.747; // mm

// The fans and widths of the whole native commands of every seam window of the bridged layer. A
// window is addressed only by exactly one command whose two endpoints and direction match it.
static SeamWindowFans seam_window_fans(const NativeLayer &layer)
{
    const Vec2d    centre = emitted_centre(layer);
    SeamWindowFans measured;
    const auto note = [&measured](SeamKind kind, int fan, double width) {
        add_width(measured.seam_widths, width);
        switch (kind) {
        case SeamKind::Complete: add_fan(measured.complete_fans, fan); ++ measured.complete; break;
        case SeamKind::Partial:  add_fan(measured.partial_fans,  fan); ++ measured.partial;  break;
        case SeamKind::Adjacent: add_fan(measured.adjacent_fans, fan); ++ measured.adjacent; break;
        case SeamKind::Detached: add_fan(measured.detached_fans, fan); ++ measured.detached; break;
        }
    };
    for (const SeamWindow &window : seam_windows) {
        const size_t index = find_window_move(layer, centre, window);
        if (index == size_t(-1)) {
            measured.unaddressed.push_back(window.name);
            continue;
        }
        note(window.kind, layer.moves[index].fan, layer.moves[index].width);
    }
    for (const NativeMove &move : layer.moves) {
        if (move.role != NativeRole::Bridge)
            continue;
        ++ measured.bridges;
        add_fan(measured.bridge_fans, move.fan);
        add_width(measured.bridge_widths, move.width);
        if (std::abs(std::abs(move.from.x() - centre.x()) - bridge_lane_x) <= native_endpoint_tolerance &&
            std::abs(std::abs(move.to.x()   - centre.x()) - bridge_lane_x) <= native_endpoint_tolerance) {
            ++ measured.bridge_lane;
            add_fan(measured.bridge_lane_fans, move.fan);
        }
    }
    for (std::vector<int> *fans : { &measured.complete_fans, &measured.partial_fans, &measured.adjacent_fans,
                                    &measured.detached_fans, &measured.bridge_fans, &measured.bridge_lane_fans })
        std::sort(fans->begin(), fans->end());
    return measured;
}

// What the packet asserts about one layer: the whole-command signature against the frozen table,
// the per-window fans, and the per-command fan expectations.
enum class SignatureField { Count, Role, Width, From, To, E };

struct SignatureMismatch
{
    size_t         row;
    SignatureField field;
};

struct FanViolation
{
    size_t row;
    int    wanted;
    int    emitted;
};

struct NativeLayerReading
{
    NativeLayer                    layer;
    Vec2d                          centre;
    SeamWindowFans                 windows;
    std::vector<SignatureMismatch> signature;
    std::vector<FanViolation>      fan;
};

static const char *signature_field_name(SignatureField field)
{
    switch (field) {
    case SignatureField::Count: return "command count";
    case SignatureField::Role:  return "role";
    case SignatureField::Width: return "width";
    case SignatureField::From:  return "start point";
    case SignatureField::To:    return "end point";
    case SignatureField::E:     return "delta E";
    }
    return "?";
}

// True when the command at `row` is among the fan violations.
static bool violated(const std::vector<FanViolation> &violations, size_t row)
{
    return std::any_of(violations.begin(), violations.end(),
                       [row](const FanViolation &violation) { return violation.row == row; });
}

// The fan a command of the given class must run at.
static int expected_fan(NativeFan fan, int bridge_percent)
{
    switch (fan) {
    case NativeFan::Bridge:   return bridge_percent;
    case NativeFan::Overhang: return overhang_fan_percent;
    case NativeFan::Layer:    break;
    }
    return layer_fan_percent;
}

// One command, for the INFO lines a failure prints.
static std::string describe_command(const NativeLayer &layer, size_t row)
{
    std::ostringstream text;
    text << "row " << row;
    if (row >= layer.moves.size())
        return text.str() + ": no such command";
    const NativeMove &move = layer.moves[row];
    text << " " << native_role_name(move.role) << " w" << move.width << " (" << move.from.x() << ","
         << move.from.y() << ")->(" << move.to.x() << "," << move.to.y() << ") E" << move.e << " at "
         << move.fan << "%";
    return text.str();
}

// Judge one layer against the frozen table and the contract without asserting anything: the case
// below asserts, and its mutants are judged by this same function.
static NativeLayerReading judge_layer(const NativeLayer &layer, const Vec2d &centre, int bridge_percent)
{
    NativeLayerReading reading;
    reading.layer   = layer;
    reading.centre  = centre;
    reading.windows = seam_window_fans(layer);

    const size_t count = layer.moves.size();
    const size_t keys  = array_count(layer5_commands);
    if (count != keys)
        reading.signature.push_back({ std::min(count, keys), SignatureField::Count });
    for (size_t row = 0; row < std::min(count, keys); ++ row) {
        const NativeMove       &move = layer.moves[row];
        const NativeCommandKey &key  = layer5_commands[row];
        if (move.role != key.role)
            reading.signature.push_back({ row, SignatureField::Role });
        else if (std::abs(move.width - key.width) > native_width_tolerance)
            reading.signature.push_back({ row, SignatureField::Width });
        else if ((move.from - centre - key.from).norm() > native_endpoint_tolerance)
            reading.signature.push_back({ row, SignatureField::From });
        else if ((move.to - centre - key.to).norm() > native_endpoint_tolerance)
            reading.signature.push_back({ row, SignatureField::To });
        else if (std::abs(move.e - key.e) > native_e_tolerance)
            reading.signature.push_back({ row, SignatureField::E });
        const int wanted = expected_fan(key.fan, bridge_percent);
        if (move.fan != wanted)
            reading.fan.push_back({ row, wanted, move.fan });
    }
    return reading;
}

// The contract, in one predicate: the frozen signature of the whole layer, every window addressed
// by exactly one command, the seam walls and the bridge at the commands' own 0.4 mm, and the fans of
// every group and of every single command as the case states them.
static bool layer_contract_holds(const NativeLayerReading &reading, int bridge_percent)
{
    const SeamWindowFans &windows = reading.windows;
    return reading.signature.empty() && reading.fan.empty() && windows.unaddressed.empty() &&
           windows.complete == 4 && windows.partial == 2 && windows.adjacent == 4 && windows.detached == 8 &&
           windows.bridges > 0 && windows.bridge_lane > 0 &&
           windows.seam_widths   == std::vector<double>{ 0.4 } &&
           windows.bridge_widths == std::vector<double>{ 0.4 } &&
           windows.bridge_fans      == std::vector<int>{ bridge_percent } &&
           windows.bridge_lane_fans == std::vector<int>{ bridge_percent } &&
           windows.complete_fans    == std::vector<int>{ bridge_percent } &&
           windows.partial_fans     == std::vector<int>{ overhang_fan_percent } &&
           windows.adjacent_fans    == std::vector<int>{ overhang_fan_percent } &&
           windows.detached_fans    == std::vector<int>{ overhang_fan_percent };
}

struct NativeSeamLayer
{
    NativeLayer layer;  // every whole command of the bridged layer
    Vec2d       centre; // the frame the frozen coordinates are measured in
};

// Slice `step` and read the bridged layer's whole native commands out of the G-code. The bridged
// layer is the one that both bridges and writes overhang walls; a second one would make the
// readings ambiguous.
static NativeSeamLayer step_seam_fans(const TriangleMesh &step, const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    init_print({ step }, print, model, config);
    // The Preview move list is deliberately not read: it subdivides one command, and one command
    // has one fan speed. A null result keeps the processor out of this measurement entirely.
    const std::string sliced = gcode(print, nullptr);
    REQUIRE(! sliced.empty());

    const NativeGCode native = read_native_gcode(sliced);
    // Whole motions only: an arc would have to be compared along its sweep, and this fixture has
    // arc fitting off, so it must not emit one.
    REQUIRE(native.arcs == 0);

    const NativeLayer *bridged        = nullptr;
    size_t             bridged_layers = 0;
    for (const NativeLayer &layer : native.layers) {
        bool has_bridge = false, has_overhang_wall = false;
        for (const NativeMove &move : layer.moves) {
            has_bridge        = has_bridge        || move.role == NativeRole::Bridge;
            has_overhang_wall = has_overhang_wall || move.role == NativeRole::OverhangWall;
        }
        if (has_bridge && has_overhang_wall) {
            ++ bridged_layers;
            bridged = &layer;
        }
    }
    REQUIRE(bridged_layers == 1);
    REQUIRE(bridged != nullptr);
    // The bridge surface of this fixture is its sixth layer.
    CHECK(std::abs(bridged->z - 1.2) <= gcode_coordinate_quantum);

    // A kept command moves and deposits material by construction; checked once, so a change to that
    // filter cannot quietly turn a window into a travel or a retract.
    size_t not_a_deposit = 0;
    for (const NativeMove &move : bridged->moves)
        if (! (move.e > 0.) || ! ((move.to - move.from).norm() > 0.))
            ++ not_a_deposit;
    REQUIRE(not_a_deposit == 0);

    NativeSeamLayer seam_layer;
    seam_layer.layer  = *bridged;
    seam_layer.centre = emitted_centre(*bridged);
    // The frozen coordinates are fixture-centred, so a different plate origin or instance shift
    // cannot move one of them onto another command.
    CHECK(std::abs(seam_layer.centre.x()) <= native_endpoint_tolerance);
    CHECK(std::abs(seam_layer.centre.y()) <= native_endpoint_tolerance);
    return seam_layer;
}

// The contract: the bridge's own fan speed reaches the overhang walls whose whole native command it
// covers on the material side, and only those. The partial walls of the same ring, the rings beside
// and behind them and every other command of the layer keep their own fan, so both spans are
// visible on one layer of one print. -1 on the same geometry leaves every one of them on the
// overhang fan.
TEST_CASE("Ordinary overhang walls touching an external bridge use its fan speed",
          "[Cooling][ExternalBridgeSeamFan][Regression]")
{
    // A 20x20x1 base under a 30x20x2 top shifted 5 mm left: the first top layer bridges 5 mm on
    // each side while three overhang walls of that same layer run beside the bridges. The layer fan
    // sits at 60%, distinct from both the explicit 30% and the 100% overhang fan.
    TriangleMesh step = make_cube(20., 20., 1.);
    TriangleMesh top  = make_cube(30., 20., 2.);
    top.translate(-5.f, 0.f, 1.f);
    step.merge(top);

    const int explicit_value = GENERATE(30, 0, -1);
    INFO("external_bridge_fan_speed " << explicit_value);

    DynamicPrintConfig config = bridge_fan_config(100);
    config.set_deserialize_strict({
        { "initial_layer_print_height",          0.2 },
        { "layer_height",                        0.2 },
        { "wall_generator",                      "classic" },
        { "wall_loops",                          3 },
        { "detect_overhang_wall",                true },
        { "enable_support",                      false },
        { "overhang_wall_overlap",               "0%" },
        // Grid cells would add their own seam walls; this case is the ordinary single bridge.
        { "external_bridge_grid_enable",         false },
        // Half of the ~0.6 mm overlap base grows the bridge over the innermost wall's centre line,
        // and still short of the next wall: real line-to-wall contact, one wall deep.
        { "external_bridge_infill_wall_overlap", "50%" },
    });
    set_external_bridge_fan_speed(config, explicit_value);

    const int                expected = explicit_value >= 0 ? explicit_value : overhang_fan_percent;
    const NativeSeamLayer    seam     = step_seam_fans(step, config);
    const NativeLayerReading measured = judge_layer(seam.layer, seam.centre, expected);
    const SeamWindowFans    &windows  = measured.windows;

    INFO("windows: complete " << windows.complete << " at " << only_fan(windows.complete_fans) << "%, partial "
                              << windows.partial << " at " << only_fan(windows.partial_fans) << "%, adjacent "
                              << windows.adjacent << " at " << only_fan(windows.adjacent_fans) << "%, detached "
                              << windows.detached << " at " << only_fan(windows.detached_fans) << "%");
    INFO("bridge " << windows.bridges << " commands at " << only_fan(windows.bridge_fans)
                   << "%, the outermost line on each side " << windows.bridge_lane << " at "
                   << only_fan(windows.bridge_lane_fans) << "%, seam width "
                   << (windows.seam_widths.empty() ? 0. : windows.seam_widths.front()) << " mm, bridge width "
                   << (windows.bridge_widths.empty() ? 0. : windows.bridge_widths.front()) << " mm");
    INFO("layer commands " << seam.layer.moves.size() << ", frozen " << array_count(layer5_commands));
    for (const std::string &window : windows.unaddressed)
        INFO("window not addressed by exactly one native command: " << window);
    for (const SignatureMismatch &mismatch : measured.signature)
        INFO(describe_command(seam.layer, mismatch.row) << ": " << signature_field_name(mismatch.field)
                                                         << " differs");
    for (const FanViolation &violation : measured.fan)
        INFO(describe_command(seam.layer, violation.row) << ": expected " << violation.wanted << "%, emitted "
                                                         << violation.emitted << "%");

    // The whole layer, command by command, as frozen: role, 0.4/0.45 width, both endpoints, delta E
    // and order. A split, reordered, reshaped or re-extruded command fails here, so no fan reading
    // below can be about a command that moved.
    REQUIRE(measured.signature.empty());
    // Every one of the layer's overhang walls is addressed by exactly one whole command: a missing,
    // duplicated or split window fails here rather than passing quietly.
    REQUIRE(windows.unaddressed.empty());
    REQUIRE(windows.complete == 4);
    REQUIRE(windows.partial == 2);
    REQUIRE(windows.adjacent == 4);
    REQUIRE(windows.detached == 8);
    CHECK(windows.complete + windows.partial + windows.adjacent + windows.detached == array_count(seam_windows));
    REQUIRE(windows.bridges > 0);
    REQUIRE(windows.bridge_lane > 0);

    // The footprint is as wide as the commands themselves: 0.4, not the 0.45 of the layer's other
    // extrusions, or the partial walls would read as covered end to end.
    CHECK(windows.seam_widths   == std::vector<double>{ 0.4 });
    CHECK(windows.bridge_widths == std::vector<double>{ 0.4 });

    // -1 keeps the old behaviour: the bridge inherits the overhang fan, and so does every wall.
    CHECK(windows.bridge_fans      == std::vector<int>{ expected });
    CHECK(windows.bridge_lane_fans == std::vector<int>{ expected });
    // The walls the bridge covers end to end take its fan, on both sides of the print.
    CHECK(windows.complete_fans == std::vector<int>{ expected });

    // The ones it only comes near keep the overhang fan: the same ring with a real gap on the
    // material side, the ring beside it, which another wall separates from the bridge, and the rest.
    CHECK(windows.partial_fans  == std::vector<int>{ overhang_fan_percent });
    CHECK(windows.adjacent_fans == std::vector<int>{ overhang_fan_percent });
    CHECK(windows.detached_fans == std::vector<int>{ overhang_fan_percent });

    // And every single command of the layer at its own expected fan, the layer's inner, outer and
    // infill lines included: this is where a fan that leaked onto a wall no group names shows up.
    CHECK(measured.fan.empty());

    // The reading is only an oracle if it rejects the answers a wrong rule gives, and if it names
    // the field that moved.
    const auto judge_mutant = [&](const char *what, const NativeLayer &layer) {
        const NativeLayerReading mutant = judge_layer(layer, seam.centre, expected);
        INFO("mutant \"" << what << "\": " << mutant.signature.size() << " signature mismatch(es), "
                         << mutant.fan.size() << " fan violation(s)");
        for (const FanViolation &violation : mutant.fan)
            INFO("  " << describe_command(layer, violation.row) << ", expected " << violation.wanted << "%");
        CHECK_FALSE(layer_contract_holds(mutant, expected));
        return mutant;
    };
    // Under -1 the bridge fan is the overhang fan and a misroute would be invisible, so the paint
    // below uses 0%: what is checked is that a wall at any fan but its own cannot pass.
    const int misrouted_fan = expected != overhang_fan_percent ? expected : 0;

    // A partial wall painted with the bridge fan: what a rule that treats proximity as a join gives.
    {
        const SeamWindow *window = seam_window_named("right partial");
        REQUIRE(window != nullptr);
        const size_t row = find_window_move(seam.layer, seam.centre, *window);
        REQUIRE(row != size_t(-1));
        NativeLayer painted = seam.layer;
        painted.moves[row].fan = misrouted_fan;
        const NativeLayerReading mutant = judge_mutant("partial wall at the bridge fan", painted);
        CHECK(mutant.signature.empty());
        CHECK(mutant.fan.size() == measured.fan.size() + 1);
        CHECK(violated(mutant.fan, row));
    }
    // The same paint on a detached wall, which no group of the contract would report.
    {
        const SeamWindow *window = seam_window_named("right adjacent low");
        REQUIRE(window != nullptr);
        const size_t row = find_window_move(seam.layer, seam.centre, *window);
        REQUIRE(row != size_t(-1));
        NativeLayer painted = seam.layer;
        painted.moves[row].fan = misrouted_fan;
        const NativeLayerReading mutant = judge_mutant("detached wall at the bridge fan", painted);
        CHECK(mutant.signature.empty());
        CHECK(mutant.fan.size() == measured.fan.size() + 1);
        CHECK(violated(mutant.fan, row));
    }
    // One command split in two at its midpoint: the writer would then be free to put a fan change
    // between the halves, which one G1 cannot have.
    {
        NativeLayer split = seam.layer;
        const NativeMove whole = split.moves[1];
        NativeMove first = whole;
        first.to = Vec2d(0.5 * (whole.from.x() + whole.to.x()), 0.5 * (whole.from.y() + whole.to.y()));
        first.e  = 0.5 * whole.e;
        NativeMove second = whole;
        second.from = first.to;
        second.e    = whole.e - first.e;
        split.moves[1] = first;
        split.moves.insert(split.moves.begin() + 2, second);
        const NativeLayerReading mutant = judge_mutant("partial wall split in two", split);
        CHECK((! mutant.signature.empty() && mutant.signature.front().field == SignatureField::Count));
    }
    // Two neighbouring commands swapped: order is part of the frozen signature.
    {
        NativeLayer swapped = seam.layer;
        std::swap(swapped.moves[1], swapped.moves[2]);
        const NativeLayerReading mutant = judge_mutant("two neighbouring commands swapped", swapped);
        CHECK(mutant.signature.size() == 2);
    }
    // One command's delta E off by one percent: a fan-only oracle would not notice.
    {
        NativeLayer drifted = seam.layer;
        drifted.moves[2].e *= 1.01;
        const NativeLayerReading mutant = judge_mutant("delta E of a seam wall drifted by 1%", drifted);
        CHECK(mutant.fan.size() == measured.fan.size());
        CHECK((mutant.signature.size() == 1 && mutant.signature.front().row == 2 &&
               mutant.signature.front().field == SignatureField::E));
    }
}

// The cooling buffer's own contract, on layers written out by hand: the paths below are what the
// generator emits around an external bridge, so the exact order of the fan commands can be read.

// Layer 0 is handled by the first-layer override as a whole: an external bridge on it takes no value of
// its own, whatever the filament says.
TEST_CASE("A layer 0 fan override covers an external bridge", "[Cooling][ExternalBridgeFan]")
{
    DynamicPrintConfig config = bridge_fan_config(80);
    config.set_deserialize_strict({ { "initial_layer_fan_speed", 40 } });
    set_external_bridge_fan_speed(config, 30);

    const std::string cooled = cool_layer(config,
        "G1 X10 Y10 F3000\n"
        ";_EXTERNAL_BRIDGE_FAN_START\n"
        ";_OVERHANG_FAN_START\n"
        "G1 X20 Y10 E0.5 F1200 ;_EXTRUDE_SET_SPEED\n"
        ";_OVERHANG_FAN_END\n"
        ";_EXTERNAL_BRIDGE_FAN_END\n"
        "G1 X30 Y10 E1 F1200 ;_EXTRUDE_SET_SPEED\n",
        0, 1);
    INFO("cooled layer 0\n" << cooled);
    CHECK(fan_percents(cooled) == std::vector<int>{ 40 });
}

// The default value is the old behaviour on the same paths: the bridge follows the overhang fan, and the
// commands run base, overhang, base.
TEST_CASE("The default external bridge value leaves the overhang commands of a layer alone",
          "[Cooling][ExternalBridgeFan]")
{
    DynamicPrintConfig config = bridge_fan_config(80);
    set_external_bridge_fan_speed(config, -1);

    const std::string cooled = cool_layer(config,
        "G1 X10 Y10 F3000\n"
        ";_EXTERNAL_BRIDGE_FAN_START\n"
        ";_OVERHANG_FAN_START\n"
        "G1 X20 Y10 E0.5 F1200 ;_EXTRUDE_SET_SPEED\n"
        ";_OVERHANG_FAN_END\n"
        ";_EXTERNAL_BRIDGE_FAN_END\n"
        "G1 X30 Y10 E1 F1200 ;_EXTRUDE_SET_SPEED\n",
        5, 1);
    INFO("cooled layer\n" << cooled);
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 80, 60 });
}

// Two explicit values that meet at a tool change inside the bridge: the new filament's value follows the
// old one directly, with no drop to the base speed in between.
TEST_CASE("A tool change between two explicit bridge speeds does not drop to the layer fan",
          "[Cooling][ExternalBridgeFan]")
{
    const std::string cooled = cool_layer(two_filament_fan_config("30,40"),
        "G1 X10 Y10 F3000\n"
        ";_EXTERNAL_BRIDGE_FAN_START\n"
        ";_OVERHANG_FAN_START\n"
        "G1 X20 Y10 E0.5 F1200 ;_EXTRUDE_SET_SPEED\n"
        "T1\n"
        "G1 X20 Y20 E1 F1200 ;_EXTRUDE_SET_SPEED\n"
        ";_OVERHANG_FAN_END\n"
        ";_EXTERNAL_BRIDGE_FAN_END\n"
        "G1 X20 Y30 E1.5 F1200 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 30, 40, 60 });
}

// The next filament inherits the value (-1): the bridge it continues goes back to its overhang fan.
TEST_CASE("A tool change from an explicit bridge speed to an inherited one uses the overhang fan",
          "[Cooling][ExternalBridgeFan]")
{
    const std::string cooled = cool_layer(two_filament_fan_config("30,-1"),
        "G1 X10 Y10 F3000\n"
        ";_EXTERNAL_BRIDGE_FAN_START\n"
        ";_OVERHANG_FAN_START\n"
        "G1 X20 Y10 E0.5 F1200 ;_EXTRUDE_SET_SPEED\n"
        "T1\n"
        "G1 X20 Y20 E1 F1200 ;_EXTRUDE_SET_SPEED\n"
        ";_OVERHANG_FAN_END\n"
        ";_EXTERNAL_BRIDGE_FAN_END\n"
        "G1 X20 Y30 E1.5 F1200 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 30, 80, 60 });
}

// The next filament forces no cooling for overhangs or bridges at all, so the bridge the previous one
// opened must not carry over: the layer fan comes back, not that filament's own bridge value.
TEST_CASE("A tool change to a filament without forced bridge cooling drops an explicit bridge speed",
          "[Cooling][ExternalBridgeFan]")
{
    const std::string cooled = cool_layer(two_filament_fan_config("30,50", "1,0"),
        "G1 X10 Y10 F3000\n"
        ";_EXTERNAL_BRIDGE_FAN_START\n"
        ";_OVERHANG_FAN_START\n"
        "G1 X20 Y10 E0.5 F1200 ;_EXTRUDE_SET_SPEED\n"
        "T1\n"
        "G1 X20 Y20 E1 F1200 ;_EXTRUDE_SET_SPEED\n"
        ";_OVERHANG_FAN_END\n"
        ";_EXTERNAL_BRIDGE_FAN_END\n"
        "G1 X20 Y30 E1.5 F1200 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 30, 60 });
}

// The same layer hands the bridge around three filaments in turn: the first takes it over itself, the
// second forces no cooling for overhangs or bridges at all, the third inherits the overhang fan. The
// third one has to come back to its overhang fan; a bridge that opens on one filament and is handed on
// twice in the same layer must not leave it on the layer fan.
TEST_CASE("A bridge handed on to a third filament keeps its overhang fan", "[Cooling][ExternalBridgeFan]")
{
    DynamicPrintConfig config = multifilament_config(3, {
        { "close_fan_the_first_x_layers", "0,0,0" },
        { "enable_arc_fitting",           "disabled" },
        { "enable_overhang_bridge_fan",   "1,0,1" },
        { "external_bridge_fan_speed",    "30,50,-1" },
        { "fan_cooling_layer_time",       "1000,1000,1000" },
        { "fan_kickstart",                0. },
        { "fan_max_speed",                "60,60,60" },
        { "fan_min_speed",                "60,60,60" },
        { "fan_speedup_time",             0. },
        { "full_fan_speed_layer",         "0,0,0" },
        { "initial_layer_fan_speed",      "-1,-1,-1" },
        { "overhang_fan_speed",           "80,80,80" },
        { "part_cooling_fan_min_pwm",     0 },
        { "slow_down_for_layer_cooling",  false },
        { "slow_down_layer_time",         "0,0,0" },
    });

    Print print;
    Model model;
    // One bridge per filament, in filament order, so a single layer prints all three in turn.
    std::vector<std::vector<ConfigBase::SetDeserializeItem>> extruders = {
        { { "extruder", 1 } },
        { { "extruder", 2 } },
        { { "extruder", 3 } },
    };
    init_print({ mesh(TestMesh::bridge), mesh(TestMesh::bridge), mesh(TestMesh::bridge) }, print, model, config, &extruders);
    GCodeProcessorResult result;
    REQUIRE(!gcode(print, &result).empty());

    const std::map<unsigned int, std::vector<int>> fans = bridge_fans_by_filament(result);
    REQUIRE(fans.count(0) == 1);
    REQUIRE(fans.count(1) == 1);
    REQUIRE(fans.count(2) == 1);
    INFO("filament 1 " << fans.at(0).front() << "%, filament 2 " << fans.at(1).front() << "%, filament 3 "
                       << fans.at(2).front() << "%");
    CHECK(fans.at(0) == std::vector<int>{ 30 }); // explicit, above its own overhang fan
    CHECK(fans.at(1) == std::vector<int>{ 60 }); // forced cooling off: the layer fan
    CHECK(fans.at(2) == std::vector<int>{ 80 }); // inherits: the overhang fan, not the layer fan it fell back to
}

// The filament a tool change brings in inherits nothing useful: no bridge value of its own, and its
// overhang fan sits below the layer fan. The bridge falls back to the layer fan, and the overhang
// request the previous filament opened must not reach into it.
TEST_CASE("A tool change to an inherited value below the layer fan drops to the layer fan",
          "[Cooling][ExternalBridgeFan]")
{
    const std::string cooled = cool_layer(two_filament_fan_config("30,-1", "1,1", "80,40"),
        "G1 X10 Y10 F3000\n"
        ";_EXTERNAL_BRIDGE_FAN_START\n"
        ";_OVERHANG_FAN_START\n"
        "G1 X20 Y10 E0.5 F1200 ;_EXTRUDE_SET_SPEED\n"
        "T1\n"
        "G1 X20 Y20 E1 F1200 ;_EXTRUDE_SET_SPEED\n"
        ";_OVERHANG_FAN_END\n"
        "G1 X20 Y30 E1.5 F1200 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    // 60% base, 30% on the first filament's bridge, then straight back to 60%: the second filament's
    // own overhang fan (40%) is below the layer fan, so it must never show up.
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 30, 60 });
}

// An old preset carries no external bridge fan speed at all, so nothing in this layer belongs to the new
// feature and the cooling buffer has to emit exactly what it always did: the layer fan, the first
// filament's overhang fan, back to the layer fan at the tool change, and - as it always did - the second
// filament's own overhang fan when the region closes, even though that fan speed sits below the layer
// fan. The trailing 40% is the old behaviour of an overhang region that a tool change interrupts; this
// feature deliberately does not change it.
TEST_CASE("A print without an external bridge fan speed keeps its old overhang commands",
          "[Cooling][ExternalBridgeFan]")
{
    // No `external_bridge_fan_speed` key: the option keeps its default and this is an old preset.
    DynamicPrintConfig config = multifilament_config(2, {
        { "close_fan_the_first_x_layers", "0,0" },
        { "enable_arc_fitting",           "disabled" },
        { "enable_overhang_bridge_fan",   "1,1" },
        { "fan_cooling_layer_time",       "0,0" },
        { "fan_kickstart",                0. },
        { "fan_max_speed",                "60,60" },
        { "fan_min_speed",                "60,60" },
        { "fan_speedup_time",             0. },
        { "full_fan_speed_layer",         "0,0" },
        { "initial_layer_fan_speed",      "-1,-1" },
        { "overhang_fan_speed",           "80,40" },
        { "part_cooling_fan_min_pwm",     0 },
        { "reduce_fan_stop_start_freq",   "1,1" },
        { "slow_down_for_layer_cooling",  false },
        { "slow_down_layer_time",         "0,0" },
    });

    const std::string cooled = cool_layer(config,
        "G1 X10 Y10 F3000\n"
        ";_OVERHANG_FAN_START\n"
        "G1 X20 Y10 E0.5 F1200 ;_EXTRUDE_SET_SPEED\n"
        "T1\n"
        "G1 X20 Y20 E1 F1200 ;_EXTRUDE_SET_SPEED\n"
        ";_OVERHANG_FAN_END\n"
        "G1 X20 Y30 E1.5 F1200 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 80, 60, 40 });
}

// ---------------------------------------------------------------------------------------------------
// External bridge seam fan: the cooling-buffer side of the reserved motion-local token.
//
// The G-code generator marks the ordinary wall line that forms the seam against an external bridge with
// the inline comment `;_EXTERNAL_BRIDGE_SEAM_FAN`. Unlike the `_EXTERNAL_BRIDGE_FAN_START` / `_END` pair
// it opens no region: it identifies exactly the depositing line it is attached to, that line takes the
// same resolved external bridge fan the bridge fill gets, and every other command the layer emits stays as
// it was. These cases drive the real CoolingBuffer with hand-written layers, so a fan reading - which
// speed is in force while a given deposit prints - is what they assert first.
// ---------------------------------------------------------------------------------------------------

// Two filaments with a flat 60% layer fan and a 80% overhang fan. `external_bridge` is the per-filament
// external bridge fan speed the seam cases vary, `overhang_bridge_fan` the per-filament "force cooling for
// overhangs and bridges" switch. The layer-level gates are off here; a case that needs one sets it on the
// config it gets back. Layer cooling cannot bend these speeds: no threshold of these layers is reachable
// (fan_cooling_layer_time 0) and the layer slowdown is off.
static DynamicPrintConfig seam_fan_config(const char *external_bridge, const char *overhang = "80,80",
                                          const char *overhang_bridge_fan = "1,1")
{
    // The option has to be part of this build: without it these cases would compare two runs that are
    // equal because neither of them wrote the key.
    REQUIRE(print_config_def.has("external_bridge_fan_speed"));
    return multifilament_config(2, {
        { "close_fan_the_first_x_layers", "0,0" },
        { "enable_arc_fitting",           "disabled" },
        { "enable_overhang_bridge_fan",   overhang_bridge_fan },
        { "external_bridge_fan_speed",    external_bridge },
        { "fan_cooling_layer_time",       "0,0" },
        { "fan_kickstart",                0. },
        { "fan_max_speed",                "60,60" },
        { "fan_min_speed",                "60,60" },
        { "fan_speedup_time",             0. },
        { "full_fan_speed_layer",         "0,0" },
        { "initial_layer_fan_speed",      "-1,-1" },
        { "overhang_fan_speed",           overhang },
        { "part_cooling_fan_min_pwm",     0 },
        { "reduce_fan_stop_start_freq",   "1,1" },
        { "slow_down_for_layer_cooling",  false },
        { "slow_down_layer_time",         "0,0" },
    });
}

// The part cooling fan speed in force while the G-code line holding `move` prints. A fan command written
// after a line changes the fan only for the moves that follow it, so the speed a deposit runs at is the
// last part fan command before its line. The deposit is found by a word that only it carries, because the
// marker comment itself is an annotation that must not reach the G-code.
static int seam_fan_in_force_at(const std::string &gcode, const std::string &move)
{
    const size_t line_at = gcode.find(move);
    REQUIRE(line_at != std::string::npos);
    int                fan = -1;
    std::istringstream before(gcode.substr(0, line_at));
    for (std::string line; std::getline(before, line);) {
        if ((line.rfind("M106", 0) != 0 && line.rfind("M107", 0) != 0) || line.find("P2") != std::string::npos)
            continue;
        const size_t s = line.find('S');
        fan = s == std::string::npos ? 0 : int(std::lround(std::atof(line.c_str() + s + 1) * 100. / 255.));
    }
    return fan;
}

// The marker is motion-local: the line it is attached to prints at the external bridge fan, and the very
// next ordinary line is back on the 60% layer fan. Each extrusion carries its own F so the cooling buffer
// keeps it as a separate line instead of folding it into the previous speed block.
TEST_CASE("A seam-marked wall line takes the external bridge fan for its own deposit only",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const std::string cooled = cool_layer(seam_fan_config("30,30"),
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    // 60% layer fan, the 30% external bridge fan on the seam, 60% again on the wall that follows it.
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 30, 60 });
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 30);
    CHECK(seam_fan_in_force_at(cooled, "X22 Y12") == 60);
    // The token is a cooling-buffer annotation: it must not survive into the emitted G-code.
    CHECK(cooled.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}

// A seam wall is deposited before the bridge fill it borders, so the seam releases the fan again and the
// real external bridge marker then takes the same value over. The seam is not the bridge take-over: after
// the bridge region closes, the layer is back on its own fan.
TEST_CASE("A seam marker hands the layer to the external bridge fill that follows it",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const std::string cooled = cool_layer(seam_fan_config("30,30"),
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        ";_EXTERNAL_BRIDGE_FAN_START\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n"
        ";_EXTERNAL_BRIDGE_FAN_END\n"
        "G1 X23 Y13 E1.5 F1400 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    // Every deposit of the same layer runs at the speed its own state resolves to: the seam wall and the
    // bridge fill it borders at the 30% external bridge fan, the ordinary wall after the bridge at 60%.
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 30);
    CHECK(seam_fan_in_force_at(cooled, "X22 Y12") == 30);
    CHECK(seam_fan_in_force_at(cooled, "X23 Y13") == 60);
}

// The seam must not be mistaken for the bridge take-over. Once a layer is under the Fix3 feature, an
// overhang region opened after it answers to the printing filament's own "force cooling for overhangs and
// bridges" switch instead of the old rule. Filament 1 switches that off, so an implementation that let the
// seam marker set the layer's bridge state would silently drop its 80% overhang fan - the fan readings
// below are what separates the two.
TEST_CASE("A seam marker leaves the bridge state and a later overhang region of the layer alone",
          "[Cooling][ExternalBridgeSeamFan]")
{
    // Filament 0 cools overhangs, filament 1 does not.
    const DynamicPrintConfig config = seam_fan_config("30,30", "80,80", "1,0");
    const std::string        cooled = cool_layer(config,
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n"
        "T1\n"
        ";_OVERHANG_FAN_START\n"
        "G1 X23 Y13 E1.5 F1400 ;_EXTRUDE_SET_SPEED\n"
        ";_OVERHANG_FAN_END\n"
        "G1 X24 Y14 E2 F1500 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    // 60% layer fan, 30% on the seam, then exactly the sequence this layer always had: the tool change to
    // the second filament, its 80% overhang fan, and the layer fan again when the region closes.
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 30, 60, 80, 60 });
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 30);
    CHECK(seam_fan_in_force_at(cooled, "X22 Y12") == 60);
    CHECK(seam_fan_in_force_at(cooled, "X23 Y13") == 80);
    CHECK(seam_fan_in_force_at(cooled, "X24 Y14") == 60);

    // The same layer without the annotation is the baseline the marked one has to keep: the seam may not
    // add, drop or reorder a fan command outside its own deposit.
    const std::string unmarked = cool_layer(config,
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n"
        "T1\n"
        ";_OVERHANG_FAN_START\n"
        "G1 X23 Y13 E1.5 F1400 ;_EXTRUDE_SET_SPEED\n"
        ";_OVERHANG_FAN_END\n"
        "G1 X24 Y14 E2 F1500 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("unmarked layer\n" << unmarked);
    CHECK(fan_percents(unmarked) == std::vector<int>{ 60, 80, 60 });
}

// The value belongs to the filament printing the line: a marked line after a tool change resolves the new
// filament's own external bridge fan, not the one the layer started with.
TEST_CASE("A seam marker after a tool change uses the new filament's external bridge fan",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const std::string cooled = cool_layer(seam_fan_config("30,25"),
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n"
        "T1\n"
        "G1 X23 Y13 E1.5 F1400 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X24 Y14 E2 F1500 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    // Both filaments have the same 60% layer fan, so the tool change itself asks for no new command.
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 30, 60, 25, 60 });
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 30);
    CHECK(seam_fan_in_force_at(cooled, "X23 Y13") == 25);
    CHECK(seam_fan_in_force_at(cooled, "X24 Y14") == 60);
}

// An old preset leaves the key at -1, so a seam marker changes nothing at all: the wall prints on the
// layer fan it always had.
TEST_CASE("A seam marker with no external bridge fan set keeps the layer fan",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const std::string cooled = cool_layer(seam_fan_config("-1,-1"),
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 60);
    CHECK(fan_percents(cooled) == std::vector<int>{ 60 });
    // The annotation never reaches the emitted G-code.
    CHECK(cooled.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}

// The value only exists while the filament forces cooling for overhangs and bridges, the same switch the
// bridge fill answers to: with it off a seam marker is inert even though the speed is set.
TEST_CASE("A seam marker does nothing while the filament forces no overhang cooling",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const std::string cooled = cool_layer(seam_fan_config("30,30", "80,80", "0,0"),
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 60);
    CHECK(fan_percents(cooled) == std::vector<int>{ 60 });
    // The annotation never reaches the emitted G-code.
    CHECK(cooled.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}

// An explicit 0 is a real value, not "unset": the seam deposit prints with the part fan off and the rest of
// the layer keeps its 60%.
TEST_CASE("A seam marker with an explicit zero turns the fan off for its own deposit",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const std::string cooled = cool_layer(seam_fan_config("0,0"),
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 0, 60 });
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 0);
    CHECK(seam_fan_in_force_at(cooled, "X22 Y12") == 60);
}

// The first-layer override is the whole layer's fan, the seam included: layer 0 prints at its 45%.
TEST_CASE("The first-layer fan override outranks a seam marker",
          "[Cooling][ExternalBridgeSeamFan]")
{
    DynamicPrintConfig config = seam_fan_config("30,30");
    config.set_deserialize_strict({ { "initial_layer_fan_speed", "45,45" } });

    const std::string cooled = cool_layer(config,
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n",
        0, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 45);
    CHECK(fan_percents(cooled) == std::vector<int>{ 45 });
    // The move the seam is written after carries no E, so it deposits nothing: the gate's value is the fan
    // in force on that travel, and the seam is not hoisted onto a move that prints no material.
    CHECK(seam_fan_in_force_at(cooled, "X10 Y10") == 45);
    // The gate outranks the marker, but the token is still a cooling-buffer annotation and is consumed on
    // this path too.
    CHECK(cooled.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}

// "No cooling for the first N layers" is a hard gate: as long as it holds, a seam marker cannot lift the
// fan off zero.
TEST_CASE("A seam marker does not lift the fan off the first-layer no-cooling gate",
          "[Cooling][ExternalBridgeSeamFan]")
{
    DynamicPrintConfig config = seam_fan_config("30,30");
    config.set_deserialize_strict({ { "close_fan_the_first_x_layers", "1,1" } });

    const std::string cooled = cool_layer(config,
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n",
        0, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 0);
    CHECK(fan_percents(cooled) == std::vector<int>{ 0 });
    // The annotation never reaches the emitted G-code.
    CHECK(cooled.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}

// The seam rides the "full fan speed at layer" ramp with the value it shares with the bridge fill: layer 1
// of a ramp that reaches full speed at layer 4 is half way, so the 30% bridge fan is 15% and the 60% layer
// fan is 30%.
TEST_CASE("A seam marker rides the fan speed ramp with the bridge fan it shares",
          "[Cooling][ExternalBridgeSeamFan]")
{
    DynamicPrintConfig config = seam_fan_config("30,30");
    config.set_deserialize_strict({ { "full_fan_speed_layer", "4,4" } });

    const std::string cooled = cool_layer(config,
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n",
        1, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(fan_percents(cooled) == std::vector<int>{ 30, 15, 30 });
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 15);
    CHECK(seam_fan_in_force_at(cooled, "X22 Y12") == 30);
}

// A belt printer's band above the belt keeps the fan off whatever role a line belongs to, the seam
// included; leaving the band brings the layer fan back.
TEST_CASE("The belt band above the belt outranks a seam marker",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const std::string cooled = cool_layer(seam_fan_config("30,30"),
        "G1 X10 Y10 F3000\n"
        ";_BELT_BAND_START\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n"
        ";_BELT_BAND_END\n"
        "G1 X23 Y13 E1.5 F1400 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 0, 60 });
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 0);
    CHECK(seam_fan_in_force_at(cooled, "X23 Y13") == 60);
    // The travel before the band carries no E, so it deposits nothing and keeps the 60% it prints at; the
    // band's off value owns the seam and does not reach back to the travel.
    CHECK(seam_fan_in_force_at(cooled, "X10 Y10") == 60);
    // The band outranks the marker, and the token must not leak on the gated path either.
    CHECK(cooled.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}

// The marker belongs to every depositing motion, not just to G1: an arc that deposits the seam is marked
// the same way and takes the same fan.
TEST_CASE("A seam-marked arc deposit takes the external bridge fan too",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const std::string cooled = cool_layer(seam_fan_config("30,30"),
        "G1 X21 Y11 F3000\n"
        "G3 X22 Y12 I1 J0 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X23 Y13 E1 F1300 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(seam_fan_in_force_at(cooled, "X22 Y12") == 30);
    CHECK(seam_fan_in_force_at(cooled, "X23 Y13") == 60);
    // The annotation never reaches the emitted G-code.
    CHECK(cooled.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}

// The shape the generator writes: GCode::_extrude() puts `;_EXTRUDE_SET_SPEED` on its own `G1 F` line, so
// the deposit that carries the token has no feedrate word and nothing on it but the annotation. That line
// is the deposit, so the token has to be honoured there too - a filter that only looks at speed-setting
// lines would miss every real seam.
TEST_CASE("A seam marker on a deposit that follows its speed line still takes the fan",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const std::string cooled = cool_layer(seam_fan_config("30,30"),
        "G1 X10 Y10 F3000\n"
        "G1 F1200 ;_EXTRUDE_SET_SPEED\n"
        "G1 X21 Y11 E0.5 ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 30, 60 });
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 30);
    CHECK(seam_fan_in_force_at(cooled, "X22 Y12") == 60);
    CHECK(cooled.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}

// ---------------------------------------------------------------------------------------------------
// External bridge seam fan: state isolation in the merged cooling-buffer output.
//
// The cases above check one deposit or one region at a time, so an implementation that released the
// fan at the end of the line block, or that kept it for the rest of the layer, would still read the
// same on them. These cases drive the state the buffer really carries: a role region that is still
// open, two marked deposits in a row, the layer boundary, and a tool change with an unmarked deposit
// in between.
// ---------------------------------------------------------------------------------------------------

// Two layers through one CoolingBuffer, the way a print runs them. With `merge` the lower layer is
// buffered and the upper one flushes the pair, so both are parsed and cooled together; without it each
// layer is flushed on its own, which is what an object layer does. Either way the state the lower layer
// leaves is the state the upper layer starts from. No product-side factory is needed for that: the
// public constructor and `process_layer(..., flush)` are the whole interface, and a buffered layer is
// not emitted by contract.
static std::string seam_cool_two_layers(const DynamicPrintConfig &config, const std::string &lower_layer,
                                        const std::string &upper_layer, size_t lower_layer_id,
                                        unsigned int filaments, bool merge)
{
    CoolingTestGCode gcodegen(config, filaments);
    CoolingBuffer     buffer(gcodegen);
    buffer.set_current_extruder(0, 0);
    if (! merge) {
        // Sequenced one call at a time: the order of a print, not the order of a `+` expression.
        std::string out = buffer.process_layer(std::string(lower_layer), lower_layer_id, true);
        out += buffer.process_layer(std::string(upper_layer), lower_layer_id + 1, true);
        return out;
    }
    const std::string buffered = buffer.process_layer(std::string(lower_layer), lower_layer_id, false);
    REQUIRE(buffered.empty());
    return buffer.process_layer(std::string(upper_layer), lower_layer_id + 1, true);
}

// The overhang region the generator opened is still live when the seam wall prints. The marker changes
// the fan of that one deposit only: the next deposit of the region is back on the overhang fan, not on
// the layer fan, and the region's own end still returns to the layer fan.
TEST_CASE("A seam-marked deposit inside a live overhang region returns to the overhang fan",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const std::string cooled = cool_layer(seam_fan_config("30,30"),
        "G1 X10 Y10 F3000\n"
        ";_OVERHANG_FAN_START\n"
        "G1 X20 Y10 E0.5 F1200 ;_EXTRUDE_SET_SPEED\n"
        "G1 X21 Y11 E1 F1300 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1.5 F1400 ;_EXTRUDE_SET_SPEED\n"
        ";_OVERHANG_FAN_END\n"
        "G1 X23 Y13 E2 F1500 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    // 60% layer fan, the 80% overhang fan of the open region, 30% on the marked deposit, 80% again
    // while that region is still open, and 60% once it closes.
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 80, 30, 80, 60 });
    CHECK(seam_fan_in_force_at(cooled, "X20 Y10") == 80);
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 30);
    CHECK(seam_fan_in_force_at(cooled, "X22 Y12") == 80);
    CHECK(seam_fan_in_force_at(cooled, "X23 Y13") == 60);
    CHECK(cooled.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}

// Two marked deposits that touch. The fan stays at the external bridge value across the boundary
// between them - no pulse back to the layer fan in between - and only the ordinary wall after them
// drops back. The fan sequence is the whole order, so a 60% pulse between the two 30% deposits would
// show up as an extra entry.
TEST_CASE("Two adjacent seam-marked deposits keep one uninterrupted external bridge fan",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const std::string cooled = cool_layer(seam_fan_config("30,30"),
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X23 Y13 E1.5 F1400 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 30, 60 });
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 30);
    CHECK(seam_fan_in_force_at(cooled, "X22 Y12") == 30);
    CHECK(seam_fan_in_force_at(cooled, "X23 Y13") == 60);
    CHECK(cooled.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}

// The layer boundary is a buffer boundary too: the last deposit of the lower layer is marked, the first
// real move of the upper layer is not, and both layers go through one CoolingBuffer. The upper layer
// starts on its own layer fan - the 30% belongs to one deposit and must not be what the buffer hands on
// to the layer above. Both shapes a print uses are covered: the two layers parsed together (a buffered
// layer and the object layer that flushes it) and each layer flushed on its own.
TEST_CASE("A seam-marked last deposit does not hand its fan to the layer above",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const DynamicPrintConfig config = seam_fan_config("30,30");
    const std::string        lower =
        "G1 X10 Y10 F3000\n"
        "G1 X20 Y10 E0.5 F1200 ;_EXTRUDE_SET_SPEED\n"
        "G1 X21 Y11 E1 F1300 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n";
    const std::string        upper =
        "G1 X21 Y12 E1.5 F1400 ;_EXTRUDE_SET_SPEED\n"
        "G1 X22 Y13 E2 F1500 ;_EXTRUDE_SET_SPEED\n";

    // The pair parsed and cooled as one, the way a buffered layer and the object layer above it are.
    const std::string merged = seam_cool_two_layers(config, lower, upper, 5, 2, true);
    INFO("merged layers\n" << merged);
    CHECK(fan_percents(merged) == std::vector<int>{ 60, 30, 60 });
    CHECK(seam_fan_in_force_at(merged, "X21 Y11") == 30);
    // The first deposit above the boundary: the layer fan, not the marked deposit's 30%.
    CHECK(seam_fan_in_force_at(merged, "X21 Y12") == 60);
    CHECK(seam_fan_in_force_at(merged, "X22 Y13") == 60);
    CHECK(merged.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);

    // The same two layers with each one flushed on its own: the buffer carries the fan state across the
    // boundary, and what it must not carry is the marker.
    const std::string separate = seam_cool_two_layers(config, lower, upper, 5, 2, false);
    INFO("layers flushed one by one\n" << separate);
    CHECK(fan_percents(separate) == std::vector<int>{ 60, 30, 60 });
    CHECK(seam_fan_in_force_at(separate, "X21 Y11") == 30);
    CHECK(seam_fan_in_force_at(separate, "X21 Y12") == 60);
    CHECK(separate.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}

// The multi-filament sequence of the case above, with the unmarked deposit the generator also emits
// after a tool change: the unmarked deposit prints on the new filament's layer fan (it does not carry
// the previous filament's 30%), the marker after it takes the new filament's own external bridge fan,
// and the wall after that returns to the layer fan. Both filaments print their layer fan at 60%, so the
// tool change itself asks for no new command and the unmarked deposit is the only witness.
TEST_CASE("An unmarked deposit after a tool change does not carry the previous filament's seam fan",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const std::string cooled = cool_layer(seam_fan_config("30,25"),
        "G1 X10 Y10 F3000\n"
        "G1 X21 Y11 E0.5 F1200 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X22 Y12 E1 F1300 ;_EXTRUDE_SET_SPEED\n"
        "T1\n"
        "G1 X23 Y13 E1.5 F1400 ;_EXTRUDE_SET_SPEED\n"
        "G1 X24 Y14 E2 F1500 ;_EXTRUDE_SET_SPEED ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X25 Y15 E2.5 F1600 ;_EXTRUDE_SET_SPEED\n",
        5, 2);
    INFO("cooled layer\n" << cooled);
    CHECK(fan_percents(cooled) == std::vector<int>{ 60, 30, 60, 25, 60 });
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 30);
    CHECK(seam_fan_in_force_at(cooled, "X22 Y12") == 60);
    CHECK(seam_fan_in_force_at(cooled, "X23 Y13") == 60);
    CHECK(seam_fan_in_force_at(cooled, "X24 Y14") == 25);
    CHECK(seam_fan_in_force_at(cooled, "X25 Y15") == 60);
    CHECK(cooled.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}

TEST_CASE("A seam-marked clockwise arc takes the active external bridge fan",
          "[Cooling][ExternalBridgeSeamFan]")
{
    const int external_fan = GENERATE(30, 0, -1);
    const std::string fan = std::to_string(external_fan);
    const std::string logical_filament_fans = fan + "," + fan;
    const std::string cooled = cool_layer(seam_fan_config(logical_filament_fans.c_str()),
        "G1 X21 Y11 F3000\n"
        "G1 F1200 ;_EXTRUDE_SET_SPEED\n"
        "G2 X22 Y12 I1 J0 E0.5 ;_EXTERNAL_BRIDGE_SEAM_FAN\n"
        "G1 X23 Y13 E1\n",
        5, 2);
    INFO("clockwise arc external fan " << external_fan << "\n" << cooled);
    // The whole main fan sequence of the layer, not only the arc's own reading: an explicit value is one
    // pulse between the two 60% states, and `-1` leaves the layer on the single base speed it started on.
    const std::vector<int> expected_fans = external_fan >= 0 ? std::vector<int>{ 60, external_fan, 60 }
                                                             : std::vector<int>{ 60 };
    CHECK(fan_percents(cooled) == expected_fans);
    // The move before the arc carries no E, so it deposits nothing and stays on the layer fan: the seam
    // value belongs to the arc and to the deposit after it, not to the travel that leads into it.
    CHECK(seam_fan_in_force_at(cooled, "X21 Y11") == 60);
    CHECK(seam_fan_in_force_at(cooled, "X22 Y12") == (external_fan >= 0 ? external_fan : 60));
    CHECK(seam_fan_in_force_at(cooled, "X23 Y13") == 60);
    CHECK(cooled.find("_EXTERNAL_BRIDGE_SEAM_FAN") == std::string::npos);
}
