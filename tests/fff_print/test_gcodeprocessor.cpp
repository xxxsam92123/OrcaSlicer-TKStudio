#include <catch2/catch_all.hpp>

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/libslic3r.h"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"

#include "test_helpers.hpp"
#include "test_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <initializer_list>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Point.hpp"
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;

// Bambu firmware uses the " FEATURE: " style reserved tags, everything else the Slic3r-compatible
// "TYPE:" style, so which list applies depends on the printer kind passed in.
TEST_CASE("Reserved keyword detection follows the printer kind it is given", "[GCodeProcessor]")
{
    struct Case
    {
        const char* name;
        std::string gcode;
        bool        reserved_on_bbl;
        bool        reserved_on_non_bbl;
    };

    const auto test_case = GENERATE(values<Case>({
        {"compatible feature tag", ";TYPE:Prime tower", false, true},
        {"compatible layer tag", ";LAYER_CHANGE", false, true},
        {"bbl feature tag", "; FEATURE: Outer wall", true, false},
        {"tag shared by both lists", ";_GP_FIRST_LINE_M73_PLACEHOLDER", true, true},
        {"bbl spells this one with a leading space", ";COLOR_CHANGE", false, true},
        {"ordinary comment", "; heat the bed", false, false},
        {"not a comment at all", "G1 X10 Y10 F3000", false, false},
        // A tag counts only as the whole comment's prefix, so neither a tag mentioned mid-comment
        // nor one trailing a real command is a reserved use.
        {"tag text later in the comment", "; the TYPE:Prime tower marker", false, false},
        {"tag trailing a command", "G1 X10 ;TYPE:Prime tower", false, false},
    }));

    DYNAMIC_SECTION(test_case.name)
    {
        std::vector<std::string> tags;
        REQUIRE(GCodeProcessor::contains_reserved_tags(test_case.gcode, 5, tags, true) == test_case.reserved_on_bbl);

        tags.clear();
        REQUIRE(GCodeProcessor::contains_reserved_tags(test_case.gcode, 5, tags, false) == test_case.reserved_on_non_bbl);
    }
}

TEST_CASE("Temperature preview follows the targeted nozzle across mapped tool changes", "[GCodeProcessor]")
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    config.single_extruder_multi_material.value = true;
    config.filament_diameter.values = {1.75, 1.75};
    config.filament_map.values = {2, 1};
    config.physical_extruder_map.values = {1, 0};
    config.nozzle_diameter.values = {0.4, 0.4};
    config.nozzle_volume.values = {0.0, 0.0};

    ScopedTemporaryFile temp(".gcode");
    {
        std::ofstream output(temp.string());
        output << "M83\n"
                  "M104 S270 T1\n"
                  "T1 H-1\n"
                  "; FEATURE: Outer wall\n"
                  "G1 X10 Y10 Z0.2 F600\n"
                  "G1 X20 Y20 E1 F1200\n"
                  "M104 S140 T0\n"
                  "G1 X30 Y30 E1\n"
                  "M109 S230 T0\n"
                  "G1 X35 Y35 E1\n"
                  "T0 H-1\n"
                  "G1 X40 Y40 E1\n";
    }

    GCodeProcessor processor;
    processor.apply_config(config);
    processor.process_file(temp.string());

    std::vector<float> extrusion_temperatures;
    for (const GCodeProcessorResult::MoveVertex& move : processor.get_result().moves) {
        if (move.type == EMoveType::Extrude &&
            (move.position[Y] == 20.0f || move.position[Y] == 30.0f || move.position[Y] == 35.0f ||
            move.position[Y] == 40.0f))
            extrusion_temperatures.push_back(move.temperature);
    }

    REQUIRE(extrusion_temperatures.size() == 4);
    CHECK_THAT(extrusion_temperatures[0], Catch::Matchers::WithinAbs(270.0f, 0.01f));
    CHECK_THAT(extrusion_temperatures[1], Catch::Matchers::WithinAbs(270.0f, 0.01f));
    CHECK_THAT(extrusion_temperatures[2], Catch::Matchers::WithinAbs(270.0f, 0.01f));
    CHECK_THAT(extrusion_temperatures[3], Catch::Matchers::WithinAbs(230.0f, 0.01f));
}

namespace {

// Temperatures of the extrusion vertices at the given Y positions, in file order (MoveVertex::position
// is in mm).
std::vector<float> extrusion_temperatures_at(const Slic3r::GCodeProcessor& processor, const std::vector<float>& ys)
{
    std::vector<float> temperatures;
    for (const Slic3r::GCodeProcessorResult::MoveVertex& move : processor.get_result().moves) {
        if (move.type != Slic3r::EMoveType::Extrude)
            continue;
        for (float y : ys)
            if (move.position[Slic3r::Y] == y) {
                temperatures.push_back(move.temperature);
                break;
            }
    }
    return temperatures;
}

// Two filaments on one physical hotend: the temperature commands carry no T, and the A parameter is a
// plain "every hotend" flag.
void write_single_hotend_two_filament_gcode(const std::string& path)
{
    std::ofstream output(path);
    output << "M83\n"
              "M104 S240\n"
              "T0 H-1\n"
              "; FEATURE: Outer wall\n"
              "G1 X10 Y10 Z0.2 F600\n"
              "G1 X20 Y20 E1 F1200\n"
              "M104 S230\n"
              "T1 H-1\n"
              "G1 X30 Y30 E1 F1200\n";
}

} // namespace

// The A parameter is a "every hotend" flag only in its valueless form. Other dialects use it as a
// valued parameter (Snapmaker U1 ships `M104 S0 T0 A0`), where it must not swallow the T target and
// must not fan the temperature out to every hotend.
TEST_CASE("Temperature preview does not let a valued A parameter override the T target", "[GCodeProcessor]")
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    config.single_extruder_multi_material.value = true;
    config.filament_diameter.values = {1.75, 1.75};
    config.filament_map.values = {2, 1};
    config.physical_extruder_map.values = {1, 0};
    config.nozzle_diameter.values = {0.4, 0.4};
    config.nozzle_volume.values = {0.0, 0.0};

    ScopedTemporaryFile temp(".gcode");
    {
        std::ofstream output(temp.string());
        output << "M83\n"
                  "M104 S270 T1\n"
                  "T1 H-1\n"
                  "; FEATURE: Outer wall\n"
                  "G1 X10 Y10 Z0.2 F600\n"
                  "G1 X20 Y20 E1 F1200\n"
                  "M104 S0 T0 A0\n"
                  "G1 X30 Y30 E1 F1200\n"
                  "T0 H-1\n"
                  "M104 S240 A\n"
                  "T1 H-1\n"
                  "G1 X40 Y40 E1 F1200\n";
    }

    GCodeProcessor processor;
    processor.apply_config(config);
    processor.process_file(temp.string());

    const std::vector<float> temperatures = extrusion_temperatures_at(processor, {20.0f, 30.0f, 40.0f});
    REQUIRE(temperatures.size() == 3);
    CHECK_THAT(temperatures[0], Catch::Matchers::WithinAbs(270.0f, 0.01f));
    // `M104 S0 T0 A0` addresses hotend 0 only: the active hotend (1) keeps its own temperature.
    CHECK_THAT(temperatures[1], Catch::Matchers::WithinAbs(270.0f, 0.01f));
    // The valueless `M104 S240 A` does address every hotend. It is issued while hotend 0 is the active
    // one, so hotend 1 reading 240 afterwards is only possible if the command really wrote every slot:
    // a predicate that missed the valueless form would leave hotend 1 at its previous 270.
    CHECK_THAT(temperatures[2], Catch::Matchers::WithinAbs(240.0f, 0.01f));
}

// The same resolution on a printer that does not map filaments to physical hotends: the commands carry
// no T, so every filament's path has to show the temperature that was set while it was printing.
TEST_CASE("Temperature preview follows the active hotend without a physical extruder map", "[GCodeProcessor]")
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    config.single_extruder_multi_material.value = true;
    config.filament_diameter.values = {1.75, 1.75};
    config.filament_map.values = {1, 1};
    config.physical_extruder_map.values = {0, 0};
    config.nozzle_diameter.values = {0.4, 0.4};
    config.nozzle_volume.values = {0.0, 0.0};

    ScopedTemporaryFile temp(".gcode");
    write_single_hotend_two_filament_gcode(temp.string());

    GCodeProcessor processor;
    processor.apply_config(config);
    processor.process_file(temp.string());

    const std::vector<float> temperatures = extrusion_temperatures_at(processor, {20.0f, 30.0f});
    REQUIRE(temperatures.size() == 2);
    CHECK_THAT(temperatures[0], Catch::Matchers::WithinAbs(240.0f, 0.01f));
    CHECK_THAT(temperatures[1], Catch::Matchers::WithinAbs(230.0f, 0.01f));
}

TEST_CASE("Reserved keyword detection reports every offending line", "[GCodeProcessor]")
{
    const std::string gcode = ";TYPE:Prime tower\nG1 X10\n;LAYER_CHANGE\n";

    std::vector<std::string> tags;
    REQUIRE(GCodeProcessor::contains_reserved_tags(gcode, 5, tags, false));
    REQUIRE(tags.size() == 2);
    // Reported in the order they appear, which is what makes the max_count cut-off meaningful.
    CHECK(tags[0] == "TYPE:Prime tower");
    CHECK(tags[1] == "LAYER_CHANGE");

    SECTION("the reported count is capped at max_count")
    {
        tags.clear();
        REQUIRE(GCodeProcessor::contains_reserved_tags(gcode, 1, tags, false));
        CHECK(tags.size() == 1);
        CHECK(tags[0] == "TYPE:Prime tower");
    }

    SECTION("a max_count of zero still reports the first tag")
    {
        tags.clear();
        REQUIRE(GCodeProcessor::contains_reserved_tags(gcode, 0, tags, false));
        CHECK(tags.size() == 1);
    }

    SECTION("g-code with nothing reserved in it reports nothing")
    {
        tags.clear();
        CHECK_FALSE(GCodeProcessor::contains_reserved_tags("G28\n; home all axes\n", 5, tags, false));
        CHECK(tags.empty());
    }
}

namespace {

// s_IsBBLPrinter selects the "; FEATURE: " role tags the G-code uses.
void process_gcode(const std::string &gcode, GCodeProcessorResult &result, const FullPrintConfig &config)
{
    const bool       was_bbl_printer = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl_printer([was_bbl_printer] { GCodeProcessor::s_IsBBLPrinter = was_bbl_printer; });
    GCodeProcessor::s_IsBBLPrinter = true;
    ScopedTemporaryFile temp(".gcode");
    std::ofstream(temp.string()) << gcode;
    GCodeProcessor processor;
    processor.apply_config(config);
    processor.process_file(temp.string());
    result = std::move(processor.extract_result());
}

void process_gcode(const std::string &gcode, GCodeProcessorResult &result)
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    process_gcode(gcode, result, config);
}

// Closed outer-wall squares, each after a fast travel and before an inner-wall move, so the processor
// records seams and inserts actual speed moves. virtual_moves adds a VG1 move after each square.
void process_squares(int squares, GCodeProcessorResult &result, bool virtual_moves = false)
{
    std::ostringstream gcode;
    gcode << "M83\nG90\n";
    for (int i = 0; i < squares; ++i) {
        gcode << "G1 X10 Y10 Z" << 0.2 * (i + 1) << " F12000\n"
              << "; FEATURE: Outer wall\n"
              << "G1 X50 Y10 E2 F3000\nG1 X50 Y50 E2\nG1 X10 Y50 E2\nG1 X10 Y10 E2\n"
              << "; FEATURE: Inner wall\n"
              << "G1 X12 Y12 E0.1\nG1 X30 Y12 E1\n";
        if (virtual_moves)
            gcode << "VG1 X20 Y30 F12000\n";
    }
    process_gcode(gcode.str(), result);
}

// Objects A and B on the first layer and A again on the second, with A's brim and support. The skirt and the
// prime tower belong to neither.
void process_two_objects(GCodeProcessorResult &result)
{
    std::ostringstream gcode;
    gcode << "M83\nG90\n"
          << "; CHANGE_LAYER\n; LAYER_HEIGHT: 0.2\nG1 Z0.2 F12000\n"
          << "; FEATURE: Skirt\nG1 X0 Y100 E5 F3000\n"
          << "; FEATURE: Brim\nG1 X8 Y8 F12000\nG1 X12 Y8 E1 F3000\n"
          << "; FEATURE: Support\nG1 X10 Y20 F12000\nG1 X10 Y30 E1 F3000\n"
          << "; FEATURE: Outer wall\nG1 X10 Y10 F12000\nG1 X20 Y10 E1 F3000\n"
          << "; FEATURE: Outer wall\nG1 X50 Y50 F12000\nG1 X60 Y50 E2 F3000\n"
          << "; FEATURE: Prime tower\nG1 X80 Y80 F12000\nG1 X90 Y80 E1 F3000\n"
          << "; CHANGE_LAYER\n; LAYER_HEIGHT: 0.2\nG1 Z0.4 F12000\n"
          << "; FEATURE: Outer wall\nG1 X10 Y10 F12000\nG1 X20 Y10 E1 F3000\n";
    process_gcode(gcode.str(), result);
}

// Bead centers of process_two_objects(), half the 0.2 mm layer below the nozzle.
const Vec3d a_brim(10., 8., 0.1), a_support(10., 25., 0.1), a_wall_0(15., 10., 0.1), a_wall_1(15., 10., 0.3), b_wall(55., 50., 0.1);

Vec3d center_of(const GCodeProcessorResult::ObjectMass::Sum &sum) { return sum.moment / sum.mass; }

// One filament, so each bead weighs as much as the E it was extruded with.
Vec3d weighted_center(std::initializer_list<std::pair<double, Vec3d>> beads)
{
    double mass = 0.;
    Vec3d  moment = Vec3d::Zero();
    for (const auto &[e, center] : beads) {
        mass += e;
        moment += e * center;
    }
    return moment / mass;
}

bool is_block_move(const GCodeProcessorResult::MoveVertex &move)
{
    return !move.internal_only && (move.type == EMoveType::Extrude || move.type == EMoveType::Travel);
}

} // namespace

// The start G-code runs in a "prepare" stage before the printed part: heat, home, prime. The processor
// marks its moves with MoveVertex::prepare_stage and counts their time as prepare time instead of a
// layer's, which is what lets the preview leave them out of the first printed layer's paths.
TEST_CASE("Start G-code prepare-stage moves are marked apart from the first printed layer", "[GCodeProcessor]")
{
    // The role tag before the first G1 opens the prepare stage. The prime line is an extrusion that
    // belongs to no layer, pinned to the first layer's Z.
    const std::string gcode =
        "M83\n"
        "G90\n"
        "; FEATURE: Custom\n"
        "G1 Z5.8 F600\n"
        "G1 X290 Y10 E10 F12000\n"
        "; CHANGE_LAYER\n"
        "; LAYER_HEIGHT: 0.2\n"
        "; FEATURE: Outer wall\n"
        "G1 X10 Y50 Z0.2 F12000\n"
        "G1 X200 Y50 E5 F1800\n";

    GCodeProcessorResult result;
    process_gcode(gcode, result);

    const auto extrusion_ending_at = [&result](float x) {
        return std::find_if(result.moves.begin(), result.moves.end(), [x](const GCodeProcessorResult::MoveVertex& move) {
            return move.type == EMoveType::Extrude && std::fabs(move.position[X] - x) < 0.001f;
        });
    };

    const auto prime = extrusion_ending_at(290.0f);
    REQUIRE(prime != result.moves.end());
    CHECK(prime->prepare_stage);

    const auto wall = extrusion_ending_at(200.0f);
    REQUIRE(wall != result.moves.end());
    CHECK_FALSE(wall->prepare_stage);
    // A prepare-stage move precedes the first printed one, so the preview has the position to start
    // the first printed path from instead of the origin.
    CHECK(std::any_of(result.moves.begin(), wall, [](const GCodeProcessorResult::MoveVertex& move) { return move.prepare_stage; }));

    SECTION("every vertex of the prepare-stage extrusion is marked, its actual-speed splits too")
    {
        // The prime line's vertices are the only extrusions below Y 20, and the move is long and fast
        // enough for the acceleration to split it into more than its own vertex.
        size_t vertices = 0;
        for (const GCodeProcessorResult::MoveVertex& move : result.moves) {
            if (move.type != EMoveType::Extrude || move.position[Y] >= 20.0f)
                continue;
            CAPTURE(move.position[X], move.position[Y], move.internal_only);
            ++vertices;
            CHECK(move.prepare_stage);
        }
        CHECK(vertices > 1);
    }

    SECTION("no vertex of the first printed extrusion is marked")
    {
        size_t vertices = 0;
        for (const GCodeProcessorResult::MoveVertex& move : result.moves) {
            if (move.type != EMoveType::Extrude || move.position[Y] < 20.0f)
                continue;
            CAPTURE(move.position[X], move.position[Y], move.internal_only);
            ++vertices;
            CHECK_FALSE(move.prepare_stage);
        }
        CHECK(vertices > 1);
    }

    SECTION("the prepare stage's time is counted as prepare time, not as a layer's")
    {
        const size_t normal = static_cast<size_t>(PrintEstimatedStatistics::ETimeMode::Normal);
        double       prepare_move_time = 0.0;
        for (const GCodeProcessorResult::MoveVertex& move : result.moves)
            if (move.prepare_stage)
                prepare_move_time += double(move.time[normal]);
        CHECK(prepare_move_time > 0.0);
        // The same time the time blocks tagged, so the total print time still holds it.
        CHECK_THAT(double(result.print_statistics.modes[normal].prepare_time), Catch::Matchers::WithinRel(prepare_move_time, 1e-3));
        CHECK(result.print_statistics.modes[normal].prepare_time < result.print_statistics.modes[normal].time);
    }
}

// Arc fitting can leave a segment shorter than the 0.001 mm the writer prints, with a positive E still
// on it. Dividing that filament by a length no nozzle travelled reports a bead cross section of tens of
// mm2, and the preview shows a flow of thousands of mm3/s for a move it cannot even draw.
TEST_CASE("A sub-resolution extrusion reports no bead cross section", "[GCodeProcessor][Regression]")
{
    // The middle move covers 3.32e-6 mm in X and Y, five and thirteen steps of the float the processor
    // holds Marlin coordinates in at this magnitude: below both the 0.001 mm the writer prints and the
    // 1e-4 EPSILON the processor treats as no displacement. Only its E is a real, positive amount.
    const std::string gcode =
        "M83\n"
        "G90\n"
        "; FEATURE: Outer wall\n"
        "G1 Z0.2 F12000\n"
        "G1 X8 Y2 F12000\n"
        "G1 X2 Y2 E0.2 F3000\n"
        "G1 X2.000001192 Y2.0000031 E.00003\n"
        "G1 X12 Y2 E0.33 F3000\n";

    GCodeProcessorResult result;
    process_gcode(gcode, result);

    // A bead cannot be wider than a nozzle nor taller than a layer: the real moves here are 0.4 x 0.2 mm,
    // 0.08 mm2, and even 1 mm2 would take a nozzle this printer does not have. The division above reports
    // about 21 mm2 instead.
    //
    // Flow alone is not enough to pin the defect down: clamping mm3_per_mm would satisfy it while still
    // recording a bead on a move the head never made. So the recorded geometry is judged too, but only for
    // vertices whose reported flow is already plausible - a vertex that fails the flow check is reported
    // once, not twice. The head travelled 3.32e-6 mm, which the processor itself treats as no displacement
    // (EPSILON, 1e-4), so that vertex must not survive as an extrusion vertex within EPSILON of the
    // extrusion vertex before it. Every Extrude move does carry some XY displacement (a move without one
    // is not typed Extrude), so a legitimate extrusion vertex cannot trip this check.
    bool  has_previous_extrusion = false;
    Vec2f previous_extrusion     = Vec2f::Zero();
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        CAPTURE(move.position[X], move.position[Y], move.mm3_per_mm, move.volumetric_rate());
        CHECK(move.mm3_per_mm < 1.0f);
        const Vec2f end(move.position[X], move.position[Y]);
        if (move.mm3_per_mm < 1.0f && has_previous_extrusion) {
            CAPTURE(previous_extrusion[X], previous_extrusion[Y], (end - previous_extrusion).norm());
            CHECK((end - previous_extrusion).norm() >= float(EPSILON));
        }
        previous_extrusion     = end;
        has_previous_extrusion = true;
    }

    // Leaving the sub-resolution move out must keep the two real extrusions, and their flow, where they
    // were: the one before it ends at X 2 and the one after it at X 12. The sub-resolution endpoint sits
    // 1.19e-6 further in X (and 3.32e-6 further in the plane), so a 5e-7 window - about two float steps at
    // this magnitude - matches the intended endpoint and can capture neither of the other two.
    const auto extrusion_ending_at = [&result](float x) {
        return std::find_if(result.moves.begin(), result.moves.end(), [x](const GCodeProcessorResult::MoveVertex &move) {
            return move.type == EMoveType::Extrude && std::fabs(move.position[X] - x) < 5.0e-7f;
        });
    };
    // 1.75 mm filament is 2.405 mm2 across, so 0.2 mm of it over 6 mm is a 0.080 mm2 bead.
    const double filament_cross_section = M_PI * 0.875 * 0.875;
    // Both real extrusions run at F3000, i.e. 50 mm/s, so their preview flow is their bead times 50.
    const double feedrate_mm_s = 3000.0 / 60.0;
    const auto   before        = extrusion_ending_at(2.0f);
    REQUIRE(before != result.moves.end());
    CAPTURE(before->position[X], before->position[Y], before->mm3_per_mm, before->volumetric_rate());
    CHECK_THAT(double(before->mm3_per_mm), Catch::Matchers::WithinRel(filament_cross_section * 0.2 / 6.0, 1e-3));
    CHECK_THAT(double(before->volumetric_rate()), Catch::Matchers::WithinRel(filament_cross_section * 0.2 / 6.0 * feedrate_mm_s, 1e-3));

    // And 0.33 mm of it over 10 mm is a 0.079 mm2 bead.
    const auto after = extrusion_ending_at(12.0f);
    REQUIRE(after != result.moves.end());
    CAPTURE(after->position[X], after->position[Y], after->mm3_per_mm, after->volumetric_rate());
    CHECK_THAT(double(after->mm3_per_mm), Catch::Matchers::WithinRel(filament_cross_section * 0.33 / 10.0, 1e-3));
    CHECK_THAT(double(after->volumetric_rate()), Catch::Matchers::WithinRel(filament_cross_section * 0.33 / 10.0 * feedrate_mm_s, 1e-3));
}

TEST_CASE("Loaded manual tool change comments switch subsequent moves when enabled", "[GCodeProcessor][Regression]")
{
    const bool manual_filament_change = GENERATE(false, true);
    CAPTURE(manual_filament_change);
    const DynamicPrintConfig config = Test::multifilament_config(2, {
        { "gcode_flavor",           "marlin" },
        { "filament_type",          "PLA;PETG" },
        { "manual_filament_change", manual_filament_change },
    });
    const bool was_bbl_printer = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl_printer([was_bbl_printer] { GCodeProcessor::s_IsBBLPrinter = was_bbl_printer; });
    GCodeProcessor::s_IsBBLPrinter = false;

    ScopedTemporaryFile temp(".gcode");
    {
        std::ofstream out(temp.string());
        REQUIRE(out.is_open());
        out << "; generated by OrcaSlicer\n"
            << "G90\nM83\nT0\n"
            << "G1 X10 Y10 Z0.2 F3000\n"
            << "G1 X20 E1\n"
            << "; MANUAL_TOOL_CHANGE T1\n"
            << "G1 X30\nG1 X40 E1\nG1 X50\nG1 X60 E1\n";
        // Let process_file load the embedded DynamicPrintConfig, as the standalone viewer does.
        out << "; CONFIG_BLOCK_START\n";
        for (const std::string &key : config.keys())
            out << "; " << key << " = " << config.opt_serialize(key) << '\n';
        out << "; CONFIG_BLOCK_END\n";
        REQUIRE(out.good());
    }

    GCodeProcessor processor;
    processor.process_file(temp.string());
    const auto &result = processor.get_result();
    REQUIRE(result.filaments_count == 2);

    constexpr unsigned int tool_change_line = 7;
    size_t before = 0;
    size_t after = 0;
    for (const auto &move : result.moves) {
        if (!is_block_move(move))
            continue;
        CAPTURE(move.gcode_id, move.type);
        if (move.gcode_id < tool_change_line) {
            ++before;
            CHECK(move.extruder_id == 0);
        } else if (move.gcode_id > tool_change_line) {
            ++after;
            CHECK(move.extruder_id == (manual_filament_change ? 1 : 0));
        }
    }
    REQUIRE(before > 0);
    REQUIRE(after > 0);
}

TEST_CASE("Actual speed moves are inserted on their block's segment just before its move", "[GCodeProcessor]")
{
    // 60 squares take several planner passes, which remap the blocks kept between passes.
    const int                  squares       = GENERATE(10, 60);
    const bool                 virtual_moves = GENERATE(false, true);
    GCodeProcessorResult       result;
    process_squares(squares, result, virtual_moves);
    const auto                &moves   = result.moves;
    constexpr size_t           normal  = size_t(PrintEstimatedStatistics::ETimeMode::Normal);

    size_t inserted = 0;
    for (size_t i = 1; i < moves.size(); ++i) {
        if (!moves[i].internal_only)
            continue;
        ++inserted;
        // Inserted moves have zero time, but a VG1 block's time is written to whatever move its move_id names.
        if (!virtual_moves)
            CHECK(moves[i].time[normal] == 0.f);
        size_t block = i + 1;
        while (block < moves.size() && moves[block].internal_only)
            ++block;
        size_t previous = i - 1;
        while (previous > 0 && moves[previous].internal_only)
            --previous;
        REQUIRE(block < moves.size());
        CHECK(moves[block].gcode_id == moves[i].gcode_id);
        const Vec3f segment = moves[block].position - moves[previous].position;
        const Vec3f offset  = moves[i].position - moves[previous].position;
        CHECK(segment.cross(offset).norm() / segment.norm() < 1e-3f);
    }
    REQUIRE(inserted > 0);
}

TEST_CASE("A seam takes the actual speed of the move it follows", "[GCodeProcessor]")
{
    GCodeProcessorResult result;
    // 10 squares fit in one planner pass, so the seam's move and the block after it are timed together.
    process_squares(10, result);
    const auto                &moves  = result.moves;

    size_t seams = 0;
    for (size_t i = 1; i < moves.size(); ++i)
        if (moves[i].type == EMoveType::Seam && is_block_move(moves[i - 1])) {
            ++seams;
            CHECK_THAT(moves[i].actual_feedrate, Catch::Matchers::WithinAbs(moves[i - 1].actual_feedrate, 1e-4));
        }
    REQUIRE(seams > 0);
}

TEST_CASE("A pause keeps the moves around it in order and on their own layers", "[GCodeProcessor][Regression]")
{
    // libvgcode numbers layers from the order of the moves, so the moves around a pause must stay in layer and
    // line order. The pause vertex itself moves to the first extrusion after it and keeps its own line.
    std::ostringstream gcode;
    gcode << "M83\nG90\n";
    for (int layer = 1; layer <= 3; ++layer) {
        gcode << "; CHANGE_LAYER\n; LAYER_HEIGHT: 0.2\n";
        if (layer == 2)
            gcode << "G1 E-0.8 F3000\n; PAUSE_PRINTING\nM601\n";
        gcode << "G1 X10 Y10 F12000\nG1 Z" << 0.2 * layer << "\n";
        if (layer == 2)
            gcode << "G1 E0.8 F3000\n";
        gcode << "; FEATURE: Outer wall\n"
              << "G1 X50 Y10 E2 F3000\nG1 X50 Y50 E2\nG1 X10 Y50 E2\nG1 X10 Y10 E2\n";
    }
    GCodeProcessorResult result;
    process_gcode(gcode.str(), result);
    const auto &moves = result.moves;

    REQUIRE(result.custom_gcode_per_print_z.size() == 1);
    CHECK_THAT(result.custom_gcode_per_print_z.front().print_z, Catch::Matchers::WithinAbs(0.4, 1e-4));
    REQUIRE(std::count_if(moves.begin(), moves.end(), [](const auto &move) { return move.internal_only; }) > 0);

    const auto pause = std::find_if(moves.begin(), moves.end(), [](const auto &move) { return move.type == EMoveType::Pause_Print; });
    REQUIRE(pause != moves.end());
    CHECK(pause->layer_id == 1);
    CHECK_THAT(pause->position.z(), Catch::Matchers::WithinAbs(0.4f, 1e-4f));

    const GCodeProcessorResult::MoveVertex *previous = nullptr;
    for (const auto &move : moves) {
        if (move.type == EMoveType::Pause_Print)
            continue;
        if (previous != nullptr) {
            CAPTURE(previous->gcode_id, move.gcode_id, static_cast<int>(move.type));
            CHECK(move.layer_id >= previous->layer_id);
            CHECK(move.gcode_id >= previous->gcode_id);
        }
        previous = &move;
    }
}

TEST_CASE("Line ends of the exported G-code mark every newline in the file", "[GCodeProcessor]")
{
    struct Case
    {
        const char* name;
        bool        preheat_backtrace;
        bool        pre_heating;
    };
    const auto test_case = GENERATE(values<Case>({
        { "written by size", false, false },
        { "written by time for the preheat backtrace", true, false },
        { "rewritten by the pre-heating pass", false, true },
    }));
    INFO(test_case.name);
    DynamicPrintConfig config = Test::multifilament_config(2, {
        { "single_extruder_multi_material", 0 },
        { "ooze_prevention",                test_case.preheat_backtrace },
        { "preheat_time",                   30 },
        { "enable_pre_heating",             test_case.pre_heating },
    });
    Print print;
    Model model;
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{ { { "extruder", 1 } }, { { "extruder", 2 } } };
    Test::init_print({ Test::cube(20), Test::cube(20) }, print, model, config, &overrides);
    GCodeProcessorResult result;
    const std::string    gcode = Test::gcode(print, &result);
    REQUIRE((gcode.find("preheat T") != std::string::npos) == test_case.preheat_backtrace);
    REQUIRE((gcode.find(GCodeProcessor::Machine_Start_GCode_End_Tag) != std::string::npos) == test_case.pre_heating);
    REQUIRE(gcode.size() > GCodeProcessor::Output_Block_Size);

    std::vector<size_t> newline_ends;
    for (size_t i = gcode.find('\n'); i != std::string::npos; i = gcode.find('\n', i + 1))
        newline_ends.push_back(i + 1);
    REQUIRE(result.lines_ends.size() == newline_ends.size());
    const auto difference = std::mismatch(result.lines_ends.begin(), result.lines_ends.end(), newline_ends.begin());
    INFO("first difference at line " << difference.first - result.lines_ends.begin() + 1);
    CHECK(difference.first == result.lines_ends.end());
}

TEST_CASE("Reloaded moves name their lines in G-code a script rewrote in place", "[GCodeProcessor]")
{
    Print print;
    Model model;
    Test::init_print({ Test::cube(20) }, print, model);
    GCodeProcessorResult result;
    const std::string    gcode          = Test::gcode(print, &result);
    const auto           exported_moves = result.moves;

    // A script that prepends one comment and, writing in text mode on Windows, turns every LF into CRLF.
    const std::string prepended = ";EDITED\r\n";
    std::string       edited    = prepended;
    for (const char c : gcode) {
        if (c == '\n')
            edited += '\r';
        edited += c;
    }
    ScopedTemporaryFile temp(".gcode");
    save_string_file(temp.path(), edited);
    result.filename = temp.string();
    print.reload_gcode_moves(&result);

    std::vector<size_t> newline_ends;
    for (size_t i = edited.find('\n'); i != std::string::npos; i = edited.find('\n', i + 1))
        newline_ends.push_back(i + 1);
    CHECK(result.lines_ends == newline_ends);

    // Every move that came from a line now names the same line one further down.
    REQUIRE(result.moves.size() == exported_moves.size());
    const auto difference = std::mismatch(exported_moves.begin(), exported_moves.end(), result.moves.begin(),
                                          [](const auto &exported, const auto &reloaded) {
                                              return reloaded.gcode_id == (exported.gcode_id == 0 ? 0 : exported.gcode_id + 1);
                                          });
    INFO("first difference at move " << difference.first - exported_moves.begin());
    CHECK(difference.first == exported_moves.end());
}

TEST_CASE("Rewritten G-code that cannot be re-read keeps the moves and hides the G-code window", "[GCodeProcessor]")
{
    Print print;
    Model model;
    Test::init_print({ Test::cube(20) }, print, model);
    GCodeProcessorResult result;
    const std::string    gcode          = Test::gcode(print, &result);
    const auto           exported_moves = result.moves;

    // A script that strips the trailing config block, which the G-code reader needs.
    const size_t config_block = gcode.find("; CONFIG_BLOCK_START");
    REQUIRE(config_block != std::string::npos);
    ScopedTemporaryFile temp(".gcode");
    save_string_file(temp.path(), gcode.substr(0, config_block));
    result.filename = temp.string();
    print.reload_gcode_moves(&result);

    CHECK(result.lines_ends.empty());
    REQUIRE(result.moves.size() == exported_moves.size());
    CHECK(result.moves.back().gcode_id == exported_moves.back().gcode_id);
}

TEST_CASE("The plate's center of mass takes every extrusion of G-code without a print behind it", "[GCodeProcessor]")
{
    GCodeProcessorResult result;
    process_two_objects(result);

    CHECK(result.object_masses.empty());
    CHECK(result.body_masses.empty());
    const GCodeProcessorResult::ObjectMass &plate = result.plate_mass;
    REQUIRE(plate.printed_up_to_layer.size() == 2);
    CHECK_THAT((center_of(plate.printed_up_to_layer.front()) -
                weighted_center({ { 1., a_brim }, { 1., a_support }, { 1., a_wall_0 }, { 2., b_wall } })).norm(),
               Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT((center_of(plate.printed_up_to_layer.back()) -
                weighted_center({ { 1., a_brim }, { 1., a_support }, { 1., a_wall_0 }, { 2., b_wall }, { 1., a_wall_1 } })).norm(),
               Catch::Matchers::WithinAbs(0., 1e-5));

    // Each bead weighs its volume at the default density and spreads along its move, (a^2 + ab + b^2) / 3 for one from
    // a to b: the brim from x 8 to 12 at y 8, the support at x 10 from y 20 to 30, A's walls from x 10 to 20 at y 10 and
    // B's from x 50 to 60 at y 50 with twice the filament, all at z 0.1 but A's second wall at 0.3.
    const GCodeProcessorResult::ObjectMass::Sum total = plate.total();
    CHECK_THAT(total.mass / total.volume, Catch::Matchers::WithinRel(double(DEFAULT_FILAMENT_DENSITY), 1e-6));
    const Vec3d second = total.second / total.mass;
    CHECK_THAT(second.x(), Catch::Matchers::WithinRel((304. / 3. + 100. + 2. * 700. / 3. + 2. * 9100. / 3.) / 6., 1e-6));
    CHECK_THAT(second.y(), Catch::Matchers::WithinRel((64. + 1900. / 3. + 2. * 100. + 2. * 2500.) / 6., 1e-6));
    CHECK_THAT(second.z(), Catch::Matchers::WithinRel((5. * 0.01 + 0.09) / 6., 1e-5));
    // The beads' center lines, brim and support included, from the first layer's bottom to the second's top.
    CHECK_THAT((plate.box.min - Vec3d(8., 8., 0.)).norm(), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT((plate.box.max - Vec3d(60., 50., 0.4)).norm(), Catch::Matchers::WithinAbs(0., 1e-5));
}

TEST_CASE("Each sliced cube's center of mass is its center, and the brim lowers the plate's printed one", "[GCodeProcessor]")
{
    const bool copies = GENERATE(false, true);
    INFO((copies ? "two copies of one cube" : "two cubes"));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "skirt_loops", 0 }, { "brim_type", "outer_only" }, { "brim_width", 5 }, { "combine_brims", 0 } });
    std::vector<TriangleMesh> cubes{ Test::cube(20) };
    if (!copies)
        cubes.emplace_back(Test::cube(20));
    Print print;
    Model model;
    Test::init_print(std::move(cubes), print, model, config, nullptr, true, copies ? 2 : 1);
    GCodeProcessorResult result;
    Test::gcode(print, &result);

    CHECK(result.body_masses.empty());
    REQUIRE(result.object_masses.size() == 2);
    for (const ModelObject *object : model.objects)
        for (size_t instance = 0; instance < object->instances.size(); ++instance) {
            const Vec3d center = object->instance_bounding_box(instance).center();
            const auto  mass   = std::min_element(result.object_masses.begin(), result.object_masses.end(), [&center](const auto &l, const auto &r) {
                return (center_of(l.total()) - center).squaredNorm() < (center_of(r.total()) - center).squaredNorm();
            });
            // Off the center only by the infill's alignment and the top and bottom shells.
            const Vec3d part = center_of(mass->total());
            CHECK_THAT(part.x(), Catch::Matchers::WithinAbs(center.x(), 0.5));
            CHECK_THAT(part.y(), Catch::Matchers::WithinAbs(center.y(), 0.5));
            CHECK_THAT(part.z(), Catch::Matchers::WithinAbs(center.z(), 1.));
            // The outer walls' center lines run half a line inside the cube's sides, of copies touching each other too.
            const BoundingBoxf3 box = object->instance_bounding_box(instance);
            for (int axis = 0; axis < 3; ++axis) {
                CHECK_THAT(mass->box.min[axis], Catch::Matchers::WithinAbs(box.min[axis], 0.3));
                CHECK_THAT(mass->box.max[axis], Catch::Matchers::WithinAbs(box.max[axis], 0.3));
            }
        }
    GCodeProcessorResult::ObjectMass::Sum objects;
    for (const GCodeProcessorResult::ObjectMass &object : result.object_masses)
        objects.add(object.total());
    const GCodeProcessorResult::ObjectMass::Sum plate = result.plate_mass.total();
    CHECK(plate.mass > objects.mass);
    CHECK(center_of(plate).z() < center_of(objects).z());
}

TEST_CASE("Each cube's raft is its support, centered below it", "[GCodeProcessor]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "skirt_loops", 0 }, { "brim_type", "no_brim" }, { "raft_layers", 3 } });
    Print print;
    Model model;
    Test::init_print({ Test::cube(20), Test::cube(20) }, print, model, config);
    GCodeProcessorResult result;
    Test::gcode(print, &result);

    REQUIRE(result.support_masses.size() == 2);
    for (size_t i = 0; i < 2; ++i) {
        const GCodeProcessorResult::ObjectMass::Sum support = result.support_masses[i].total();
        const Vec3d                                 object  = center_of(result.object_masses[i].total());
        REQUIRE(support.mass > 0.);
        CHECK_THAT(center_of(support).x(), Catch::Matchers::WithinAbs(object.x(), 1.));
        CHECK_THAT(center_of(support).y(), Catch::Matchers::WithinAbs(object.y(), 1.));
        CHECK(center_of(support).z() < 1.);
    }
}

TEST_CASE("A spiral vase cube counts all its extrusions, rising through each layer", "[GCodeProcessor]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "skirt_loops", 0 }, { "brim_type", "no_brim" }, { "spiral_mode", 1 }, { "wall_loops", 1 },
                                    { "top_shell_layers", 0 }, { "sparse_infill_density", 0 } });
    Print print;
    Model model;
    Test::init_print({ Test::cube(20) }, print, model, config);
    GCodeProcessorResult result;
    Test::gcode(print, &result);

    REQUIRE(result.object_masses.size() == 1);
    CHECK_THAT(result.object_masses.front().total().mass, Catch::Matchers::WithinRel(result.plate_mass.total().mass, 1e-6));
}

TEST_CASE("Each separate part of an assembly gets its center of mass, overlapping parts one", "[GCodeProcessor]")
{
    const bool overlapping = GENERATE(false, true);
    // Separated infills finds the bodies first, which the G-code export then takes.
    const bool separated = GENERATE(false, true);
    INFO((overlapping ? "overlapping parts" : "separate parts") << (separated ? ", separated infills" : ""));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "skirt_loops", 0 }, { "brim_type", "no_brim" }, { "separated_infills", separated ? 1 : 0 } });
    TriangleMesh first = make_cube(20, 20, 20);
    TriangleMesh second = make_cube(20, 20, 20);
    first.translate(50, 50, 0);
    second.translate(overlapping ? 60 : 90, 50, 0);
    Print print;
    Model model;
    Test::init_print({ first }, print, model, config, nullptr, false);
    model.objects.front()->add_volume(std::move(second), ModelVolumeType::MODEL_PART, false);
    print.apply(model, config);
    GCodeProcessorResult result;
    Test::gcode(print, &result);

    REQUIRE(result.object_masses.size() == 1);
    CHECK(result.object_masses.front().assembly);
    // One body is the object itself.
    if (overlapping) {
        CHECK(result.body_masses.empty());
        return;
    }
    REQUIRE(result.body_masses.size() == 2);
    CHECK(print.objects().front()->separated_body_bboxes().size() == (separated ? 2 : 0));
    const ModelObject &object = *model.objects.front();
    for (const ModelVolume *volume : object.volumes) {
        const Vec3d center = volume->mesh().transformed_bounding_box(object.instances.front()->get_matrix() * volume->get_matrix()).center();
        const auto  body   = std::min_element(result.body_masses.begin(), result.body_masses.end(), [&center](const auto &l, const auto &r) {
            return (center_of(l.total()) - center).squaredNorm() < (center_of(r.total()) - center).squaredNorm();
        });
        const Vec3d part = center_of(body->total());
        CHECK_THAT(part.x(), Catch::Matchers::WithinAbs(center.x(), 0.5));
        CHECK_THAT(part.y(), Catch::Matchers::WithinAbs(center.y(), 0.5));
        CHECK_THAT(part.z(), Catch::Matchers::WithinAbs(center.z(), 1.));
    }
}

TEST_CASE("Each extrusion weighs its filament's density", "[GCodeProcessor]")
{
    // Two like cubes, the second's filament three times as dense.
    DynamicPrintConfig config = Test::multifilament_config(2, { { "filament_density", "1,3" }, { "skirt_loops", 0 }, { "brim_type", "no_brim" } });
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{ { { "extruder", 1 } }, { { "extruder", 2 } } };
    Print print;
    Model model;
    Test::init_print({ Test::cube(20), Test::cube(20) }, print, model, config, &overrides);
    GCodeProcessorResult result;
    Test::gcode(print, &result);

    REQUIRE(result.object_masses.size() == 2);
    std::vector<const GCodeProcessorResult::ObjectMass *> masses;
    for (const ModelObject *object : model.objects) {
        const Vec3d center = object->instance_bounding_box(0).center();
        masses.emplace_back(&*std::min_element(result.object_masses.begin(), result.object_masses.end(), [&center](const auto &l, const auto &r) {
            return (center_of(l.total()) - center).squaredNorm() < (center_of(r.total()) - center).squaredNorm();
        }));
    }
    CHECK_THAT(masses[1]->total().mass / masses[0]->total().mass, Catch::Matchers::WithinRel(3., 0.02));
    // The plate's center lies three quarters of the way to the dense cube.
    const Vec3d plate = center_of(result.plate_mass.total());
    const Vec3d light = center_of(masses[0]->total());
    const Vec3d dense = center_of(masses[1]->total());
    CHECK_THAT((plate - light).dot(dense - light) / (dense - light).squaredNorm(), Catch::Matchers::WithinAbs(0.75, 0.01));
}

// A G1 that repeats the current point exactly asks for no path at all, yet its positive E still has
// to be classified. The pre-existing zero-displacement rule already does it: E moving alone is an
// Unretract, never an Extrude, so nothing is recorded as a bead on a segment the head did not travel.
// Task 2's guard is not even consulted here - it asks for type == Extrude first.
TEST_CASE("A G1 that repeats the current point records no extrusion vertex", "[GCodeProcessor][Regression]")
{
    // These boundary cases assert flow in mm3_per_mm, i.e. extruded filament volume over path length,
    // so the filament diameter they depend on is pinned instead of inherited from the config defaults.
    FullPrintConfig config;
    config.gcode_flavor.value       = gcfMarlinFirmware;
    config.filament_diameter.values = { 1.75 };
    config.nozzle_diameter.values   = { 0.4 };

    const std::string gcode =
        "M83\n"
        "G90\n"
        "; FEATURE: Outer wall\n"
        "G1 Z0.2 F12000\n"
        "G1 X20 Y10 F12000\n"
        "G1 X30 Y10 E1 F1800\n"
        "G1 X30 Y10 E.00003\n"
        "G1 X40 Y10 E1 F1800\n";

    GCodeProcessorResult result;
    process_gcode(gcode, result, config);

    // The time estimator splits long moves into internal_only vertices of its own; those are not what
    // the G-code asked for, so every count below is over the command vertices.
    std::vector<size_t> extrusions;
    for (size_t i = 0; i < result.moves.size(); ++i)
        if (result.moves[i].type == EMoveType::Extrude && !result.moves[i].internal_only)
            extrusions.push_back(i);
    REQUIRE(extrusions.size() == 2);

    // 1 mm of 1.75 mm filament (2.405 mm2) over a 10 mm path is a 0.2405 mm2 bead.
    const double filament_cross_section = M_PI * 0.875 * 0.875;
    CHECK_THAT(result.moves[extrusions[0]].position[X], Catch::Matchers::WithinAbs(30.0f, 1e-5f));
    CHECK_THAT(result.moves[extrusions[1]].position[X], Catch::Matchers::WithinAbs(40.0f, 1e-5f));
    CHECK_THAT(double(result.moves[extrusions[0]].mm3_per_mm), Catch::Matchers::WithinRel(filament_cross_section / 10.0, 1e-3));
    CHECK_THAT(double(result.moves[extrusions[1]].mm3_per_mm), Catch::Matchers::WithinRel(filament_cross_section / 10.0, 1e-3));

    // At the repeated point itself there is one arriving extrusion and then the E-only move, which is
    // typed Unretract. A second Extrude there would be a bead on a zero-length path.
    size_t extrusions_at_repeat = 0;
    bool   unretract_at_repeat  = false;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        if (move.internal_only || std::fabs(move.position[X] - 30.0f) > 1e-6f || std::fabs(move.position[Y] - 10.0f) > 1e-6f)
            continue;
        CAPTURE(static_cast<int>(move.type), move.delta_extruder);
        extrusions_at_repeat += (move.type == EMoveType::Extrude) ? 1 : 0;
        unretract_at_repeat |= (move.type == EMoveType::Unretract);
    }
    CHECK(extrusions_at_repeat == 1);
    CHECK(unretract_at_repeat);
}

// A retract and its unretract do not move the head, so their XYZ delta is exactly zero. They are typed
// by their E sign alone (Retract / Unretract) and are never Extrude, which is what keeps Task 2's
// extrusion guard from touching them.
TEST_CASE("A retract and its unretract keep their own types at a standstill", "[GCodeProcessor][Regression]")
{
    FullPrintConfig config;
    config.gcode_flavor.value       = gcfMarlinFirmware;
    config.filament_diameter.values = { 1.75 };
    config.nozzle_diameter.values   = { 0.4 };

    const std::string gcode =
        "M83\n"
        "G90\n"
        "; FEATURE: Outer wall\n"
        "G1 Z0.2 F12000\n"
        "G1 X50 Y10 F12000\n"
        "G1 X70 Y10 E1 F1800\n"
        "G1 E-.8 F1800\n"
        "G1 E.8 F1800\n"
        "G1 X90 Y10 E1 F1800\n";

    GCodeProcessorResult result;
    process_gcode(gcode, result, config);

    const auto vertex_at = [&result](float x, EMoveType type) {
        return std::find_if(result.moves.begin(), result.moves.end(), [x, type](const GCodeProcessorResult::MoveVertex &move) {
            return move.type == type && !move.internal_only && std::fabs(move.position[X] - x) < 1e-6f;
        });
    };

    size_t retracts = 0;
    size_t unretracts = 0;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        retracts += (move.type == EMoveType::Retract) ? 1 : 0;
        unretracts += (move.type == EMoveType::Unretract) ? 1 : 0;
    }
    CHECK(retracts == 1);
    CHECK(unretracts == 1);

    const auto retract = vertex_at(70.0f, EMoveType::Retract);
    REQUIRE(retract != result.moves.end());
    CAPTURE(retract->position[X], retract->position[Y], retract->delta_extruder, retract->travel_dist);
    CHECK_THAT(retract->position[Y], Catch::Matchers::WithinAbs(10.0f, 1e-6f));
    CHECK_THAT(double(retract->delta_extruder), Catch::Matchers::WithinAbs(-0.8, 1e-6));
    // No path at all: the whole move is E, so its travel distance is zero, not a sub-resolution sliver.
    CHECK_THAT(double(retract->travel_dist), Catch::Matchers::WithinAbs(0.0, 1e-6));

    const auto unretract = vertex_at(70.0f, EMoveType::Unretract);
    REQUIRE(unretract != result.moves.end());
    CAPTURE(unretract->position[X], unretract->position[Y], unretract->delta_extruder, unretract->travel_dist);
    CHECK_THAT(unretract->position[Y], Catch::Matchers::WithinAbs(10.0f, 1e-6f));
    CHECK_THAT(double(unretract->delta_extruder), Catch::Matchers::WithinAbs(0.8, 1e-6));
    CHECK_THAT(double(unretract->travel_dist), Catch::Matchers::WithinAbs(0.0, 1e-6));

    // The two real extrusions around them are untouched: 1 mm of filament over 20 mm of path each.
    const double filament_cross_section = M_PI * 0.875 * 0.875;
    const auto   before = vertex_at(70.0f, EMoveType::Extrude);
    REQUIRE(before != result.moves.end());
    CHECK_THAT(double(before->mm3_per_mm), Catch::Matchers::WithinRel(filament_cross_section / 20.0, 1e-3));
    const auto after = vertex_at(90.0f, EMoveType::Extrude);
    REQUIRE(after != result.moves.end());
    CHECK_THAT(double(after->mm3_per_mm), Catch::Matchers::WithinRel(filament_cross_section / 20.0, 1e-3));
}

// The cut Task 2 added is EPSILON, 1e-4 mm. A legitimate path one order of magnitude above it - the
// 0.001 mm the writer quantizes its output to - must stay an extrusion with the flow of its real
// length, otherwise a working filter would be eating normal print paths.
TEST_CASE("A 0.001 mm XY extrusion keeps its path length and its flow", "[GCodeProcessor][Regression]")
{
    FullPrintConfig config;
    config.gcode_flavor.value       = gcfMarlinFirmware;
    config.filament_diameter.values = { 1.75 };
    config.nozzle_diameter.values   = { 0.4 };

    const std::string gcode =
        "M83\n"
        "G90\n"
        "; FEATURE: Outer wall\n"
        "G1 Z0.2 F12000\n"
        "G1 X60 Y10 F12000\n"
        "G1 X60.001 Y10 E0.0000333 F1800\n"
        "G1 X60.051 Y10 E0.001665 F1800\n";

    GCodeProcessorResult result;
    process_gcode(gcode, result, config);

    std::vector<size_t> extrusions;
    for (size_t i = 0; i < result.moves.size(); ++i)
        if (result.moves[i].type == EMoveType::Extrude && !result.moves[i].internal_only)
            extrusions.push_back(i);
    REQUIRE(extrusions.size() == 2);

    // Walk back over the internal_only vertices the time estimator may have added to the travel, so
    // the path length is measured from the vertex the G-code itself asked for.
    size_t previous = extrusions[0];
    while (previous > 0 && result.moves[previous - 1].internal_only)
        --previous;
    REQUIRE(previous > 0);
    const GCodeProcessorResult::MoveVertex &travel_end = result.moves[previous - 1];
    REQUIRE(travel_end.type == EMoveType::Travel);

    const GCodeProcessorResult::MoveVertex &shortest = result.moves[extrusions[0]];
    const double distance = std::hypot(double(shortest.position[X]) - double(travel_end.position[X]),
                                       double(shortest.position[Y]) - double(travel_end.position[Y]));
    CAPTURE(distance, shortest.position[X], shortest.mm3_per_mm);
    // 60.001 is held as a float, i.e. 60.0009995, so the recorded length is 9.9945e-4 mm.
    CHECK_THAT(distance, Catch::Matchers::WithinAbs(0.001, 1e-6));
    // Ten EPSILONs: the sub-resolution cut must be far below this move.
    CHECK(distance > 5.0 * double(EPSILON));

    const double filament_cross_section = M_PI * 0.875 * 0.875;
    const double expected = filament_cross_section * 0.0000333 / distance;
    CHECK_THAT(double(shortest.mm3_per_mm), Catch::Matchers::WithinRel(expected, 2e-3));
    // A bead, not the blow-up the sub-resolution division produced (tens of mm2).
    CHECK(shortest.mm3_per_mm < 1.0f);

    // The following move is fifty times longer and carries fifty times the E, so it is the same bead:
    // the position bookkeeping still makes the two agree.
    const GCodeProcessorResult::MoveVertex &follow_up = result.moves[extrusions[1]];
    const double follow_distance = std::hypot(double(follow_up.position[X]) - double(shortest.position[X]),
                                              double(follow_up.position[Y]) - double(shortest.position[Y]));
    CAPTURE(follow_distance, follow_up.position[X], follow_up.mm3_per_mm);
    CHECK_THAT(follow_distance, Catch::Matchers::WithinAbs(0.05, 1e-6));
    CHECK_THAT(double(follow_up.mm3_per_mm),
               Catch::Matchers::WithinRel(filament_cross_section * 0.001665 / follow_distance, 2e-3));
    CHECK_THAT(double(follow_up.mm3_per_mm), Catch::Matchers::WithinRel(double(shortest.mm3_per_mm), 2e-3));
}

// A G2/G3 arc is discretized into many short G1 extrusions inside process_G2_G3, all of them handed to
// process_G1 with G1DiscretizationOrigin::G2G3. The guard Task 2 added only fires for origin == G1, so
// the arc keeps every segment it had before: same count, same geometry, same E.
TEST_CASE("A G2/G3 semicircle keeps every discretized extrusion segment", "[GCodeProcessor][Regression]")
{
    const bool clockwise = GENERATE(false, true);
    CAPTURE(clockwise);

    FullPrintConfig config;
    config.gcode_flavor.value       = gcfMarlinFirmware;
    config.filament_diameter.values = { 1.75 };
    config.nozzle_diameter.values   = { 0.4 };

    // Centre (40, 10), radius 10, half turn, F1200 = 20 mm/s.
    const std::string gcode =
        std::string("M83\n"
                    "G90\n"
                    "; FEATURE: Outer wall\n"
                    "G1 Z0.2 F12000\n"
                    "G1 X50 Y10 F12000\n") +
        (clockwise ? "G2" : "G3") + " X30 Y10 I-10 J0 E0.5 F1200\n";

    GCodeProcessorResult result;
    process_gcode(gcode, result, config);

    std::vector<size_t> arc_vertices;
    for (size_t i = 0; i < result.moves.size(); ++i)
        if (result.moves[i].type == EMoveType::Extrude && !result.moves[i].internal_only)
            arc_vertices.push_back(i);
    REQUIRE(!arc_vertices.empty());

    // segment_mm = clamp(min(sqrt(8 * 10 * 0.02), 20 / 50), 0.1, 2.0) = 0.4 and
    // segments = max(10 * pi / 0.4 + 0.8, 1) = 79, the Marlin branch's own count. Every segment ends in
    // its own extrusion vertex; a dropped one would show up here.
    CHECK(arc_vertices.size() == 79);

    // From the first to the last arc vertex every move is an extrusion: the arc is not interrupted.
    for (size_t i = arc_vertices.front(); i <= arc_vertices.back(); ++i) {
        CAPTURE(i, static_cast<int>(result.moves[i].type), result.moves[i].position[X], result.moves[i].position[Y]);
        CHECK(result.moves[i].type == EMoveType::Extrude);
    }

    const Vec2f  center(40.0f, 10.0f);
    double       total_e     = 0.0;
    double       min_segment = 1e9;
    for (size_t k = 0; k < arc_vertices.size(); ++k) {
        const GCodeProcessorResult::MoveVertex &move = result.moves[arc_vertices[k]];
        const Vec2f                              point(move.position[X], move.position[Y]);
        CAPTURE(k, point[X], point[Y], move.delta_extruder);
        // On the circle the arc was asked for.
        CHECK_THAT(double((point - center).norm()), Catch::Matchers::WithinAbs(10.0, 1e-3));
        total_e += double(move.delta_extruder);
        if (k > 0) {
            const GCodeProcessorResult::MoveVertex &previous = result.moves[arc_vertices[k - 1]];
            const Vec2f                              previous_point(previous.position[X], previous.position[Y]);
            min_segment = std::min(min_segment, double((point - previous_point).norm()));
        }
    }
    // The arc delivers exactly the E the file asked for, so no segment's filament was dropped.
    CAPTURE(total_e, min_segment);
    CHECK_THAT(total_e, Catch::Matchers::WithinRel(0.5, 1e-4));
    // Segments are ~0.3976 mm, three orders of magnitude above EPSILON, so the guard's delta_xyz < 1e-4
    // test is false on every one of them: the pre-Task-2 code and the current code take the same branch,
    // and the arc path is untouched by construction.
    CHECK_THAT(min_segment, Catch::Matchers::WithinAbs(0.3976, 1e-3));
    CHECK(min_segment > 100.0 * double(EPSILON));

    // First vertex one segment along the circle, last exactly at the commanded endpoint.
    const GCodeProcessorResult::MoveVertex &first_arc = result.moves[arc_vertices.front()];
    CAPTURE(first_arc.position[X], first_arc.position[Y]);
    CHECK_THAT(first_arc.position[X], Catch::Matchers::WithinAbs(49.9921f, 1e-3f));
    CHECK_THAT(first_arc.position[Y], Catch::Matchers::WithinAbs(clockwise ? 9.6024f : 10.3976f, 1e-3f));
    // The turn direction survives the discretization: G2 runs below Y 10, G3 above it.
    CHECK((first_arc.position[Y] < 10.0f) == clockwise);

    const GCodeProcessorResult::MoveVertex &last_arc = result.moves[arc_vertices.back()];
    CAPTURE(last_arc.position[X], last_arc.position[Y], last_arc.delta_extruder);
    CHECK_THAT(last_arc.position[X], Catch::Matchers::WithinAbs(30.0f, 1e-5f));
    CHECK_THAT(last_arc.position[Y], Catch::Matchers::WithinAbs(10.0f, 1e-5f));
}
