#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Config.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libvgcode/include/GCodeInputData.hpp"
#include "libvgcode/include/PathVertex.hpp"
#include "libvgcode/include/Types.hpp"
#include "libvgcode/src/Layers.hpp"
#include "slic3r/GUI/LibVGCode/LibVGCodeWrapper.hpp"
#include "test_utils.hpp"

// The preview keeps the start G-code's prepare stage on screen without letting it back into the
// numbers the legends and the layer times are built from. The exclusion lives in libvgcode, whose
// viewer needs a GL context to load(), but both halves are reachable without one: the conversion is
// GUI code (this suite links libslic3r_gui) and the layer accumulator is a plain class, so both are
// exercised here through the real code rather than re-implemented for the test.
namespace {

using Slic3r::EMoveType;
using Slic3r::GCodeProcessorResult;

// Runs the real processor over `gcode`, so the prepare-stage flags and bead values the preview
// filters on are the ones the emitted G-code produces.
void process_gcode(const std::string &gcode, GCodeProcessorResult &result)
{
    Slic3r::FullPrintConfig config;
    config.gcode_flavor.value = Slic3r::gcfMarlinFirmware;

    // "; FEATURE: " role tags are the BBL flavour, which selects the prepare stage here.
    struct BblPrinterFlag
    {
        const bool saved = Slic3r::GCodeProcessor::s_IsBBLPrinter;
        BblPrinterFlag() { Slic3r::GCodeProcessor::s_IsBBLPrinter = true; }
        ~BblPrinterFlag() { Slic3r::GCodeProcessor::s_IsBBLPrinter = saved; }
    } const bbl_printer_flag;

    ScopedTemporaryFile temp(".gcode");
    {
        std::ofstream output(temp.string());
        output << gcode;
    }

    Slic3r::GCodeProcessor processor;
    processor.apply_config(config);
    processor.process_file(temp.string());
    result = std::move(processor.extract_result());
}

libvgcode::GCodeInputData to_input_data(const GCodeProcessorResult &result)
{
    // convert() takes the viewer for the callers that need the belt transform; it reads no state
    // from it, and the viewer allocates nothing on the GL side until init() is called.
    const libvgcode::Viewer viewer;
    return libvgcode::convert(result, { "#FF0000" }, {}, viewer, nullptr);
}

// The predicate the preview screens moves with, spelled out independently of convert(): a move that
// is not an option marker starts a new path when it is the first one or when it differs from the move
// before it in type, role, or a value the preview colours by. Keeping it here makes the vertex count
// checkable against the rule instead of against itself.
bool starts_path_reference(const GCodeProcessorResult::MoveVertex &prev, const GCodeProcessorResult::MoveVertex &curr, bool first)
{
    const libvgcode::EOptionType option_type = libvgcode::move_type_to_option(libvgcode::convert(curr.type));
    if (option_type != libvgcode::EOptionType::COUNT && option_type != libvgcode::EOptionType::Travels && option_type != libvgcode::EOptionType::Wipes)
        return false;
    return first || prev.type != curr.type || prev.extrusion_role != curr.extrusion_role ||
        prev.mm3_per_mm != curr.mm3_per_mm || prev.acceleration != curr.acceleration || prev.jerk != curr.jerk;
}

// One vertex per move after the first, plus a phantom in front of every path start - the count
// convert() sizes its array by, over every move it walks.
size_t expected_vertices_count(const GCodeProcessorResult &result)
{
    size_t count = 0;
    for (size_t i = 1; i < result.moves.size(); ++i)
        count += starts_path_reference(result.moves[i - 1], result.moves[i], i == 1) ? 2 : 1;
    return count;
}

bool same_position(const libvgcode::Vec3 &a, const Slic3r::Vec3f &b)
{
    return std::fabs(a[0] - b[0]) < 1e-3f && std::fabs(a[1] - b[1]) < 1e-3f && std::fabs(a[2] - b[2]) < 1e-3f;
}

// The header an OrcaSlicer-produced file carries: the producer line, then the config block the
// processor loads to configure itself from. It is not decoration - the processor reads the
// "; LAYER_HEIGHT: " tags only in such a file, and without them the heights fall back to the Z
// difference, which a prepare line at Z 5.8 poisons. The block is written from the defaults, the way
// the slicer writes it.
std::string header_gcode()
{
    // The standalone G-code loader rejects suspiciously small config blocks. Emit the full default
    // block while omitting multiline values, whose embedded newlines would become fake config records.
    Slic3r::DynamicPrintConfig defaults = Slic3r::DynamicPrintConfig::full_print_config();
    std::string header = "; generated by OrcaSlicer\n; CONFIG_BLOCK_START\n";
    for (const std::string &key : defaults.keys()) {
        const std::string value = defaults.opt_serialize(key);
        if (value.find('\n') != std::string::npos)
            continue;
        header += "; " + key + " = " + value + "\n";
    }
    return header + "; CONFIG_BLOCK_END\nM83\nG90\n";
}
// A two-layer printed part with a wall at Z 0.2 and Z 0.4, both reaching X 200 / Y 50.
constexpr const char *printed_part_gcode =
    "; CHANGE_LAYER\n"
    "; LAYER_HEIGHT: 0.2\n"
    "; FEATURE: Outer wall\n"
    "G1 X10 Y50 Z0.2 F12000\n"
    "G1 X200 Y50 E5 F1800\n"
    "; CHANGE_LAYER\n"
    "; LAYER_HEIGHT: 0.2\n"
    "; FEATURE: Outer wall\n"
    "G1 Z0.4 F12000\n"
    "G1 X10 Y50 F12000\n"
    "G1 X200 Y50 E5 F1800\n";

// The prepare stage as the start G-code writes it: the role tag opens it before the first G1, then a
// pinned-Z travel, the prime line's extrusion, a retract and the travel to where the part begins. The
// prime line is what pollutes the legends when it is measured: its Z of 5.8 becomes the layer height,
// and the bead width and the volumetric rate are derived from it.
constexpr const char *prepare_stage_gcode =
    "; FEATURE: Custom\n"
    "G1 Z5.8 F600\n"
    "G1 X100 Y10 E10 F12000\n"
    "G1 E-1 F1800\n"
    "G1 X150 Y150 F9000\n";

// The extrusions of one stage, from the processor's own output.
std::vector<const GCodeProcessorResult::MoveVertex *> extrusions_of(const GCodeProcessorResult &result, bool prepare_stage)
{
    std::vector<const GCodeProcessorResult::MoveVertex *> ret;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves)
        if (move.type == EMoveType::Extrude && move.prepare_stage == prepare_stage)
            ret.push_back(&move);
    return ret;
}

// The vertex the preview drew for `move`, or the end of the list.
std::vector<libvgcode::PathVertex>::const_iterator vertex_of(const std::vector<libvgcode::PathVertex> &vertices,
    const GCodeProcessorResult::MoveVertex &move)
{
    return std::find_if(vertices.begin(), vertices.end(),
        [&move](const libvgcode::PathVertex &vertex) { return vertex.is_extrusion() && same_position(vertex.position, move.position); });
}

struct Extremes
{
    float height{ 0.0f };
    float width{ 0.0f };
    float volumetric_rate{ 0.0f };
};

// The maximum each legend would end up with, over the extrusion vertices `accept` lets through. This
// walks the same fields libvgcode::ViewerImpl::update_color_ranges() walks, with the acceptance rule
// under test as the only variable.
template<typename Accept>
Extremes extremes(const std::vector<libvgcode::PathVertex> &vertices, Accept accept)
{
    Extremes ret;
    for (const libvgcode::PathVertex &vertex : vertices) {
        if (!vertex.is_extrusion() || !accept(vertex))
            continue;
        ret.height = std::max(ret.height, vertex.height);
        ret.width = std::max(ret.width, vertex.width);
        ret.volumetric_rate = std::max(ret.volumetric_rate, vertex.volumetric_rate());
    }
    return ret;
}

} // namespace

// The prepare stage runs before the printed part: heat, home, prime. The preview draws it - the prime
// line is part of the G-code the user is looking at - but must not measure it: its bead is the prime
// bead's (5.8 mm tall, 1.35 mm wide once the pinned Z is taken as the height), its speeds are the
// start-up travel's and its time is the warm-up wait. All of that belongs to no printed layer, so it
// stays out of the colour ranges and out of the layer times.
TEST_CASE("The start G-code's prepare stage is drawn but not measured", "[LibVGCode]")
{
    GCodeProcessorResult result;
    process_gcode(header_gcode() + prepare_stage_gcode + printed_part_gcode, result);
    const libvgcode::GCodeInputData data = to_input_data(result);
    const std::vector<libvgcode::PathVertex> &vertices = data.vertices;

    // The prepare stage is a prefix of the move list, and the role tag opens it before the first G1.
    // index 0 is the processor's dummy move, which belongs to neither stage.
    const auto first_printed = std::find_if(std::next(result.moves.begin()), result.moves.end(),
        [](const GCodeProcessorResult::MoveVertex &move) { return !move.prepare_stage; });
    REQUIRE(first_printed != result.moves.end());
    CHECK(std::distance(result.moves.begin(), first_printed) >= 4);
    CHECK(std::none_of(first_printed, result.moves.end(),
        [](const GCodeProcessorResult::MoveVertex &move) { return move.prepare_stage; }));

    SECTION("every prepare-stage extrusion reaches the preview, carrying its flag")
    {
        // One vertex per walked move, plus the phantoms, over the whole move list: the prepare stage is
        // no longer dropped before the conversion, so the count is the unfiltered one.
        CHECK(vertices.size() == expected_vertices_count(result));

        // The prime line's geometry is on screen - all of its vertices, none missing - and flagged.
        const auto prepare_extrusions = extrusions_of(result, true);
        REQUIRE(!prepare_extrusions.empty());
        for (const GCodeProcessorResult::MoveVertex *move : prepare_extrusions) {
            CAPTURE(move->position[Slic3r::X], move->position[Slic3r::Y], move->height, move->width);
            const auto vertex = vertex_of(vertices, *move);
            REQUIRE(vertex != vertices.end());
            CHECK(vertex->prepare_stage);
            // The real bead reaches the viewer, so the exclusion is done by the flag and not by
            // rewriting the values.
            CHECK_THAT(vertex->height, Catch::Matchers::WithinRel(move->height, 1e-5f));
            CHECK_THAT(vertex->width, Catch::Matchers::WithinRel(move->width, 1e-5f));
        }

        // The prime line is split into several vertices by the acceleration ramp, so the geometry is
        // really drawn rather than collapsed into one point.
        CHECK(prepare_extrusions.size() > 1);
        // The prepare stage's travels are drawn too, not just its extrusion.
        CHECK(std::count_if(vertices.begin(), vertices.end(),
            [](const libvgcode::PathVertex &v) { return v.prepare_stage && !v.is_extrusion(); }) > 0);
    }

    SECTION("no vertex of the printed part is flagged")
    {
        const auto printed_extrusions = extrusions_of(result, false);
        REQUIRE(!printed_extrusions.empty());
        for (const GCodeProcessorResult::MoveVertex *move : printed_extrusions) {
            CAPTURE(move->position[Slic3r::X], move->position[Slic3r::Y]);
            const auto vertex = vertex_of(vertices, *move);
            REQUIRE(vertex != vertices.end());
            CHECK_FALSE(vertex->prepare_stage);
        }
        CHECK(std::count_if(vertices.begin(), vertices.end(),
            [](const libvgcode::PathVertex &v) { return v.prepare_stage; }) > 0);
    }

    SECTION("the legends take the printed values only")
    {
        // The fixture really carries the polluting values, and only the prepare stage carries them:
        // without the flag check the height legend would open at the prime line's 5.8 mm and the width
        // and flow legends at the bead derived from it.
        const Extremes all = extremes(vertices, [](const libvgcode::PathVertex &) { return true; });
        const Extremes printed = extremes(vertices, [](const libvgcode::PathVertex &v) { return v.is_measured(); });
        CAPTURE(all.height, all.width, all.volumetric_rate, printed.height, printed.width, printed.volumetric_rate);

        CHECK(all.height > 5.0f);
        CHECK(all.width > 1.0f);
        CHECK(printed.height < 0.3f);
        CHECK(printed.width < 1.0f);
        CHECK(printed.height < all.height);
        CHECK(printed.width < all.width);
        CHECK(printed.volumetric_rate < all.volumetric_rate);

        // What is left is exactly the printed part's own range, taken from the processor's output.
        float expected_height = 0.0f;
        float expected_width = 0.0f;
        for (const GCodeProcessorResult::MoveVertex *move : extrusions_of(result, false)) {
            expected_height = std::max(expected_height, move->height);
            expected_width = std::max(expected_width, move->width);
        }
        CHECK_THAT(printed.height, Catch::Matchers::WithinRel(expected_height, 1e-5f));
        CHECK_THAT(printed.width, Catch::Matchers::WithinRel(expected_width, 1e-5f));
    }

    SECTION("the layer times hold the printed time only, the prepare time staying on the vertices")
    {
        const size_t normal = static_cast<size_t>(Slic3r::PrintEstimatedStatistics::ETimeMode::Normal);

        // The prepare stage's time is real data on the vertices - it is the layer accumulator that
        // leaves it out, not a zeroed field.
        float prepare_time = 0.0f;
        float printed_time = 0.0f;
        for (const libvgcode::PathVertex &vertex : vertices) {
            if (vertex.prepare_stage)
                prepare_time += vertex.times[normal];
            else
                printed_time += vertex.times[normal];
        }
        CHECK(prepare_time > 0.0f);
        CHECK(printed_time > 0.0f);

        // The processor's own sum, as an oracle: one vertex per move, and the phantoms carry zero times.
        float expected_printed_time = 0.0f;
        for (size_t i = 1; i < result.moves.size(); ++i)
            if (!result.moves[i].prepare_stage)
                expected_printed_time += result.moves[i].time[normal];
        CHECK_THAT(printed_time, Catch::Matchers::WithinRel(expected_printed_time, 1e-4f));

        // The accumulator the layer slider and its legend read. Its two layers hold the printed time
        // and nothing else: the prepare stage's time is in neither of them.
        libvgcode::Layers layers;
        for (size_t i = 0; i < vertices.size(); ++i)
            layers.update(vertices[i], static_cast<uint32_t>(i));
        REQUIRE(layers.count() == 2);
        const float layer_0_time = layers.get_layer_time(libvgcode::ETimeMode::Normal, 0);
        const float layer_1_time = layers.get_layer_time(libvgcode::ETimeMode::Normal, 1);
        CAPTURE(layer_0_time, layer_1_time, prepare_time, expected_printed_time);
        CHECK_THAT(layer_0_time + layer_1_time, Catch::Matchers::WithinRel(expected_printed_time, 1e-4f));
        CHECK(layer_0_time + layer_1_time < expected_printed_time + prepare_time);
    }

    SECTION("the prepare stage is drawn as part of the first layer")
    {
        // libvgcode draws a layer by its vertex range, so the prepare stage has to be inside the first
        // layer's: it comes first and carries layer 0, the printed layers keeping their numbering.
        REQUIRE(!vertices.empty());
        CHECK(vertices.front().layer_id == 0);
        const auto first_printed_vertex = std::find_if(vertices.begin(), vertices.end(),
            [](const libvgcode::PathVertex &v) { return !v.prepare_stage; });
        REQUIRE(first_printed_vertex != vertices.end());
        CHECK(std::count_if(vertices.begin(), first_printed_vertex,
            [](const libvgcode::PathVertex &v) { return v.prepare_stage; }) ==
            std::distance(vertices.begin(), first_printed_vertex));
        for (auto it = vertices.begin(); it != first_printed_vertex; ++it) {
            CAPTURE(it->layer_id, it->height, it->width);
            CHECK(it->layer_id == 0);
        }

        uint32_t layer_id = vertices.front().layer_id;
        for (const libvgcode::PathVertex &vertex : vertices) {
            CAPTURE(vertex.layer_id, vertex.position[2]);
            CHECK(vertex.layer_id >= layer_id);
            CHECK(vertex.layer_id <= layer_id + 1);
            layer_id = vertex.layer_id;
        }
        // Two printed layers, numbered 0 and 1: the prepare stage did not open a layer of its own.
        CHECK(layer_id == 1);
    }
}

// Without a prepare stage nothing is flagged and the preview comes out exactly as it did before: the
// same vertex count, and the first path starting at the file's initial move.
TEST_CASE("A G-code without a prepare stage reaches the preview unchanged", "[LibVGCode]")
{
    const std::string gcode = header_gcode() + printed_part_gcode;

    GCodeProcessorResult result;
    process_gcode(gcode, result);
    const libvgcode::GCodeInputData data = to_input_data(result);
    const std::vector<libvgcode::PathVertex> &vertices = data.vertices;

    REQUIRE(!result.moves.empty());
    CHECK(std::none_of(result.moves.begin(), result.moves.end(),
        [](const GCodeProcessorResult::MoveVertex &move) { return move.prepare_stage; }));
    CHECK(std::none_of(vertices.begin(), vertices.end(),
        [](const libvgcode::PathVertex &vertex) { return vertex.prepare_stage; }));

    // The count the walk over the whole move list produces, unchanged by the prepare-stage support.
    CHECK(vertices.size() == expected_vertices_count(result));

    REQUIRE(!vertices.empty());
    // The phantom vertex takes the first move's position, the start of the file.
    CHECK(same_position(vertices.front().position, result.moves.front().position));

    uint32_t layer_id = vertices.front().layer_id;
    CHECK(layer_id == 0);
    for (const libvgcode::PathVertex &vertex : vertices) {
        CAPTURE(vertex.layer_id);
        CHECK(vertex.layer_id >= layer_id);
        CHECK(vertex.layer_id <= layer_id + 1);
        layer_id = vertex.layer_id;
    }
    CHECK(layer_id == 1);

    // Every extrusion is measured, so filtering changes nothing.
    const Extremes printed = extremes(vertices, [](const libvgcode::PathVertex &v) { return !v.prepare_stage; });
    const Extremes all = extremes(vertices, [](const libvgcode::PathVertex &) { return true; });
    CHECK(all.height == printed.height);
    CHECK(all.width == printed.width);
    CHECK(all.volumetric_rate == printed.volumetric_rate);
}
