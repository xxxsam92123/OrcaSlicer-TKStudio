#include <catch2/catch_all.hpp>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Print.hpp"

#include "test_helpers.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

// The PA value on an M900 / SET_PRESSURE_ADVANCE line, false when the line is neither.
bool parse_pa_command(const std::string &line, double &pa)
{
    if (line.rfind("M900", 0) == 0) {
        const size_t k = line.find('K');
        if (k == std::string::npos)
            return false;
        pa = std::strtod(line.c_str() + k + 1, nullptr);
        return true;
    }
    if (line.rfind("SET_PRESSURE_ADVANCE", 0) == 0) {
        const size_t advance = line.find("ADVANCE=");
        if (advance == std::string::npos)
            return false;
        pa = std::strtod(line.c_str() + advance + 8, nullptr);
        return true;
    }
    return false;
}

bool is_extrusion_move(const std::string &line)
{
    if (line.rfind("G1 ", 0) != 0)
        return false;
    return line.find('E') != std::string::npos
        && (line.find('X') != std::string::npos || line.find('Y') != std::string::npos);
}

// Both the Orca marker (";TYPE:Bridge") and the Bambu Lab one ("; FEATURE: Bridge") name the role.
bool parse_role_line(const std::string &line, std::string &role)
{
    for (const char *prefix : {";TYPE:", "; FEATURE:"}) {
        const size_t length = std::strlen(prefix);
        if (line.rfind(prefix, 0) != 0)
            continue;
        role = line.substr(length);
        const size_t first = role.find_first_not_of(' ');
        role = first == std::string::npos ? std::string() : role.substr(first);
        return true;
    }
    return false;
}

// The PA in effect on every extrusion move, paired with the role marker that move belongs to.
std::vector<std::pair<std::string, double>> pa_by_role(const std::string &gcode)
{
    std::vector<std::pair<std::string, double>> samples;
    std::string role;
    double      pa = -1.;
    size_t      pos = 0;
    while (pos < gcode.size()) {
        const size_t eol  = gcode.find('\n', pos);
        const std::string line = gcode.substr(pos, eol == std::string::npos ? std::string::npos : eol - pos);
        double value = 0.;
        if (parse_role_line(line, role)) {
            // role updated above
        } else if (parse_pa_command(line, value))
            pa = value;
        else if (is_extrusion_move(line) && !role.empty())
            samples.emplace_back(role, pa);
        pos = (eol == std::string::npos) ? gcode.size() : eol + 1;
    }
    return samples;
}

size_t count_pa_commands(const std::string &gcode)
{
    size_t n   = 0;
    size_t pos = 0;
    while (pos < gcode.size()) {
        const size_t eol = gcode.find('\n', pos);
        double value = 0.;
        if (parse_pa_command(gcode.substr(pos, eol == std::string::npos ? std::string::npos : eol - pos), value))
            ++n;
        pos = (eol == std::string::npos) ? gcode.size() : eol + 1;
    }
    return n;
}

bool is_bridge_role(const std::string &role)
{
    return role == "Bridge" || role == "Internal Bridge";
}

// How many times the extrusion sequence switches into or out of a bridge role.
size_t count_bridge_flips(const std::string &gcode)
{
    const std::vector<std::pair<std::string, double>> samples = pa_by_role(gcode);
    size_t flips = 0;
    for (size_t i = 1; i < samples.size(); ++i)
        flips += is_bridge_role(samples[i - 1].first) != is_bridge_role(samples[i].first) ? 1 : 0;
    return flips;
}

// Every distinct PA value an extrusion of `role` runs with.
std::vector<double> pa_values_for_role(const std::string &gcode, const std::string &role)
{
    std::vector<double> values;
    for (const auto &sample : pa_by_role(gcode)) {
        if (sample.first != role)
            continue;
        bool seen = false;
        for (double v : values)
            seen = seen || std::abs(v - sample.second) < 1e-9;
        if (!seen)
            values.push_back(sample.second);
    }
    return values;
}

std::vector<double> pa_values_outside_bridges(const std::string &gcode)
{
    std::vector<double> values;
    for (const auto &sample : pa_by_role(gcode)) {
        if (is_bridge_role(sample.first))
            continue;
        bool seen = false;
        for (double v : values)
            seen = seen || std::abs(v - sample.second) < 1e-9;
        if (!seen)
            values.push_back(sample.second);
    }
    return values;
}

// The configurations under test. Only the keys the assertions depend on are set; everything else
// stays at the defaults.
struct PaConfig
{
    bool        enable_pa      = true;
    const char *pressure_advance = "0.035";
    bool        adaptive_pa    = false;
    const char *bridges        = "0.02";
    const char *flavor         = "marlin2";
};

std::string slice_bridge(const PaConfig &pa_config)
{
    return slice({mesh(TestMesh::bridge)},
        {{"enable_pressure_advance", pa_config.enable_pa ? "1" : "0"},
         {"pressure_advance", pa_config.pressure_advance},
         {"adaptive_pressure_advance", pa_config.adaptive_pa ? "1" : "0"},
         {"adaptive_pressure_advance_model", "0.02,10,2000"},
         {"adaptive_pressure_advance_bridges", pa_config.bridges},
         {"gcode_flavor", pa_config.flavor}});
}

} // namespace

TEST_CASE("Bridge static pressure advance applies to bridge infill without adaptive pressure advance",
          "[BridgePA]")
{
    PaConfig config;
    const std::string gcode = slice_bridge(config);

    // init_print() forces gcode_comments on, so these markers are observable whenever the branch
    // fires; their presence here is what makes the absence assertion in the adaptive case below a
    // real negative rather than a comment flag that never had a chance to appear.
    REQUIRE(gcode.find("; Bridge static pressure advance\n") != std::string::npos);
    REQUIRE(gcode.find("; Bridge static pressure advance reverted\n") != std::string::npos);

    const std::vector<double> bridge_values = pa_values_for_role(gcode, "Bridge");
    REQUIRE(!bridge_values.empty());
    for (double value : bridge_values)
        REQUIRE(std::abs(value - 0.02) < 1e-6);

    const std::vector<double> internal_values = pa_values_for_role(gcode, "Internal Bridge");
    REQUIRE(!internal_values.empty());
    for (double value : internal_values)
        REQUIRE(std::abs(value - 0.02) < 1e-6);

    const std::vector<double> non_bridge_values = pa_values_outside_bridges(gcode);
    REQUIRE(!non_bridge_values.empty());
    for (double value : non_bridge_values)
        REQUIRE(std::abs(value - 0.035) < 1e-6);

    // One PA command per bridge-state flip plus the filament's initial one: the branch must emit on a
    // transition only, so a per-path emission shows up here as an inflated count.
    const size_t flips   = count_bridge_flips(gcode);
    const size_t emitted = count_pa_commands(gcode);
    INFO("emitted " << emitted << " PA commands, expected 1 + " << flips << " bridge-state flips");
    REQUIRE(emitted == 1 + flips);
}

TEST_CASE("A zero bridge override leaves the static pressure advance untouched", "[BridgePA]")
{
    PaConfig config;
    config.bridges = "0";
    const std::string gcode = slice_bridge(config);

    const std::vector<double> values = pa_values_outside_bridges(gcode);
    REQUIRE(!values.empty());

    for (const auto &sample : pa_by_role(gcode))
        REQUIRE(std::abs(sample.second - 0.035) < 1e-6);

    // Only the initial PA of the filament, nothing emitted per bridge.
    REQUIRE(count_pa_commands(gcode) == 1);
}

TEST_CASE("Adaptive pressure advance keeps sole control of the bridge pressure advance", "[BridgePA]")
{
    PaConfig config;
    config.adaptive_pa = true;
    const std::string gcode = slice_bridge(config);

    // The static bridge branch is gated on adaptive PA being off, so it must not emit anything here.
    // init_print() forces gcode_comments on (tests/fff_print/test_helpers.cpp:258), so a static-branch
    // emission would always carry this marker. The positive control in the first test case pins that
    // coupling: if the helper ever stops forcing the flag, that assertion fails loudly instead of this
    // one silently passing.
    REQUIRE(gcode.find("; Bridge static pressure advance") == std::string::npos);

    // The adaptive processor still applies the bridge override by itself.
    const std::vector<double> bridge_values = pa_values_for_role(gcode, "Bridge");
    REQUIRE(!bridge_values.empty());
    for (double value : bridge_values)
        REQUIRE(std::abs(value - 0.02) < 1e-6);
}

TEST_CASE("A bridge override without pressure advance enabled emits nothing", "[BridgePA]")
{
    PaConfig config;
    config.enable_pa = false;
    const std::string gcode = slice_bridge(config);

    REQUIRE(count_pa_commands(gcode) == 0);
    REQUIRE(!pa_by_role(gcode).empty());
}

TEST_CASE("Bridge static pressure advance covers internal bridge infill", "[BridgePA]")
{
    PaConfig config;
    const std::string gcode = slice_bridge(config);

    const std::vector<double> internal_values = pa_values_for_role(gcode, "Internal Bridge");
    REQUIRE(!internal_values.empty());
    for (double value : internal_values)
        REQUIRE(std::abs(value - 0.02) < 1e-6);

    // The surrounding infill runs at the filament's own PA.
    const std::vector<double> infill_values = pa_values_for_role(gcode, "Internal solid infill");
    REQUIRE(!infill_values.empty());
    for (double value : infill_values)
        REQUIRE(std::abs(value - 0.035) < 1e-6);
}

TEST_CASE("Bambu Lab printers get the bridge override in the M900 L1000 form", "[BridgePA]")
{
    PaConfig    config;
    Slic3r::Print  print;
    Slic3r::Model  model;
    init_print({mesh(TestMesh::bridge)}, print, model,
        {{"enable_pressure_advance", "1"},
         {"pressure_advance", config.pressure_advance},
         {"adaptive_pressure_advance", "0"},
         {"adaptive_pressure_advance_bridges", config.bridges},
         {"gcode_flavor", config.flavor}});
    print.is_BBL_printer() = true;

    const std::string gcode = Test::gcode(print);

    REQUIRE(gcode.find("M900 K0.02 L1000 M10 ;") != std::string::npos);
    REQUIRE(gcode.find("M900 K0.035 L1000 M10 ;") != std::string::npos);
}
