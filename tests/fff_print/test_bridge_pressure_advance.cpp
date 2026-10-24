#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Model.hpp"
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

// How many times the extrusion sequence switches into and out of a bridge role.
struct BridgeFlips
{
    size_t entries = 0; // non-bridge -> bridge
    size_t exits   = 0; // bridge -> non-bridge
};

BridgeFlips count_bridge_flips(const std::string &gcode)
{
    const std::vector<std::pair<std::string, double>> samples = pa_by_role(gcode);
    BridgeFlips flips;
    for (size_t i = 1; i < samples.size(); ++i) {
        const bool was_bridge = is_bridge_role(samples[i - 1].first);
        const bool is_bridge  = is_bridge_role(samples[i].first);
        if (was_bridge == is_bridge)
            continue;
        if (is_bridge)
            ++flips.entries;
        else
            ++flips.exits;
    }
    return flips;
}

// The markers this branch writes on a bridge-state flip, so the counts below attribute to it alone.
constexpr const char *kEntryMarker = "; Bridge static pressure advance\n";
constexpr const char *kExitMarker  = "; Bridge static pressure advance reverted\n";

size_t count_marker(const std::string &gcode, const char *marker)
{
    const std::string needle(marker);
    size_t            n = 0;
    for (size_t pos = gcode.find(needle); pos != std::string::npos; pos = gcode.find(needle, pos + needle.size()))
        ++n;
    return n;
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
    bool        bridge_switch  = false;
    const char *bridges        = "0.02";
    const char *adaptive_model = "0.02,10,2000";
    const char *flavor         = "marlin2";
};

std::string slice_bridge(const PaConfig &pa_config)
{
    return slice({mesh(TestMesh::bridge)},
        {{"enable_pressure_advance", pa_config.enable_pa ? "1" : "0"},
         {"pressure_advance", pa_config.pressure_advance},
         {"adaptive_pressure_advance", pa_config.adaptive_pa ? "1" : "0"},
         {"adaptive_pressure_advance_model", pa_config.adaptive_model},
         {"enable_bridge_pressure_advance", pa_config.bridge_switch ? "1" : "0"},
         {"adaptive_pressure_advance_bridges", pa_config.bridges},
         {"gcode_flavor", pa_config.flavor}});
}

} // namespace

TEST_CASE("Bridge static pressure advance applies to bridge infill without adaptive pressure advance",
          "[BridgePA]")
{
    PaConfig config;
    config.bridge_switch = true;
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

    // One marker per bridge-state flip, each of the branch's own: counting PA commands instead would also
    // count the ones the start G-code, the wipe tower and filament changes emit, so a failure could not be
    // attributed. The branch emits on a transition only, so a per-path emission shows up here as a surplus
    // of markers, and a missing revert shows up in the exit count.
    const BridgeFlips flips   = count_bridge_flips(gcode);
    const size_t      entries = count_marker(gcode, kEntryMarker);
    const size_t      exits   = count_marker(gcode, kExitMarker);
    INFO("entries: " << entries << " markers for " << flips.entries << " bridge entries; exits: " << exits
                     << " markers for " << flips.exits << " bridge exits");
    REQUIRE(entries == flips.entries);
    REQUIRE(exits == flips.exits);
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
    config.adaptive_pa   = true;
    config.bridge_switch = true;
    const std::string gcode = slice_bridge(config);

    // The static bridge branch is gated on adaptive PA being off, so it must not emit anything here.
    // init_print() sets gcode_comments unconditionally after config.apply(), so a static-branch emission
    // would always carry this marker. The positive control in the first test case pins that coupling: if
    // the helper ever stops forcing the flag, that assertion fails loudly instead of this one silently
    // passing.
    REQUIRE(gcode.find("; Bridge static pressure advance") == std::string::npos);

    // The adaptive processor still applies the bridge override by itself, on the switch's authority.
    const std::vector<double> bridge_values = pa_values_for_role(gcode, "Bridge");
    REQUIRE(!bridge_values.empty());
    for (double value : bridge_values)
        REQUIRE(std::abs(value - 0.02) < 1e-6);

    // The distinction the 0.02 above rests on: the model string holds a single flow point, so the
    // interpolator fails and every path outside the override runs at the extruder's static PA. Pinning
    // that keeps the bridge assertion honest: give the model a second point and it would pass on the
    // model's own value without distinguishing the override.
    size_t non_bridge_samples = 0;
    for (const auto &sample : pa_by_role(gcode)) {
        if (is_bridge_role(sample.first) || sample.first == "Overhang wall")
            continue;
        INFO("role " << sample.first << " ran at " << sample.second);
        REQUIRE(std::abs(sample.second - 0.035) < 1e-6);
        ++non_bridge_samples;
    }
    REQUIRE(non_bridge_samples > 0);

    // Overhang perimeters belong to the override's scope while adaptive PA is on - the BR flag carries
    // both roles - which is the scope the option's tooltip states.
    const std::vector<double> overhang_values = pa_values_for_role(gcode, "Overhang wall");
    REQUIRE(!overhang_values.empty());
    for (double value : overhang_values)
        REQUIRE(std::abs(value - 0.02) < 1e-6);
}

// The switch off leaves the adaptive channel as it was: 0 is not a value, so nothing overrides the
// model and the bridge paths fall back like every other path. This quadrant is the one that keeps the
// switch's gate from widening into a bare isBridge check - drop the switch and the value test from the
// override condition and this case is the only one that notices.
TEST_CASE("A zero bridge override does not disturb the adaptive model while the switch is off", "[BridgePA]")
{
    PaConfig config;
    config.adaptive_pa = true;
    config.bridges     = "0";
    const std::string gcode = slice_bridge(config);

    // The model holds a single flow point, so interpolation fails and the extruder's static PA is what
    // each of these roles runs at - the same footing the adaptive case above establishes for its
    // non-bridge paths, applied here to the roles the override would have taken.
    for (const char *role : {"Bridge", "Internal Bridge"}) {
        const std::vector<double> values = pa_values_for_role(gcode, role);
        INFO("role " << role);
        REQUIRE(!values.empty());
        for (double value : values)
            REQUIRE(std::abs(value - 0.035) < 1e-6);
    }

    // init_print() forces gcode_comments on, so an override emission would carry its comment here. The
    // static branch is gated on adaptive PA being off, so its marker cannot appear either.
    REQUIRE(gcode.find("; APA Model Override (bridge)") == std::string::npos);
    REQUIRE(gcode.find("; Bridge static pressure advance") == std::string::npos);
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
    config.bridge_switch = true;
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
    PaConfig       config;
    Slic3r::Print  print;
    Slic3r::Model  model;
    config.bridge_switch = true;
    init_print({mesh(TestMesh::bridge)}, print, model,
        {{"enable_pressure_advance", "1"},
         {"pressure_advance", config.pressure_advance},
         {"adaptive_pressure_advance", "0"},
         {"enable_bridge_pressure_advance", "1"},
         {"adaptive_pressure_advance_bridges", config.bridges},
         {"gcode_flavor", config.flavor}});
    print.is_BBL_printer() = true;

    const std::string gcode = Test::gcode(print);

    REQUIRE(gcode.find("M900 K0.02 L1000 M10 ;") != std::string::npos);
    REQUIRE(gcode.find("M900 K0.035 L1000 M10 ;") != std::string::npos);
}

// The switch makes 0 a real value on the static channel: the bridge is entered at PA 0 and left again
// at the filament's own PA. The markers still count the flips one for one, so the zero is emitted on a
// transition and not per path.
TEST_CASE("A zero bridge override is applied when the bridge pressure advance switch is on", "[BridgePA]")
{
    PaConfig config;
    config.bridges       = "0";
    config.bridge_switch = true;
    const std::string gcode = slice_bridge(config);

    for (const char *role : {"Bridge", "Internal Bridge"}) {
        const std::vector<double> values = pa_values_for_role(gcode, role);
        INFO("role " << role);
        REQUIRE(!values.empty());
        for (double value : values)
            REQUIRE(std::abs(value) < 1e-9);
    }

    // The revert restores the extruder's own PA, so the switch stays scoped to the bridge paths.
    const std::vector<double> non_bridge_values = pa_values_outside_bridges(gcode);
    REQUIRE(!non_bridge_values.empty());
    for (double value : non_bridge_values)
        REQUIRE(std::abs(value - 0.035) < 1e-6);

    const BridgeFlips flips   = count_bridge_flips(gcode);
    const size_t      entries = count_marker(gcode, kEntryMarker);
    const size_t      exits   = count_marker(gcode, kExitMarker);
    INFO("entries: " << entries << " markers for " << flips.entries << " bridge entries; exits: " << exits
                     << " markers for " << flips.exits << " bridge exits");
    REQUIRE(entries == flips.entries);
    REQUIRE(exits == flips.exits);
}

// Switched on, a non-zero value behaves exactly as it did while the switch did not exist.
TEST_CASE("The bridge pressure advance switch leaves a non-zero override unchanged", "[BridgePA]")
{
    PaConfig config;
    config.bridge_switch = true;
    const std::string gcode = slice_bridge(config);

    for (const char *role : {"Bridge", "Internal Bridge"}) {
        const std::vector<double> values = pa_values_for_role(gcode, role);
        INFO("role " << role);
        REQUIRE(!values.empty());
        for (double value : values)
            REQUIRE(std::abs(value - 0.02) < 1e-6);
    }

    const std::vector<double> non_bridge_values = pa_values_outside_bridges(gcode);
    REQUIRE(!non_bridge_values.empty());
    for (double value : non_bridge_values)
        REQUIRE(std::abs(value - 0.035) < 1e-6);
}

// The adaptive channel honours the switch on its own: with adaptive PA on the static branch is out of
// the picture, so a PA of 0 on bridge infill here can only come from the AdaptivePAProcessor override.
TEST_CASE("The bridge pressure advance switch makes zero override the adaptive model", "[BridgePA]")
{
    PaConfig config;
    config.adaptive_pa   = true;
    config.bridges       = "0";
    config.bridge_switch = true;
    const std::string gcode = slice_bridge(config);

    REQUIRE(gcode.find("; Bridge static pressure advance") == std::string::npos);

    const std::vector<double> bridge_values = pa_values_for_role(gcode, "Bridge");
    REQUIRE(!bridge_values.empty());
    for (double value : bridge_values)
        REQUIRE(std::abs(value) < 1e-9);

    // Overhang perimeters share the bridge override on this channel (the PA_Change tag carries both
    // roles), so they are excluded from the "everything else keeps the extruder PA" set.
    size_t non_bridge_samples = 0;
    for (const auto &sample : pa_by_role(gcode)) {
        if (is_bridge_role(sample.first) || sample.first == "Overhang wall")
            continue;
        INFO("role " << sample.first << " ran at " << sample.second);
        REQUIRE(std::abs(sample.second - 0.035) < 1e-6);
        ++non_bridge_samples;
    }
    REQUIRE(non_bridge_samples > 0);
}

// The switch is the master switch of the feature: while it is off the value is inert even when it is
// non-zero, so the static channel must leave the G-code exactly as it is with the value at 0 - one PA
// command for the filament and nothing per bridge.
TEST_CASE("A non-zero bridge override is inert while the switch is off", "[BridgePA]")
{
    PaConfig config;
    config.bridges       = "0.02";
    config.bridge_switch = false;
    const std::string gcode = slice_bridge(config);

    for (const auto &sample : pa_by_role(gcode)) {
        INFO("role " << sample.first << " ran at " << sample.second);
        REQUIRE(std::abs(sample.second - 0.035) < 1e-6);
    }
    REQUIRE(!pa_by_role(gcode).empty());

    // Only the filament's initial PA, nothing emitted per bridge.
    REQUIRE(count_pa_commands(gcode) == 1);

    // init_print() forces gcode_comments on, so a static-branch emission would carry its marker.
    REQUIRE(gcode.find("; Bridge static pressure advance") == std::string::npos);
}

// The adaptive channel obeys the same master switch: with adaptive PA on and the switch off, a non-zero
// bridge value must leave the model's own output alone. The model holds two flow points at one
// acceleration, so the interpolator really initialises and every path reaches the override branch - a
// single-point model falls back before that branch is even considered, which is what made an earlier
// version of this case pass with the value ungated. Its PA values sit far from the bridge value under
// test, so a leak of that value into the model's output cannot pass unnoticed. The proof is a pair of
// slices that differ only in the bridge value: with the switch off they must agree sample for sample.
TEST_CASE("The switch off stops the adaptive bridge override", "[BridgePA]")
{
    PaConfig with_value;
    with_value.adaptive_pa    = true;
    with_value.adaptive_model = "0.06,10,2000\n0.08,20,2000";
    with_value.bridges        = "0.02";
    with_value.bridge_switch  = false;

    PaConfig with_zero  = with_value;
    with_zero.bridges   = "0";

    const std::string gcode_with_value = slice_bridge(with_value);
    const std::string gcode_with_zero  = slice_bridge(with_zero);

    const std::vector<std::pair<std::string, double>> samples_with_value = pa_by_role(gcode_with_value);
    const std::vector<std::pair<std::string, double>> samples_with_zero  = pa_by_role(gcode_with_zero);
    REQUIRE(!samples_with_value.empty());
    REQUIRE(samples_with_value.size() == samples_with_zero.size());
    for (size_t i = 0; i < samples_with_value.size(); ++i) {
        INFO("sample " << i << ": role " << samples_with_value[i].first << " at " << samples_with_value[i].second
                       << " against role " << samples_with_zero[i].first << " at " << samples_with_zero[i].second);
        REQUIRE(samples_with_value[i].first == samples_with_zero[i].first);
        REQUIRE(std::abs(samples_with_value[i].second - samples_with_zero[i].second) < 1e-9);
    }

    // init_print() forces gcode_comments on, so an override emission would carry its comment - and that
    // comment sits under the same condition as the override itself.
    REQUIRE(gcode_with_value.find("; APA Model Override (bridge)") == std::string::npos);

    // The static branch is gated on adaptive PA being off, so its marker cannot appear either.
    REQUIRE(gcode_with_value.find("; Bridge static pressure advance") == std::string::npos);
}
