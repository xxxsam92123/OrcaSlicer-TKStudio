#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/Preset.hpp"
#include "libslic3r/PrintConfig.hpp"

#include <algorithm>
#include "libslic3r/Config.hpp"
#include <vector>
#include <cstddef>
#include <string>
#include <utility>

using namespace Slic3r;

// Regression test for the python-plugin branch's intentional divergence from
// upstream in add_correct_opts_to_diff() (src/libslic3r/Preset.cpp): a vector
// option entry whose index is beyond the reference vector's length is reported
// dirty even when it duplicates an existing value. On main these duplicates
// were NOT flagged. See the comment on add_correct_opts_to_diff() in src/libslic3r/Preset.cpp.
TEST_CASE("deep_diff flags new vector entries that duplicate values[0]", "[PresetDiff][Config]")
{
    // reference: single-extruder vector (one entry)
    Preset reference(Preset::TYPE_PRINTER, "ref");
    reference.config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.4});

    // edited: a second extruder entry was added whose value duplicates the first
    Preset edited(Preset::TYPE_PRINTER, "edited");
    edited.config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.4, 0.4});

    // deep_compare = true routes through deep_diff() -> add_correct_opts_to_diff()
    std::vector<std::string> diff =
        PresetCollection::dirty_options(&edited, &reference, /*deep_compare=*/true);

    // The new index #1 is reported dirty even though 0.4 == values[0] (0.4).
    REQUIRE(std::find(diff.begin(), diff.end(), "nozzle_diameter#1") != diff.end());

    // Sanity: the unchanged existing index #0 is NOT reported, so the rule is
    // specific to new indices rather than flagging the whole vector.
    REQUIRE(std::find(diff.begin(), diff.end(), "nozzle_diameter#0") == diff.end());
}

TEST_CASE("deep_diff distinguishes absolute and percentage speeds for each variant", "[PresetDiff][Config]")
{
    const size_t changed_index = GENERATE(size_t(0), size_t(1));
    Preset reference(Preset::TYPE_PRINT, "ref");
    reference.config.set_key_value("small_perimeter_speed", new ConfigOptionFloatsOrPercents{{50., false}, {50., false}});

    Preset edited = reference;
    edited.config.option<ConfigOptionFloatsOrPercents>("small_perimeter_speed")->values[changed_index].percent = true;

    const auto diff = PresetCollection::dirty_options(&edited, &reference, /*deep_compare=*/true);
    REQUIRE(diff == std::vector<std::string>{"small_perimeter_speed#" + std::to_string(changed_index)});

    DynamicPrintConfig transferred = reference.config;
    transferred.apply_only(edited.config, diff);
    REQUIRE(*transferred.option("small_perimeter_speed") == *edited.config.option("small_perimeter_speed"));
}

// Switching printers re-encodes per-variant print options: a vector option such as
// outer_wall_speed holds one slot per entry of printer_extruder_variant, and a dirty slot is
// reported as "outer_wall_speed#i". Slots have to be paired across the switch by (extruder id,
// variant name), otherwise a modified speed is either discarded or lands on another extruder's
// identically named variant.
namespace {

// Variant lists as they appear in resources/profiles/BBL/machine/*.json. X2D carries a different
// list per nozzle size, which is what makes a slot index move between two presets of one printer.
const std::vector<std::string> P1S_VARIANTS {"Direct Drive Standard", "Direct Drive High Flow"};
const std::vector<int>         P1S_EXTRUDERS {1, 1};

const std::vector<std::string> H2S_VARIANTS {"Direct Drive Standard", "Direct Drive High Flow", "Direct Drive E3D High Flow"};
const std::vector<int>         H2S_EXTRUDERS {1, 1, 1};

const std::vector<std::string> H2D_VARIANTS {"Direct Drive Standard", "Direct Drive High Flow", "Direct Drive E3D High Flow",
                                             "Direct Drive Standard", "Direct Drive High Flow", "Direct Drive TPU High Flow",
                                             "Direct Drive E3D High Flow"};
const std::vector<int>         H2D_EXTRUDERS {1, 1, 1, 2, 2, 2, 2};

const std::vector<std::string> X2D4_VARIANTS {"Direct Drive Standard", "Direct Drive High Flow", "Bowden Standard", "Bowden High Flow"};
const std::vector<int>         X2D4_EXTRUDERS {1, 1, 2, 2};

const std::vector<std::string> X2D6_VARIANTS {"Direct Drive Standard", "Direct Drive High Flow", "Direct Drive E3D High Flow",
                                              "Bowden Standard", "Bowden High Flow", "Bowden E3D High Flow"};
const std::vector<int>         X2D6_EXTRUDERS {1, 1, 1, 2, 2, 2};

constexpr size_t NO_SLOT = std::string::npos;

} // namespace

TEST_CASE("variant slots survive a switch between printers with the same variant list", "[PresetDiff][PresetTransfer]")
{
    const std::vector<std::string> keys {"outer_wall_speed#0", "outer_wall_speed#1"};
    std::vector<std::string> dropped;
    std::vector<std::pair<size_t, size_t>> indices;

    const std::vector<std::string> remapped =
        remap_variant_option_keys(keys, P1S_VARIANTS, P1S_EXTRUDERS, P1S_VARIANTS, P1S_EXTRUDERS, &dropped, &indices);

    REQUIRE(remapped == keys);
    REQUIRE(dropped.empty());
    REQUIRE(indices == std::vector<std::pair<size_t, size_t>>{{0, 0}, {1, 1}});
}

TEST_CASE("a variant slot keeps its own nozzle when the target printer gains variants", "[PresetDiff][PresetTransfer]")
{
    const std::vector<std::string> keys {"outer_wall_speed#0", "travel_acceleration#1"};
    std::vector<std::string> dropped;
    std::vector<std::pair<size_t, size_t>> indices;

    const std::vector<std::string> remapped =
        remap_variant_option_keys(keys, P1S_VARIANTS, P1S_EXTRUDERS, H2S_VARIANTS, H2S_EXTRUDERS, &dropped, &indices);

    REQUIRE(remapped == keys);
    REQUIRE(dropped.empty());
    REQUIRE(indices == std::vector<std::pair<size_t, size_t>>{{0, 0}, {1, 1}});
}

TEST_CASE("a variant slot pairs with the same extruder's variant, not a same-named one elsewhere", "[PresetDiff][PresetTransfer]")
{
    // H2D lists "Direct Drive High Flow" on extruder 1 (#1) and again on extruder 2 (#4).
    SECTION("P1S High Flow is extruder 1's slot") {
        std::vector<std::string> dropped;
        std::vector<std::pair<size_t, size_t>> indices;
        const std::vector<std::string> remapped = remap_variant_option_keys(
            {"outer_wall_speed#1"}, P1S_VARIANTS, P1S_EXTRUDERS, H2D_VARIANTS, H2D_EXTRUDERS, &dropped, &indices);

        REQUIRE(remapped == std::vector<std::string>{"outer_wall_speed#1"});
        REQUIRE(dropped.empty());
        REQUIRE(indices == std::vector<std::pair<size_t, size_t>>{{1, 1}});
    }
    SECTION("H2D's second extruder has no counterpart on a single-extruder printer") {
        std::vector<std::string> dropped;
        const std::vector<std::string> remapped = remap_variant_option_keys(
            {"outer_wall_speed#4"}, H2D_VARIANTS, H2D_EXTRUDERS, P1S_VARIANTS, P1S_EXTRUDERS, &dropped);

        REQUIRE(remapped.empty());
        REQUIRE(dropped == std::vector<std::string>{"outer_wall_speed#4"});
    }
}

TEST_CASE("slots with no counterpart on the target printer are dropped, the rest migrate", "[PresetDiff][PresetTransfer]")
{
    // X2D 0.4 nozzle has 6 slots: [Std1, HF1, E3D1, BowdenStd2, BowdenHF2, BowdenE3D2].
    // H2S has 3: [Std1, HF1, E3D1]. The Bowden variants have no counterpart there.
    const std::vector<std::string> keys {"outer_wall_speed#0", "outer_wall_speed#3", "travel_speed#5", "outer_wall_speed#2"};
    std::vector<std::string> dropped;
    std::vector<std::pair<size_t, size_t>> indices;

    const std::vector<std::string> remapped =
        remap_variant_option_keys(keys, X2D6_VARIANTS, X2D6_EXTRUDERS, H2S_VARIANTS, H2S_EXTRUDERS, &dropped, &indices);

    REQUIRE(remapped == std::vector<std::string>{"outer_wall_speed#0", "outer_wall_speed#2"});
    REQUIRE(dropped == std::vector<std::string>{"outer_wall_speed#3", "travel_speed#5"});
    REQUIRE(indices == std::vector<std::pair<size_t, size_t>>{{0, 0}, {2, 2}});
}

TEST_CASE("options without a variant slot pass through unchanged and in order", "[PresetDiff][PresetTransfer]")
{
    const std::vector<std::string> keys {"layer_height", "outer_wall_speed#1", "wall_loops"};
    std::vector<std::string> dropped;
    std::vector<std::pair<size_t, size_t>> indices;

    const std::vector<std::string> remapped =
        remap_variant_option_keys(keys, P1S_VARIANTS, P1S_EXTRUDERS, H2D_VARIANTS, H2D_EXTRUDERS, &dropped, &indices);

    REQUIRE(remapped == keys);
    REQUIRE(dropped.empty());
    REQUIRE(indices.size() == keys.size());
    REQUIRE(indices[0] == std::pair<size_t, size_t>{NO_SLOT, NO_SLOT});
    REQUIRE(indices[1] == std::pair<size_t, size_t>{1, 1});
    REQUIRE(indices[2] == std::pair<size_t, size_t>{NO_SLOT, NO_SLOT});
}

TEST_CASE("a slot the old printer does not describe is dropped instead of remapped", "[PresetDiff][PresetTransfer]")
{
    SECTION("slot index beyond the old variant list") {
        std::vector<std::string> dropped;
        const std::vector<std::string> remapped = remap_variant_option_keys(
            {"outer_wall_speed#5"}, P1S_VARIANTS, P1S_EXTRUDERS, H2D_VARIANTS, H2D_EXTRUDERS, &dropped);

        REQUIRE(remapped.empty());
        REQUIRE(dropped == std::vector<std::string>{"outer_wall_speed#5"});
    }
    SECTION("slot suffix that is not a number") {
        std::vector<std::string> dropped;
        const std::vector<std::string> remapped = remap_variant_option_keys(
            {"outer_wall_speed#x"}, P1S_VARIANTS, P1S_EXTRUDERS, H2D_VARIANTS, H2D_EXTRUDERS, &dropped);

        REQUIRE(remapped.empty());
        REQUIRE(dropped == std::vector<std::string>{"outer_wall_speed#x"});
    }
    SECTION("old extruder ids missing falls back to matching by variant name") {
        std::vector<std::string> dropped;
        std::vector<std::pair<size_t, size_t>> indices;
        const std::vector<std::string> remapped = remap_variant_option_keys(
            {"outer_wall_speed#1"}, P1S_VARIANTS, {}, H2D_VARIANTS, H2D_EXTRUDERS, &dropped, &indices);

        REQUIRE(remapped == std::vector<std::string>{"outer_wall_speed#1"});
        REQUIRE(dropped.empty());
        REQUIRE(indices == std::vector<std::pair<size_t, size_t>>{{1, 1}});
    }
    SECTION("target printer without variants drops every slot") {
        std::vector<std::string> dropped;
        const std::vector<std::string> remapped = remap_variant_option_keys(
            {"outer_wall_speed#0"}, P1S_VARIANTS, P1S_EXTRUDERS, {}, {}, &dropped);

        REQUIRE(remapped.empty());
        REQUIRE(dropped == std::vector<std::string>{"outer_wall_speed#0"});
    }
    SECTION("two old slots pairing with one target slot keep the first") {
        // A degenerate old list with the same (extruder, variant) twice: only one target slot exists.
        std::vector<std::string> dropped;
        const std::vector<std::string> remapped = remap_variant_option_keys(
            {"outer_wall_speed#0", "outer_wall_speed#1"},
            {"Direct Drive Standard", "Direct Drive Standard"}, {1, 1},
            {"Direct Drive Standard"}, {1}, &dropped);

        REQUIRE(remapped == std::vector<std::string>{"outer_wall_speed#0"});
        REQUIRE(dropped == std::vector<std::string>{"outer_wall_speed#1"});
    }
}

TEST_CASE("a slot that moved keeps the value it had on the old printer", "[PresetDiff][PresetTransfer]")
{
    // X2D 0.2 nozzle -> 0.4 nozzle: Bowden Standard moves from slot #2 to slot #3.
    Preset source(Preset::TYPE_PRINT, "source");
    source.config.set_key_value("outer_wall_speed", new ConfigOptionFloats{10.f, 20.f, 30.f, 40.f});
    Preset target(Preset::TYPE_PRINT, "target");
    target.config.set_key_value("outer_wall_speed", new ConfigOptionFloats{0.f, 0.f, 0.f, 0.f, 0.f, 0.f});

    std::vector<std::string> dropped;
    std::vector<std::pair<size_t, size_t>> indices;
    const std::vector<std::string> remapped = remap_variant_option_keys(
        {"outer_wall_speed#2"}, X2D4_VARIANTS, X2D4_EXTRUDERS, X2D6_VARIANTS, X2D6_EXTRUDERS, &dropped, &indices);

    REQUIRE(remapped == std::vector<std::string>{"outer_wall_speed#3"});
    REQUIRE(indices == std::vector<std::pair<size_t, size_t>>{{2, 3}});

    // apply_only() reads the source and writes the target at the same slot index, so the old #2
    // value has to be placed at the target slot first -- that is what indices is for.
    DynamicPrintConfig moved = source.config;
    moved.option<ConfigOptionFloats>("outer_wall_speed")->values[3] =
        source.config.option<ConfigOptionFloats>("outer_wall_speed")->values[2];
    DynamicPrintConfig applied = target.config;
    applied.apply_only(moved, remapped);

    const ConfigOptionFloats *speed = applied.option<ConfigOptionFloats>("outer_wall_speed");
    REQUIRE(speed->values.size() == 6);
    REQUIRE_THAT(speed->values[3], Catch::Matchers::WithinAbs(30.f, 1e-4));
    // The slot the option came from is not the one being written, and no other slot is touched.
    REQUIRE_THAT(speed->values[4], Catch::Matchers::WithinAbs(0.f, 1e-4));
    REQUIRE_THAT(speed->values[2], Catch::Matchers::WithinAbs(0.f, 1e-4));
}

TEST_CASE("several options edited on one variant slot all migrate together", "[PresetDiff][PresetTransfer]")
{
    // A user who tunes one nozzle edits a set of speeds, not a single option: every key carrying the
    // same old slot has to resolve to the same target slot instead of taking it away from each other.
    const std::vector<std::string> keys {"outer_wall_speed#1", "inner_wall_speed#1",
                                         "sparse_infill_speed#1", "travel_speed#1"};
    std::vector<std::string> dropped;
    std::vector<std::pair<size_t, size_t>> indices;

    const std::vector<std::string> remapped =
        remap_variant_option_keys(keys, P1S_VARIANTS, P1S_EXTRUDERS, H2S_VARIANTS, H2S_EXTRUDERS, &dropped, &indices);

    REQUIRE(remapped == keys);
    REQUIRE(dropped.empty());
    REQUIRE(indices == std::vector<std::pair<size_t, size_t>>{{1, 1}, {1, 1}, {1, 1}, {1, 1}});
}

TEST_CASE("several options on the default variant slot all migrate together", "[PresetDiff][PresetTransfer]")
{
    const std::vector<std::string> keys {"outer_wall_speed#0", "inner_wall_speed#0", "travel_speed#0"};
    std::vector<std::string> dropped;
    std::vector<std::pair<size_t, size_t>> indices;

    const std::vector<std::string> remapped =
        remap_variant_option_keys(keys, P1S_VARIANTS, P1S_EXTRUDERS, H2S_VARIANTS, H2S_EXTRUDERS, &dropped, &indices);

    REQUIRE(remapped == keys);
    REQUIRE(dropped.empty());
    REQUIRE(indices == std::vector<std::pair<size_t, size_t>>{{0, 0}, {0, 0}, {0, 0}});
}

TEST_CASE("a second old slot competing for one target slot drops each of its options", "[PresetDiff][PresetTransfer]")
{
    // Two old slots describe the same (extruder, variant) and only one target slot exists: the old
    // slot that resolves first keeps it and every option of the other one is dropped.
    std::vector<std::string> dropped;
    const std::vector<std::string> remapped = remap_variant_option_keys(
        {"outer_wall_speed#0", "inner_wall_speed#0", "outer_wall_speed#1", "inner_wall_speed#1"},
        {"Direct Drive Standard", "Direct Drive Standard"}, {1, 1},
        {"Direct Drive Standard"}, {1}, &dropped);

    REQUIRE(remapped == std::vector<std::string>{"outer_wall_speed#0", "inner_wall_speed#0"});
    REQUIRE(dropped == std::vector<std::string>{"outer_wall_speed#1", "inner_wall_speed#1"});
}
