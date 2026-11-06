#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCode/GCodeStatistics.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Utils.hpp"

#include "test_helpers.hpp"
#include "test_utils.hpp"

#include <nlohmann/json.hpp>

#include <cstddef>
#include <initializer_list>
#include <set>
#include <string>
#include <vector>

using namespace Slic3r;

TEST_CASE("The statistics of a sliced plate add up per layer, feature and object instance", "[GCodeStatistics]")
{
    // A 20 mm cube at 0.2 mm layers prints 100 of them; without skirt or brim all its filament goes to its instances.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "layer_height", 0.2 },
                                    { "initial_layer_print_height", 0.2 },
                                    { "skirt_loops", 0 },
                                    { "brim_type", "no_brim" },
                                    { "filament_density", 1.25 } });
    Print print;
    Model model;
    Test::init_print({ Test::cube(20) }, print, model, config, nullptr, true, 2);
    GCodeProcessorResult result;
    Test::gcode(print, &result);

    ScopedTemporaryFile file(".json");
    export_gcode_statistics(result, file.string());
    std::string text;
    load_string_file(file.path(), text);
    const nlohmann::json statistics = nlohmann::json::parse(text);

    const nlohmann::json &summary = statistics["summary"];
    const nlohmann::json &layers  = statistics["layers"];
    REQUIRE(layers.size() == 100);
    CHECK(summary["layer_count"] == 100);
    CHECK_THAT(layers.front()["z"].get<double>(), Catch::Matchers::WithinAbs(0.2, 1e-3));
    CHECK_THAT(layers.back()["z"].get<double>(), Catch::Matchers::WithinAbs(20., 1e-3));

    double time = 0., volume = 0.;
    for (const nlohmann::json &layer : layers) {
        time += layer["time"].get<double>();
        volume += layer["volume"].get<double>();
        double features_time = 0.;
        for (const auto &[name, feature] : layer["features"].items())
            features_time += feature["time"].get<double>();
        CHECK(features_time <= layer["time"].get<double>() + 1e-3);
    }
    // Each value is rounded to the millisecond and the cubic millimeter.
    CHECK_THAT(time, Catch::Matchers::WithinAbs(summary["time"].get<double>(), 0.1));
    CHECK_THAT(volume, Catch::Matchers::WithinAbs(summary["volume"].get<double>(), 0.1));
    CHECK_THAT(summary["weight"].get<double>(), Catch::Matchers::WithinRel(summary["volume"].get<double>() * 1.25 / 1000., 1e-4));
    CHECK(summary.contains("filament_cost"));

    // The filament splits by use as the preview's table does.
    REQUIRE(summary["filaments"].size() == 1);
    const nlohmann::json &filament = summary["filaments"].front();
    double used = 0.;
    for (const char *use : { "model", "support", "prime_tower", "flush" })
        used += filament[use]["volume"].get<double>();
    CHECK_THAT(used, Catch::Matchers::WithinAbs(filament["volume"].get<double>(), 0.01));

    const nlohmann::json &features = summary["features"];
    CHECK(features.contains("Outer wall"));
    CHECK(features.contains("Sparse infill"));
    double model_volume = 0.;
    for (const auto &[name, feature] : features.items())
        if (name != "Custom")
            model_volume += feature["volume"].get<double>();

    const nlohmann::json &objects = statistics["objects"];
    REQUIRE(objects.size() == 2);
    double objects_volume = 0.;
    for (size_t i = 0; i < objects.size(); ++i) {
        CHECK(objects[i]["name"] == model.objects.front()->name);
        CHECK(objects[i]["instance"] == int(i));
        CHECK(objects[i]["support_volume"].get<double>() == 0.);
        objects_volume += objects[i]["volume"].get<double>();
    }
    CHECK_THAT(objects_volume, Catch::Matchers::WithinAbs(model_volume, 0.1));
}

TEST_CASE("Copies sliced as objects of their own keep their number in the model object", "[GCodeStatistics]")
{
    // A copy of another scale slices as a print object of its own.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "skirt_loops", 0 }, { "brim_type", "no_brim" } });
    Print print;
    Model model;
    Test::init_print({ Test::cube(20) }, print, model, config, nullptr, true, 2);
    model.objects.front()->instances.back()->set_scaling_factor(Vec3d(0.5, 0.5, 0.5));
    model.objects.front()->ensure_on_bed();
    print.apply(model, config);
    REQUIRE(print.objects().size() == 2);
    GCodeProcessorResult result;
    Test::gcode(print, &result);

    const nlohmann::json objects = nlohmann::json::parse(gcode_statistics_json(result))["objects"];
    std::set<int>        instances;
    for (const nlohmann::json &object : objects)
        instances.insert(object["instance"].get<int>());
    CHECK(instances == std::set<int>{ 0, 1 });
}

TEST_CASE("An export that does not keep the preview payload reports the same G-code and statistics", "[GCodeStatistics]")
{
    // What the command line does: export without the per-move payload, because it never renders a
    // preview from the result. Neither the G-code nor the statistics document may move - the
    // document is summed from an accumulator fed in the move list's order (see
    // GCodeProcessorResult::PreviewStatistics), so this is the comparison that proves it.
    const auto slice_once = [](bool preview_payload, GCodeProcessorResult &result,
                               std::initializer_list<ConfigBase::SetDeserializeItem> extra = {},
                               size_t *masked_header_lines = nullptr) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({ { "layer_height", 0.2 },
                                        { "initial_layer_print_height", 0.2 },
                                        { "skirt_loops", 0 },
                                        { "brim_type", "no_brim" },
                                        { "filament_density", 1.25 },
                                        { "enable_arc_fitting", "enabled" } });
        config.set_deserialize_strict(extra);
        Print print;
        Model model;
        Test::init_print({ Test::cube(20) }, print, model, config, nullptr, true, 2);
        return Test::gcode(print, &result, preview_payload, masked_header_lines);
    };

    GCodeProcessorResult stored;
    size_t               stored_masked = 0;
    const std::string    gcode_stored = slice_once(true, stored, {}, &stored_masked);
    GCodeProcessorResult skipped;
    size_t               skipped_masked = 0;
    const std::string    gcode_skipped = slice_once(false, skipped, {}, &skipped_masked);

    // The byte comparison below only holds while the export timestamp is masked. If the header's
    // format changes so that no line is masked any more, these fail here instead of letting the
    // comparison start flaking whenever the two exports straddle a second.
    REQUIRE(stored_masked > 0);
    REQUIRE(skipped_masked > 0);

    REQUIRE(!stored.moves.empty());
    REQUIRE(!stored.statistics_json.empty());
    CHECK(skipped.moves.empty());
    CHECK(skipped.preview_payload_skipped);
    // The G-code is written from the time machine's g1 cache, not from the moves, so dropping the
    // moves must not touch a byte of it - the M73 lines included.
    CHECK(gcode_skipped == gcode_stored);
    CHECK(skipped.statistics_json == stored.statistics_json);
    // The document still describes the plate, not an empty one.
    const nlohmann::json statistics = nlohmann::json::parse(skipped.statistics_json);
    CHECK(statistics["summary"]["layer_count"] == 100);
    CHECK(statistics["summary"]["time"].get<double>() > 0.);
    CHECK(statistics["summary"]["volume"].get<double>() > 0.);

    // The multi-nozzle pre-heat/pre-cool injector resolves its idle windows against the moves, so a
    // machine that enables pre-heating overrules the caller: the payload comes back even though this
    // export said it would not render it. The control is the *same* machine with the payload stored,
    // because turning enable_pre_heating on moves the G-code by design, before any of this is
    // involved: the writer emits `; MACHINE_START_GCODE_END` / `; MACHINE_END_GCODE_START` for the
    // injector's usage blocks (GCode.cpp, gated on the same flag: they are what activates it) and the
    // config block echoes `enable_pre_heating = 1`. Against the non-preheat run those lines alone
    // differ, so comparing across configs would test the flag, not the caller override.
    GCodeProcessorResult preheated_stored;
    size_t               preheated_stored_masked = 0;
    const std::string    gcode_preheated_stored = slice_once(true, preheated_stored, { { "enable_pre_heating", "1" } },
                                                             &preheated_stored_masked);
    GCodeProcessorResult preheated;
    size_t               preheated_masked = 0;
    const std::string    gcode_preheated = slice_once(false, preheated, { { "enable_pre_heating", "1" } },
                                                      &preheated_masked);
    REQUIRE(preheated_stored_masked > 0);
    REQUIRE(preheated_masked > 0);
    CHECK(!preheated.preview_payload_skipped);
    CHECK(!preheated.moves.empty());
    CHECK(gcode_preheated == gcode_preheated_stored);
    CHECK(preheated.statistics_json == preheated_stored.statistics_json);
}

TEST_CASE("The statistics file takes the G-code's name with the statistics extension", "[GCodeStatistics]")
{
    CHECK(gcode_statistics_path("plate_1.gcode") == "plate_1.statistics.json");
    CHECK(gcode_statistics_path("out/Cube_PLA_1h.gcode") == "out/Cube_PLA_1h.statistics.json");
}
