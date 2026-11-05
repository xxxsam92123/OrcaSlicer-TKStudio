#pragma once

#include <string>

namespace Slic3r {

struct GCodeProcessorResult;

// The statistics of a plate's G-code as JSON, in seconds, millimeters and grams: totals, per layer and per object instance.
std::string gcode_statistics_json(const GCodeProcessorResult &result);
// Whether gcode_statistics_json() can produce a document for `result`: it has to have the document
// cached (from the export, or read back from the project it came from), or the moves it is summed
// from. A result whose preview payload was released and whose document was never cached has
// neither, and would sum to an empty one.
bool has_gcode_statistics(const GCodeProcessorResult &result);
// Whether `statistics_json` holds a document with statistics in it. Summing a result without moves
// produces a well formed document whose "layers" array is empty and whose totals are all zero, so a
// non-empty string does not mean there is anything to report. This tells the two apart, for the
// documents read back out of a project as well as for the ones summed here.
bool statistics_json_has_data(const std::string &statistics_json);
// Where the statistics of the G-code at gcode_path are written.
std::string gcode_statistics_path(const std::string &gcode_path);
// Throws ExportError when the file cannot be written.
void export_gcode_statistics(const GCodeProcessorResult &result, const std::string &path);

} // namespace Slic3r
