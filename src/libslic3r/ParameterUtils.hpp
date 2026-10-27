#ifndef slic3r_Parameter_Utils_hpp_
#define slic3r_Parameter_Utils_hpp_

#include <utility>
#include <string>
#include <vector>
#include <map>
#include "Config.hpp"
#include "PrintConfig.hpp"

namespace Slic3r {
using LayerPrintSequence = std::pair<std::pair<int, int>, std::vector<int>>;
std::vector<LayerPrintSequence> get_other_layers_print_sequence(int sequence_nums, const std::vector<int> &sequence);
void get_other_layers_print_sequence(const std::vector<LayerPrintSequence> &customize_sequences, int &sequence_nums, std::vector<int> &sequence);

extern int get_index_for_extruder_parameter(const DynamicPrintConfig &config, const std::string &opt_key, int cur_extruder_id, ExtruderType extruder_type, NozzleVolumeType nozzle_volume_type);

// Whether the option is stored as one column per nozzle variant, i.e. its length matches the variant column
// count (variant count x stride). config is the config holding the option (e.g. Tab::m_config), variant_name
// the variant column name of its domain ("print_extruder_variant" for print, "printer_extruder_variant" for
// printer). Array indices of other vector options are plain slots, not nozzle slots.
bool is_variant_column_option(const DynamicPrintConfig &config, const std::string &opt_key,
                              const std::string &variant_name, unsigned int stride);

// Reads value idx of a vector option as a display string. Returns false and leaves out untouched when opt is
// empty or idx is outside of it.
bool variant_column_value(const ConfigOptionVectorBase &opt, int idx, std::string &out);
} // namespace Slic3r

#endif // slic3r_Parameter_Utils_hpp_
