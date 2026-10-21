// Tree supports by Thomas Rahm, losely based on Tree Supports by CuraEngine.
// Original source of Thomas Rahm's tree supports:
// https://github.com/ThomasRahm/CuraEngine
//
// Original CuraEngine copyright:
// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef slic3r_TreeSupportCommon_hpp
#define slic3r_TreeSupportCommon_hpp

#include "../libslic3r.h"
#include "../Polygon.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Support/SupportParameters.hpp"
#include "libslic3r/Support/SupportLayer.hpp"
#include "SupportCommon.hpp"
#include "libslic3r/Utils.hpp"

#include <algorithm>
#include <cstddef>
#include <math.h>
#include <cmath>
#include <limits>
#include <cassert>
#include <cstdio>
#include <iterator>
#include <mutex>
#include <string_view>
#include <vector>
#include <utility>

namespace Slic3r
{
    // The number of vertices in each circle.
    static constexpr const size_t SUPPORT_TREE_CIRCLE_RESOLUTION = 25;
namespace TreeSupport3D
{
using LayerIndex = int;

// A support layer thinner than this is never emitted: the contact layer falls back to the layer
// below instead, which only makes the gap larger than requested (the safe direction). Mirrors the
// slicer's own floor for object layers (MIN_LAYER_HEIGHT in Slicing.cpp).
static constexpr double MIN_SUPPORT_LAYER_HEIGHT = 0.01;

/*!
 * \brief Z of every support layer, one entry per layer index.
 *
 * The organic generator used to derive every layer Z from a single layer height
 * (SlicingParameters::layer_height), which is only correct while the object is sliced with a
 * uniform layer height and the top / bottom gap is a multiple of it. Both assumptions break with
 * custom (variable) layer heights and with a gap the user did not round to the layer height. The
 * real object layers are the authority here, and one support layer per object layer keeps the layer
 * index space (and with it TreeModelVolumes' outline lookup at index num_raft_layers + object layer)
 * exactly as it was.
 *
 * A layer may additionally have its top lowered: the top contact layer of an overhang is placed
 * exactly `support_top_z_distance` below the overhang instead of on the object layer Z, which makes
 * that layer thinner than the object's. Such a layer is marked as a gap layer.
 */
struct TreeSupportLayerGrid
{
    // Millimetres to scaled units, rounded to the nearest unit instead of truncated: the grid Zs come
    // from double arithmetic, so a 0.19999999999999996 would otherwise end up a unit short of the real
    // 200 microns and shift every growth limit derived from it.
    [[nodiscard]] static coord_t scaled_rounded(const double mm)
    {
        return SCALING_FACTOR > 0. ? coord_t(std::llround(mm / SCALING_FACTOR)) : coord_t(0);
    }

    struct Entry
    {
        // Z of the top of the layer.
        double     print_z  { 0. };
        // Z of the bottom of the layer, equals the print_z of the layer below (0 for the first one).
        // Lowering or inserting a layer moves this, so it describes the support stack, not the object.
        double     bottom_z { 0. };
        // Z where the object layer this entry belongs to really starts. Unlike bottom_z it is never
        // moved when a contact layer is lowered, so collision clearances are measured against the
        // object rather than against the support stack.
        double     obj_bottom_z { 0. };
        // print_z - bottom_z. Lowering the top of a layer shrinks it.
        double     height   { 0. };
        // Object layer this support layer belongs to, -1 for the raft layers below the object.
        LayerIndex obj_layer_idx { -1 };
        // The top was lowered to satisfy an exact "overhang - support_top_z_distance" Z gap.
        bool       is_gap_layer { false };
    };

    std::vector<Entry> entries;
    size_t             raft_layers { 0 };

    bool   empty() const { return entries.empty(); }
    size_t size()  const { return entries.size(); }

    // Top Z of the layer. Extrapolates with the height of the first / last layer beyond the grid: a
    // branch tip near the bed spans `tip_layers` of grid, which reach under the first layer, and the
    // layers above the highest support layer are asked for by the growth code. Both have to keep the
    // Z axis linear, so that a Z span stays layers x height; clamping it at the first layer would
    // shorten the span and with it the radius derived from it (contract §4.1/§4.4).
    double z(LayerIndex layer_idx) const
    {
        if (entries.empty())
            return 0.;
        if (layer_idx < 0)
            // Extrapolate below the first layer: z(-1) is the bottom of the first layer and each
            // further step down is one more of its heights, so a Z span stays layers x height.
            return entries.front().bottom_z + double(layer_idx + 1) * entries.front().height;
        if (layer_idx >= LayerIndex(entries.size())) {
            const Entry &last = entries.back();
            return last.print_z + double(layer_idx - LayerIndex(entries.size()) + 1) * last.height;
        }
        return entries[layer_idx].print_z;
    }

    double h(LayerIndex layer_idx) const
    {
        if (entries.empty())
            return 0.;
        if (layer_idx < 0)
            return entries.front().height;
        if (layer_idx >= LayerIndex(entries.size()))
            return entries.back().height;
        return entries[layer_idx].height;
    }

    double bottom_z(LayerIndex layer_idx) const { return z(layer_idx) - h(layer_idx); }

    // Move the Z where the object's material inside `layer_idx` really starts. A layer whose slice
    // reaches an overhang starts above the face when the face falls inside the layer below, and the
    // clearance of the contact layer below has to be measured against the face, not against the
    // layer's bottom: with the exact gap the contact top sits exactly `top_z_distance` under the face,
    // so that layer is not within the clearance and must not be treated as an obstacle there.
    // Lowering it is never done - the layer's slices do start at its bottom everywhere else.
    void set_obj_bottom_z(LayerIndex layer_idx, const double obj_bottom_z_req)
    {
        if (layer_idx < 0 || layer_idx >= LayerIndex(entries.size()))
            return;
        entries[layer_idx].obj_bottom_z = std::max(entries[layer_idx].obj_bottom_z, obj_bottom_z_req);
    }

    // Z where the object inside this layer really starts. Stays put when the support stack above is
    // pulled down by a lowered contact layer.
    double obj_bottom_z(LayerIndex layer_idx) const
    {
        if (entries.empty())
            return 0.;
        if (layer_idx < 0)
            return entries.front().obj_bottom_z;
        if (layer_idx >= LayerIndex(entries.size()))
            return entries.back().obj_bottom_z;
        return entries[layer_idx].obj_bottom_z;
    }

    // Vertical distance between the tops of two layers. The layers tile the Z axis, so this is the
    // sum of the layer heights in between.
    double z_span(LayerIndex from, LayerIndex to) const { return z(to) - z(from); }

    // How many layers a Z distance spans. Walking the grid keeps these counts meaningful with
    // variable layer heights; the three roundings reproduce the historical formulas exactly on a
    // uniform grid, which is what keeps existing output unchanged (contract §4.4).
    // Layers that fit completely into `distance` - matches a truncated `distance / layer_height`.
    size_t layers_fitting(const double distance, const LayerIndex from = 0) const
    {
        double accumulated = 0.;
        size_t layers      = 0;
        const size_t start = from > 0 ? size_t(from) : 0;
        for (size_t idx = start; idx < entries.size(); ++ idx) {
            const double height = entries[idx].height;
            if (height <= EPSILON)
                continue;
            if (accumulated + height > distance + EPSILON)
                break;
            accumulated += height;
            ++ layers;
        }
        return layers;
    }

    // Layers needed to cover `distance` - matches round_up_divide(distance, layer_height).
    size_t layers_covering(const double distance, const LayerIndex from = 0) const
    {
        double accumulated = 0.;
        size_t layers      = 0;
        const size_t start = from > 0 ? size_t(from) : 0;
        for (size_t idx = start; idx < entries.size() && accumulated < distance - EPSILON; ++ idx) {
            if (entries[idx].height <= EPSILON)
                continue;
            accumulated += entries[idx].height;
            ++ layers;
        }
        return layers;
    }

    // Layers closest to `distance`, ties rounding up - matches round(distance / layer_height).
    size_t layers_rounding(const double distance, const LayerIndex from = 0) const
    {
        double accumulated = 0.;
        size_t layers      = 0;
        const size_t start = from > 0 ? size_t(from) : 0;
        for (size_t idx = start; idx < entries.size(); ++ idx) {
            const double height = entries[idx].height;
            if (height <= EPSILON)
                continue;
            if (accumulated + height >= distance - EPSILON) {
                // One more layer crosses the distance; take it when it lands closer than stopping.
                if (distance - accumulated >= 0.5 * height - EPSILON)
                    ++ layers;
                break;
            }
            accumulated += height;
            ++ layers;
        }
        return layers;
    }

    // Lowest layer whose top reaches z, i.e. the layer that would have to be lowered to end at z.
    LayerIndex idx_ceil(double z_req) const
    {
        const auto it = std::lower_bound(entries.begin(), entries.end(), z_req - EPSILON,
            [](const Entry &entry, double z) { return entry.print_z < z; });
        return LayerIndex(it - entries.begin());
    }

    // Highest layer whose top does not exceed z.
    LayerIndex idx_floor(double z_req) const
    {
        const auto it = std::upper_bound(entries.begin(), entries.end(), z_req + EPSILON,
            [](double z, const Entry &entry) { return z < entry.print_z; });
        return std::max<LayerIndex>(0, LayerIndex(it - entries.begin()) - 1);
    }

    // Lower the top of `layer_idx` to `print_z_req`. Returns false when that would leave no layer
    // below the new top (the requested gap is thinner than MIN_SUPPORT_LAYER_HEIGHT).
    //
    // The top is lowered rather than a separate gap layer being inserted: the grid is one global stack,
    // so a layer inserted at the contact Z would leave the object layer above it in the stack and the
    // support would still reach that layer's top instead of the requested Z. Lowering the contact
    // layer's own top is the only way to end the support exactly there, and the layer below keeps its
    // own bottom, so the stack stays tiled.
    bool lower_top(LayerIndex layer_idx, double print_z_req)
    {
        if (layer_idx < 0 || layer_idx >= LayerIndex(entries.size()))
            return false;
        Entry &entry = entries[layer_idx];
        if (print_z_req >= entry.print_z - EPSILON)
            return true; // Already at or below the requested Z, nothing to do.
        if (print_z_req <= entry.bottom_z + MIN_SUPPORT_LAYER_HEIGHT)
            return false;
        entry.print_z      = print_z_req;
        entry.is_gap_layer = true;
        // Keep the grid tiling: the layer above now starts at the lowered top instead of at the old
        // one, otherwise the two neighbouring layers would leave a vertical hole of the difference
        // (contract §4.1 rule 4 - heights are recomputed after a top changes).
        if (layer_idx + 1 < LayerIndex(entries.size())) {
            Entry &above   = entries[layer_idx + 1];
            above.bottom_z = entry.print_z;
            above.height   = above.print_z - above.bottom_z;
        }
        entry.height       = entry.print_z - entry.bottom_z;
        return true;
    }

    // Layer that carries the top contact of an overhang whose lower face is at `overhang_bottom_z`.
    // The contact layer is lowered so its top ends up exactly `top_z_distance` below the overhang; when
    // that would make it thinner than a printable layer the contact stays one layer lower instead, which
    // only widens the gap and can therefore never fuse the support to the object.
    //
    // Layer 0 is the build plate face and has no support below it, and a raft layer has no object above
    // it to gain distance from, so the contact never lands in either of them. Returns -1 when no layer
    // is left to carry it.
    LayerIndex place_top_contact(const double overhang_bottom_z, const double top_z_distance)
    {
        const LayerIndex lowest = std::max<LayerIndex>(1, LayerIndex(raft_layers));
        if (entries.size() < size_t(lowest) + 1)
            return -1;
        const double     z_gap     = overhang_bottom_z - top_z_distance;
        const LayerIndex layer_idx = std::clamp(idx_ceil(z_gap), lowest, LayerIndex(entries.size()) - 1);
        if (lower_top(layer_idx, z_gap))
            return layer_idx;
        // One layer lower only widens the gap, but the raft layers are below the object: falling back
        // into them would draw the contact inside the raft instead of below the overhang.
        const LayerIndex lowered = layer_idx - 1;
        return lowered >= LayerIndex(raft_layers) ? lowered : -1;
    }
};

enum class InterfacePreference
{
    InterfaceAreaOverwritesSupport,
    SupportAreaOverwritesInterface,
    InterfaceLinesOverwriteSupport,
    SupportLinesOverwriteInterface,
    Nothing
};

struct TreeSupportMeshGroupSettings {
    TreeSupportMeshGroupSettings() = default;
    explicit TreeSupportMeshGroupSettings(const PrintObject &print_object)
    {
        const PrintConfig       &print_config       = print_object.print()->config();
        const PrintObjectConfig &config             = print_object.config();
        const SlicingParameters &slicing_params     = print_object.slicing_parameters();
    //    const std::vector<unsigned int>  printing_extruders = print_object.object_extruders();

        // Support must be enabled and set to Tree style.
        //assert(config.support_material);
        //assert(config.support_material_style == smsTree || config.support_material_style == smsOrganic);

        // Calculate maximum external perimeter width over all printing regions, taking into account the default layer height.
        coordf_t external_perimeter_width = 0.;
        for (size_t region_id = 0; region_id < print_object.num_printing_regions(); ++ region_id) {
            const PrintRegion &region = print_object.printing_region(region_id);
            external_perimeter_width = std::max<coordf_t>(external_perimeter_width, region.flow(print_object, frExternalPerimeter, config.layer_height).width());
        }

        this->layer_height              = scaled<coord_t>(config.layer_height.value);
        this->resolution                = scaled<coord_t>(print_config.resolution.value);
        // Arache feature
        this->min_feature_size          = scaled<coord_t>(config.min_feature_size.value);
        // +1 makes the threshold inclusive
        this->support_angle             = 0.5 * M_PI - std::clamp<double>((config.support_threshold_angle + 1) * M_PI / 180., 0., 0.5 * M_PI);
        this->support_line_width        = support_material_flow(&print_object, config.layer_height).scaled_width();
        this->support_roof_line_width   = support_material_interface_flow(&print_object, config.layer_height).scaled_width();
        const int bottom_interface_layers = number_of_support_interface_bottom_layers(config);
        this->support_bottom_enable     = bottom_interface_layers > 0;
        this->support_bottom_height     = this->support_bottom_enable ?
            bottom_interface_layers * this->layer_height :
            0;
        this->support_material_buildplate_only = config.support_on_build_plate_only;
        this->support_xy_distance       = scaled<coord_t>(config.support_object_xy_distance.value);
        this->support_xy_distance_1st_layer = scaled<coord_t>(config.support_object_first_layer_gap.value);
        // Separation of interfaces, it is likely smaller than support_xy_distance.
        this->support_xy_distance_overhang = std::min(this->support_xy_distance, scaled<coord_t>(0.5 * external_perimeter_width));
        // Quantised clearance, as SlicingParameters computed it for the layer synchronised generators.
        // The Cura derived engines (tree_slim / strong / hybrid) read it through
        // z_distance_top_layers below, so it must stay rounded to whole layers (review round 1, issue 2).
        this->support_top_distance      = scaled<coord_t>(slicing_params.gap_support_object);
        this->support_bottom_distance   = scaled<coord_t>(slicing_params.gap_object_support);
        // Raw millimetres for the organic generator, which places its contact layers exactly this far
        // below the overhang instead of a whole number of layers. Only the organic paths read these.
        this->support_top_z_distance_mm    = config.support_top_z_distance.value;
        this->support_bottom_z_distance_mm = config.support_bottom_z_distance.value;
        this->support_roof_enable       = config.support_interface_top_layers.value > 0;
        this->support_roof_layers       = config.support_interface_top_layers.value;
        this->support_floor_enable      = bottom_interface_layers > 0;
        this->support_floor_layers      = bottom_interface_layers;
        this->support_roof_pattern      = config.support_interface_pattern;
        this->support_pattern           = config.support_base_pattern;
        this->support_line_spacing      = scaled<coord_t>(config.support_base_pattern_spacing.value);
        this->support_wall_count        = std::max(1, config.tree_support_wall_count.value);  // at least 1 wall for organic tree support
        this->support_roof_line_distance = scaled<coord_t>(config.support_interface_spacing.value) + this->support_roof_line_width;
        this->support_tree_branch_distance = scaled<coord_t>(config.tree_support_branch_distance_organic.value);
        this->support_tree_angle          = std::clamp<double>(config.tree_support_branch_angle_organic * M_PI / 180., 0., 0.5 * M_PI - EPSILON);
        this->support_tree_angle_slow     = std::clamp<double>(config.tree_support_angle_slow * M_PI / 180., 0., this->support_tree_angle - EPSILON);
        this->support_tree_branch_diameter = scaled<coord_t>(config.tree_support_branch_diameter_organic.value);
        this->support_tree_branch_diameter_angle  = std::clamp<double>(config.tree_support_branch_diameter_angle * M_PI / 180., 0., 0.5 * M_PI - EPSILON);
        this->support_tree_top_rate       = config.tree_support_top_rate.value; // percent
    //    this->support_tree_tip_diameter = this->support_line_width;
        this->support_tree_tip_diameter = std::clamp(scaled<coord_t>(config.tree_support_tip_diameter.value), (coord_t)0, this->support_tree_branch_diameter);

        this->build_layer_grid(print_object);
    }

    // Fill layer_grid with the raft layers followed by the object layers as they were sliced, so that
    // every layer index keeps resolving to the object layer it is generated for. The belt Z offset is
    // removed because the organic generator works in the object's local frame.
    void build_layer_grid(const PrintObject &print_object)
    {
        const SlicingParameters &slicing_params = print_object.slicing_parameters();
        const double             z_offset       = print_object.belt_global_z_offset();

        layer_grid.entries.clear();
        layer_grid.entries.reserve(slicing_params.raft_layers() + print_object.layer_count());

        if (slicing_params.raft_layers() > 0) {
            // Same layer Zs as the raft generator produces: first layer, base, interface, contact and
            // the fill layers between the contact layer and the object.
            double z = slicing_params.first_print_layer_height;
            auto   add_raft_layer = [this](double print_z) {
                TreeSupportLayerGrid::Entry entry;
                entry.print_z  = print_z;
                entry.bottom_z = layer_grid.entries.empty() ? 0. : layer_grid.entries.back().print_z;
                entry.obj_bottom_z = entry.bottom_z;
                entry.height   = entry.print_z - entry.bottom_z;
                layer_grid.entries.emplace_back(entry);
            };
            add_raft_layer(z);
            for (size_t i = 1; i < slicing_params.base_raft_layers; ++ i)
                add_raft_layer(z += slicing_params.base_raft_layer_height);
            for (size_t i = 0; i + 1 < slicing_params.interface_raft_layers; ++ i)
                add_raft_layer(z += slicing_params.interface_raft_layer_height);
            if (slicing_params.raft_layers() > 1) {
                z = slicing_params.raft_contact_top_z;
                add_raft_layer(z);
            }
            if (double dist_to_go = slicing_params.object_print_z_min - z; dist_to_go > EPSILON) {
                // Layers between the raft contacts and bottom of the object.
                // ORCA: Bias by EPSILON so near-equal gaps do not get an extra split from FP noise.
                auto   nsteps = int(ceil((dist_to_go - EPSILON) / slicing_params.max_suport_layer_height));
                double step   = dist_to_go / nsteps;
                for (int i = 0; i < nsteps; ++ i)
                    add_raft_layer(z += step);
            }
        }
        layer_grid.raft_layers = layer_grid.entries.size();

        for (size_t i = 0; i < print_object.layer_count(); ++ i) {
            const Layer &layer = *print_object.get_layer(i);
            TreeSupportLayerGrid::Entry entry;
            entry.print_z       = layer.print_z - z_offset;
            entry.bottom_z      = entry.print_z - layer.height;
            entry.obj_bottom_z  = entry.bottom_z;
            entry.height        = layer.height;
            entry.obj_layer_idx = LayerIndex(i);
            layer_grid.entries.emplace_back(entry);
        }
    }

/*********************************************************************/
/* Print parameters, not support specific:                           */
/*********************************************************************/
    coord_t                         layer_height                            { scaled<coord_t>(0.15) };
    // Maximum Deviation (meshfix_maximum_deviation)
    // The maximum deviation allowed when reducing the resolution for the Maximum Resolution setting. If you increase this, 
    // the print will be less accurate, but the g-code will be smaller. Maximum Deviation is a limit for Maximum Resolution, 
    // so if the two conflict the Maximum Deviation will always be held true.
    coord_t                         resolution                              { scaled<coord_t>(0.025) };
    // Minimum Feature Size (aka minimum line width) - Arachne specific
    // Minimum thickness of thin features. Model features that are thinner than this value will not be printed, while features thicker 
    // than the Minimum Feature Size will be widened to the Minimum Wall Line Width.
    coord_t                         min_feature_size                        { scaled<coord_t>(0.1) };

/*********************************************************************/
/* General support parameters:                                       */
/*********************************************************************/

    // Support Overhang Angle
    // The minimum angle of overhangs for which support is added. At a value of 0° all overhangs are supported, 90° will not provide any support.
    double                          support_angle                           { 50. * M_PI / 180. };
    // Support Line Width
    // Width of a single support structure line.
    coord_t                         support_line_width                      { scaled<coord_t>(0.4) };
    // Support Roof Line Width: Width of a single support roof line.
    coord_t                         support_roof_line_width                 { scaled<coord_t>(0.4) };
    // Enable Support Floor (aka bottom interfaces)
    // Generate a dense slab of material between the bottom of the support and the model. This will create a skin between the model and support.
    bool                            support_bottom_enable                   { false };
    // Support Floor Thickness
    // The thickness of the support floors. This controls the number of dense layers that are printed on top of places of a model on which support rests.
    coord_t                         support_bottom_height                   { scaled<coord_t>(1.) };
    bool                            support_material_buildplate_only        { false };
    // Support X/Y Distance
    // Distance of the support structure from the print in the X/Y directions.
    // minimum: 0, maximum warning: 1.5 * machine_nozzle_tip_outer_diameter
    coord_t                         support_xy_distance                     { scaled<coord_t>(0.7) };
    coord_t                         support_xy_distance_1st_layer           { scaled<coord_t>(0.7) };
    // Minimum Support X/Y Distance
    // Distance of the support structure from the overhang in the X/Y directions.
    // minimum_value: 0,  minimum warning": support_xy_distance - support_line_width * 2, maximum warning: support_xy_distance
    coord_t                         support_xy_distance_overhang            { scaled<coord_t>(0.2) };
    // Support Top Distance
    // Distance from the top of the support to the print.
    coord_t                         support_top_distance                    { scaled<coord_t>(0.1) };
    // Support Bottom Distance
    // Distance from the print to the bottom of the support.
    coord_t                         support_bottom_distance                 { scaled<coord_t>(0.1) };
    // Raw (unquantised) clearances in millimetres. The organic generator places its contact layers
    // from these; the quantised members above stay for the layer synchronised engines (review round 1,
    // issue 2).
    double                          support_top_z_distance_mm               { 0.1 };
    double                          support_bottom_z_distance_mm            { 0.1 };
    //FIXME likely not needed, optimization for clipping of interface layers
    // When checking where there's model above and below the support, take steps of the given height. Lower values will slice slower, while higher values 
    // may cause normal support to be printed in some places where there should have been support interface.
    coord_t                         support_interface_skip_height           { scaled<coord_t>(0.3) };
    // Support Infill Line Directions
    // A list of integer line directions to use. Elements from the list are used sequentially as the layers progress and when the end 
    // of the list is reached, it starts at the beginning again. The list items are separated by commas and the whole list is contained 
    // in square brackets. Default is an empty list which means use the default angle 0 degrees.
//    std::vector<double>             support_infill_angles                   {};
    // Enable Support Roof
    // Generate a dense slab of material between the top of support and the model. This will create a skin between the model and support.
    bool                            support_roof_enable                     { false };
    // Support Roof Thickness
    // The thickness of the support roofs. This controls the amount of dense layers at the top of the support on which the model rests.
    coord_t                         support_roof_layers                     { 2 };
    bool                            support_floor_enable                    { false };
    coord_t                         support_floor_layers                    { 2 };
    // Minimum Support Roof Area
    // Minimum area size for the roofs of the support. Polygons which have an area smaller than this value will be printed as normal support.
    double                          minimum_roof_area                       { scaled<double>(scaled<double>(1.)) };
    // A list of integer line directions to use. Elements from the list are used sequentially as the layers progress 
    // and when the end of the list is reached, it starts at the beginning again. The list items are separated
    // by commas and the whole list is contained in square brackets. Default is an empty list which means
    // use the default angles (alternates between 45 and 135 degrees if interfaces are quite thick or 90 degrees).
    std::vector<double>             support_roof_angles                     {};
    // Support Roof Pattern (aka top interface)
    // The pattern with which the roofs of the support are printed.
    SupportMaterialInterfacePattern support_roof_pattern                    { smipAuto };
    // Support Pattern
    // The pattern of the support structures of the print. The different options available result in sturdy or easy to remove support.
    SupportMaterialPattern          support_pattern                         { smpRectilinear };
    // Support Line Distance
    // Distance between the printed support structure lines. This setting is calculated by the support density.
    coord_t                         support_line_spacing                    { scaled<coord_t>(2.66 - 0.4) };
    // Support Floor Horizontal Expansion
    // Amount of offset applied to the floors of the support.
    coord_t                         support_bottom_offset                   { scaled<coord_t>(0.) };
    // Support Wall Line Count
    // The number of walls with which to surround support infill. Adding a wall can make support print more reliably 
    // and can support overhangs better, but increases print time and material used.
    // tree: 1, zig-zag: 0, concentric: 1
    int                             support_wall_count                      { 1 };
    // Support Roof Line Distance
    // Distance between the printed support roof lines. This setting is calculated by the Support Roof Density, but can be adjusted separately.
    coord_t                         support_roof_line_distance              { scaled<coord_t>(0.4) };
    // Minimum Support Area
    // Minimum area size for support polygons. Polygons which have an area smaller than this value will not be generated.
    coord_t                         minimum_support_area                    { scaled<coord_t>(0.) };
    // Minimum Support Floor Area
    // Minimum area size for the floors of the support. Polygons which have an area smaller than this value will be printed as normal support.
    coord_t                         minimum_bottom_area                     { scaled<coord_t>(1.0) };
    // Support Horizontal Expansion
    // Amount of offset applied to all support polygons in each layer. Positive values can smooth out the support areas and result in more sturdy support.
    coord_t                         support_offset                          { scaled<coord_t>(0.) };

/*********************************************************************/
/* Parameters for the Cura tree supports implementation:             */
/*********************************************************************/

    // Tree Support Maximum Branch Angle
    // The maximum angle of the branches, when the branches have to avoid the model. Use a lower angle to make them more vertical and more stable. Use a higher angle to be able to have more reach.
    // minimum: 0, minimum warning: 20, maximum: 89, maximum warning": 85
    double                          support_tree_angle                      { 60. * M_PI / 180. };
    // Tree Support Branch Diameter Angle
    // The angle of the branches' diameter as they gradually become thicker towards the bottom. An angle of 0 will cause the branches to have uniform thickness over their length. 
    // A bit of an angle can increase stability of the tree support.
    // minimum: 0, maximum: 89.9999, maximum warning: 15
    double                          support_tree_branch_diameter_angle      { 5.  * M_PI / 180. };
    // Tree Support Branch Distance
    // How far apart the branches need to be when they touch the model. Making this distance small will cause 
    // the tree support to touch the model at more points, causing better overhang but making support harder to remove.
    coord_t                         support_tree_branch_distance            { scaled<coord_t>(1.) };
    // Tree Support Branch Diameter
    // The diameter of the thinnest branches of tree support. Thicker branches are more sturdy. Branches towards the base will be thicker than this.
    // minimum: 0.001, minimum warning: support_line_width * 2
    coord_t                         support_tree_branch_diameter            { scaled<coord_t>(2.) };

/*********************************************************************/
/* Parameters new to the Thomas Rahm's tree supports implementation: */
/*********************************************************************/

    // Tree Support Preferred Branch Angle
    // The preferred angle of the branches, when they do not have to avoid the model. Use a lower angle to make them more vertical and more stable. Use a higher angle for branches to merge faster.
    // minimum: 0, minimum warning: 10, maximum: support_tree_angle, maximum warning: support_tree_angle-1
    double                          support_tree_angle_slow                 { 50. * M_PI / 180. };
    // Tree Support Diameter Increase To Model
    // The most the diameter of a branch that has to connect to the model may increase by merging with branches that could reach the buildplate.
    // Increasing this reduces print time, but increases the area of support that rests on model
    // minimum: 0
    coord_t                         support_tree_max_diameter_increase_by_merges_when_support_to_model { scaled<coord_t>(1.0) };
    // Tree Support Minimum Height To Model
    // How tall a branch has to be if it is placed on the model. Prevents small blobs of support. This setting is ignored when a branch is supporting a support roof.
    // minimum: 0, maximum warning: 5
    coord_t                         support_tree_min_height_to_model        { scaled<coord_t>(1.0) };
    // Tree Support Inital Layer Diameter
    // Diameter every branch tries to achieve when reaching the buildplate. Improves bed adhesion.
    // minimum: 0, maximum warning: 20
    coord_t                         support_tree_bp_diameter                { scaled<coord_t>(7.5) };
    // Tree Support Branch Density
    // Adjusts the density of the support structure used to generate the tips of the branches. A higher value results in better overhangs,
    // but the supports are harder to remove. Use Support Roof for very high values or ensure support density is similarly high at the top.
    // ->
    // Adjusts the density of the support structure used to generate the tips of the branches.
    // A higher value results in better overhangs but the supports are harder to remove, thus it is recommended to enable top support interfaces
    // instead of a high branch density value if dense interfaces are needed.
    // 5%-35%
    double                          support_tree_top_rate                   { 15. };
    // Tree Support Tip Diameter
    // The diameter of the top of the tip of the branches of tree support.
    // minimum: min_wall_line_width, minimum warning: min_wall_line_width+0.05, maximum_value: support_tree_branch_diameter, value: support_line_width
    coord_t                         support_tree_tip_diameter               { scaled<coord_t>(0.4) };

    // Z of every support layer (raft layers, then the object layers). See TreeSupportLayerGrid.
    TreeSupportLayerGrid            layer_grid;

    // Support Interface Priority
    // How support interface and support will interact when they overlap. Currently only implemented for support roof.
    //enum                           support_interface_priority { support_lines_overwrite_interface_area };
};

/*!
 * \brief This struct contains settings used in the tree support. Thanks to this most functions do not need to know of meshes etc. Also makes the code shorter.
 */
struct TreeSupportSettings
{
public:
    TreeSupportSettings() = default; // required for the definition of the config variable in the TreeSupportGenerator class.
    explicit TreeSupportSettings(const TreeSupportMeshGroupSettings &mesh_group_settings, const SlicingParameters &slicing_params)
        : support_line_width(mesh_group_settings.support_line_width),
          layer_height(mesh_group_settings.layer_height),
          support_tree_angle(mesh_group_settings.support_tree_angle),
          support_tree_angle_slow(mesh_group_settings.support_tree_angle_slow),
          branch_diameter_angle(mesh_group_settings.support_tree_branch_diameter_angle),
          branch_radius(mesh_group_settings.support_tree_branch_diameter / 2),
          min_radius(mesh_group_settings.support_tree_tip_diameter / 2), // The actual radius is 50 microns larger as the resulting branches will be increased by 50 microns to avoid rounding errors effectively increasing the xydistance
          maximum_move_distance((mesh_group_settings.support_tree_angle < M_PI / 2.) ? (coord_t)(tan(mesh_group_settings.support_tree_angle) * layer_height) : std::numeric_limits<coord_t>::max()),
          maximum_move_distance_slow((mesh_group_settings.support_tree_angle_slow < M_PI / 2.) ? (coord_t)(tan(mesh_group_settings.support_tree_angle_slow) * layer_height) : std::numeric_limits<coord_t>::max()),
          support_bottom_layers(mesh_group_settings.support_bottom_enable ? (mesh_group_settings.support_bottom_height + layer_height / 2) / layer_height : 0),
          // Ensure lines always stack nicely even if layer height is large.
          tip_layers(std::max((branch_radius - min_radius) / (support_line_width / 3), branch_radius / layer_height)),
          branch_radius_increase_per_layer(tan(mesh_group_settings.support_tree_branch_diameter_angle) * layer_height),
          max_to_model_radius_increase(mesh_group_settings.support_tree_max_diameter_increase_by_merges_when_support_to_model / 2),
          min_dtt_to_model(round_up_divide(mesh_group_settings.support_tree_min_height_to_model, layer_height)),
          increase_radius_until_radius(mesh_group_settings.support_tree_branch_diameter / 2),
          increase_radius_until_layer(increase_radius_until_radius <= branch_radius ? tip_layers * (increase_radius_until_radius / branch_radius) : (increase_radius_until_radius - branch_radius) / branch_radius_increase_per_layer),
          support_rests_on_model(! mesh_group_settings.support_material_buildplate_only),
          xy_distance(mesh_group_settings.support_xy_distance),
          xy_min_distance(std::min(mesh_group_settings.support_xy_distance, mesh_group_settings.support_xy_distance_overhang)),
          bp_radius(mesh_group_settings.support_tree_bp_diameter / 2),
          // Increase by half a line overlap, but not faster than 40 degrees angle (0 degrees means zero increase in radius).
          bp_radius_increase_per_layer(std::min(tan(0.7) * layer_height, 0.5 * support_line_width)),
          z_distance_bottom_layers(size_t(round(double(mesh_group_settings.support_bottom_distance) / double(layer_height)))),
          z_distance_top_layers(size_t(round(double(mesh_group_settings.support_top_distance) / double(layer_height)))),
    //              support_infill_angles(mesh_group_settings.support_infill_angles),
          support_roof_angles(mesh_group_settings.support_roof_angles),
          roof_pattern(mesh_group_settings.support_roof_pattern),
          support_pattern(mesh_group_settings.support_pattern),
          support_roof_line_width(mesh_group_settings.support_roof_line_width),
          support_line_spacing(mesh_group_settings.support_line_spacing),
          support_bottom_offset(mesh_group_settings.support_bottom_offset),
          support_wall_count(mesh_group_settings.support_wall_count),
          resolution(mesh_group_settings.resolution),
          support_roof_line_distance(mesh_group_settings.support_roof_line_distance), // in the end the actual infill has to be calculated to subtract interface from support areas according to interface_preference.
          settings(mesh_group_settings),
          min_feature_size(mesh_group_settings.min_feature_size)
    {
        // At least one tip layer must be defined.
        assert(tip_layers > 0);
    
        layer_start_bp_radius = (bp_radius - branch_radius) / bp_radius_increase_per_layer;

        // The layer Zs of this group. Copied so that a contact layer may be lowered to an exact Z gap
        // without touching the settings the volumes are built from.
        layer_grid = mesh_group_settings.layer_grid;
        raft_layers.clear();
        for (size_t i = 0; i < layer_grid.raft_layers; ++ i)
            raft_layers.emplace_back(layer_grid.entries[i].print_z);
        has_top_gap    = mesh_group_settings.support_top_z_distance_mm > EPSILON;
        has_bottom_gap = mesh_group_settings.support_bottom_z_distance_mm > EPSILON;

        // Derive the layer counts from the grid, so a variable layer height does not silently change how
        // many layers a millimetre covers (contract §4.1). On a uniform grid every count resolves to the
        // value the initialisation list derived from the nominal layer height.
        if (! layer_grid.empty()) {
            support_bottom_layers = mesh_group_settings.support_bottom_enable ?
                layer_grid.layers_rounding(unscale<double>(mesh_group_settings.support_bottom_height)) : 0;
            tip_layers = std::max<size_t>(size_t(1),
                std::max<size_t>(size_t((branch_radius - min_radius) / std::max<coord_t>(support_line_width / 3, 1)),
                                 layer_grid.layers_fitting(unscale<double>(branch_radius))));
            min_dtt_to_model = layer_grid.layers_covering(unscale<double>(mesh_group_settings.support_tree_min_height_to_model));
            increase_radius_until_layer = increase_radius_until_radius <= branch_radius ?
                size_t(tip_layers * (increase_radius_until_radius / branch_radius)) :
                layers_for_branch_radius_growth(double(increase_radius_until_radius - branch_radius));
            layer_start_bp_radius = LayerIndex(layers_for_bp_radius_growth(double(bp_radius - branch_radius)));
        }

        if (TreeSupportSettings::zero_top_z_gap) {
            // safeOffsetInc can only work in steps of the size xy_min_distance in the worst case => xy_min_distance has to be a bit larger than 0 in this worst case and should be large enough for performance to not suffer extremely
            // When for all meshes the z bottom and top distance is more than one layer though the worst case is xy_min_distance + min_feature_size
            // This is not the best solution, but the only one to ensure areas can not lag though walls at high maximum_move_distance.
            xy_min_distance = std::max(xy_min_distance, scaled<coord_t>(0.1));
            xy_distance     = std::max(xy_distance, xy_min_distance);
        }
    
    //            const std::unordered_map<std::string, InterfacePreference> interface_map = { { "support_area_overwrite_interface_area", InterfacePreference::SupportAreaOverwritesInterface }, { "interface_area_overwrite_support_area", InterfacePreference::InterfaceAreaOverwritesSupport }, { "support_lines_overwrite_interface_area", InterfacePreference::SupportLinesOverwriteInterface }, { "interface_lines_overwrite_support_area", InterfacePreference::InterfaceLinesOverwriteSupport }, { "nothing", InterfacePreference::Nothing } };
    //            interface_preference = interface_map.at(mesh_group_settings.get<std::string>("support_interface_priority"));
    //FIXME this was the default
    //            interface_preference = InterfacePreference::SupportLinesOverwriteInterface;
        //interface_preference = InterfacePreference::SupportAreaOverwritesInterface;
        interface_preference = InterfacePreference::InterfaceAreaOverwritesSupport;
    }

    // some static variables dependent on other meshes that are not currently processed.
    // Has to be static because TreeSupportConfig will be used in TreeModelVolumes as this reduces redundancy.
    inline static bool zero_top_z_gap = false;
    /*!
     * \brief Width of a single line of support.
     */
    coord_t support_line_width;
    /*!
     * \brief Height of a single layer
     */
    coord_t layer_height;
    /*!
     * \brief Angles the per-layer growth limits are derived from. The members below hold those limits
     * for the nominal layer height; the *_at() accessors apply the same angle to the actual thickness
     * of one layer of the grid (contract §4.1/§4.4).
     */
    double support_tree_angle;
    double support_tree_angle_slow;
    double branch_diameter_angle;
    /*!
     * \brief Radius of a branch when it has left the tip.
     */
    coord_t branch_radius;
    /*!
     * \brief smallest allowed radius, required to ensure that even at DTT 0 every circle will still be printed
     */
    coord_t min_radius;
    /*!
     * \brief How far an influence area may move outward every layer at most.
     */
    coord_t maximum_move_distance;
    /*!
     * \brief How far every influence area will move outward every layer if possible.
     */
    coord_t maximum_move_distance_slow;
    /*!
     * \brief Amount of bottom layers. 0 if disabled.
     */
    size_t support_bottom_layers;
    /*!
     * \brief Amount of effectiveDTT increases are required to reach branch radius.
     */
    size_t tip_layers;
    /*!
     * \brief How much a branch radius increases with each layer to guarantee the prescribed tree widening.
     */
    double branch_radius_increase_per_layer;
    /*!
     * \brief How much a branch resting on the model may grow in radius by merging with branches that can reach the buildplate.
     */
    coord_t max_to_model_radius_increase;
    /*!
     * \brief If smaller (in layers) than that, all branches to model will be deleted
     */
    size_t min_dtt_to_model;
    /*!
     * \brief Increase radius in the resulting drawn branches, even if the avoidance does not allow it. Will be cut later to still fit.
     */
    coord_t increase_radius_until_radius;
    /*!
     * \brief Same as increase_radius_until_radius, but contains the DTT at which the radius will be reached.
     */
    size_t increase_radius_until_layer;
    /*!
     * \brief True if the branches may connect to the model.
     */
    bool support_rests_on_model;
    /*!
     * \brief How far should support be from the model.
     */
    coord_t xy_distance;
    /*!
     * \brief A minimum radius a tree trunk should expand to at the buildplate if possible.
     */
    coord_t bp_radius;
    /*!
     * \brief The layer index at which an increase in radius may be required to reach the bp_radius.
     */
    LayerIndex layer_start_bp_radius;
    /*!
     * \brief How much one is allowed to increase the tree branch radius close to print bed to reach the required bp_radius at layer 0.
     * Note that this radius increase will not happen in the tip, to ensure the tip is structurally sound.
     */
    double bp_radius_increase_per_layer;
    /*!
     * \brief minimum xy_distance. Only relevant when Z overrides XY, otherwise equal to xy_distance-
     */
    coord_t xy_min_distance;
    /*!
     * \brief Nominal amount of layers between the top of the support and the model.
     *
     * round(distance / layer height). Kept for the heuristics that only need a layer count; the
     * geometric clearance is derived from layer_grid, which knows the real Z of every layer and
     * therefore also holds for a varying layer height.
     */
    size_t z_distance_top_layers;
    /*!
     * \brief Nominal amount of layers between the top of the model and the bottom of a support structure.
     */
    size_t z_distance_bottom_layers;
    /*!
     * \brief Z of every support layer, plus the object layer it was generated for.
     */
    TreeSupportLayerGrid layer_grid;
    /*!
     * \brief True when a non-zero vertical gap between the object and the support has to be kept.
     */
    bool has_top_gap    { false };
    bool has_bottom_gap { false };
    /*!
     * \brief User specified angles for the support infill.
     */
//        std::vector<double> support_infill_angles;
    /*!
     * \brief User specified angles for the support roof infill.
     */
    std::vector<double> support_roof_angles;
    /*!
     * \brief Pattern used in the support roof. May contain non relevant data if support roof is disabled.
     */
    SupportMaterialInterfacePattern roof_pattern;
    /*!
     * \brief Pattern used in the support infill.
     */
    SupportMaterialPattern support_pattern;
    /*!
     * \brief Line width of the support roof.
     */
    coord_t support_roof_line_width;
    /*!
     * \brief Distance between support infill lines.
     */
    coord_t support_line_spacing;
    /*!
     * \brief Offset applied to the support floor area.
     */
    coord_t support_bottom_offset;
    /*
     * \brief Amount of walls the support area will have.
     */
    int support_wall_count;
    /*
     * \brief Maximum allowed deviation when simplifying.
     */
    coord_t resolution;
    /*
     * \brief Distance between the lines of the roof.
     */
    coord_t support_roof_line_distance;
    /*
     * \brief How overlaps of an interface area with a support area should be handled.
     */
    InterfacePreference interface_preference;

    /*
     * \brief The infill class wants a settings object. This one will be the correct one for all settings it uses.
     */
    TreeSupportMeshGroupSettings settings;

    /*
     * \brief Minimum thickness of any model features.
     */
    coord_t min_feature_size;

    // Extra raft layers below the object.
    std::vector<coordf_t> raft_layers;

public:
    bool operator==(const TreeSupportSettings& other) const
    {
        return branch_radius == other.branch_radius && tip_layers == other.tip_layers && branch_radius_increase_per_layer == other.branch_radius_increase_per_layer && layer_start_bp_radius == other.layer_start_bp_radius && bp_radius == other.bp_radius && 
               // as a recalculation of the collision areas is required to set a new min_radius.
               bp_radius_increase_per_layer == other.bp_radius_increase_per_layer && min_radius == other.min_radius && xy_min_distance == other.xy_min_distance &&
               xy_distance - xy_min_distance == other.xy_distance - other.xy_min_distance && // if the delta of xy_min_distance and xy_distance is different the collision areas have to be recalculated.
               support_rests_on_model == other.support_rests_on_model && increase_radius_until_layer == other.increase_radius_until_layer && min_dtt_to_model == other.min_dtt_to_model && max_to_model_radius_increase == other.max_to_model_radius_increase && maximum_move_distance == other.maximum_move_distance && maximum_move_distance_slow == other.maximum_move_distance_slow && z_distance_bottom_layers == other.z_distance_bottom_layers && support_line_width == other.support_line_width && 
               support_line_spacing == other.support_line_spacing && support_roof_line_width == other.support_roof_line_width && // can not be set on a per-mesh basis currently, so code to enable processing different roof line width in the same iteration seems useless.
               support_bottom_offset == other.support_bottom_offset && support_wall_count == other.support_wall_count && support_pattern == other.support_pattern && roof_pattern == other.roof_pattern && // can not be set on a per-mesh basis currently, so code to enable processing different roof patterns in the same iteration seems useless.
               support_roof_angles == other.support_roof_angles && 
               //support_infill_angles == other.support_infill_angles && 
               increase_radius_until_radius == other.increase_radius_until_radius && support_bottom_layers == other.support_bottom_layers && layer_height == other.layer_height && z_distance_top_layers == other.z_distance_top_layers && resolution == other.resolution && // Infill generation depends on deviation and resolution.
               support_roof_line_distance == other.support_roof_line_distance && interface_preference == other.interface_preference
               && min_feature_size == other.min_feature_size // interface_preference should be identical to ensure the tree will correctly interact with the roof.
               // The infill class now wants the settings object and reads a lot of settings, and as the infill class is used to calculate support roof lines for interface-preference. Not all of these may be required to be identical, but as I am not sure, better safe than sorry
#if 0
                && (interface_preference == InterfacePreference::InterfaceAreaOverwritesSupport || interface_preference == InterfacePreference::SupportAreaOverwritesInterface
                // Perimeter generator parameters
                   || 
                        (settings.get<bool>("fill_outline_gaps") == other.settings.get<bool>("fill_outline_gaps") && 
                         settings.get<coord_t>("min_bead_width") == other.settings.get<coord_t>("min_bead_width") && 
                         settings.get<double>("wall_transition_angle") == other.settings.get<double>("wall_transition_angle") && 
                         settings.get<coord_t>("wall_transition_length") == other.settings.get<coord_t>("wall_transition_length") && 
                         settings.get<Ratio>("wall_split_middle_threshold") == other.settings.get<Ratio>("wall_split_middle_threshold") && 
                         settings.get<Ratio>("wall_add_middle_threshold") == other.settings.get<Ratio>("wall_add_middle_threshold") && 
                         settings.get<int>("wall_distribution_count") == other.settings.get<int>("wall_distribution_count") && 
                         settings.get<coord_t>("wall_transition_filter_distance") == other.settings.get<coord_t>("wall_transition_filter_distance") && 
                         settings.get<coord_t>("wall_transition_filter_deviation") == other.settings.get<coord_t>("wall_transition_filter_deviation") && 
                         settings.get<coord_t>("wall_line_width_x") == other.settings.get<coord_t>("wall_line_width_x") && 
                         settings.get<int>("meshfix_maximum_extrusion_area_deviation") == other.settings.get<int>("meshfix_maximum_extrusion_area_deviation"))
                    )
#endif
               && raft_layers == other.raft_layers
            ;
    }

    // ---- Per-layer growth (contract §4.1/§4.4) ------------------------------------------------
    // The branch moves and widens once per layer. With a variable layer height that step has to follow
    // the layer's own thickness, not the nominal layer height the members above are derived from. On a
    // uniform grid every accessor returns exactly the member it replaces, so existing output is
    // unchanged.

    [[nodiscard]] static coord_t move_distance_for_height(const double height, const double angle)
    {
        return angle < M_PI / 2. ? coord_t(std::tan(angle) * double(TreeSupportLayerGrid::scaled_rounded(height))) : std::numeric_limits<coord_t>::max();
    }

    // How far an influence area may move outward from `layer_idx` down to the layer below it.
    // A layer as thick as the nominal layer height returns the legacy scalar verbatim, so a uniform
    // layer grid stays bit-identical to the scalar implementation this replaced (contract §5.1).
    [[nodiscard]] bool nominal_thickness(const LayerIndex layer_idx) const
    { return TreeSupportLayerGrid::scaled_rounded(layer_grid.h(layer_idx)) == layer_height; }

    [[nodiscard]] coord_t maximum_move_distance_at(const LayerIndex layer_idx) const
    {
        return nominal_thickness(layer_idx) ? maximum_move_distance :
            move_distance_for_height(layer_grid.h(layer_idx), support_tree_angle);
    }

    [[nodiscard]] coord_t maximum_move_distance_slow_at(const LayerIndex layer_idx) const
    {
        return nominal_thickness(layer_idx) ? maximum_move_distance_slow :
            move_distance_for_height(layer_grid.h(layer_idx), support_tree_angle_slow);
    }

    // Radius a branch gains while `layer_idx` is printed. Scaled units, like
    // `branch_radius_increase_per_layer`, which it replaces layer by layer.
    [[nodiscard]] double branch_radius_increase_at(const LayerIndex layer_idx) const
    {
        return nominal_thickness(layer_idx) ? branch_radius_increase_per_layer :
            std::tan(branch_diameter_angle) * double(TreeSupportLayerGrid::scaled_rounded(layer_grid.h(layer_idx)));
    }

    // Radius an elephant foot gains while `layer_idx` is printed. Scaled units.
    [[nodiscard]] double bp_radius_increase_at(const LayerIndex layer_idx) const
    {
        return nominal_thickness(layer_idx) ? bp_radius_increase_per_layer :
            std::min(std::tan(0.7) * double(TreeSupportLayerGrid::scaled_rounded(layer_grid.h(layer_idx))), 0.5 * double(support_line_width));
    }

    // Layers until the branch radius grew by `radius_delta`, accumulated over the actual layer heights.
    [[nodiscard]] size_t layers_for_branch_radius_growth(const double radius_delta) const
    {
        double accumulated = 0.;
        size_t layers      = 0;
        for (size_t idx = 0; idx < layer_grid.size(); ++ idx) {
            const double growth = branch_radius_increase_at(LayerIndex(idx));
            if (growth <= EPSILON || accumulated + growth > radius_delta + EPSILON)
                break;
            accumulated += growth;
            ++ layers;
        }
        return layers;
    }

    // Layers until an elephant foot grew by `radius_delta`, accumulated over the actual layer heights.
    [[nodiscard]] size_t layers_for_bp_radius_growth(const double radius_delta) const
    {
        double accumulated = 0.;
        size_t layers      = 0;
        for (size_t idx = 0; idx < layer_grid.size(); ++ idx) {
            const double growth = bp_radius_increase_at(LayerIndex(idx));
            if (growth <= EPSILON || accumulated + growth > radius_delta + EPSILON)
                break;
            accumulated += growth;
            ++ layers;
        }
        return layers;
    }

    /*!
     * \brief Get the Radius part will have based on numeric values.
     * \param distance_to_top[in] The effective distance_to_top of the element
     * \param elephant_foot_increases[in] The elephant_foot_increases of the element.
     * \return The radius an element with these attributes would have.
     */
    [[nodiscard]] inline coord_t getRadius(size_t distance_to_top, const double elephant_foot_increases = 0) const
    {
        return (distance_to_top <= tip_layers ? min_radius + (branch_radius - min_radius) * distance_to_top / tip_layers : // tip
                       branch_radius + // base
                       (distance_to_top - tip_layers) * branch_radius_increase_per_layer)
               + // gradual increase
               elephant_foot_increases * (std::max(bp_radius_increase_per_layer - branch_radius_increase_per_layer, 0.0));
    }

    /*!
     * \brief Same radius, but integrated over the actual layer heights between the branch tip and
     * the element.
     *
     * `grown_layers` counts layers below the tip (the tip is the element with distance_to_top == 0,
     * which sits `distance_to_top` layers above the element it describes), while a branch grows over
     * a Z distance. On a uniform grid the two expressions agree exactly; with a variable layer height
     * the growth has to be accumulated over the grid heights instead of multiplied by the nominal
     * layer height (contract §4.1/§4.4).
     */
    [[nodiscard]] inline coord_t getRadius_at(const LayerIndex tip_layer, const size_t grown_layers,
                                              const double elephant_foot_increases = 0) const
    {
        if (layer_grid.empty() || tip_layer < 0 || tip_layer >= LayerIndex(layer_grid.size()))
            return getRadius(grown_layers, elephant_foot_increases);

        // The element sits `grown_layers` layers *below* the tip, so the Z it grew over is the height
        // between that layer and the tip, not the height of the layers above the tip (which is a
        // different range of the grid, and on a variable layer height a different number of microns).
        const LayerIndex grown_layer = tip_layer - LayerIndex(grown_layers);
        const coord_t    z_above     = TreeSupportLayerGrid::scaled_rounded(layer_grid.z_span(grown_layer, tip_layer));
        const coord_t    z_tip       = TreeSupportLayerGrid::scaled_rounded(layer_grid.z_span(tip_layer - LayerIndex(tip_layers), tip_layer));
        // Both spans being whole layers means the grid carries no thickness information the scalar
        // implementation lacks, so take its value verbatim: the expressions are mathematically equal
        // there, but only the scalar one is bit-for-bit reproducible (products are evaluated in a
        // different order), and that is what keeps a uniform grid byte-identical (contract §5.1).
        if (z_above == coord_t(grown_layers) * layer_height && z_tip == coord_t(tip_layers) * layer_height)
            return getRadius(grown_layers, elephant_foot_increases);
        const coord_t    radius      = grown_layers <= tip_layers ?
            min_radius + (branch_radius - min_radius) * z_above / std::max<coord_t>(z_tip, 1) :
            branch_radius + coord_t(std::tan(branch_diameter_angle) * double(z_above - z_tip));
        return radius + coord_t(elephant_foot_increases *
                                std::max(bp_radius_increase_at(grown_layer) - branch_radius_increase_at(grown_layer), 0.0));
    }

    /*!
     * \brief Get the Radius an element should at least have at a given layer.
     * \param layer_idx[in] The layer.
     * \return The radius every element should aim to achieve.
     */
    [[nodiscard]] inline coord_t recommendedMinRadius(LayerIndex layer_idx) const
    {
        const double num_layers_widened = double(layer_start_bp_radius - layer_idx);
        if (num_layers_widened <= 0.)
            return 0;
        // The historical formula multiplies the number of layers by the nominal per-layer growth of the
        // elephant foot. The foot itself grows by `bp_radius_increase_at()`, i.e. by the real thickness
        // of each layer, so on a variable layer height the recommendation has to accumulate the same
        // increments - multiplying them by the count would ask for a radius the branches below never
        // reach, and the branch spacing derived from it would be wrong (review round 4, issue 1).
        // A range of nominal layers carries no thickness information beyond the scalar, so it takes the
        // scalar expression verbatim and a uniform grid stays bit-identical (contract §5.1).
        bool all_nominal = ! layer_grid.empty();
        for (LayerIndex idx = layer_idx + 1; all_nominal && idx <= layer_start_bp_radius; ++ idx)
            all_nominal = nominal_thickness(idx);
        if (! all_nominal) {
            double radius = branch_radius;
            for (LayerIndex idx = layer_idx + 1; idx <= layer_start_bp_radius; ++ idx)
                radius += bp_radius_increase_at(idx);
            return coord_t(radius);
        }
        return branch_radius + num_layers_widened * bp_radius_increase_per_layer;
    }

#if 0
    /*!
     * \brief Return on which z in microns the layer will be printed. Used only for support infill line generation.
     * \param layer_idx[in] The layer.
     * \return The radius every element should aim to achieve.
     */
    [[nodiscard]] inline coord_t getActualZ(LayerIndex layer_idx)
    {
        return layer_idx < coord_t(known_z.size()) ? known_z[layer_idx] : (layer_idx - known_z.size()) * layer_height + known_z.size() ? known_z.back() : 0;
    }

    /*!
     * \brief Set the z every Layer is printed at. Required for getActualZ to work
     * \param z[in] The z every LayerIndex is printed. Vector is used as a map<LayerIndex,coord_t> with the index of each element being the corresponding LayerIndex
     * \return The radius every element should aim to achieve.
     */
    void setActualZ(std::vector<coord_t>& z)
    {
        known_z = z;
    }
#endif

private:
//    std::vector<coord_t> known_z;
};

inline double tiny_area_threshold() { return sqr(scaled<double>(0.001)); }

inline void tree_supports_show_error(std::string_view message, bool critical)
{ // todo Remove!  ONLY FOR PUBLIC BETA!!
    printf("Error: %s, critical: %d\n", message.data(), int(critical));
#ifdef TREE_SUPPORT_SHOW_ERRORS_WIN32
    static bool g_showed_critical_error = false;
    static bool g_showed_performance_warning = false;
    auto bugtype = std::string(critical ? " This is a critical bug. It may cause missing or malformed branches.\n" : "This bug should only decrease performance.\n");
    bool show    = (critical && !g_showed_critical_error) || (!critical && !g_showed_performance_warning);
    (critical ? g_showed_critical_error : g_showed_performance_warning) = true;
    if (show)
        MessageBoxA(nullptr, std::string("TreeSupport_2 MOD detected an error while generating the tree support.\nPlease report this back to me with profile and model.\nRevision 5.0\n" + std::string(message) + "\n" + bugtype).c_str(), 
            "Bug detected!", MB_OK | MB_SYSTEMMODAL | MB_SETFOREGROUND | MB_ICONWARNING);
#endif // TREE_SUPPORT_SHOW_ERRORS_WIN32
}

// Print Z of a support layer. Comes from the layer grid, which holds the real object layer Zs, so
// a varying layer height and a contact layer lowered to an exact Z gap both resolve correctly.
inline double layer_z(const TreeSupportSettings &config, const size_t layer_idx)
{
    return config.layer_grid.z(LayerIndex(layer_idx));
}

// Overload for the caller that still passes the slicing parameters (the tree skeleton preview in
// TreeSupport.cpp, which is outside this change's scope). The grid carries the same Zs, so the
// slicing parameters only serve as a fallback for a settings object whose grid was never built.
inline double layer_z(const SlicingParameters &slicing_params, const TreeSupportSettings &config, const size_t layer_idx)
{
    if (! config.layer_grid.empty())
        return layer_z(config, layer_idx);
    return layer_idx >= config.raft_layers.size() ?
        slicing_params.object_print_z_min + slicing_params.first_object_layer_height + (layer_idx - config.raft_layers.size()) * slicing_params.layer_height :
        config.raft_layers[layer_idx];
}

// Lowest collision layer: the lowest layer whose top reaches z.
inline LayerIndex layer_idx_ceil(const TreeSupportSettings &config, const double z)
{
    return config.layer_grid.idx_ceil(z);
}

// Highest collision layer: the highest layer whose top does not exceed z.
inline LayerIndex layer_idx_floor(const TreeSupportSettings &config, const double z)
{
    return config.layer_grid.idx_floor(z);
}

inline SupportGeneratorLayer& layer_initialize(
    SupportGeneratorLayer     &layer_new,
    const TreeSupportSettings &config, 
    const size_t               layer_idx)
{
    layer_new.print_z  = layer_z(config, layer_idx);
    // The grid carries the bottom of every layer: z = 0 for the first layer on a flat bed, and the
    // (possibly negative) local Z below the belt for a belt printer, whose virtual support layers
    // continue below zero. Taking 0 there made the bottom-most belt layer's height come out as its
    // own print_z, which reached Flow::with_height() and threw FlowErrorNegativeFlow, so tree support
    // could not slice any belt model whose branches reached down that far.
    layer_new.bottom_z = config.layer_grid.bottom_z(LayerIndex(layer_idx));
    layer_new.height   = layer_new.print_z - layer_new.bottom_z;
    return layer_new;
}

// Using the std::deque as an allocator.
inline SupportGeneratorLayer& layer_allocate_unguarded(
    SupportGeneratorLayerStorage      &layer_storage,
    SupporLayerType                    layer_type,
    const TreeSupportSettings         &config, 
    size_t                             layer_idx)
{
    SupportGeneratorLayer &layer = layer_storage.allocate_unguarded(layer_type);
    return layer_initialize(layer, config, layer_idx);
}

inline SupportGeneratorLayer& layer_allocate(
    SupportGeneratorLayerStorage      &layer_storage,
    SupporLayerType                    layer_type,
    const TreeSupportSettings         &config, 
    size_t                             layer_idx)
{
    SupportGeneratorLayer &layer = layer_storage.allocate(layer_type);
    return layer_initialize(layer, config, layer_idx);
}

// Used by generate_initial_areas() in parallel by multiple layers.
class InterfacePlacer {
public:
    InterfacePlacer(
        const SupportParameters         &support_parameters,
        const TreeSupportSettings       &config, 
        SupportGeneratorLayerStorage    &layer_storage, 
        SupportGeneratorLayersPtr       &top_contacts,
        SupportGeneratorLayersPtr       &top_interfaces,
        SupportGeneratorLayersPtr       &top_base_interfaces) 
    :
        support_parameters(support_parameters), config(config),
        layer_storage(layer_storage), top_contacts(top_contacts), top_interfaces(top_interfaces), top_base_interfaces(top_base_interfaces)  
    {}
    InterfacePlacer(const InterfacePlacer& rhs) :
        support_parameters(rhs.support_parameters), config(rhs.config),
        layer_storage(rhs.layer_storage), top_contacts(rhs.top_contacts), top_interfaces(rhs.top_interfaces), top_base_interfaces(rhs.top_base_interfaces) 
    {}

    const SupportParameters    &support_parameters;
    const TreeSupportSettings  &config;
    SupportGeneratorLayersPtr&  top_contacts_mutable() { return this->top_contacts; }

public:
    // Insert the contact layer and some of the interface and base interface layers below.
    void add_roofs(std::vector<Polygons> &&new_roofs, const size_t insert_layer_idx)
    {
        if (! new_roofs.empty()) {
            std::lock_guard<std::mutex> lock(m_mutex_layer_storage);
            for (size_t idx = 0; idx < new_roofs.size(); ++ idx)
                if (! new_roofs[idx].empty())
                    add_roof_unguarded(std::move(new_roofs[idx]), insert_layer_idx - idx, idx);
        }
    }

    void add_roof(Polygons &&new_roof, const size_t insert_layer_idx, const size_t dtt_tip)
    {
        std::lock_guard<std::mutex> lock(m_mutex_layer_storage);
        add_roof_unguarded(std::move(new_roof), insert_layer_idx, dtt_tip);
    }

    // called by sample_overhang_area()
    void add_roof_build_plate(Polygons &&overhang_areas, size_t dtt_roof)
    {
        std::lock_guard<std::mutex> lock(m_mutex_layer_storage);
        this->add_roof_unguarded(std::move(overhang_areas), 0, std::min(dtt_roof, this->support_parameters.num_top_interface_layers));
    }

    void add_roof_unguarded(Polygons &&new_roofs, const size_t insert_layer_idx, const size_t dtt_roof)
    {
        assert(support_parameters.has_top_contacts);
        assert(dtt_roof <= support_parameters.num_top_interface_layers);
        // ORCA: Reserve one top interface layer but only when top base-interface layers exist.
        // This prevents all interface layers from being classified as base-interface layers
        // and preserves correct top contact and interface behavior.
        size_t interface_threshold = support_parameters.num_top_interface_layers_only();
        if (interface_threshold > 0 && support_parameters.num_top_base_interface_layers > 0)
            --interface_threshold;
        SupportGeneratorLayersPtr &layers =
            dtt_roof == 0 ? this->top_contacts :
            dtt_roof <= interface_threshold ? this->top_interfaces : this->top_base_interfaces;
        SupportGeneratorLayer*& l = layers[insert_layer_idx];
        if (l == nullptr)
            l = &layer_allocate_unguarded(layer_storage, dtt_roof == 0 ? SupporLayerType::TopContact : SupporLayerType::TopInterface, 
                    config, insert_layer_idx);
        // will be unioned in finalize_interface_and_support_areas()
        append(l->polygons, std::move(new_roofs));
    }

private:
    // Outputs
    SupportGeneratorLayerStorage                       &layer_storage;
    SupportGeneratorLayersPtr                          &top_contacts;
    SupportGeneratorLayersPtr                          &top_interfaces;
    SupportGeneratorLayersPtr                          &top_base_interfaces;

    // Mutexes, guards
    std::mutex                                          m_mutex_layer_storage;
};

enum class LineStatus
{
    INVALID,
    TO_MODEL,
    TO_MODEL_GRACIOUS,
    TO_MODEL_GRACIOUS_SAFE,
    TO_BP,
    TO_BP_SAFE
};


} // namespace TreeSupport3D

} // namespace Slic3r

#endif // slic3r_TreeSupportCommon_hpp
