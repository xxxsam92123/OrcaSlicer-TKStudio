// Tree supports by Thomas Rahm, losely based on Tree Supports by CuraEngine.
// Original source of Thomas Rahm's tree supports:
// https://github.com/ThomasRahm/CuraEngine
//
// Original CuraEngine copyright:
// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef slic3r_TreeSupportContact_hpp
#define slic3r_TreeSupportContact_hpp

#include "TreeSupportCommon.hpp"

#include "libslic3r/Polygon.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace Slic3r {

class TriangleMesh;

namespace TreeSupport3D {

/*!
 * \brief Every facet of a mesh with its Z range, indexed so the facets that overlap a Z slab can be
 * listed without walking the whole mesh once per overhang area.
 *
 * The top contact of an overhang is placed from the overhang face itself, not from a layer boundary,
 * so the face has to be searched once per overhang area. Only the facets overlapping the searched slab
 * matter, and the index answers that query with a walk over the start of the facet order plus the
 * facets that really reach into the slab, instead of a walk over every triangle for every area.
 */
struct OverhangFacetZIndex
{
    std::vector<double> z_min; // lowest vertex Z of each facet, in mesh order, mesh frame
    std::vector<double> z_max; // highest vertex Z of each facet, in mesh order, mesh frame
    std::vector<size_t> by_z;  // facet indices, ascending z_min, ties in mesh order
    // Segment tree over by_z: the highest z_max inside a node's range. A node whose facets all end
    // below the slab is skipped in one step, which keeps the query proportional to the facets it
    // returns rather than to the facets below them.
    std::vector<double> node_z_max;

    void build(const TriangleMesh &mesh);
};

/*!
 * \brief Lowest Z a down facing facet of `area` can have inside the closed window [z_lo, z_hi], or
 * nothing when no facet reaches into it, in the mesh frame.
 *
 * The result is a lower bound of the face under `area`: a facet is considered when its Z range
 * overlaps the window at all, and a facet that starts below the window contributes the window bottom
 * (its face inside the window is at or above it), so measuring the gap from the result can widen the
 * gap but never close it. The mesh is in the frame it was sliced in, `trafo_sliced()`, and the window is
 * in that same frame - the caller takes both from the layer grid, see place_overhang_contact().
 */
[[nodiscard]] std::optional<double> overhang_face_z(const TriangleMesh &mesh, const OverhangFacetZIndex &facet_z,
                                                    const Polygons &area, double z_lo, double z_hi);

/*!
 * \brief Places the top contact of the overhang area that was sliced into `layer_idx`.
 *
 * A layer's slices are taken at its mid-plane, so the material that first appears in layer `layer_idx`
 * starts below that layer, in the slab of the layers underneath: `[bottom_z(layer_idx - 1), z(layer_idx)]`
 * - closed, and with the round-off tolerance of overhang_face_z() - is the window the face is searched in,
 * and it is two layers tall on a uniform grid. `contact_layer` is the highest whole layer whose top keeps
 * at least `top_z_distance` of air under that face, and `contact_top_z` is where the support material
 * may really end: the face minus the requested distance. The two differ by less than one layer height;
 * the caller adds a support layer of exactly that remainder on top of `contact_layer` (its `height` is
 * the remainder and its extrusions are built with that height), so the printed air gap is the requested
 * one rather than the next whole object layer. The object layers, their Zs and heights are untouched.
 *
 * When no facet overlaps the window at all, the gap is measured from the bottom of that same window
 * instead. The face over an overhang area lies above the mid-plane slice of the layer below it - the
 * area is the material of this layer's slice that the layer below does not support - so that bottom is
 * a lower bound of the material start as well, and the gap stays a lower bound.
 *
 * Also raises the object material start of `layer_idx` to the face, so the clearance of the contact
 * layer below is measured against the face rather than against a layer bottom that sits below it.
 *
 * `mesh` and `grid` are in one frame and no conversion happens in here: the caller transforms `mesh` by
 * `trafo_sliced()` and lifts it by `object_print_z_min` when the object stands on a raft (slicing itself
 * uses `slice_z`, which does not include the raft, while `Layer::print_z` does). `grid` holds those same
 * Zs: `build_layer_grid()` copies each object layer's print_z minus the belt global Z offset, which is
 * added to `Layer::print_z` only after slicing.
 */
struct OverhangContact
{
    LayerIndex            contact_layer { -1 };  // -1 when no whole layer is left to carry it
    // The Z the air gap of this area is measured from, in the frame of the layer grid: the mesh as
    // transformed by `trafo_sliced()` (lifted by `object_print_z_min` on a raft), which is the grid's frame.
    std::optional<double> face_z;
    // Z the support material of this overhang may reach: `face_z - top_z_distance`, i.e. the exact air
    // gap, and never below the top of `contact_layer`. When it lands between two grid layers the caller
    // finishes the contact with a thin layer of its own on top of `contact_layer` (see
    // ExactContactLayer in TreeSupport3D.cpp); when it lands on a grid layer top it changes nothing.
    // `contact_layer` itself and every object layer keep their Z and height either way.
    double                contact_top_z { 0. };
};

[[nodiscard]] OverhangContact place_overhang_contact(TreeSupportLayerGrid &grid, LayerIndex layer_idx,
                                                    const TriangleMesh &mesh, const OverhangFacetZIndex &facet_z,
                                                    const Polygons &area, double top_z_distance);

} // namespace TreeSupport3D

} // namespace Slic3r

#endif /* slic3r_TreeSupportContact_hpp */
