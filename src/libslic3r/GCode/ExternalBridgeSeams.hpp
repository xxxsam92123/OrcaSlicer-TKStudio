#ifndef slic3r_ExternalBridgeSeams_hpp_
#define slic3r_ExternalBridgeSeams_hpp_

#include <cstddef>
#include <memory>

namespace Slic3r {

class ArcSegment;
class ExtrusionPath;
class Layer;
class Point;

// ORCA: Read-only, layer-local classifier for ordinary overhang-wall segments that sit on the
// actual material interface of an external bridge deposit. It answers one question for the
// G-code writer: is *this whole* already emitted ordinary wall move part of such a seam?
//
// Authority, in this order:
//   * identity and local material side come from the generation-time provenance record the
//     perimeter generator attached to the path (`ExtrusionPath::wall_span_key` ->
//     `Layer::seam_wall_span()`). A path without such a key, or with a key whose record is gone, is
//     never certified: proximity to a bridge is not ownership and is not a fallback. The key is
//     `(generation << 32) | record`, and the record additionally carries the owning layer id and the
//     publishing `print_region_id`; all three are validated, so a stale generation, a key from
//     another layer and a key used from another region all miss;
//   * the walls are the final post-region-split `LayerRegion::perimeters` paths (never `fills`,
//     so the external bridge grid's shared walls can never become a target);
//   * the bridge face is the layer's own `stBottomBridge` surfaces, intersected with the clean
//     perimeter-derived `external_bridge_fill_expolygons` boundary. The intersection is split into
//     its connected components and every component is its own source/domain: each deposit centre
//     line is clipped to the component that owns it, and contact, first interface, blocking and
//     coverage are all evaluated inside one component. A move that two components could certify is
//     ambiguous and misses; there is no "union of all bridge faces" distance anywhere;
//   * the deposits are the actual positive-flow `erBridgeInfill` extrusion paths of the same region.
//
// Geometry therefore only certifies contact, the first material interface and the absence of an
// intervening wall; it never decides which wall a move belongs to or which side its material is on.
//
// Motion scope: a move is certified as the whole native motion the emitter is about to write. G1 is
// checked against its chord, G2/G3 against the emitted arc's own length (plus its analytic sagitta),
// and neither is clipped, chord-split or re-fitted. The travel direction of a move is taken from two
// interior probe points of that native motion, so a closed span whose end parameter coincides with
// its start cannot flip its own side.
//
// Arachne scope: only erOverhangPerimeter paths that live inside an ExtrusionLoop of the innermost
// overhang inset carry a key. Open Arachne fragments (plain ExtrusionPath / ExtrusionMultiPath
// output) have no ring and therefore no provable single material side; they are deliberately unkeyed
// and counted on the layer (`SeamProvenanceStats`). That is a conservative miss and is NOT support
// for Arachne open walls.
//
// Everything else is rejected: an unseen cell, a missing cache or an ambiguous ownership never
// produces a positive. The classifier is built for one layer and one object-local coordinate
// frame; the caller must not keep it past that layer's export step and must not assume that the
// pointers it was built from survive cloning (lookup is by value key plus geometry).
class ExternalBridgeSeams
{
public:
    explicit ExternalBridgeSeams(const Layer &layer);
    ~ExternalBridgeSeams();

    ExternalBridgeSeams(const ExternalBridgeSeams &) = delete;
    ExternalBridgeSeams &operator=(const ExternalBridgeSeams &) = delete;

    // `print_region_id` is the PrintRegion of the current `extrude_perimeters` traversal.
    // `a` and `b` are the endpoints of the whole native emitted move in scaled object-local
    // coordinates, oriented as written; `arc` describes the same move when it is emitted as a
    // single G2/G3. True is returned only when [a,b] is continuously covered by a certified seam
    // span of exactly one ordinary overhang wall of that region. Partial contact, an ambiguous
    // wall match, a missing cache or an unprovable arc all return false.
    bool contains(size_t print_region_id, const ExtrusionPath &wall, const Point &a,
                  const Point &b, const ArcSegment *arc = nullptr) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace Slic3r

#endif // slic3r_ExternalBridgeSeams_hpp_
