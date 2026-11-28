#include "ExternalBridgeSeams.hpp"

#include "libslic3r/Circle.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Surface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace Slic3r {

namespace {

using P2 = Vec2d;

// Sampling of a candidate span.  Coverage against the actual bridge deposits is certified
// exactly with Clipper below; these steps only sample the ownership guards (side, intervening
// wall) and the continuous-gap certificate.
constexpr double SEAM_STEP_FINE_MM   = 0.002;
constexpr double SEAM_NORMAL_MM      = 0.005;
constexpr double SEAM_GRID_CELL_MM   = 1.0;
// Hard budget so a pathological layer cannot stall export; past it the remaining walls stay
// uncertified (a conservative miss).
constexpr size_t SEAM_MAX_SAMPLES_PER_LAYER = 600000;

inline double mm2scaled(double mm) { return mm / SCALING_FACTOR; }
inline P2     to_p2(const Point &p) { return p.cast<double>(); }

// Axis-aligned box in object-local scaled coordinates. It is only used for cheap rejection of pairs
// that cannot interact (face vs deposit, face vs wall); it never contributes to a positive decision.
struct Box
{
    double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
    bool   valid = false;

    void add(const P2 &p)
    {
        if (! valid) {
            x0 = x1 = p.x();
            y0 = y1 = p.y();
            valid   = true;
            return;
        }
        x0 = std::min(x0, p.x());
        y0 = std::min(y0, p.y());
        x1 = std::max(x1, p.x());
        y1 = std::max(y1, p.y());
    }
};

inline Box box_of(const Points &points)
{
    Box box;
    for (const Point &p : points)
        box.add(to_p2(p));
    return box;
}

inline Box box_of(const Polyline &line) { return box_of(line.points); }

inline Box box_of(const ExPolygon &expolygon)
{
    Box box = box_of(expolygon.contour.points);
    for (const Polygon &hole : expolygon.holes)
        for (const Point &p : hole.points)
            box.add(to_p2(p));
    return box;
}

// True when the two boxes are closer than `margin` on both axes (touching counts as near).
inline bool box_near(const Box &a, const Box &b, double margin)
{
    return a.valid && b.valid && a.x0 - margin <= b.x1 && b.x0 - margin <= a.x1 &&
           a.y0 - margin <= b.y1 && b.y0 - margin <= a.y1;
}

inline double point_seg_distance(const P2 &q, const P2 &a, const P2 &b, P2 &closest)
{
    const P2     d  = b - a;
    const double l2 = d.squaredNorm();
    double       t  = 0.0;
    if (l2 > 0.0)
        t = std::clamp((q - a).dot(d) / l2, 0.0, 1.0);
    closest = a + t * d;
    return (q - closest).norm();
}

// Ray (p + t * dir, t > 0) against segment [a,b]; returns false when parallel or behind.
inline bool ray_segment(const P2 &p, const P2 &dir, const P2 &a, const P2 &b, double &t_out)
{
    const P2     v  = b - a;
    const double den = dir.x() * v.y() - dir.y() * v.x();
    if (std::abs(den) < std::numeric_limits<double>::epsilon())
        return false;
    const P2     ap = a - p;
    const double t  = (ap.x() * v.y() - ap.y() * v.x()) / den;
    const double u  = (ap.x() * dir.y() - ap.y() * dir.x()) / den;
    if (t <= 0.0 || u < -1e-12 || u > 1.0 + 1e-12)
        return false;
    t_out = t;
    return true;
}

// Uniform-grid index over 2D segments.  Buckets are keyed by integer cell coordinates, so the
// query radius is a handful of buckets instead of a whole layer scan.
class SegmentIndex
{
public:
    struct Item {
        P2  a, b;
        int payload;
    };

    void add(const P2 &a, const P2 &b, int payload, double cell)
    {
        m_cell = cell;
        const int idx = int(m_items.size());
        m_items.push_back({a, b, payload});
        const int x0 = cell_x(std::min(a.x(), b.x())), x1 = cell_x(std::max(a.x(), b.x()));
        const int y0 = cell_y(std::min(a.y(), b.y())), y1 = cell_y(std::max(a.y(), b.y()));
        for (int x = x0; x <= x1; ++ x)
            for (int y = y0; y <= y1; ++ y)
                m_bins[{x, y}].push_back(idx);
    }

    template<class Fn> void for_each_near(const P2 &p, double radius, Fn &&fn) const
    {
        if (m_items.empty())
            return;
        const int r  = int(std::ceil(std::max(radius, 0.0) / m_cell));
        const int cx = cell_x(p.x()), cy = cell_y(p.y());
        for (int x = cx - r; x <= cx + r; ++ x)
            for (int y = cy - r; y <= cy + r; ++ y) {
                auto it = m_bins.find({x, y});
                if (it == m_bins.end())
                    continue;
                for (int idx : it->second)
                    fn(m_items[idx]);
            }
    }

    bool   empty() const { return m_items.empty(); }
    size_t size() const { return m_items.size(); }

private:
    int cell_x(double x) const { return int(std::floor(x / m_cell)); }
    int cell_y(double y) const { return int(std::floor(y / m_cell)); }

    double                                             m_cell = 1.0;
    std::vector<Item>                                  m_items;
    std::map<std::pair<int, int>, std::vector<int>>    m_bins;
};

// Even-odd containment over a ring set (a bridge face, contours and holes alike).  Edges are
// bucketed by Y so a query only walks the edges that can cross the horizontal ray.
class EvenOddIndex
{
public:
    void add_ring(const Points &points, double ybin)
    {
        m_ybin = ybin;
        const size_t n = points.size();
        if (n < 3)
            return;
        for (size_t i = 0; i < n; ++ i) {
            const P2 a = to_p2(points[i]);
            const P2 b = to_p2(points[(i + 1) % n]);
            if (a.y() == b.y())
                continue;
            const int lo = int(std::floor(std::min(a.y(), b.y()) / m_ybin));
            const int hi = int(std::floor(std::max(a.y(), b.y()) / m_ybin));
            for (int k = lo; k <= hi; ++ k)
                m_bins[k].emplace_back(a, b);
        }
    }

    bool contains(const P2 &q) const
    {
        bool inside = false;
        auto it = m_bins.find(int(std::floor(q.y() / m_ybin)));
        if (it == m_bins.end())
            return false;
        for (const std::pair<P2, P2> &e : it->second)
            if ((e.first.y() > q.y()) != (e.second.y() > q.y())) {
                const double x = e.first.x() + (q.y() - e.first.y()) *
                                 (e.second.x() - e.first.x()) / (e.second.y() - e.first.y());
                if (x > q.x())
                    inside = !inside;
            }
        return inside;
    }

    bool empty() const { return m_bins.empty(); }

private:
    double                                              m_ybin = 1.0;
    std::map<int, std::vector<std::pair<P2, P2>>>       m_bins;
};

// Boundary edges of a bridge face, for the first-interface ray.
class EdgeIndex
{
public:
    void add(const P2 &a, const P2 &b, double cell)
    {
        m_cell = cell;
        const int idx = int(m_edges.size());
        m_edges.emplace_back(a, b);
        const int x0 = int(std::floor(std::min(a.x(), b.x()) / m_cell));
        const int x1 = int(std::floor(std::max(a.x(), b.x()) / m_cell));
        const int y0 = int(std::floor(std::min(a.y(), b.y()) / m_cell));
        const int y1 = int(std::floor(std::max(a.y(), b.y()) / m_cell));
        for (int x = x0; x <= x1; ++ x)
            for (int y = y0; y <= y1; ++ y)
                m_bins[{x, y}].push_back(idx);
    }

    template<class Fn> void for_each_near(const P2 &p, double radius, Fn &&fn) const
    {
        if (m_edges.empty())
            return;
        const int r  = int(std::ceil(std::max(radius, 0.0) / m_cell));
        const int cx = int(std::floor(p.x() / m_cell)), cy = int(std::floor(p.y() / m_cell));
        for (int x = cx - r; x <= cx + r; ++ x)
            for (int y = cy - r; y <= cy + r; ++ y) {
                auto it = m_bins.find({x, y});
                if (it == m_bins.end())
                    continue;
                for (int idx : it->second)
                    fn(m_edges[idx].first, m_edges[idx].second);
            }
    }

    bool empty() const { return m_edges.empty(); }

private:
    double                                          m_cell = 1.0;
    std::vector<std::pair<P2, P2>>                  m_edges;
    std::map<std::pair<int, int>, std::vector<int>> m_bins;
};

// Walks one ExtrusionEntityCollection tree and yields every ExtrusionPath with its enclosing
// loop inset (-1 for multipath/plain paths).  Grid shared-wall collections are skipped: they are
// not ordinary wall scope even though they are erOverhangPerimeter.
template<class Fn> void for_each_path(const ExtrusionEntityCollection &collection, Fn &&fn)
{
    for (const ExtrusionEntity *entity : collection.entities) {
        if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity)) {
            fn(*path, -1);
        } else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath *>(entity)) {
            for (const ExtrusionPath &path : multipath->paths)
                fn(path, -1);
        } else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity)) {
            for (const ExtrusionPath &path : loop->paths)
                fn(path, int(loop->inset_idx));
        } else if (const auto *nested = dynamic_cast<const ExtrusionEntityCollection *>(entity)) {
            if (! nested->external_bridge_grid_wall)
                for_each_path(*nested, fn);
        }
    }
}

} // namespace

// -------------------------------------------------------------------------------------------------
// Per-layer index
// -------------------------------------------------------------------------------------------------
struct ExternalBridgeSeams::Impl
{
    // One connected component of the region's bridge face (`stBottomBridge` surfaces intersected
    // with the clean external-bridge-fill boundary). It is the unit of *source*: the deposits it
    // owns, the contact test, the first-interface ray, the blocker test and the coverage
    // certificate are all evaluated inside one Face, so a move is only ever certified from the local
    // bridge face that actually produced it. A region with two disjoint bridge bands therefore has
    // two independent sources instead of one union.
    struct Face {
        ExPolygon     poly;
        Box           bbox;
        bool          active = false;
        std::vector<size_t> deposit_ids;       // indices into Region::deposits
        SegmentIndex        deposit_index;     // this face's own deposit centre lines
        SegmentIndex        wall_index;        // ordinary overhang walls near this face (blockers)
        EdgeIndex           joint_edges;
        EvenOddIndex        joint_evenodd;
        double              max_deposit_half = 0.0;
        mutable std::map<int, Polygons> capsule_cache;
    };

    struct Wall {
        Polyline      line;
        double        half     = 0.0;
        int           loop_id  = -1;
        // ORCA: generation-time provenance key of the span this path belongs to (0 = none). It is
        // the only way a wall can become a candidate: geometry alone never selects a wall.
        uint64_t      span_key = 0;
        // Per (source face, material side) certificates of this wall, computed lazily on the first
        // candidate move and never merged across faces: an interval certified by face A cannot be
        // used by face B. The side sign is part of the identity because it selects which side of the
        // wall is sampled, so a forward and a reversed copy of the same wall must not share one
        // certificate.
        struct FaceIntervals {
            size_t      face      = 0;
            int         side_sign = 0;
            std::vector<std::pair<double, double>> intervals;
        };
        mutable std::vector<FaceIntervals> face_intervals;

        const FaceIntervals *certificate(size_t face, int side_sign) const
        {
            for (const FaceIntervals &certified : face_intervals)
                if (certified.face == face && certified.side_sign == side_sign)
                    return &certified;
            return nullptr;
        }
    };

    struct Region {
        int        print_region_id   = -1;
        size_t     layer_region_index = 0;
        bool       active            = false;
        bool       built             = false;
        ExPolygons joint;                       // stBottomBridge surfaces ∩ clean bridge boundary
        // Deposit centre lines, clipped per source face. The same bead may appear once per face it
        // genuinely crosses; it is never shared as one unclipped line.
        std::vector<Polyline>    deposits;
        std::vector<double>      deposit_half;
        std::vector<Wall>        walls;         // ordinary overhang walls of this region
        std::vector<Face>        faces;         // one per connected component of `joint`
        double                   max_wall_half = 0.0;
        mutable size_t           samples = 0;
    };

    explicit Impl(const Layer &layer) : m_layer(layer)
    {
        // Fix3 fast path: without an explicitly configured external bridge fan on at least one
        // logical filament, and the overhang/bridge fan switch that arms it, nothing can be
        // classified.  The per-filament decision stays in the emitter; this only skips the work.
        const PrintObject *object = m_layer.object();
        const Print       *print  = object != nullptr ? object->print() : nullptr;
        if (print == nullptr)
            return;
        const PrintConfig &config = print->config();
        bool any_speed = false;
        for (int speed : config.external_bridge_fan_speed.values)
            if (speed >= 0)
                any_speed = true;
        bool any_switch = false;
        for (unsigned char on : config.enable_overhang_bridge_fan.values)
            if (on != 0)
                any_switch = true;
        if (! any_speed || ! any_switch)
            return;

        const LayerRegionPtrs &regions = m_layer.regions();
        m_regions.reserve(regions.size());
        for (size_t i = 0; i < regions.size(); ++ i) {
            const LayerRegion *lr = regions[i];
            if (lr == nullptr)
                continue;
            Region region;
            region.print_region_id    = lr->region().print_region_id();
            region.layer_region_index = i;
            m_regions.emplace_back(std::move(region));
            m_region_by_print_id[lr->region().print_region_id()].push_back(m_regions.size() - 1);
        }
        m_active = ! m_regions.empty();
    }

    bool contains(size_t print_region_id, const ExtrusionPath &wall, const Point &a, const Point &b,
                  const ArcSegment *arc) const
    {
        if (! m_active)
            return false;
        if (wall.role() != erOverhangPerimeter || wall.is_force_no_extrusion() || !(wall.width > 0.f))
            return false;
        if (a == b)
            return false;
        // ORCA: provenance decides *which* wall this move belongs to. A move without a
        // generation-time span key is never certified, however close its geometry happens to be to
        // a bridge: proximity is not ownership and must not become a fallback. This branch is also
        // the explicit, auditable miss for open Arachne fragments: an open fragment carries no key
        // (the producer counts it in SeamProvenanceStats) and can therefore never be certified.
        if (wall.wall_span_key == 0)
            return false;
        const SeamWallSpan *record = m_layer.seam_wall_span(wall.wall_span_key);
        if (record == nullptr || record->side == SeamWallSide::Unknown || record->line.points.size() < 2)
            // Missing table (old cache, unsupported generator path), a key of a previous perimeter
            // generation of this layer, or an unprovable material side: a conservative miss.
            return false;
        // Layer and region consistency. The key already carries the generation; these two checks add
        // the layer identity and the publishing print region, so a key can only certify a path that
        // is emitted from the region that generated it. `print_region_id` here is the same
        // PrintRegion::print_region_id() the emitter got, never the object-local region index.
        if (record->layer_id != int(m_layer.id()))
            return false;
        if (record->print_region_id != int(print_region_id))
            return false;
        auto map_it = m_region_by_print_id.find(int(print_region_id));
        if (map_it == m_region_by_print_id.end() || map_it->second.size() != 1)
            // Unknown or ambiguous region ownership: never borrow another region.
            return false;
        Region &region = m_regions[map_it->second.front()];
        if (! region.built)
            this->build_region(region, *m_layer.regions()[region.layer_region_index]);
        if (! region.active)
            return false;

        const double wall_half = 0.5 * mm2scaled(double(wall.width));
        // A key that ended up on a path whose width the generator never recorded for this span is
        // not this span (e.g. a later rewrite that changed the flow).
        if (wall_half + mm2scaled(0.02) < record->min_half_width ||
            wall_half > record->max_half_width + mm2scaled(0.02))
            return false;
        const P2     pa        = to_p2(a);
        const P2     pb        = to_p2(b);
        const double chord     = (pb - pa).norm();
        if (!(chord > 0.0))
            return false;

        // The native motion of this move: a G1 is its chord, a G2/G3 is the emitted arc itself. The
        // move is certified as a whole - it is never clipped against the span, never split into
        // chords and never re-fitted.
        const bool is_arc = arc != nullptr && arc->is_arc;
        P2         probe0 = pa;
        P2         probe1 = pb;
        double     move_length = chord;
        double     deviation   = 0.0;
        if (is_arc) {
            if (!(arc->radius > 0.0) || !(arc->length > 0.0) ||
                arc->direction == ArcDirection::Arc_Dir_unknow)
                // An arc whose own geometry is not authoritative cannot be certified, and it must
                // not be approximated by its chord either.
                return false;
            const double sweep  = arc->angle_radians;      // signed by direction
            const P2     center = to_p2(arc->center);
            const auto   at     = [&](double t) {
                const double theta = arc->polar_start_theta + t * sweep;
                return center + arc->radius * P2(std::cos(theta), std::sin(theta));
            };
            // Interior probes of the native arc: no endpoint is used for the direction, so a closed
            // span whose start and end parameters coincide cannot flip its own side.
            probe0      = at(1.0 / 3.0);
            probe1      = at(2.0 / 3.0);
            move_length = arc->length;
            // Analytic sagitta of the emitted arc: the arc may bulge this far off the wall
            // centreline, so the certified span must be wider by that much on both ends.
            deviation   = arc->radius * (1.0 - std::cos(0.5 * std::abs(sweep)));
        }
        if (!((probe1 - probe0).squaredNorm() > 0.0))
            return false;
        // Travel direction of the emitted move relative to the canonical span, taken from two
        // interior probes. A closed span is parameterised circularly, so "the end point is the start
        // point" is the total arc length, not a reversal.
        const int travel_sign = travel_sign_on_span(record->line, probe0, probe1, record->closed);
        if (travel_sign == 0)
            // The move cannot be parameterised on its own recorded span: conservative miss.
            return false;
        const int side_sign = (record->side == SeamWallSide::Left ? 1 : -1) * travel_sign;

        const size_t none = std::numeric_limits<size_t>::max();
        bool   any_wall          = false;
        bool   all_walls_covered = true;
        bool   ambiguous_face    = false;
        size_t move_face         = none;
        for (size_t wi = 0; wi < region.walls.size(); ++ wi) {
            const Wall &w = region.walls[wi];
            // Only the very span this path was generated on may certify it. A different wall of
            // the same region is never a candidate, whatever its distance to the bridge.
            if (w.span_key != wall.wall_span_key)
                continue;
            if (w.line.points.size() < 2 || !(w.half > 0.0))
                continue;
            double sa = 0.0, sb = 0.0;
            const double reach = w.half + mm2scaled(0.002);
            if (! project_on(w.line, a, sa) || ! project_on(w.line, b, sb))
                continue;
            if (! within_band(w.line, a, reach) || ! within_band(w.line, b, reach))
                continue;
            const double lo   = std::min(sa, sb);
            const double hi   = std::max(sa, sb);
            const double span = hi - lo;
            if (is_arc) {
                const double tol = std::max(mm2scaled(0.05), 0.05 * move_length);
                if (std::abs(span - move_length) > tol)
                    continue;
            } else if (std::abs(span - chord) > mm2scaled(0.01)) {
                continue;
            }

            // The move lies on this wall. It is certified only if the whole of it is covered by the
            // certificate of exactly one local bridge face.
            any_wall = true;
            size_t wall_face = none;
            for (size_t fi = 0; fi < region.faces.size(); ++ fi) {
                const Face &face = region.faces[fi];
                if (! face.active || ! face_reaches_wall(face, w))
                    continue;
                this->certify(region, fi, wi, side_sign);
                bool inside = false;
                const Wall::FaceIntervals *certified = region.walls[wi].certificate(fi, side_sign);
                if (certified != nullptr)
                    for (const std::pair<double, double> &interval : certified->intervals) {
                        if (interval.first + deviation <= lo + 1e-9 &&
                            hi <= interval.second - deviation + 1e-9) {
                            inside = true;
                            break;
                        }
                    }
                if (! inside)
                    continue;
                if (wall_face == none)
                    wall_face = fi;
                else if (wall_face != fi)
                    // Two bridge faces can both certify this move: the deposits cannot be tied to a
                    // single local source, so this is exactly the ambiguity that must stay a miss.
                    ambiguous_face = true;
            }
            if (wall_face == none)
                all_walls_covered = false;
            else if (move_face == none)
                move_face = wall_face;
            else if (move_face != wall_face)
                ambiguous_face = true;
        }
        return any_wall && all_walls_covered && ! ambiguous_face;
    }

private:
    void build_region(Region &region, const LayerRegion &lr) const
    {
        region.built = true;
        // Bridge face that actually generated erBridgeInfill: the final stBottomBridge surfaces of
        // this region, narrowed by the clean perimeter-derived external bridge boundary.
        ExPolygons surfaces;
        for (const Surface &surface : lr.fill_surfaces.surfaces)
            if (surface.surface_type == stBottomBridge && ! surface.expolygon.empty())
                surfaces.emplace_back(surface.expolygon);
        if (surfaces.empty() || lr.external_bridge_fill_expolygons.empty())
            return;
        region.joint = intersection_ex(surfaces, lr.external_bridge_fill_expolygons);
        if (region.joint.empty())
            return;
        // Every connected component of this face is its own source/domain (see Face). Two disjoint
        // bridge bands of one region stay two independent sources: nothing below unions them, and no
        // nearest-deposit query can cross from one band to the other.

        size_t deposit_segments = 0;
        for_each_path(lr.fills, [&](const ExtrusionPath &path, int) {
            // ORCA: only an actual positive-flow bridge deposit is material evidence. Zero or
            // non-finite flow, a force-no-extrusion wipe and a non-bridge role can never certify
            // contact, even when their geometry sits exactly on the seam.
            if (path.role() != erBridgeInfill || path.is_force_no_extrusion() ||
                !(path.width > 0.f) || !(path.mm3_per_mm > 0.0) ||
                path.polyline.points.size() < 2)
                return;
            Polyline line = path.polyline.to_polyline();
            deposit_segments += line.points.size();
            region.deposits.emplace_back(std::move(line));
            region.deposit_half.push_back(0.5 * mm2scaled(double(path.width)));
        });
        if (region.deposits.empty() || deposit_segments > 200000)
            // No actual deposit, or a deposit set too large to index conservatively.
            return;

        collect_walls(lr.perimeters, region);
        if (region.walls.empty())
            return;

        const size_t raw_deposits = region.deposits.size();
        const double cell         = mm2scaled(SEAM_GRID_CELL_MM);
        const double edge_cell    = mm2scaled(0.5);
        for (size_t wi = 0; wi < region.walls.size(); ++ wi)
            region.max_wall_half = std::max(region.max_wall_half, region.walls[wi].half);

        for (size_t fi = 0; fi < region.joint.size(); ++ fi) {
            Face face;
            // The face *is* the expolygon: move it instead of keeping two copies of the same rings.
            face.poly  = std::move(region.joint[fi]);
            face.bbox  = box_of(face.poly);
            if (! face.bbox.valid)
                continue;
            face.joint_evenodd.add_ring(face.poly.contour.points, edge_cell);
            for (const Polygon &hole : face.poly.holes)
                face.joint_evenodd.add_ring(hole.points, edge_cell);
            const size_t ring_count = 1 + face.poly.holes.size();
            for (size_t r = 0; r < ring_count; ++ r) {
                const Points &points = r == 0 ? face.poly.contour.points : face.poly.holes[r - 1].points;
                const size_t  n      = points.size();
                if (n < 3)
                    continue;
                for (size_t i = 0; i < n; ++ i)
                    face.joint_edges.add(to_p2(points[i]), to_p2(points[(i + 1) % n]), edge_cell);
            }
            // The deposits this face owns: every bead centre line clipped to the face itself. A bead
            // that genuinely crosses two bands contributes its own piece to each of them; it is
            // never shared as one unclipped line, so "nearest deposit" can not jump a band gap.
            for (size_t pi = 0; pi < raw_deposits; ++ pi) {
                const double half = region.deposit_half[pi];
                Polyline     deposit = region.deposits[pi];    // copy: the pool grows below
                if (! box_near(box_of(deposit), face.bbox, mm2scaled(0.05)))
                    continue;
                Polylines pieces = intersection_pl(deposit, face.poly);
                for (Polyline &piece : pieces) {
                    if (piece.points.size() < 2)
                        continue;
                    const size_t id = region.deposits.size();
                    region.deposits.emplace_back(std::move(piece));
                    region.deposit_half.push_back(half);
                    face.deposit_ids.push_back(id);
                }
            }
            if (face.deposit_ids.empty())
                continue;
            for (size_t id : face.deposit_ids) {
                face.max_deposit_half = std::max(face.max_deposit_half, region.deposit_half[id]);
                const Polyline &line = region.deposits[id];
                for (size_t k = 0; k + 1 < line.points.size(); ++ k)
                    face.deposit_index.add(to_p2(line.points[k]), to_p2(line.points[k + 1]), int(id), cell);
            }
            // Blocker index of this face: every ordinary overhang wall that could physically sit
            // between a sample of this face and its deposits. The margin covers the largest wall half
            // width plus the whole interface depth a sample can look at (<= its own half width) plus
            // a safety millimetre, so no real blocker can be excluded by this filter.
            const double blocker_margin = region.max_wall_half + mm2scaled(1.0);
            for (size_t wi = 0; wi < region.walls.size(); ++ wi) {
                const Polyline &line = region.walls[wi].line;
                if (line.points.size() < 2)
                    continue;
                if (! box_near(box_of(line), face.bbox, blocker_margin))
                    continue;
                for (size_t k = 0; k + 1 < line.points.size(); ++ k)
                    face.wall_index.add(to_p2(line.points[k]), to_p2(line.points[k + 1]), int(wi), cell);
            }
            face.active = ! face.joint_evenodd.empty() && ! face.deposit_index.empty() &&
                          ! face.wall_index.empty();
            region.faces.emplace_back(std::move(face));
        }
        region.active = false;
        for (const Face &face : region.faces)
            region.active = region.active || face.active;
    }

    void collect_walls(const ExtrusionEntityCollection &collection, Region &region) const
    {
        for_each_path(collection, [&](const ExtrusionPath &path, int loop_id) {
            if (path.role() != erOverhangPerimeter || path.is_force_no_extrusion() || !(path.width > 0.f) ||
                path.polyline.points.size() < 2)
                return;
            push_wall(path.polyline.to_polyline(), 0.5 * mm2scaled(double(path.width)), loop_id,
                      path.wall_span_key, region);
        });
    }

    void push_wall(Polyline &&line, double half, int loop_id, uint64_t span_key, Region &region) const
    {
        Wall wall;
        wall.line     = std::move(line);
        wall.half     = half;
        wall.loop_id  = loop_id;
        wall.span_key = span_key;
        region.walls.emplace_back(std::move(wall));
    }

    static bool project_on(const Polyline &line, const Point &q, double &s)
    {
        const P2 query = to_p2(q);
        double   acc = 0.0, best = std::numeric_limits<double>::infinity(), best_s = 0.0;
        for (size_t i = 0; i + 1 < line.points.size(); ++ i) {
            const P2 a = to_p2(line.points[i]);
            const P2 b = to_p2(line.points[i + 1]);
            P2       c;
            const double d = point_seg_distance(query, a, b, c);
            if (d < best) {
                best   = d;
                best_s = acc + (c - a).norm();
            }
            acc += (b - a).norm();
        }
        s = best_s;
        return std::isfinite(best);
    }

    static bool project_on(const Polyline &line, const P2 &q, double &s)
    {
        const P2 query = q;
        double   acc = 0.0, best = std::numeric_limits<double>::infinity(), best_s = 0.0;
        for (size_t i = 0; i + 1 < line.points.size(); ++ i) {
            const P2 a = to_p2(line.points[i]);
            const P2 b = to_p2(line.points[i + 1]);
            P2       c;
            const double d = point_seg_distance(query, a, b, c);
            if (d < best) {
                best   = d;
                best_s = acc + (c - a).norm();
            }
            acc += (b - a).norm();
        }
        s = best_s;
        return std::isfinite(best);
    }

    static double polyline_length(const Polyline &line)
    {
        double acc = 0.0;
        for (size_t i = 0; i + 1 < line.points.size(); ++ i)
            acc += (to_p2(line.points[i + 1]) - to_p2(line.points[i])).norm();
        return acc;
    }

    // Direction of the emitted move (probe0 -> probe1, both interior points of the *native* motion)
    // relative to the canonical span. Returns +1 forward, -1 reversed and 0 when the move cannot be
    // parameterised on this span (the caller then misses conservatively).
    //
    // Endpoints are deliberately not used: for a closed span the last point equals the first, so
    // both endpoint projections are 0 (or "0 and the total length") and any comparison of them would
    // read a reversal as forward. For a closed span the arc length is a circular parameter, so a
    // move that crosses the seam is measured through the wrap and a projection that lands on the
    // shared start/end vertex counts as either 0 or the total length, never as a reversal.
    static int travel_sign_on_span(const Polyline &canonical, const P2 &probe0, const P2 &probe1,
                                   bool closed)
    {
        double s0 = 0.0, s1 = 0.0;
        if (! project_on(canonical, probe0, s0) || ! project_on(canonical, probe1, s1))
            return 0;
        double delta = s1 - s0;
        if (closed) {
            const double total = polyline_length(canonical);
            if (!(total > 0.0))
                return 0;
            delta -= total * std::round(delta / total);   // wrap into [-total/2, total/2]
        }
        if (std::abs(delta) <= mm2scaled(1e-3))
            return 0;
        return delta > 0.0 ? 1 : -1;
    }

    // True when this face can interact with this wall at all. Only a rejection filter.
    static bool face_reaches_wall(const Face &face, const Wall &wall)
    {
        if (wall.line.size() < 2)
            return false;
        return box_near(face.bbox, box_of(wall.line), face.max_deposit_half + wall.half + mm2scaled(0.1));
    }

    static bool within_band(const Polyline &line, const Point &q, double reach)
    {
        const P2 query = to_p2(q);
        for (size_t i = 0; i + 1 < line.points.size(); ++ i) {
            P2 c;
            if (point_seg_distance(query, to_p2(line.points[i]), to_p2(line.points[i + 1]), c) <= reach)
                return true;
        }
        return false;
    }

    static void point_at(const Polyline &line, double s, P2 &p, P2 &tangent)
    {
        double acc = 0.0;
        for (size_t i = 0; i + 1 < line.points.size(); ++ i) {
            const P2     a   = to_p2(line.points[i]);
            const P2     b   = to_p2(line.points[i + 1]);
            const double len = (b - a).norm();
            if (len <= 0.0)
                continue;
            if (acc + len >= s || i + 2 == line.points.size()) {
                const double f = std::clamp(len > 0.0 ? (s - acc) / len : 0.0, 0.0, 1.0);
                p       = a + f * (b - a);
                tangent = (b - a) / len;
                return;
            }
            acc += len;
        }
        p       = to_p2(line.points.front());
        tangent = P2(1.0, 0.0);
    }

    // Swept band of every deposit this face owns, widened by this wall's half width. Per face and
    // per half width, so the certificate of one bridge band can never be built from another band's
    // deposits.
    const Polygons *capsule_union(Region &region, const Face &face, double wall_half) const
    {
        const int key = int(std::lround(wall_half * 1000.0));
        auto it = face.capsule_cache.find(key);
        if (it != face.capsule_cache.end())
            return &it->second;
        Polygons raw;
        for (size_t id : face.deposit_ids) {
            Polylines one;
            one.emplace_back(region.deposits[id]);
            // Swept band of one real deposit plus this wall's half width, exactly the contact
            // criterion used by the frozen-layer witness.  Round caps keep the band honest at
            // bridge ends instead of squaring them off.
            append(raw, offset(one, float(wall_half + region.deposit_half[id]), jtRound, 0.0, etOpenRound));
        }
        return &face.capsule_cache.emplace(key, union_safety_offset(raw)).first->second;
    }

    double nearest_deposit(const Region &region, const Face &face, const P2 &p, double wall_half,
                           P2 &closest) const
    {
        double best   = std::numeric_limits<double>::infinity();
        const double radius = wall_half + face.max_deposit_half + mm2scaled(0.05);
        face.deposit_index.for_each_near(p, radius, [&](const SegmentIndex::Item &item) {
            P2           c;
            const double d   = point_seg_distance(p, item.a, item.b, c);
            const double gap = d - (wall_half + region.deposit_half[item.payload]);
            if (gap < best) {
                best    = gap;
                closest = c;
            }
        });
        return best;
    }

    // Depth of the first point that is inside both the bridge surface and the clean external
    // bridge fill boundary, marching from p along `dir`.  Negative when unreachable.
    double interface_depth(const Face &face, const P2 &p, const P2 &dir, double limit) const
    {
        if (face.joint_evenodd.contains(p))
            return 0.0;
        std::vector<double> crossings;
        const double        slack = mm2scaled(0.05);
        face.joint_edges.for_each_near(p, limit + slack, [&](const P2 &a, const P2 &b) {
            double t = 0.0;
            if (ray_segment(p, dir, a, b, t) && t <= limit + mm2scaled(1e-3))
                crossings.push_back(t);
        });
        if (crossings.empty())
            return -1.0;
        std::sort(crossings.begin(), crossings.end());
        double prev = 0.0;
        for (double t : crossings) {
            const double mid = 0.5 * (prev + t);
            if (mid > prev && face.joint_evenodd.contains(p + mid * dir))
                return prev;
            prev = t;
        }
        if (face.joint_evenodd.contains(p + (crossings.back() + mm2scaled(1e-4)) * dir))
            return crossings.back();
        return -1.0;
    }

    bool blocked(const Region &region, const Face &face, const P2 &p, const P2 &tangent,
                 const P2 &side_normal, double depth, size_t own_wall) const
    {
        if (depth <= 2.0 * mm2scaled(SEAM_NORMAL_MM))
            return false;
        const double normal_step = mm2scaled(SEAM_NORMAL_MM);
        bool hit = false;
        face.wall_index.for_each_near(p, depth + mm2scaled(0.1) + region.max_wall_half,
                                      [&](const SegmentIndex::Item &item) {
            if (size_t(item.payload) == own_wall)
                return;
            P2           c;
            point_seg_distance(p, item.a, item.b, c);
            const P2     delta = c - p;
            const double nd    = delta.dot(side_normal);
            const double td    = std::abs(delta.dot(tangent));
            if (nd > normal_step && nd < depth - normal_step && td <= 1.5 * region.walls[size_t(item.payload)].half)
                hit = true;
        });
        return hit;
    }

    bool sample(const Region &region, const Face &face, const P2 &p, const P2 &tangent, double wall_half,
                size_t own_wall, int side_sign, double &gap_out) const
    {
        P2           closest(0.0, 0.0);
        const double gap = this->nearest_deposit(region, face, p, wall_half, closest);
        gap_out          = gap;
        if (! std::isfinite(gap) || gap > 0.0)
            return false;
        const P2 normal(-tangent.y(), tangent.x());
        // The material side is recorded at generation time, not derived from whichever deposit
        // happens to be closest: `side_sign` only maps that record onto this path's travel
        // direction, it never chooses the side.
        const P2 side_normal = side_sign > 0 ? normal : P2(-normal.x(), -normal.y());
        if (face.joint_evenodd.contains(p)) {
            // The wall centreline already lies inside the joint bridge face. Still require the
            // face to continue on the recorded material side, so a wall that merely touches a
            // hole edge from the outside cannot use the material on the other side.
            return face.joint_evenodd.contains(p + (wall_half + mm2scaled(SEAM_NORMAL_MM)) * side_normal);
        }
        const double contact = (closest - p).dot(side_normal);
        if (contact <= mm2scaled(1e-4))
            // The closest actual deposit is not on the recorded material side: whether it is a
            // detached neighbour, the wrong side of a hole edge or an unowned bead, it may not
            // borrow this wall.
            return false;
        const double depth   = this->interface_depth(face, p, side_normal, wall_half);
        if (depth < 0.0)
            return false;                       // no same-side interface inside the wall band
        if (this->blocked(region, face, p, tangent, side_normal, depth, own_wall))
            return false;                       // another ordinary overhang wall is in between
        return true;
    }

    // Certifies `region.walls[wall_index]` against one source face and stores the resulting arc
    // length intervals on that (wall, face) pair. Faces are certified independently and their
    // results are never merged: a move must be covered by a single face.
    void certify(Region &region, size_t face_index, size_t wall_index, int side_sign) const
    {
        Wall &wall = region.walls[wall_index];
        const Face &face = region.faces[face_index];
        if (wall.certificate(face_index, side_sign) != nullptr)
            return;
        std::vector<std::pair<double, double>> intervals;
        if (wall.line.points.size() < 2 || !(wall.half > 0.0)) {
            wall.face_intervals.push_back({face_index, side_sign, std::move(intervals)});
            return;
        }

        const Polygons *capsules = this->capsule_union(region, face, wall.half);
        if (capsules == nullptr || capsules->empty()) {
            wall.face_intervals.push_back({face_index, side_sign, std::move(intervals)});
            return;
        }
        const Polylines covered = intersection_pl(wall.line, *capsules);
        if (covered.empty()) {
            wall.face_intervals.push_back({face_index, side_sign, std::move(intervals)});
            return;
        }

        // Arc-length spans where the wall centreline sits inside the swept band of a real deposit.
        std::vector<std::pair<double, double>> spans;
        for (const Polyline &piece : covered) {
            if (piece.points.size() < 2)
                continue;
            double s0 = 0.0, s1 = 0.0;
            if (! project_on(wall.line, piece.points.front(), s0) || ! project_on(wall.line, piece.points.back(), s1))
                continue;
            if (s1 < s0)
                std::swap(s0, s1);
            spans.emplace_back(s0, s1);
        }
        if (spans.empty()) {
            wall.face_intervals.push_back({face_index, side_sign, std::move(intervals)});
            return;
        }
        std::sort(spans.begin(), spans.end());
        // Union of the exact capsule intersections; touching spans stay touching.
        std::vector<std::pair<double, double>> merged;
        for (const std::pair<double, double> &span : spans) {
            if (! merged.empty() && span.first <= merged.back().second + mm2scaled(1e-6))
                merged.back().second = std::max(merged.back().second, span.second);
            else
                merged.emplace_back(span);
        }

        const double fine = mm2scaled(SEAM_STEP_FINE_MM);
        for (const std::pair<double, double> &span : merged) {
            double s = span.first;
            double e = span.second;
            if (e - s <= 2.0 * fine)
                continue;
            const int    n    = int(std::ceil((e - s) / fine));
            const double step = (e - s) / n;
            bool         have_prev = false, prev_ok = false;
            double       prev_gap = 0.0, run_start = 0.0, run_end = 0.0;
            bool         run_open = false;
            for (int k = 0; k <= n; ++ k) {
                if (region.samples >= SEAM_MAX_SAMPLES_PER_LAYER)
                    break;
                const double sc = s + k * step;
                P2 p, tangent;
                point_at(wall.line, sc, p, tangent);
                double gap = 0.0;
                const bool ok = this->sample(region, face, p, tangent, wall.half, wall_index, side_sign, gap);
                ++ region.samples;
                if (have_prev) {
                    // 1-Lipschitz certificate: between two samples the signed gap cannot exceed
                    // max(gap_prev, gap) + step/2, so a negative bound covers the whole sub-span.
                    const bool pair_ok = ok && prev_ok && std::max(gap, prev_gap) + 0.5 * step <= 0.0;
                    if (pair_ok) {
                        if (! run_open) {
                            run_start = sc - step;
                            run_open  = true;
                        }
                        run_end = sc;
                    } else if (run_open) {
                        this->close_interval(intervals, run_start, run_end, fine);
                        run_open = false;
                    }
                }
                have_prev = true;
                prev_ok   = ok;
                prev_gap  = gap;
            }
            if (run_open)
                this->close_interval(intervals, run_start, run_end, fine);
        }
        wall.face_intervals.push_back({face_index, side_sign, std::move(intervals)});
    }

    static void close_interval(std::vector<std::pair<double, double>> &intervals, double run_start,
                               double run_end, double fine)
    {
        // One full sample at each end is always a boundary sample, so shrink by two to stay
        // strictly inside the certified run.
        const double lo = run_start + 2.0 * fine;
        const double hi = run_end - 2.0 * fine;
        if (hi - lo <= 4.0 * fine)
            return;
        // Only truly overlapping runs are unioned: two runs separated by even one fine step must
        // stay two intervals so a move spanning the gap is refused.
        if (! intervals.empty() && lo <= intervals.back().second)
            intervals.back().second = std::max(intervals.back().second, hi);
        else
            intervals.emplace_back(lo, hi);
    }

    const Layer &m_layer;
    bool         m_active = false;
    mutable std::vector<Region> m_regions;
    std::map<int, std::vector<size_t>> m_region_by_print_id;
};

// -------------------------------------------------------------------------------------------------
ExternalBridgeSeams::ExternalBridgeSeams(const Layer &layer) : m_impl(new Impl(layer)) {}
ExternalBridgeSeams::~ExternalBridgeSeams() = default;

bool ExternalBridgeSeams::contains(size_t print_region_id, const ExtrusionPath &wall, const Point &a,
                                   const Point &b, const ArcSegment *arc) const
{
    return m_impl->contains(print_region_id, wall, a, b, arc);
}

} // namespace Slic3r
