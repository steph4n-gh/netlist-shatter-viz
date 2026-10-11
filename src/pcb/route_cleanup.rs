//! Bounded automatic repair of imported route widths and local clearance misses.
//! Every trial preserves net connectivity according to DRC; the caller still runs KiCad/CAM.
use super::*;
use serde::Serialize;
#[derive(Debug, Serialize)]
pub struct RouteCleanupReport {
    pub widths_normalized: usize,
    pub vertices_adjusted: usize,
    pub trials: usize,
    pub clean: bool,
}
fn cost(r: &DrcResult) -> f64 {
    (r.short_circuits.len()
        + r.unrouted_nets.len()
        + r.edge_keepout_violations.len()
        + r.annular_ring_violations.len()) as f64
        * 1e6
        + r.clearance_violations
            .iter()
            .map(|v| 100.0 + (v.required - v.actual).max(0.0))
            .sum::<f64>()
}
pub fn cleanup_route(board: &mut PcbBoard, rules: &RouterConfig) -> RouteCleanupReport {
    let mut candidate = board.clone();
    let mut report = RouteCleanupReport {
        widths_normalized: 0,
        vertices_adjusted: 0,
        trials: 0,
        clean: false,
    };
    for t in &mut candidate.tracks {
        if t.width < rules.default_trace_width && matches!(t.layer, Layer::FCu | Layer::BCu) {
            t.width = rules.default_trace_width;
            report.widths_normalized += 1;
        }
    }
    let copper = candidate.tracks.clone();
    candidate.tracks.retain(|dot| {
        if dot.start.distance_to(dot.end) > 1e-8 {
            return true;
        }
        !copper.iter().any(|t| {
            if t.net != dot.net
                || t.layer != dot.layer
                || t.width + 1e-8 < dot.width
                || t.start.distance_to(t.end) < 1e-8
            {
                return false;
            }
            let v = t.end - t.start;
            let u = ((dot.start - t.start).dot(v) / v.length_sq()).clamp(0.0, 1.0);
            dot.start.distance_to(t.start + v * u) <= (t.width - dot.width) / 2.0 + 1e-8
        })
    });
    let mut drc = run_drc(&candidate, rules);
    // This cleanup cannot invent missing routes or repair shorts/edge violations.
    if !drc.unrouted_nets.is_empty()
        || !drc.short_circuits.is_empty()
        || !drc.edge_keepout_violations.is_empty()
        || !drc.annular_ring_violations.is_empty()
    {
        return report;
    }
    for _ in 0..32 {
        if drc.is_clean() {
            report.clean = true;
            *board = candidate;
            return report;
        }
        let violation = &drc.clearance_violations[0];
        let location = violation.location;
        let mut vertices = vec![];
        for t in &candidate.tracks {
            if Some(t.net) != violation.net1 && Some(t.net) != violation.net2 {
                continue;
            }
            for p in [t.start, t.end] {
                if p.distance_to(location) > 3.0
                    || vertices
                        .iter()
                        .any(|(q, n): &(Point, NetId)| *n == t.net && p.distance_to(*q) < 1e-8)
                {
                    continue;
                }
                // Exact pad centers are terminals, not bend points.
                if candidate.footprints.iter().any(|f| {
                    f.pads.iter().any(|pad| {
                        pad.net == Some(t.net) && f.pad_world_position(pad).distance_to(p) < 1e-6
                    })
                }) {
                    continue;
                }
                vertices.push((p, t.net));
            }
        }
        vertices.sort_by(|(a, an), (b, bn)| {
            a.distance_to(location)
                .total_cmp(&b.distance_to(location))
                .then(an.cmp(bn))
                .then(a.x.total_cmp(&b.x))
                .then(a.y.total_cmp(&b.y))
        });
        let mut improved = None;
        'search: for (p, net) in vertices.into_iter().take(24) {
            for step in [0.025, 0.05, 0.1] {
                for delta in [
                    Point::new(step, 0.),
                    Point::new(-step, 0.),
                    Point::new(0., step),
                    Point::new(0., -step),
                ] {
                    let mut trial = candidate.clone();
                    for t in &mut trial.tracks {
                        if t.net == net {
                            if t.start.distance_to(p) < 1e-8 {
                                t.start += delta;
                            }
                            if t.end.distance_to(p) < 1e-8 {
                                t.end += delta;
                            }
                        }
                    }
                    for v in &mut trial.vias {
                        if v.net == net && v.at.distance_to(p) < 1e-8 {
                            v.at += delta;
                        }
                    }
                    let check = run_drc(&trial, rules);
                    report.trials += 1;
                    if check.unrouted_nets.is_empty()
                        && check.short_circuits.is_empty()
                        && cost(&check) < cost(&drc) - 1e-8
                    {
                        improved = Some((trial, check));
                        break 'search;
                    }
                }
            }
        }
        if let Some((next, check)) = improved {
            candidate = next;
            drc = check;
            report.vertices_adjusted += 1;
        } else {
            break;
        }
    }
    if drc.is_clean() {
        report.clean = true;
        *board = candidate;
    }
    report
}
