# Reviewed RP2040 routing fixtures

`framework.ses` now records the cold, unseeded legal-fanout route for the corrected
35-part RP2040 USB prototype. Its exact draft DSN is fingerprinted in
`framework.dsn.sha256`; source pose, pad or rule changes invalidate it. The normal
application independently regenerates this route from zero copper in 8.87 seconds
including native and CAM checks. See `fabrication/framework-cold-completed-engineering.zip`
and `build/progress-review-2026-09-30/framework-cold-proof.json`. This is a generic
USB prototype with physical qualification still pending.

`bringup.ses` is a local Freerouting 2.4.1 session, reviewed against the exact DSN SHA-256 recorded in `bringup.dsn.sha256`. The build script refuses it after changes to placement, pads, netlist or rules.

Routing used 0.15 mm traces/clearance, 0.6/0.3 mm vias, two copper layers. Freerouting's fanout produced 33 undersized segments despite automatic neckdown being disabled. The reviewed session widens these to 0.15 mm and moves two shared bend points by 0.05 mm: GND (26.7752,23.8733) → (26.8252,23.8733), and CC2 (10.0248,4.2267) → (9.9748,4.2267). Both the internal checker and KiCad then report zero violations and no unconnected items. The session importer checks placement before replacing any copper.

This is an engineering routing fixture, not evidence of working hardware. Generic land patterns and high-speed/power layout still require qualification. A fresh external route may fail checks and needs review; the build script does not silently fix or waive violations.

On 2026-09-29 the DSN fingerprint was refreshed after replacing decimal points in padstack identifiers with `p`. Freerouting 2.4.1 removes decimal suffixes when reading pad names, so decimal identifiers could alias geometrically different pads. Geometry and the reviewed cached session are unchanged; internal and KiCad checks were rerun before refreshing the fingerprint. The new Layout Lab workflow performs fresh routing instead of replaying this fixture and includes bounded automatic cleanup (`--cleanup-route`).
