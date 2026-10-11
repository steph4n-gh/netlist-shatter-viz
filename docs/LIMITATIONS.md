# Alpha limitations

`v0.1.0-alpha.2` releases the local software workbench. It does not release a
qualified hardware product or approve any board for fabrication or assembly.

## Pico native Finish scope

The alpha 2 completion work concerns the retained original CORE03 native Pico
project, including **`RGB_DATA` and `I2C_SCL`**, through the ordinary **Finish**
workflow. The software discovers bounded copper cuts and restores affected nets
without a prepared cut-board upload or manual trace coordinates. It keeps the
input unchanged and checks surviving source contacts and original rules before
accepting a result.

Qualification of this input requires all **66 original restoration obligations**
and **64 supplied profile rules**, zero enabled native errors, warnings or open
connections, and the workflow's independent export checks. The release's
`VERIFICATION.json` records the exact input identity and executed results.
The source package does not bundle that development input or a Pico
manufacturing archive. This scope does not establish cold routing of the
separate built-in Pico model or completion of arbitrary Pico revisions.

Alpha 1 retained those two open connections. Earlier target-only candidate
paths did not establish complete native acceptance; that history remains in the
[release notes](RELEASE_NOTES.md).

A search timeout or exhausted state limit is a software search miss. It does
not establish that a physical route is impossible.

## What a passing result means

Candidate guards screen a proposed change. Native KiCad DRC/connectivity,
source-preservation checks, supplied profile checks and independent CAM checks
provide additional software evidence for an exact resulting board. These are
different stages; a legal candidate path is not a completed board.

An engineering archive is available only when the workflow's required checks
pass. Read its report and qualification holds. Passing geometry, connectivity
or CAM checks does not establish complete circuit correctness, component
suitability, reliable electrical operation or successful assembly.

No physical bring-up, firmware operation, thermal qualification, machine
placement convention or manufacturing release has been established for these
demonstrations.

## Geometry and design scope

- Automatic initial native placement supports rectangular boards and supplied
  anchors/groups. It does not infer critical bypass, clock or connector rules.
- Native board routing keeps the existing component placement. The model
  editor and built-in model generators use a more limited two-layer geometry
  representation; model and native support are not interchangeable.
- The model representation supports round plated drills and selected simple
  pad shapes. It does not represent blind/buried vias, plated slots, NPTH
  separation, arbitrary cutouts or filled zones.
- Blind/buried vias are unsupported by the native routing workflow. Imported
  projects require their matching project rules and referenced libraries.
- Differential-pair coupling, impedance, high-speed timing and thermal behavior
  are not solved. Supplied length, width, layer, via and matching bounds are
  explicit checks, not automatic electrical qualification.
- Optional DC and passive RC scenarios check only the supplied loads, limits
  and numerical models. They do not validate a regulator control loop,
  inductance, switching behavior or complete board operation.
- Optional reference-plane rules have a limited single-layer signal scope.
  Missing profiles leave their associated properties unchecked.

Review actual footprints and pin functions against the selected parts before
building hardware. Generic footprint names and clean DRC do not qualify a
manufacturer's land pattern.

## Search and operational scope

Routing and placement use finite search and time budgets. Dense boards can be
slow or remain incomplete. A failed attempt retains its diagnostics and is
excluded from passing engineering downloads. There is no universal routing
success rate or demonstrated dominance over a skilled hand layout.

The server is intended for local use on loopback. It launches local tools and
writes jobs under `build/`; it is not designed as a public multi-user service.
The alpha has no backward-compatibility promise for its files or interfaces.
Platform validation is limited to the installation evidence recorded for this
release; other systems may need tool-path adjustments.
