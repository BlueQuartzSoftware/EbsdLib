# EbsdLib v3.0.0 Stabilization Plan

`topic/pole_figure_updates` is a major behavioral overhaul of EbsdLib's
pole-figure pipeline, IPF coloring, and Laue-class direction
conventions. The branch is feature-complete but has not yet been
hardened against downstream callers (DREAM3D-NX, SIMPLNX). This
document records the current state, the known gaps, and the concrete
steps to declare the branch stable enough to merge as the v3.0.0
release.

Living document — update as items get resolved.

---

## What changed (1-paragraph summary)

`LaueOps::_calcRodNearestOrigin` was rewritten in quaternion space to
fix an undefined-behavior bug for 180° rotations. Hexagonal and
trigonal direction triplets were swapped to the MTEX X||a*
convention. Several Laue classes (`3`, `4`, `6`, `23`, `32`) had
under-enumerated symmetry orbits expanded so each pole figure now
shows the full crystal-symmetry-equivalent set; this required bumping
`k_SymSize` constants for 4 of those classes and renaming default
pole-figure labels for `4/m` and `6/m`. `GriddedColorKey` was fixed
to honor caller-supplied `angleLimits`. A new `PUCMColorKey` was
added (BSD-3 port of wlenthe's reference implementation, matches
EDAX's perceptually uniform IPF colors). All TIFF outputs in apps
and tests switched to PNG via stb. The legend / pole-figure
comparison harness now emits per-class PNGs against MTEX and EDAX
references.

Total: ~30 commits, ~5,000 LOC changes / additions.

---

## Confidence assessment

### High confidence (quantitative external validation)

| Area | Validation | Result |
| --- | --- | --- |
| TSL IPF coloring | Pixel-diff vs `EDAX_TSL_IPF.bmp` (96×100 input, 12 phases) | 11/12 classes mean diff < 3/255; whole-image mean = 3.74; only 5.8% pixels differ |
| PUCM IPF coloring | Pixel-diff vs `EDAX_PUCM_IPF.bmp` | 9/12 classes mean diff < 3/255; whole-image mean = 9.01 |
| 180° FZ-reduction fix | New regression test on `(180°, 90°, 0°)` Euler input | RED → GREEN; physical orientation preserved through `getODFFZRod` |
| GriddedColorKey 3-arg path | 4 targeted regression tests covering: angleLimits passthrough, negative-eta classes, boundary chi-clamp, triclinic full-disk | All RED before fix, all GREEN after |
| C-axis IPF corner test | `IPFLegendTest::CAxisIsRed` across all 11 unique Laue classes | All produce pure red at chi=0 |

### Medium confidence (visual MTEX comparison, no quantitative ground truth)

| Area | What was checked | Caveat |
| --- | --- | --- |
| X||a* convention in hex/trig | Per-class IPF legend visual diff vs MTEX `ipfTSLKey` | MTEX uses a *different* TSL formula (polar/HSL); our agreement is at the convention level (vertex labels), not pixel-perfect. Documented in `coloring_schemes_vs_mtex.md`. |
| Direction triplet expansions for `3`, `6`, `32`, `4` | Visual pole-figure comparison vs MTEX with synthetic Eulers | Eyeball pass only — no canonical-textures assertion. |
| Pole-figure pipeline end-to-end | `PoleFigureLaueComparisonTest` generates one PF per class, MATLAB script generates MTEX equivalent | Visual side-by-side. |

### Low confidence — known suspect, treat as "needs investigation"

| Area | Symptom | Confidence drag |
| --- | --- | --- |
| Mono-c (`112/m`) | mean diff 31.82 (TSL) / 74.67 (PUCM) vs EDAX, ≥46% of pixels affected | Same outlier under TWO independent color schemes ⇒ phase-mapping bug, not formula bug. Likely b-setting vs c-setting axis convention mismatch in `MonoclinicOps`. |
| Tet `4/m` and ditet `4/mmm` PUCM | mean diff 10.05 / 11.78 vs EDAX_PUCM, while same classes match TSL exactly (mean ~1.5) | wlenthe-specific dispatch nuance; not yet diagnosed. |
| `CubicLowOps` 4th `{011}` typo fix | One-line change | Only verified through legend-image diff, no isolated unit test. |
| `getDefaultPoleFigureNames` renames | `<010>` → `<110>` (tet-low), `<11-20>/<2-1-10>` → `<10-10>/<11-20>` (hex-low) | Strings — any downstream code that string-matches the old labels will break silently. |
| Bulk behavior of `_calcRodNearestOrigin` rewrite | Tested for the 180° case only | The new quaternion path SHOULD produce equivalent FZ representatives for all inputs, but no round-trip / equivalence sweep verifies it. |

---

## Risk register: things that may break in DREAM3D-NX / SIMPLNX

1. **Pixel-exemplar tests** — anything that pixel-compares pole-figure or
   IPF map output against a stored reference will fail. The reference
   needs to be regenerated from this branch.
2. **Hard-coded RGB assertions** — any test asserting specific RGB
   triples for specific orientations will fail for hex/trig classes
   (X||a* convention shift) and for the 5 low-symmetry classes whose
   `k_SymSize` changed.
3. **Pole-figure label assertions** — code that string-matches
   `<010>`, `<11-20>`, or the old `<2-1-10>` label position will
   silently take the wrong path.
4. **Mono-c displays anywhere** — will be visibly wrong vs OIM Analysis
   reference output until fixed.
5. **TIFF consumers** — any tooling that reads `.tif`/`.tiff` output
   from EbsdLib's apps now needs to read `.png`.
6. **`_calcRodNearestOrigin` and `getODFFZRod`** — different sym op may
   be picked for an equivalent representation. Anything testing the
   exact 4-component Rodrigues *value* will fail; tests of physical
   equivalence (orientation matrix or quaternion equality) should pass.
7. **Hex/trig Bunge angles read by simplnx must be bridged inside
   LaueOps, not at the file reader.** EbsdLib v3 internal hex/trig
   is now X‖a* (matching Oxford and MTEX). All released DREAM3D /
   DREAM3DNX / SIMPL / SIMPLNX versions normalized hex/trig to X‖a
   at file read time (the +30° rule for Oxford). Strategy: keep
   simplnx readers unchanged; LaueOps gains an `inputDataConvention`
   parameter that defaults to `X‖a` (legacy) and applies the
   transient `phi2 -= 30°` for hex/trig phases. Tracked as Phase 0
   below. Without this work every hex/trig pole figure / IPF /
   Schmid factor / misorientation under v3 will be silently 30°
   wrong on upgrade.
8. **Hex_Low / Trigonal_High / Trigonal_Low Oxford data has *always*
   been broken in released DREAM3D.** The historical +30° rule in
   simplnx only fires when the phase is `Hexagonal_High`. Trigonal
   and hex-low Oxford imports never received the normalization,
   silently producing 30°-rotated data for those phases. This is
   pre-v3 buggery, surfaced by the convention work, and worth fixing
   alongside v3 (P0.3).

---

## Stabilization plan

Ordered by priority. Each item is a discrete piece of work.

### Phase 0: Hex/trig convention bridge inside LaueOps (release blockers)

EbsdLib v3 swapped its hex/trig direction tables to the MTEX / Oxford
**X‖a\*** convention. EDAX/TSL/OIM Analysis and every prior released
version of EbsdLib used the **X‖a** convention. Bunge angles describing
the same physical orientation differ by a 30° rotation about the
c-axis between these two conventions.

**Foundational guarantee from the released DREAM3D family.** Every
released version of DREAM.3D, DREAM3DNX, SIMPL, and SIMPLNX has
applied the +30° to phi2 normalization when reading Oxford `.ctf` /
`.h5oina` / HKL-tagged `.h5ebsd` files for hex phases. As a result,
**every hex/trig EulerAngles array stored in any `.dream3d` / SIMPL
HDF5 file produced by a released DREAM3D tool is in X‖a (TSL) form**.
The only escape is a hand-crafted `.ang` file authored against the
documented warning, which is explicitly out of scope as a user-policy
problem.

That guarantee removes the entire legacy migration question. If the
v3 design treats absence-of-metadata as "data is in X‖a (TSL) form",
that interpretation is correct by codebase history for 100% of files
produced by released tools.

**Strategy.** Do *not* change simplnx file readers. Existing pipelines
keep working. The convention bridge lives inside EbsdLib v3's LaueOps:
LaueOps gains an `inputDataConvention` parameter that defaults to
`X‖a` (legacy). For hex/trig phases, LaueOps applies a transient
`phi2 -= 30°` to a local copy of the orientation before consuming it
with the new X‖a* direction tables, then proceeds normally. Filters
that go through LaueOps inherit correct behavior with no code change.
A filter that wants to opt into X‖a* input (e.g. data imported from
MTEX) sets the parameter explicitly.

Ground truth and methodology: `Data/Pole_Figure_Validation/ReadMe.md`
and `Docs/x_parallel_a_star_convention.svg`.

- [ ] **P0.1 — Add `HexConvention` parameter to LaueOps.**
  Define `enum class HexConvention { XParallelA, XParallelAStar };`
  on `LaueOps` (or a top-level `ebsdlib::HexConvention`) with default
  `XParallelA`. Add `setInputDataConvention(HexConvention)` /
  `getInputDataConvention()` accessors.

  For non-hex/trig Laue classes the parameter is a no-op (the
  convention only affects basal-plane geometry). The accessor is
  still present so callers don't have to switch on Laue class before
  setting it — filters set it once on each LaueOps instance.

- [ ] **P0.2 — Apply the convention shift inside hex/trig LaueOps.**
  Implement the transient `phi2 -= 30°` (radians) at the entry of
  every hex/trig LaueOps method that consumes a Bunge angle and
  produces a crystal-frame-dependent result. At minimum:

  - `generateSphereCoordsFromEulers`
  - `generateIPFColor` (all overloads)
  - `generateRodriguesColor`
  - `getMDFFZRod`, `getODFFZRod`
  - `getNearestQuat`, `getMisoQuat`
  - `getSchmidFactorAndSS`, `getmPrime`, `getF1`, `getF1spt`, `getF7`

  Apply only when `m_InputDataConvention == XParallelA` and the
  Laue class is one of `Hexagonal_High`, `Hexagonal_Low`,
  `Trigonal_High`, `Trigonal_Low`. The shift is on a local copy —
  the caller's data is never mutated.

  Methods that take quaternions, Rodrigues vectors, or orientation
  matrices directly (not Bunge angles) don't need the shift at the
  entry point — those representations are already convention-bound
  by the LaueOps instance they were produced under. (See Filter
  Audit P0.4.)

- [ ] **P0.3 — Extend the historical +30° rule in simplnx to
  Hex_Low / Trigonal_High / Trigonal_Low.**
  Independent of the v3 convention work, the existing simplnx code
  in `ReadCtfData.cpp:138`, `ReadH5OinaData.cpp:42`, and
  `ReadH5Ebsd.cpp:237` only applies the +30° normalization when the
  phase is `Hexagonal_High`. `Hexagonal_Low`, `Trigonal_High`, and
  `Trigonal_Low` use the same basal-plane convention and have always
  needed the same normalization. This was already broken pre-v3 but
  doesn't get noticed because trigonal samples are uncommon.

  Update the predicate to cover all four:

      crystalStructures[cellPhases[i]] == Hexagonal_High  ||
      crystalStructures[cellPhases[i]] == Hexagonal_Low   ||
      crystalStructures[cellPhases[i]] == Trigonal_High   ||
      crystalStructures[cellPhases[i]] == Trigonal_Low

  This is a fix to existing simplnx logic, not a v3-specific change.
  Worth landing alongside v3 since the area is in scope.

- [ ] **P0.4 — Filter audit: which filters actually need to interact
  with the convention parameter, and which are convention-invariant.**

  *Convention-bound (must propagate `XParallelA` default to LaueOps;
  hex/trig path is affected):*
    - Generate IPF Colors
    - Write/Generate Pole Figure
    - Generate IPF Legend (renders in v3 X‖a* — no input data, no shift,
      already correct after the LaueOps direction-table change)
    - Find Misorientations / Avg Misorientations / Feature Reference
      Misorientations / Boundary Misorientation
    - Find Average Orientations
    - Generate Misorientation Colors (Patala 2010)
    - Find Schmid Factors
    - Find ODF / Texture Components / MDF

  *Convention-invariant (no change needed):*
    - Convert Orientations (Euler ↔ Quat ↔ Rod ↔ AxisAngle ↔ Matrix)
    - Find Average C-Axis
    - Find KAM (angle-only)
    - Misorientation magnitude / disorientation angle (scalar, sym-invariant)
    - Anything operating only on cubic, tetragonal, or orthorhombic phases

  Default `XParallelA` means no filter code change is *required* for
  the convention-bound list to behave correctly on legacy data. The
  audit is to confirm each filter actually passes through LaueOps
  rather than reimplementing the math privately. Any filter that
  reimplements crystal-frame math without going through LaueOps is a
  silent landmine.

- [ ] **P0.5 — User-facing UI and labeling for the convention.**
  - Filter UI shows the convention being applied as read-only status:
    *"Input EulerAngles convention: X‖a (legacy DREAM3D, default)"*
    with an *"Override…"* affordance for advanced users.
  - Pole-figure and IPF outputs embed the rendering convention in
    the image title or caption: e.g.
    *"Pole Figure — Phase: Mg (Hexagonal-High 6/mmm) — rendered in X‖a* (MTEX/Oxford)"*.
  - An optional *"render in input convention"* toggle on PF/IPF
    filters skips the LaueOps conversion for users who want to
    visually verify what's literally in the EulerAngles array.

  This addresses the verification-friction concern. Most users never
  touch the override; the default labeling means a user opening a
  PF in MATLAB knows what convention the DREAM3D-NX render is in,
  and what to convert if they want to compare.

- [ ] **P0.6 — Update simplnx reader documentation.**
  Files:
  - `simplnx/src/Plugins/OrientationAnalysis/docs/ReadCtfDataFilter.md`
  - `simplnx/src/Plugins/OrientationAnalysis/docs/ReadChannel5DataFilter.md`
  - `simplnx/src/Plugins/OrientationAnalysis/docs/ReadH5OinaDataFilter.md`
  - `simplnx/src/Plugins/OrientationAnalysis/docs/ReadH5EbsdFilter.md`
  - the corresponding `.ang` reader doc
  - The relevant Generate Pole Figure / IPF Color / Misorientation /
    Schmid Factor / etc. filter docs

  The reader docs are mostly unchanged — readers still apply the +30°
  Oxford normalization. What changes is the description of what
  happens *downstream*: the filter docs (per P0.4) should mention the
  hex convention story and link to
  `Data/Pole_Figure_Validation/ReadMe.md` and
  `Docs/x_parallel_a_star_convention.svg`.

  Correct the convention attribution table everywhere it appears:

  | Convention | Tools / acquisition systems |
  | ---------- | --------------------------- |
  | `X‖a`      | EDAX / TSL / OIM Analysis, pre-v3 EbsdLib, all released DREAM3D family stored hex/trig in this form |
  | `X‖a*`     | Oxford Instruments / HKL (Channel 5, AZtec), MTEX, EbsdLib v3+ |

### Phase 1: Surface the unknown unknowns

- [ ] **P1.1 — Build DREAM3D-NX against this branch and run its tests.**
  Build dir: `/Users/mjackson/Workspace7/DREAM3D-Build/NX-Com-Qt69-Vtk95-Rel-EbsdLib`,
  configured from
  `cd /Users/mjackson/Workspace7/DREAM3DNX && cmake --preset NX-Com-Qt69-Vtk95-Rel-EbsdLib`.
  Capture every failing test with file:line and the kind of failure
  (pixel diff, RGB assertion, label string, runtime crash). The
  classification matters: pixel-exemplar failures → regenerate goldens;
  RGB / label assertions → real downstream change, requires migration
  notes; runtime crashes → real bug that must be fixed before release.
- [ ] **P1.2 — Same for SIMPLNX** if it has its own test set independent
  of DREAM3D-NX.

### Phase 2: Fix the known issues

- [ ] **P2.1 — Mono-c.** Diagnose b-setting vs c-setting in
  `MonoclinicOps`. The .ang file
  `Data/ipf_color_tests/AllLaueClasses_RandO.ang` contains a mono-c
  phase (Phase 11, Symmetry=2 c-setting); use it as the reproducer.
  Fix should bring mean diff against `EDAX_TSL_IPF.bmp` and
  `EDAX_PUCM_IPF.bmp` for the mono-c strip down from 31.82 / 74.67 to
  the same ~1–3 range the other classes show.
- [ ] **P2.2 — Tet 4/m and ditet 4/mmm under PUCM.** Diagnose why these
  two diverge under PUCM (mean ≈ 10–12) when every other class is at
  ~1–3. Likely a wlenthe-specific dispatch detail; the same classes
  match TSL exactly so the orientation/symmetry side is fine.

### Phase 3: Harden tests

- [ ] **P3.1 — Canonical orientations test.** Add a test that asserts
  IPF-Z RGB AND pole-figure position for 10–15 textbook textures,
  cross-referenced against MTEX or DREAM3D-NX. Define the textbook
  Bunge tuples (Cube, Goss, Brass, Copper, S, etc.) once in the
  test, then feed them through `Texture::CalculateODFData` to expand
  each component into a small cloud — that exercises the same code
  path real callers use for orientation grouping rather than testing
  isolated Euler triples.

| Texture | Crystal | Orientation | Expected pole position |
| ------- | ------- |--|------------------- |
| "Brass" | Ebsd::CrystalStructure::Cubic_High |  35.0, 45.0, 0.0 |  | 
| "Copper" | Ebsd::CrystalStructure::Cubic_High |  90.0, 35.0, 45.0 |  | 
| "Goss" |  Ebsd::CrystalStructure::Cubic_High |  0.0, 45.0, 0.0 |  | 
| "Cube" |  Ebsd::CrystalStructure::Cubic_High |  0.0, 0.0, 0.0 |  | 
| "S" |  Ebsd::CrystalStructure::Cubic_High |  59.0, 37.0, 63.0 |  | 
| "S1" | Ebsd::CrystalStructure::Cubic_High |  55.0, 30.0, 65.0 |  | 
| "S2" |  Ebsd::CrystalStructure::Cubic_High |  45.0, 35.0, 65.0 |  | 
| "RC(rd1)" | Ebsd::CrystalStructure::Cubic_High |  0.0, 20.0, 0.0 | | 
| "RC(rd2)" | Ebsd::CrystalStructure::Cubic_High |  0.0, 35.0, 0.0 |  | 
| "RC(nd1)" | Ebsd::CrystalStructure::Cubic_High |  20.0, 0.0, 0.0 |  | 
| "RC(nd2)" | Ebsd::CrystalStructure::Cubic_High |  35.0, 0.0, 0.0 |  | 
| "P" | Ebsd::CrystalStructure::Cubic_High |  70.0, 45.0, 0.0 |  | 
| "Q" | Ebsd::CrystalStructure::Cubic_High |  55.0, 20.0, 0.0 |  | 
| "R" | Ebsd::CrystalStructure::Cubic_High |  55.0, 75.0, 25.0 |  | 

  Each texture asserts: IPF-Z RGB within ±3/255 of expected, pole
  positions within 1° of expected. Pole positions are
  projection-space (x,y) — they survive the future PF rendering
  rewrite. Sources: Bunge, Randle & Engler textbooks, MTEX example
  datasets.

- [ ] **P3.2 — Symmetry-equivalence sweep test.** For each Laue class,
  generate 1000 random orientations and assert: (a) all 24/12/8/etc.
  symmetry-equivalent orientations of an input give the same IPF
  color and same pole-figure position; (b) `getODFFZRod` of an
  orientation and any sym-equivalent of it returns the same physical
  rotation (compare via quaternion `|q1·q2|` ≈ 1).

- [ ] **P3.3 — Cubic-low {011} round-trip.** The one-line typo fix to
  the 4th {011} direction in `CubicLowOps` deserves an isolated test:
  for cubic m-3, the 12 {011} directions are well-defined; assert
  `generateSphereCoordsFromEulers` (or equivalent) emits all 12 unique
  positions for an identity orientation.

- [ ] **P3.4 — IPF exemplar regeneration.** Once P2.1 / P2.2 land
  and the canonical-orientations test passes, regenerate the IPF
  *coloring* exemplars in DREAM3D-NX. Document which exemplars
  were regenerated and why in the commit message.

  **Pole-figure pixel exemplars are deliberately NOT regenerated
  here** — see "Future work" below. The PF *rendering* technique
  is being rewritten away from Lambert-square; regenerating PF
  pixel goldens against the current renderer would just create
  goldens we throw away in the next release. Consumers should
  pin against position-space assertions (see P3.1, P3.2) until
  the new PF renderer lands.

### Phase 4: Documentation and release prep

- [ ] **P4.1 — `RELEASE_NOTES.md` for v3.0.0.** Draft below; update with
  any P1 findings.
- [ ] **P4.2 — Migration guide for downstream.** Concrete code changes
  callers must make to upgrade — see "Migration notes" below.
- [ ] **P4.3 — Bump `EbsdLibVersion.cpp`.** Update version string to
  `3.0.0` and verify SOVERSION on the dynamic library is bumped if
  the project uses semantic versioning for its `.dylib`/`.so`/`.dll`.
- [ ] **P4.4 — Squash the per-step commits if desired.** The branch
  has ~30 commits; some are exploratory. Consider rebasing into a
  cleaner narrative before merge.

---

## v3.0.0 release notes draft

### Breaking changes

1. **Hexagonal and trigonal direction conventions changed to X||a\*.**
   Previously EbsdLib used real-space `a` along the X axis (matching
   EDAX / TSL / OIM Analysis); now uses reciprocal-space `a*` along X
   (matching MTEX and Oxford Instruments / HKL acquisition systems).
   Affects: `HexagonalOps`, `HexagonalLowOps`, `TrigonalOps`,
   `TrigonalLowOps` direction vectors; output pole-figure positions
   for hex/trig phases now match MTEX exactly.

   **API surface change:** `LaueOps` gains an
   `inputDataConvention` parameter (default `X‖a`, matching every
   prior released DREAM3D family file). Filters that pass Bunge
   angles through LaueOps inherit correct legacy behavior with no
   code change; filters or callers wanting MTEX-form input set the
   parameter to `X‖a*` explicitly. Simplnx file readers are
   unchanged. Existing `.dream3d` files just work — their hex/trig
   EulerAngles are in X‖a form by codebase guarantee, which is the
   default LaueOps assumes.

   Methodology and infographic:
   `Data/Pole_Figure_Validation/ReadMe.md` and
   `Docs/x_parallel_a_star_convention.svg`.

2. **`getDefaultPoleFigureNames` renamed for two classes.**
   - `TetragonalLowOps`: `<010>` → `<110>`
   - `HexagonalLowOps`: `<11-20>` → `<10-10>`, `<2-1-10>` → `<11-20>`
   Code matching the old strings will silently take the wrong path.

3. **`k_SymSize` increased for 4 Laue classes.**
   Pole figures now show the full crystal-symmetry orbit (was
   under-enumerated):
   - `TetragonalLowOps`: `{2,2,2}` → `{2,4,4}`
   - `HexagonalLowOps`:  `{2,2,2}` → `{2,6,6}`
   - `TrigonalOps`:      `{2,2,2}` → `{2,6,6}`
   - `TrigonalLowOps`:   `{2,2,2}` → `{2,6,6}`
   Output array sizes from `generateSphereCoordsFromEulers` change
   accordingly.

4. **`LaueOps::_calcRodNearestOrigin` rewritten in quaternion space.**
   Fixes undefined behavior for 180° input rotations. Output
   represents the same physical orientation as before for non-180°
   inputs but the specific 4-component Rodrigues representation may
   differ (different equivalent sym-op chosen).

5. **TIFF outputs replaced with PNG everywhere in apps and tests.**
   `make_ipf`, `generate_pole_figure`, `generate_ipf_legends`, etc.
   now write `.png`; output filenames in tests follow the same.

6. **`make_ipf` gained a 3rd optional argument `tsl|pucm`.**
   Default is `tsl` (backward-compatible behavior).

### New features

- **`PUCMColorKey`** — perceptually uniform IPF color scheme (EDAX
  PUCM-compatible), implemented via a vendored BSD-3 port of
  wlenthe's reference code.
- **`GriddedColorKey`** — decorator that wraps any `IColorKey` to
  produce 1° flat-shaded cells (MTEX-style rendering).
- **`NolzeHielscherColorKey`** — academic Nolze-Hielscher 2016
  implementation.
- **`PngWriter`** utility — STB-backed.
- **EDAX-quality reference data** in `Data/ipf_color_tests/` for
  regression testing.

### Bug fixes

- 180° rotation FZ-reduction undefined behavior (commit `6c831e1`).
- `CubicLowOps` 4th `{011}` direction typo causing 10/12 instead of
  12/12 unique poles (commit `873e61c` and ancestors).
- `GriddedColorKey` 3-arg overload silently using cubic angle limits
  for every Laue class (commit `2c20533`).
- Stray vertical pixel column in 622 IPF legend (commit `873e61c`).

### Known issues (deferred to v3.1)

- Mono-c (point group `112/m`) renders incorrectly vs EDAX reference
  (likely b-setting vs c-setting axis convention bug in
  `MonoclinicOps`).
- Tet 4/m and ditet 4/mmm under PUCM color key have minor (mean ≈ 10
  per channel) divergence from EDAX_PUCM reference.

### Explicitly out of scope for v3.0 — see "Future work"

- Pole-figure rendering technique remains the existing Lambert-square
  pipeline. A rewrite toward an MTEX-style direct-projection /
  contouring renderer is planned for a subsequent release. PF
  pixel-exemplar consumers should expect another breaking change
  there and should pin to crystallographic-position assertions
  (P3.1 / P3.2 style) rather than to v3.0 PF pixel output.
- IPF legend images leave significant whitespace margin around the
  fundamental sector for several Laue classes. Tightening the legend
  rendering so the FZ sector fills the canvas is a targeted follow-up.

---

## Migration notes for downstream callers

For DREAM3D-NX, SIMPLNX, and other consumers upgrading from EbsdLib
v2.x to v3.0:

| Old | New | Action |
| --- | --- | --- |
| `TiffWriter::WriteColorImage(...)` | `PngWriter::WriteColorImage(...)` | Replace include + namespace; output extension `.tiff` → `.png`. `TiffWriter` still exists for callers that want TIFF specifically. |
| Pole-figure label `<010>` (tet-low) | `<110>` | Update string matchers. |
| Pole-figure label `<11-20>` (hex-low) | `<10-10>` | Update string matchers. |
| Hard-coded RGB exemplars for hex/trig phases | Regenerate against v3.0 | Automate via `IPFLegendTest::*Compare_MTEX_IPF_Legends` |
| Pixel exemplars for pole figure rendering | Pin to position-space assertions (projection x,y within tolerance), not to v3.0 PF pixel output | The PF renderer is being rewritten in a follow-up release; v3.0 PF goldens would be invalidated again. See "Future work" §F1. |
| Direct calls to `_calcRodNearestOrigin` testing the 4-component Rodrigues | Convert to quaternion / orientation-matrix comparison | The new code may produce a different sym-equivalent representation |
| Existing IPF coloring tests asserting specific RGB on hex/trig phases | Re-run under v3 with default LaueOps `inputDataConvention = X‖a` | Default convention matches legacy stored data; if the test continues to fail, the old assertion encoded a pre-v3 bug. Use the canonical-orientations test (P3.1) as the fresh reference. |
| Bunge angles passed directly to `LaueOps::generateSphereCoordsFromEulers` and similar APIs | Same call signature; LaueOps now interprets hex/trig Bunge angles per `inputDataConvention` (default `X‖a`) | Legacy code keeps working with no change. Callers feeding MTEX-form data must explicitly call `setInputDataConvention(HexConvention::XParallelAStar)`. |
| Stored hex/trig EulerAngles in legacy `.dream3d` files | No action required | All released DREAM3D family versions stored hex/trig in X‖a (TSL) form via the +30° Oxford normalization. The default LaueOps convention matches. Hex/trig pole figures and IPF colors render correctly without any user intervention. |

---

## DREAM3D-NX validation harness (Phase 1.1)

Build:

```bash
cd /Users/mjackson/Workspace7/DREAM3DNX
cmake --preset NX-Com-Qt69-Vtk95-Rel-EbsdLib
cmake --build /Users/mjackson/Workspace7/DREAM3D-Build/NX-Com-Qt69-Vtk95-Rel-EbsdLib
```

Run:

```bash
cd /Users/mjackson/Workspace7/DREAM3D-Build/NX-Com-Qt69-Vtk95-Rel-EbsdLib
ctest --output-on-failure 2>&1 | tee /tmp/dream3dnx_v3_failures.log
```

Triage failures into three buckets:

1. **Pixel-exemplar failures** (test does an HDF5 / image diff against
   stored reference) — almost all expected; regenerate references and
   verify visually.
2. **Hard-coded RGB or label assertions** — these are real
   conventional changes; document in migration notes and update
   downstream tests.
3. **Runtime errors / crashes / NaN propagation** — real bugs; must
   fix before declaring the branch stable.

Capture the bucket counts in this document under a "Phase 1.1 results"
heading once the run completes.

---

## Future work (post-v3.0)

These are out of scope for the v3.0 stabilization but are tracked
here because they shape what testing we do (and don't do) for v3.0.

### F1 — Pole-figure rendering rewrite

Replace the Lambert-square accumulator with a direct-projection /
MTEX-style renderer (likely density estimation in spherical
coordinates with smooth contouring rather than a square binning
grid). This will change PF *pixel output* again, which is why
P3.4 explicitly skips regenerating PF goldens — there's no point
producing v3.0 PF pixel exemplars that will be invalidated by the
next release. Crystallographic-position correctness (P3.1, P3.2)
is the durable contract; pixel output is not.

Candidate libraries / references: MTEX `plotPDF` source,
spherical kernel density estimators, native MATLAB-equivalent
contouring. `Texture.hpp` orientation generators feed naturally
into either renderer.

### F2 — IPF legend canvas-fill rendering

Several Laue classes' IPF legends currently take up only a small
fraction of the output image (the FZ sector is drawn at its
"natural" eta-chi extent inside a fixed canvas, leaving large
white margins). Targeted change: compute the FZ sector's bounding
box in eta-chi space and stretch the rendering so the sector
fills the canvas, producing a tight, label-ready legend image.
Per-class because the FZ sector aspect ratio varies.

This is non-disruptive to API or color values — it's a
rendering-only change — so it can land independently of F1 and
of any v3.x point release.

---

## Open questions tracked elsewhere

See `Code_Review/coloring_schemes_vs_mtex.md` for the per-Laue-class
comparison details and the running list of color-scheme convention
notes. That document is the more granular companion to this plan.
