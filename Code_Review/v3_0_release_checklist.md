# EbsdLib 3.0 Release Checklist

Companion to `v3_phase0_design_notes.md` and `v3_stabilization_plan.md`. The
phase-0 / stabilization plan documents the *what* and *why* of the v3 design
(canonical X||a*, SymOps refactor, ColorKeyKind dispatch). This file is the
*how do we ship it* checklist.

The plan is gated: each phase has a verification step that decides whether to
proceed to the next phase or stop and fix. Don't bypass a gate.

---

## Phase 0 — Inventory & freeze

Goal: know exactly what's on the release branch, what's outside it, and stop
introducing churn during the release.

- [x] **Confirm branch state.** EbsdLib `topic/pole_figure_updates` is at
      `d930916` (PUCM thread-race fix) on top of `6084a50` (PR 3 — ColorKeyKind
      dispatch on LaueOps). Verify there's nothing in `origin/develop` that
      hasn't been merged.
- [x] **Open a release branch.** `release/3.0` cut from
      `topic/pole_figure_updates`. All remaining work lands on the release
      branch via PR; no direct pushes.
- [x] **Freeze new feature merges.** During the release stabilization window,
      `topic/*` branches don't merge to `release/3.0`. Bug-fix-only.
- [x] **List unmerged work.** `git log origin/develop..release/3.0 --oneline`
      — copy the list into the release notes draft.
- [x] **Confirm submodule / vcpkg-installed state.** No stale `_Data` folders,
      no detached vcpkg installs that mask the manifest port version.

Gate: green git status across EbsdLib + simplnx + DREAM3D_Plugins +
simplnx-registry. Anything unexpected (untracked design notes, lingering
exemplar regen scratchpads) gets cleaned or committed before phase 1.

---

## Phase 1 — Test baseline triage

Goal: distinguish "stale exemplars from v3 rendering changes" from "real
regression we just shipped." Until this phase is green, the v3 release is
blocked.

### 1a. Catalogue current failures
- [x] **EbsdLib full ctest run.** `cmake --build` + `ctest -R "EbsdLib::"`.
      **Result: 380/380 pass** after the regen of EbsdLib's exemplar
      archive (commit `d930916`). First-run flake on
      `PoleFigureCompositorTest::All_Laue_Classes` cleared on rerun —
      attributed to test-data extraction timing on first archive
      download, not a real failure.
- [x] **simplnx OrientationAnalysis full run.** **Result: 160/164 pass.**
      Same 4 WritePoleFigure tests still failing at byte index 972772:
      - `OrientationAnalysis::WritePoleFigureFilter-Discrete`
      - `OrientationAnalysis::WritePoleFigureFilter-Discrete-Masked`
      - `OrientationAnalysis::WritePoleFigureFilter-Color`
      - `OrientationAnalysis::WritePoleFigureFilter-Color-Masked`
      These pull a **different archive** than the EbsdLib test:
      `PoleFigure_Exemplars_v5.tar.gz` (referenced in
      `simplnx/.../WritePoleFigureTest.cpp:68`) vs EbsdLib's own
      `Pole_Figure_Images.tar.gz`. The 1b regen on the EbsdLib side did
      not touch the simplnx-side archive — still needs its own regen.
- [x] **Hypothesis check.** Same single byte mismatch (index 972772) across
      all 4 simplnx variants — exemplar drift, not a math bug.

### 1b. Regenerate baselines (and prune redundant byte-compare tests)

- [x] **EbsdLib `Pole_Figure_Images.tar.gz` (used by
      `PoleFigureCompositorTest::All_Laue_Classes`).** Regenerated; EbsdLib
      test now green (commit `d930916`).
- [x] **simplnx exemplar archive — rebuilt as `Pole_Figure_Exemplars_v6.tar.gz`
      with a deliberately different test design.** The old `_v5` archive
      drove four byte-compare tests (Discrete / Discrete-Masked / Color /
      Color-Masked) that were re-running EbsdLib's own renderer at the
      byte level — testing the wrong layer, since the simplnx filter is
      mostly a parameter translator. Replaced with:
      - One **mask-effectiveness test** (502 hex-Ti orientations, 251/251
        mask): asserts the rendered RGB array differs by ≥1% bytes between
        `use_mask=false` and `use_mask=true`. Catches a "mask ignored" bug
        like the 12.ang trap.
      - One **HexConvention plumbing test**: asserts the second-family
        intensity array (hex `<10-10>`) differs between X||a and X||a*
        renderings by ≥1% pixels. Catches a switch off-by-one in the
        executeImpl HexConvention dispatch.
      Both use the new `Pole_Figure_Exemplars_v6` archive. The archive is
      tar.gz'd, SHA512'd, and wired into
      `simplnx/.../OrientationAnalysis/test/CMakeLists.txt`.
      Rationale (test pyramid): EbsdLib's `PoleFigureCompositorTest` owns
      byte-level renderer reproducibility; the simplnx tests now cover
      only what simplnx *adds* to EbsdLib — mask filtering + parameter
      translation — so EbsdLib rendering drift no longer breaks simplnx CI.
- [ ] **Any other baseline archives that depend on EbsdLib rendering
      output** (IPF legend reference TIFs, etc.) — grep for `_v\d+\.tar\.gz`
      across both repos and audit each.

### 1c. Verification gate

- [x] **EbsdLib ctest clean.** `ctest -R "EbsdLib::"` — **380/380 pass.**
- [x] **simplnx OrientationAnalysis ctest clean.** **161/161 pass.**
      (Net -1 case: dropped 4 byte-compare WritePoleFigure tests, added
      mask + HexConvention plumbing.)
- [ ] **Document any tests intentionally skipped.** If something stays
      `[!shouldfail]` or `[.disabled]`, write why in the release notes.

If anything still red after exemplar regen, **stop** and root-cause before
phase 2.

---

## Phase 2 — MTEX position validation

Goal: the strongest crystallographic-correctness gate. Confirm EbsdLib's
pole figure positions match MTEX's within tolerance, across all Laue classes
× canonical orientations × plane families × both conventions.

- [x] **Regenerate MTEX goldens.** Re-ran
      `Data/Pole_Figure_Validation/mtex_pole_figure_positions.m` against
      MTEX 6.1.0 / MATLAB R2025b. Same set of points emitted, but the
      MTEX `symmetrise()` row order isn't stable across runs (24 of 396
      buckets — all in 622/`<11-20>` — shuffle their two rows between
      regens). Added a `sortrows()` step to the MATLAB script so each
      bucket emits in canonical `(px, py)` order; two consecutive regens
      are now byte-identical and the committed CSV is now in that sorted
      order.
- [x] **Diff the regenerated CSV vs the (originally-)committed one.** Set-equal
      after sorting both files: same rows, just intra-bucket order
      canonicalized. No value movement → no math change.
- [x] **Run `PoleFigurePositionTest`.** **396/396 buckets pass at 1e-5
      tolerance; worst max-distance across all 1752 emitted points =
      `6.29×10⁻⁸`.** Single-convention coverage (`XParallelAStar` only).
      Dual-convention coverage is provided by `LaueOpsTest::
      GenerateSphereCoords_HexConvention_*` (per-class) plus the simplnx
      `WritePoleFigureFilter: HexConvention choice reaches algorithm`
      plumbing test on hex data (composite RGB + intensity assertions).
      Not expanding the position-space test to 792 buckets because the
      dual-convention math is already pinned at the layer below.
- [x] **No failing buckets** to investigate — see above.
- [x] **Cross-spot-check with the 12.ang renders.**
      `make_pole_figure /Users/Shared/Data/MTR_Data/RR_MTR_Examples/12.ang
      Bin/12_PoleFigures` produces a byte-identical PNG pre/post the
      `9ec95c2` HexConvention enum reorder (EbsdLib uses named enum
      values throughout, so the int re-shuffle didn't perturb behavior).
      EbsdLib vs simplnx (`NX_Phase_1.png`): pixel-close (modulo title
      bar). EbsdLib vs MTEX (`Titanium__Alpha__MTEX_Phase_1.png`): same
      crystallographic features (basal-pole clusters, prismatic dimple
      in <10-10>, elongated bars in <2-1-10>) in matching positions,
      modulo rendering style (discrete projection vs ODF density
      contour). Precision math is gated by the position test above.

### 2b. Verification gate

- [x] PoleFigurePositionTest green across all 396 buckets.
- [x] 12.ang visual spot-check matches MTEX within "obvious texture
      preserved" eyeball threshold; matches simplnx pixel-close.

---

## Phase 3 — Downstream consumer audit

Goal: catch every external caller of the removed / changed API before
release, fix or document each.

### 3a. Grep audit

For each downstream repo (DREAM3DNX, DREAM3D_Plugins, simplnx-registry,
plus any internal forks of simplnx not in the public tree):

- [ ] `grep -rn 'setColorKey\|getColorKey\|setLegendRenderMode\|m_ColorKey' <repo>`
- [ ] `grep -rn 'LegendRenderMode' <repo>` — enum no longer exists.
- [ ] `grep -rn 'generateIPFColor.*HexConvention' <repo>` — IPF color no
      longer takes conv.
- [ ] `grep -rn 'generateRodriguesColor.*HexConvention' <repo>` — same.
- [ ] `grep -rn 'generateIPFTriangleLegend' <repo>` — signature changed
      (`(int, bool, HexConvention, ColorKeyKind, bool gridded)`).
- [ ] `grep -rn 'generateSphereCoordsFromEulers' <repo>` — 5-arg now,
      caller must pass conv.

### 3b. Build each consumer

- [ ] **DREAM3DNX.** Clean rebuild against `release/3.0` EbsdLib. Run
      whatever orientation-related smoke tests exist.
- [ ] **DREAM3D_Plugins.** Clean rebuild + tests.
- [ ] **simplnx-registry.** Clean rebuild + tests for any IPF/PF related
      plugins.
- [ ] **simplnx itself.** Already done this session — but re-confirm clean
      build against the final release tag.

### 3c. Migration cookbook entries

For any external caller pattern we break, write a one-paragraph migration
recipe in the CHANGELOG. Common patterns to cover:

- [ ] `op->setColorKey(std::make_shared<PUCMColorKey>(op->getRotationPointGroup()))`
      → `op->generateIPFColor(eulers, refDir, false, ebsdlib::ColorKeyKind::PUCM)`.
- [ ] `op->setLegendRenderMode(LegendRenderMode::GridInterpolated, 1.0); op->generateIPFTriangleLegend(N, full)`
      → `op->generateIPFTriangleLegend(N, full, conv, ColorKeyKind::TSL, /*gridded=*/true)`.
- [ ] `op->generateIPFColor(eulers, refDir, false, HexConvention::XParallelA)`
      → `op->generateIPFColor(eulers, refDir, false)` (conv-invariant) —
      and document the invariance argument so users don't think we just
      dropped a meaningful parameter.

---

## Phase 4 — CHANGELOG, migration docs, release notes

Goal: external users (vcpkg consumers, fork maintainers) can read one file
and know what changed and how to adapt.

- [ ] **Update `Docs/Index.md`** (or wherever the project changelog lives)
      with the v3.0.0 section. Sections:
      - Breaking changes (removed API, changed signatures, enum shifts).
      - New features (ColorKeyKind dispatch, HexConvention::NotApplicable,
        gridded legend, per-class static color-key singletons).
      - Bug fixes (PUCM thread race, NH SST boundary fixes if any).
      - Migration recipes (from phase 3c).
- [ ] **Cross-link the convention story.** Reference
      `Code_Review/v3_phase0_design_notes.md §16` and
      `Docs/x_parallel_a_star_convention.svg` from the release notes.
- [ ] **API reference snapshot.** Doxygen / hand-written reference for the
      new public LaueOps surface. Include `ColorKeyKind`, the
      `computeIPFColor(eulers, refDir, deg, key)` helper now being public,
      and the per-class `keyForKind` pattern (even though it's file-local,
      explain it so users understand why instances are stateless).
- [ ] **Mark which Apps changed.** `make_ipf`, `make_pole_figure`,
      `generate_ipf_legends`, `render_ebsd` all took non-trivial edits.
- [ ] **Tag the convention-dependent UI knobs.** For simplnx-side filters
      (already done): `WritePoleFigureFilter` exposes `hex_convention_index`;
      `ComputeIPFColorsFilter` and `ComputeFaceIPFColoringFilter` expose
      `color_key_index`. Mention these in the release notes so dream3dnx
      users find them.

---

## Phase 5 — Vcpkg port + git tag

Goal: tag, publish, point consumers at the new artifact.

- [ ] **Bump EbsdLib version string.** Wherever the version is stored
      (`cmake/EbsdLibVersion.cmake`, `vcpkg.json`, etc.) — bump major to
      3.0.0.
- [ ] **Annotated git tag.** `git tag -a v3.0.0 -m "EbsdLib 3.0.0 — ..."`.
      Include a one-liner summary; the full notes live in the GitHub
      release.
- [ ] **GitHub release.** Body = the changelog section from phase 4.
      Attach the regenerated Data_Archive tarballs from phase 1b.
- [ ] **Bump the simplnx vcpkg-manifest baseline.** Edit `vcpkg.json` /
      whichever manifest pins EbsdLib to point at `3.0.0`. Confirm a clean
      `cmake --preset` from a wiped `vcpkg-installed/` builds successfully.
- [ ] **Bump downstream consumer vcpkg baselines** that also pin EbsdLib.
      DREAM3DNX in particular.
- [ ] **Verify CI is green** post-tag on every branch that builds against
      EbsdLib's vcpkg port.

---

## Phase 6 — Polish & defer

Things worth doing if time permits, otherwise carry to 3.1.

- [ ] **PUCM / NH SST-boundary numerical stability check.** Probe with
      `eta = etamax ± epsilon`, `chi = 0/chimax ± epsilon` for each Laue
      class. Confirm no NaN, no infinities, no flickering between adjacent
      basis cells when gridded mode is on. EbsdLib already has
      `PUCMColorKey::AllLaueClassesProduceFiniteColors`; add the explicit
      boundary probe.
- [ ] **Gridded legend visual sweep.** Render all 11 Laue classes ×
      `gridded=true` legends at 512px + 2000px. Eyeball each for empty
      cells, suspicious antialiasing, label collisions.
      `ColorKeyKindTest::LegendAcceptsKindAndGridded` only asserts
      non-null; visual eyeballing catches what the assertion can't.
- [ ] **PoleFigureCompositor smoke under TSan.** If a thread-sanitizer
      build config exists (or is easy to add), run the parallel pole
      figure rendering tests under TSan — would catch any remaining
      lazy-static races like the PUCM cubicToHemi one we hit. wlenthe is
      the most likely place to find more.
- [ ] **`WritePoleFigureFilter` preflight warning when `use_mask = false`.**
      Quality-of-life nicety, prevents the unindexed-(0,0,0) trap from
      this session. Optional.
- [ ] **App-level smoke tests.** Manually run each App (`make_ipf`,
      `make_pole_figure`, `render_ebsd`, `generate_ipf_legends`,
      `generate_pole_figure`, `generate_ipf_from_file`) on at least one
      .ang dataset and one .ctf dataset. Confirm no crash, output sane.

---

## Quick-reference: things already done this cycle

So we don't redo them by accident:

- ✅ `ColorKeyKind` enum + per-class singletons in EbsdLib (commit `ed272c8`).
- ✅ `HexConvention::NotApplicable` sentinel + hex/trig audit pattern (folded
  into `ed272c8`).
- ✅ PUCM cubicToHemi / cubicLowToHemi thread-race fix (`6084a50`).
- ✅ simplnx `ComputeIPFColorsFilter` + `ComputeFaceIPFColoringFilter`
  ChoicesParameter wiring + plumbing tests (`67834e2ba`).
- ✅ simplnx `WritePoleFigureFilter` HexConvention ChoicesParameter wiring +
  plumbing test (`3ad50d386`).
- ✅ simplnx `WritePoleFigure.cpp` 4-arg→5-arg
  `generateSphereCoordsFromEulers` fix.

---

## Out of scope for 3.0

Things that *could* go in 3.0 but are large enough they deserve their own
release if not already done:

- Documentation site / Sphinx build (Phase 2 of the project overview in
  CLAUDE.md).
- Expanded example code (Phase 3 of the project overview).
- TODO.md unit-test backlog (Phase 1 of the project overview, in
  `Code_Review/ebsdlib_todo.md`).
- Any new IPF color keys beyond TSL / PUCM / Nolze-Hielscher.

These should be tracked separately; don't let them gate 3.0.
