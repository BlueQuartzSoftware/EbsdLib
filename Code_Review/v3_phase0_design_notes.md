# EbsdLib v3 — Phase 0 Design Notes

Companion to `v3_stabilization_plan.md`. The main plan describes *what*
Phase 0 has to do at the milestone level. This doc captures the design
decisions that emerged through several rounds of discussion: API shape,
code layout, sequencing, and the empirical findings that informed each
choice. If you're picking up Phase 0 cold, read this first, then
consult the plan for the surrounding context.

This document supersedes earlier exploratory drafts of the Phase 0
design. The earlier ideas (a marker-based metadata scheme, a
constructor-bound `LaueOps::HexConvention` for the entire library, a
default of X‖a*) were considered and set aside; the rationale is in
§11.

---

## 1. The problem in one paragraph

EbsdLib's hex/trig direction tables describe crystal-frame plane
normals and Laue symmetry rotations in *some* Cartesian basis. Two
conventions exist in the field: **X‖a** (real-lattice a along
Cartesian X — used by EDAX/TSL/OIM Analysis and every released
DREAM.3D / DREAM3DNX / SIMPL / SIMPLNX file) and **X‖a\*** (reciprocal-
lattice a* along Cartesian X — used by Oxford/HKL acquisition systems
and MTEX). The two are related by a 30° rotation about c. For a
hex/trig pole figure, the choice of convention rotates the visible
{10-10} and {2-1-10} clusters by 30° on the disk.

There is no "correct" convention; both are crystallographically valid.
The decision is which one EbsdLib renders by default and how it lets
callers opt into the other.

---

## 2. Design decisions

### 2.1 — Convention is a rendering choice, not a default-correctness choice

Bunge angles, quaternions, and Rodrigues vectors describe rotations.
A rotation R is the same physical thing regardless of which Cartesian
basis we attach to the crystal frame. Misorientation angles are
invariant. FZ-reduced orientations are invariant. The Laue group
itself is invariant. Most of EbsdLib's outputs (computed average
orientations, misorientation magnitudes, KAM, ODFs) don't care about
the convention.

The convention only matters when we *project* a rotation into a
specific crystal direction (e.g. "where does the [10-10] pole sit in
the sample frame?"). That projection happens inside EbsdLib's
**rendering paths**: pole figures, IPF colors, IPF legends,
Patala-style misorientation colors. Those are the only places that
need to be convention-aware.

Putting a convention parameter on the LaueOps constructor (the
earlier proposal) was overkill — it made every method
convention-aware, which in turn forced every caller to think about
the convention even for operations where it doesn't matter. Putting
the parameter only on the rendering paths localizes the choice to
where it has visible meaning.

### 2.2 — Default = X‖a (legacy DREAM3D convention)

Existing DREAM3DNX users have:
- TSL `.ang` and Oxford `.ctf` data flowing through saved pipelines
- Published figures, archived datasets, training material with PFs
  rendered in X‖a / OIM-Analysis form
- An expectation that "running my pipeline produces what it always did"

A default convention change to X‖a* would silently rotate every
hex/trig PF in every saved pipeline by 30°. The conservative choice
— and the right one for the production user base — is to default
to X‖a and make X‖a* an explicit opt-in.

This is a reversal of the original v3 framing ("now matches MTEX
out of the box"). The MTEX-compatibility work isn't wasted; it
becomes a feature available *on demand* rather than the default
behavior.

### 2.3 — Convention exposed via config-struct extension and per-method parameter, not the constructor

The `PoleFigureConfiguration_t` and `CompositePoleFigureConfiguration_t`
structs already carry rendering settings (`imageDim`, `numColors`,
`labels`, etc.). Adding `HexConvention` to them is the natural place:

```cpp
struct PoleFigureConfiguration_t
{
  // ... existing fields ...
  ebsdlib::HexConvention hexConvention = ebsdlib::HexConvention::XParallelA;
};
```

For methods that don't take a config struct (`generateIPFColor`,
`generateIPFTriangleLegend`, `generateRodriguesColor`), add a default
argument directly:

```cpp
virtual Rgb generateIPFColor(double* eulers,
                             double* refDir,
                             bool convertDegrees,
                             ebsdlib::HexConvention conv = ebsdlib::HexConvention::XParallelA) const = 0;
```

The default at the API level preserves current behavior. simplnx
filters that want to expose the choice to the user pass it explicitly
based on a UI parameter; filters that don't care just use the default.

### 2.4 — A separate "Convert Hex/Trig Euler Angles Between Cartesian Conventions" filter handles MTEX export

Users who want to *export* their data in X‖a* form (for MTEX import
or archival) run this dedicated filter. It produces a new EulerAngles
array with `phi2 ± 30°` applied to hex/trig phases (cubic / tet /
ortho / mono / tri pass through unchanged). This is a one-shot data
transformation, not a per-render option.

See §6 for the filter spec.

---

## 3. The `HexConvention` enum

```cpp
namespace ebsdlib
{
enum class HexConvention : uint8_t
{
  XParallelA,     // Real-lattice a along Cartesian X.
                  // EDAX/TSL/OIM Analysis. Every released DREAM.3D /
                  // DREAM3DNX / SIMPL / SIMPLNX file is in this form
                  // by codebase guarantee. Default for all rendering
                  // paths to preserve backward compatibility.
  XParallelAStar  // Reciprocal-lattice a* along Cartesian X.
                  // Oxford / HKL (Channel 5, AZtec) / MTEX. Opt-in
                  // for users wanting MTEX-comparable visual output.
};
}
```

Header location: `Source/EbsdLib/Core/EbsdLibConstants.h` (or a new
small dedicated header `Source/EbsdLib/Core/HexConvention.h` if we
don't want to bloat the constants file).

---

## 4. API shape — what changes, what doesn't

### Methods that gain the parameter (rendering paths)

| Method | How |
|---|---|
| `LaueOps::generatePoleFigure(PoleFigureConfiguration_t&)` | New `hexConvention` field on the config struct, default `XParallelA` |
| `PoleFigureCompositor::generateCompositeImage(CompositePoleFigureConfiguration_t&)` | New `hexConvention` field on the config struct, default `XParallelA` |
| `LaueOps::generateIPFColor(...)` (all overloads) | New trailing `HexConvention conv = XParallelA` parameter |
| `LaueOps::generateIPFTriangleLegend(int, bool)` | New trailing `HexConvention conv = XParallelA` parameter |
| `LaueOps::generateRodriguesColor(...)` | New trailing `HexConvention conv = XParallelA` parameter |

For non-hex/trig Laue classes (cubic, tet, ortho, mono, tri), the
parameter is accepted but ignored — no basal-plane convention exists.
Uniform API is worth more than micro-optimizing away an unused argument.

### Methods that don't change

Everything else. Specifically, the following stay convention-agnostic:

| Method | Why |
|---|---|
| `getODFFZRod`, `getMDFFZRod` | FZ reduction is sym-group invariant; same physical rotation regardless of basis |
| `getNearestQuat`, `getMisoQuat` | Misorientation rotation is the same physical thing under any convention; the angle is invariant |
| `calculateMisorientation` | Same |
| `getSchmidFactorAndSS`, `getmPrime`, `getF1`, `getF1spt`, `getF7` | Scalar outputs; LaueOps' internal convention (X‖a) is self-consistent for these |
| `getNumSymOps`, `getQuatSymOp(i)`, `getMatSymOp(i)` | These return convention-bound data, but they're returning EbsdLib's *internal* form (X‖a). Callers who consume these directly are operating in X‖a. |
| All non-hex/trig Laue class methods | No basal-plane convention exists |

### Rationale for not putting the parameter on `LaueOps` constructor

The constructor-bound design (earlier proposal) made convention a
property of the `LaueOps` *instance* — every method consulted the
instance's convention. That has the consistency-by-construction
property, which is appealing, but it forces every caller to make
the convention choice even for operations where it has no effect.
For a typical filter that does FZ reduction or misorientation work,
the convention parameter would just be noise.

The rendering-only design accepts that misorientation axes computed
in X‖a need a small bridge if a downstream filter wants to render
them in X‖a* form (see §9.2), in exchange for a much smaller API
surface change.

---

## 5. Internal architecture for sym op constants

EbsdLib's hex/trig classes carry compile-time arrays of sym ops:

```cpp
namespace HexagonalHigh {
  static const std::array<QuatD, 12>          k_QuatSym = { ... };
  static const std::array<Matrix3X3D, 12>     k_MatSym  = { ... };
  static const std::array<RodriguesDType, 12> k_RodSym  = { ... };
}
```

These describe symmetry rotations in the crystal frame, and the
basal-plane axes are convention-dependent — so the numerical values
differ between X‖a and X‖a*.

### 5.1 — Approach: canonical + derived via templated factory

> **Note:** This section captures the design *pattern* (single
> canonical hand-maintained set + derived alternate via templated
> factory + two static instances + pointer-flip dispatch). The
> *direction* of canonical-to-derived was revised during PR 2e —
> see §16 for the implemented choice (canonical = X‖a*, derived =
> X‖a) and the reasoning behind it. The pseudocode below uses the
> originally-planned direction (canonical = X‖a, derived = X‖a*)
> for narrative continuity with the rest of this document; the real
> code in `Source/EbsdLib/LaueOps/HexagonalOps.cpp` and the three
> peer files is in the §16 direction.

Hand-maintain *one* set of constants in the canonical form. Derive
the alternate-convention version algorithmically at TU static-init
via a `SymOps` helper struct with an `if constexpr` factory. Both
static instances exist in the binary; the rendering methods pick
one based on the `HexConvention` parameter.

```cpp
namespace HexagonalHigh
{
// One canonical set, hand-maintained, in X||a form.
static const std::array<QuatD, 12>          k_CanonicalQuatSym = { /* X||a values */ };
static const std::array<Matrix3X3D, 12>     k_CanonicalMatSym  = { /* X||a values */ };
static const std::array<RodriguesDType, 12> k_CanonicalRodSym  = { /* X||a values */ };

struct SymOps
{
  std::array<QuatD, 12>          quat;
  std::array<Matrix3X3D, 12>     mat;
  std::array<RodriguesDType, 12> rod;

  template <ebsdlib::HexConvention Conv>
  static SymOps build()
  {
    SymOps out;
    if constexpr (Conv == ebsdlib::HexConvention::XParallelA)
    {
      // Trivial copy of the canonical set.
      out.quat = k_CanonicalQuatSym;
      out.mat  = k_CanonicalMatSym;
      out.rod  = k_CanonicalRodSym;
    }
    else // XParallelAStar — derive by 30°-about-c similarity transform
    {
      const QuatD q30    = QuatD::FromAxisAngle(0.0, 0.0, 1.0, 30.0 * ebsdlib::constants::k_PiOver180D);
      const QuatD q30Inv = q30.conjugate();
      for (size_t i = 0; i < 12; ++i)
      {
        out.quat[i] = q30 * k_CanonicalQuatSym[i] * q30Inv;
        out.mat[i]  = /* matrix similarity:  R(30) · M · R(-30)  */;
        out.rod[i]  = /* rod conjugation                          */;
      }
    }
    return out;
  }
};

// Two static instances. Built once at TU static-init.
// Order is well-defined because they sit BELOW k_CanonicalQuatSym.
static const SymOps k_SymOps_XParallelA     = SymOps::build<ebsdlib::HexConvention::XParallelA>();
static const SymOps k_SymOps_XParallelAStar = SymOps::build<ebsdlib::HexConvention::XParallelAStar>();
} // namespace HexagonalHigh
```

The rendering methods select the right table based on the incoming
`HexConvention` argument:

```cpp
std::vector<UInt8ArrayType::Pointer>
HexagonalOps::generatePoleFigure(PoleFigureConfiguration_t& config) const
{
  const HexagonalHigh::SymOps* sym
    = (config.hexConvention == ebsdlib::HexConvention::XParallelAStar)
        ? &HexagonalHigh::k_SymOps_XParallelAStar
        : &HexagonalHigh::k_SymOps_XParallelA;

  // Use sym->quat[i], sym->mat[i], sym->rod[i] in the rendering loop.
  // Apply the transient phi2 shift to the local Euler copy when
  // config.hexConvention == XParallelAStar (see §7).
  // Use the convention-appropriate direction tables for the plane families.
}
```

### 5.2 — Why this shape

- **One canonical hand-maintained set** per Laue class — no risk of
  drift between two parallel hand-typed tables.
- **Compile-time elimination** of the unused branch in each
  template instantiation. `SymOps::build<XParallelA>()` has no
  conjugation code in its emitted form; `build<XParallelAStar>()`
  has no copy-from-canonical code.
- **Zero per-instance memory cost.** Both static instances exist
  once in the binary regardless of how many `HexagonalOps`
  instances are constructed.
- **Convention dispatch localized** to `SymOps::build<>` and the
  pointer-pick at the top of each rendering method — not smeared
  through method bodies.
- **Same architecture pattern across all four hex/trig classes**
  (`HexagonalOps`, `HexagonalLowOps`, `TrigonalOps`,
  `TrigonalLowOps`), each with its own `SymOps` namespace block.

### 5.3 — Plane-family direction tables

The same idea applies to the per-Laue plane-family direction lists
used inside `generateSphereCoordsFromEulers` (the hardcoded
`direction[0] = 1.0; direction[1] = 0.0; ...` blocks for {10-10},
{2-1-10}, etc.). Add convention-aware pairs of these tables — X‖a
canonical, X‖a* derived — and pick the right one based on the
incoming `HexConvention`. Same `SymOps`-style helper applies.

### 5.4 — `getDefaultPoleFigureNames()`

Returns string labels for the three default plane families. The slot
*order* (c-axis / prism / a-family) is the same under both conventions
— the rendering pipeline always emits family-0 / family-1 / family-2
in that fixed order. What changes between conventions is only the
*string* the third slot prints, because OIM and MTEX pick different
orbit-member representatives for the `{2-1-10}` a-family:

| Slot | X‖a (OIM-style) | X‖a* (MTEX-style) |
|------|-----------------|--------------------|
| 0    | `<0001>`        | `<0001>`           |
| 1    | `<10-10>`       | `<10-10>`          |
| 2    | `<2-1-10>`      | `<11-20>`          |

`<2-1-10>` and `<11-20>` are sym-equivalent under the 6-fold about c
— they describe the same physical family, just with different
"first orbit member" choices that match each tribe's tooling.

```cpp
virtual std::array<std::string, 3>
getDefaultPoleFigureNames(ebsdlib::HexConvention conv = HexConvention::XParallelAStar) const = 0;
```

> **PR 2i implementation note:** Earlier drafts of this section
> proposed swapping slot order between conventions (e.g. X‖a returns
> `{<0001>, <2-1-10>, <10-10>}`). That was wrong — the renderer
> doesn't reorder families, so swapping the labels in the array would
> have misaligned labels with rendered content. The implementation
> only swaps the string in slot 2; slot order is fixed.

> **Trigonal classes (-3m, -3):** TrigonalHigh has two distinct prism
> families; the OIM/MTEX label-tradition split that hex 6/mmm has
> doesn't apply cleanly. Both `TrigonalOps` and `TrigonalLowOps`
> accept the `conv` parameter for API uniformity but currently return
> the same strings under both conventions. Revisit if a user reports
> a specific OIM/MTEX label divergence for trigonal phases.

### 5.5 — Caveats to verify before committing

1. `QuatD` and `Matrix3X3D` need to be usable in static-init context.
   Almost certainly fine (POD-ish types with simple math), but worth
   a smoke test.
2. Static-init order within the TU: keep `k_Canonical*` declarations
   ABOVE `k_SymOps_*` declarations in the same `.cpp` file. Within
   a single TU, top-to-bottom order is guaranteed.
3. If `QuatD` has `constexpr` constructors and operators, the whole
   thing can become `constexpr` and the derivation moves to compile
   time. Worth a follow-up; not required for correctness.

---

## 6. The "Convert Hex/Trig Euler Angles Between Cartesian Conventions" filter

This is a separate simplnx filter, not part of LaueOps proper. It
handles the case where the user wants to *export* orientation data
in X‖a* form (e.g. for MTEX import) or *import* X‖a* data into a
DREAM3DNX pipeline that's otherwise running in X‖a.

### 6.1 — Filter spec

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `InputEulerAngles` | DataPath (Float32, 3-component) | — | Source data |
| `InputCrystalStructures` | DataPath (UInt32) | — | Per-phase Laue class indices |
| `InputPhases` | DataPath (Int32) | — | Per-tuple phase index |
| `InputConvention` | dropdown | `XParallelA` | What convention the input data is in |
| `OutputConvention` | dropdown | `XParallelAStar` | What convention to write |
| `OutputEulerAngles` | DataPath (output array name) | `EulerAngles_X||a*` | Default name encodes the convention |

### 6.2 — Algorithm

```
For each tuple i:
  phase = InputPhases[i]
  cs    = InputCrystalStructures[phase]
  if cs is one of {Hexagonal_High, Hexagonal_Low, Trigonal_High, Trigonal_Low}:
    if InputConvention == XParallelA  and OutputConvention == XParallelAStar:
      OutputEulerAngles[i] = (phi1, Phi, phi2 - 30°)   // sign TBD per §7
    else if InputConvention == XParallelAStar and OutputConvention == XParallelA:
      OutputEulerAngles[i] = (phi1, Phi, phi2 + 30°)
    else if InputConvention == OutputConvention:
      OutputEulerAngles[i] = InputEulerAngles[i]      // no-op
  else:
    OutputEulerAngles[i] = InputEulerAngles[i]        // non-hex/trig pass through
```

### 6.3 — Properties to verify in tests

- **Round-trip lossless.** `Convert(A→A*)` then `Convert(A*→A)` returns the original Bunge angles within FP precision.
- **Cubic invariance.** Cubic data passes through unchanged (no shift applied).
- **Phase-wise gating.** Multi-phase scans with mixed cubic + hex/trig only shift the hex/trig points.

### 6.4 — Documentation requirements

The filter doc should:
- Reference the convention infographic at `Docs/x_parallel_a_star_convention.svg`.
- Reference the methodology in `Data/Pole_Figure_Validation/ReadMe.md`.
- State explicitly that this filter operates on Bunge Euler angles only — converting derived parameterizations (Quaternions, Rodrigues, etc.) requires running this filter on the source Bunge first, *then* re-running Convert Orientations.
- Note that the round-trip is lossless, so the user can convert back without losing precision.

---

## 7. The phi2 sign — empirically TBD

The closed-form derivation says `X||a → X||a*` is `phi2 -= 30°`.
Empirical testing during the conversation showed that the
`make_pole_figure.cpp` matrix-path transformation produces output
consistent with `phi2 += 30°` instead. The likely cause is in
EbsdLib's `AxisAngle::toOrientationMatrix()` returning a passive form
for `(z, +90°)` that equals `R_z(-90°)` in active terms, which
inverts the closed-form derivation. The conjugation direction in
the sym op similarity transform may flip correspondingly.

**Action during PR 2**: implement the bridge with `phi2 -= 30°` per
the closed-form derivation, validate against `make_pole_figure`'s
output on a real `.ang` file. If it doesn't match, flip the sign and
the `SymOps::build<XParallelAStar>()` similarity-transform direction
together. This is a 5-line edit; the full architecture is unchanged.

The conversion filter (§6) needs the same empirical confirmation
before being published — just spot-check round-trip and cross-check
against MTEX output once.

---

## 8. Affected simplnx filters

Three categories.

### 8.1 — Filters that gain a `HexConvention` UI parameter

| Filter | Parameter | Default |
|---|---|---|
| `Generate IPF Colors` | `Hex/Trig Convention` | X‖a |
| `Generate IPF Legend` | `Hex/Trig Convention` | X‖a |
| `Write Pole Figure` | `Hex/Trig Convention` | X‖a |
| `Generate Misorientation Colors` (Patala) | `Hex/Trig Convention` | X‖a |

Each filter passes the user's choice into the corresponding LaueOps
method or config struct.

### 8.2 — A new filter

| Filter | Purpose |
|---|---|
| `Convert Hex/Trig Euler Angles Between Cartesian Conventions` | One-shot Bunge-angle conversion for MTEX export / import (§6) |

### 8.3 — Filters that need no change

Everything else. Specifically:

- `Convert Orientations`
- `Find Misorientations` (and Avg / FRM / Boundary variants)
- `Find Average Orientations`
- `Find KAM`
- `Find Schmid Factors`
- `Find Average C-Axis`
- `Find ODF / Texture Components / MDF`

These run convention-agnostic. The numerical outputs are the same as
they always were under v2.

---

## 9. Subtleties to be aware of

### 9.1 — "Compute under X‖a, render under X‖a*" works for direct rendering

A workflow like `ReadAngData → FindAverageOrientation → WritePoleFigure (X‖a* render)`:

- `FindAverageOrientation` produces an orientation that is the same
  *physical* rotation regardless of convention. Its Bunge representation
  in the output array is in X‖a form (since LaueOps internal is X‖a).
- `WritePoleFigure` with X‖a* render configuration takes that
  orientation, applies the convention bridge inside its rendering
  method (transient phi2 shift + X‖a* sym ops + X‖a* direction tables),
  produces the picture.
- Result: the same picture you'd get if every step had been in X‖a*
  — because the rotation is the same physical rotation; only the
  cartesian labeling changes.

### 9.2 — Misorientation axis rendering needs a small bridge

`FindMisorientations` outputs an axis-angle pair. The axis is in
*crystal frame* — and that's the X‖a crystal frame, since that's
EbsdLib's internal default. If a downstream filter wants to render
that axis on an X‖a* IPF/Patala-color map, the rendering filter has
to apply a basal-plane bridge to the axis component before plotting.

Practically: `Generate Misorientation Colors` with `XParallelAStar`
configured needs to apply the convention shift to its input axis
array before invoking the color generation. Worth flagging in the
filter implementation so we don't end up with a 30°-off Patala color
map and assume EbsdLib is broken.

### 9.3 — Cubic / tet / ortho / mono / triclinic ignore the parameter

These have no basal-plane convention. The parameter is accepted on
the API for uniformity but is ignored internally. Filter UIs can
either suppress the parameter widget when the active phase is not
hex/trig, or always show it with a note that it has no effect for
non-hex/trig phases. Probably the latter — easier to implement and
makes the convention story consistent across the UI.

---

## 10. UI labeling guidance

Three signals to help users get this right at a glance:

1. **Default IS what they used to see.** Dropdowns default to
   `X||a (TSL/EDAX, OIM Analysis-compatible)`. Existing pipelines
   reproduce existing output unless the user explicitly changes the
   dropdown.

2. **Opt-in is honestly labeled, not editorialized.** The other
   choice reads `X||a* (Oxford / HKL / MTEX-compatible)`. Not
   "modern" or "correct" — just naming the camps and what they're
   compatible with.

3. **Output PNG embeds the convention.** Same as the new MTEX
   script in `Data/Pole_Figure_Validation/`: stamp the convention
   into the title or filename of the rendered output so the user
   looking at the picture later knows which form they're seeing
   without needing to consult the pipeline JSON.

For the conversion filter, expose two dropdowns (`InputConvention`
and `OutputConvention`) so the direction is explicit and the filter
is reusable both ways.

---

## 11. Why earlier ideas were set aside

Captured here so future-you doesn't re-walk the same paths.

### 11.1 — Marker-based metadata stamping on EulerAngles arrays (rejected)

The idea: stamp an `HexConvention` HDF5 attribute on each EulerAngles
array; readers stamp on import; downstream filters consult the marker;
absence of marker → assume X‖a (legacy guarantee).

Why rejected: simplnx metadata feature is untested; would require
propagation through every orientation-derived filter (Convert
Orientations etc.); user-perceptible value is small given that the
codebase guarantee already tells us legacy data is X‖a.

### 11.2 — Constructor-bound convention on `LaueOps` (rejected)

The idea: every `LaueOps` instance carries the convention; every
method consults `m_InputDataConvention`; compile fails everywhere if
the default is removed.

Why rejected: convention is only relevant for rendering paths; making
every method convention-aware was overkill and forced every caller
to think about the convention even where it has no effect. The
config-struct + per-method-default approach gives a smaller
API-surface change.

### 11.3 — Default = X‖a* (matches MTEX out of the box) (rejected)

The idea: v3 rebranded as "now matches MTEX"; existing users who
want OIM-form output set an explicit override.

Why rejected: would silently rotate every hex/trig PF in every
saved DREAM3DNX pipeline by 30° on upgrade. The production user base
doesn't compare to MTEX; they want PFs that look like what OIM
Analysis produced. Behavior preservation is more valuable than the
"matches MTEX out of the box" headline.

### 11.4 — Per-pipeline-level "convention mode" parameter (rejected)

The idea: a single global pipeline-scoped switch; all filters honor it.

Why rejected: simplnx has no clean place for pipeline-level state;
introducing one for this single concern would set a bad precedent.
Per-filter parameters are more honest — they make the choice visible
at the point where it has effect.

---

## 12. PR sequencing

### PR 1 — Plumbing only

- Add `HexConvention` enum to a shared header.
- Add `hexConvention` field to `PoleFigureConfiguration_t` and
  `CompositePoleFigureConfiguration_t` with default `XParallelA`.
- Add `HexConvention conv = XParallelA` parameter to
  `LaueOps::generateIPFColor` (all overloads),
  `generateIPFTriangleLegend`, `generateRodriguesColor`,
  `getDefaultPoleFigureNames`.
- Inside method bodies: parameter unused for now. No internal
  refactoring of sym op tables. Output stays bit-identical because
  the existing internal X‖a* tables work for the X‖a default the
  same way they did before — wait, they don't. (See note below.)

**Note**: PR 1 has a wrinkle that doesn't exist if we keep v3's X‖a*
internal tables. Currently, EbsdLib renders hex/trig under X‖a*. If
we want PR 1 to be truly bit-identical to current v3 output, the
default needs to be `XParallelAStar` even though the long-term
default is `XParallelA`. Then PR 2 flips the canonical tables to
X‖a, the default to X‖a, and that's where output for legacy data
*starts matching what they had pre-v3*.

So a more accurate sequencing:

### PR 1 (revised) — Plumbing with current-behavior default

- Add `HexConvention` enum.
- Add the parameter to all rendering APIs (config-struct fields and
  method args) with default `XParallelAStar` (matches current v3
  internal behavior).
- No internal change. Output bit-identical to current v3.

### PR 2 — Internal architecture: SymOps struct + per-class dispatch

> **Revised during execution.** The original sketch below assumed
> the canonical hand-maintained tables would be hand-flipped back
> to X‖a (v2-style) values. PR 2e revealed that v2 → v3 was not a
> uniform basis rotation — see §16. The implemented sequencing
> kept canonical = X‖a* (current v3 hand-typed, MTEX-validated) and
> derived X‖a via the conjugation transform.

- For the four hex/trig Ops files: introduce the `SymOps` struct,
  the templated `build<>` factory, the two static instances.
- Replace internal `HexagonalHigh::k_QuatSym[i]` references with
  `sym->quat[i]` where `sym` is picked at the top of each rendering
  method from `config.hexConvention` / the per-method `conv` arg.
- *(Originally planned, dropped per §16:)* Hand-flip the canonical
  tables to X‖a using git-history-recovered v2 values.
- *(Originally planned, dropped per §16:)* Flip
  `getDefaultPoleFigureNames` to return X‖a strings by default.
- Validate: with `XParallelAStar` (current default), output is
  bit-identical to current v3 / matches MTEX. With `XParallelA`,
  the conjugation-derived path produces a self-consistent X‖a
  rendering (basal-plane content rotated 30° about c on the disk).

### PR 3 — Remove default values; force simplnx audit

- Remove the `= XParallelAStar` default from every rendering API
  (config-struct fields and per-method args).
- Every `LaueOps` construction site and every rendering call site
  in simplnx (and any other consumer) becomes a compile error
  until the caller explicitly supplies a `HexConvention`.
- This forces a deliberate, audited choice at every call site
  rather than letting filters quietly inherit a default.
- The eventual long-term default (X‖a, per §2.2) is then enforced
  at the *filter UI* level (PR 4), not at the LaueOps API. The
  LaueOps API itself stays default-free.

### PR 4 — simplnx UI: per-filter dropdowns

- Add `Hex/Trig Convention` dropdown to `Generate IPF Colors`,
  `Generate IPF Legend`, `Write Pole Figure`, `Generate Misorientation Colors`.
- Default = `X‖a (TSL/EDAX)`.
- Plumb the user's choice into the LaueOps call.

### PR 5 — `Convert Hex/Trig Euler Angles Between Cartesian Conventions` filter

- New simplnx filter per §6 spec.
- Tests: round-trip lossless, cubic pass-through, phase-wise gating.

### PR 6 — Documentation

- Update `Data/Pole_Figure_Validation/ReadMe.md` to explain dual
  convention support.
- Update `Docs/x_parallel_a_star_convention.svg` footer to reflect
  default = X‖a, opt-in = X‖a*.
- Filter docs for the four UI-changed filters and the new conversion filter.
- Release notes: "v3 now supports both X‖a (default, OIM-compatible)
  and X‖a* (opt-in, MTEX-compatible) for hex/trig rendering. No
  default behavior change for existing pipelines."

---

## 13. Validation evidence

### 13.1 — `12.ang` test (Titanium Alpha, Hex 6/mmm)

Three rendering paths compared on the same TSL `.ang` input:

| Path | Treats Bunge as | Renders in |
|---|---|---|
| `make_pole_figure` (with in-app `phi2 ± 30°` + matrix path) | X‖a (after the shift) | X‖a / OIM-form |
| DREAM3DNX (current v3, no shift) | X‖a* (misinterpretation of X‖a-stored data) | X‖a* / MTEX-form |
| MTEX (with `phi2 += 30°` in MATLAB after CI filter) | X‖a (via the script's add) | X‖a / OIM-form |

Under the new design (PR 2 onward, default X‖a):

- DREAM3DNX (no special config) → X‖a render → matches `make_pole_figure` → matches OIM Analysis → matches MTEX-with-shift.
- DREAM3DNX with explicit X‖a* config → X‖a* render → matches MTEX-without-shift.

The `12.ang` test set lives at `/Users/Shared/Data/MTR_Data/RR_MTR_Examples/`.
The MTEX-side processing for the comparison is in
`Data/Pole_Figure_Validation/mtex_ang_to_pole_figures.m`.

### 13.2 — Cubic textbook orientations

The position-space validation (1752 buckets, 12 canonical orientations
× 11 Laue classes × 3 plane families, max distance 6×10⁻⁸ vs MTEX) at
`Data/Pole_Figure_Validation/` is the regression guard. It runs under
v3's X‖a* internal default and continues to pass after PR 2 because:

1. The current default convention is still `XParallelAStar`, so the
   regression test is unchanged in its calling convention.
2. The canonical sym op + direction tables are unchanged — they're
   still the v3 hand-typed values. (The implementation chose
   canonical = X‖a*, see §16.) The MTEX-side numbers therefore
   match bit-for-bit what they did before PR 2 landed.

When PR 3 removes the default values, the 1752-bucket harness will
be updated to pass `XParallelAStar` explicitly so the regression
remains apples-to-apples against MTEX.

The X‖a (derived) path is exercised by `LaueOpsTest`'s convention
regression suite (see PR 2b/2d): for each of the four hex/trig Ops
classes, `generateSphereCoordsFromEulers` is invoked under both
conventions with a zero-Euler input, and the X‖a output is asserted
to be `R_z(+30°)` applied to the X‖a* canonical first-family entry.
That gives a self-consistency guard on the conjugation transform
without requiring a second MTEX validation pass.

---

## 14. Related: simplnx `WritePoleFigure` stale-label issue

Independent of the LaueOps work but lives in the same area: the
simplnx `WritePoleFigure` filter takes three user-supplied label
strings (`IntensityPlot1Name` etc.) that are stamped onto the output
PNG but do *not* drive what gets plotted. Pipelines built against
pre-v3 hex/trig defaults have stale strings — the plotted content
matches v3's `getDefaultPoleFigureNames` output but the labels say
something else. Visible to the user as "the {10-10} and {2-1-10}
images appear swapped vs `make_pole_figure`".

After PR 2 lands and the canonical tables are X‖a, the *content*
under `<10-10>` and `<2-1-10>` labels reverts to the legacy v2
positions, so old saved pipelines automatically display correctly
labeled images. Nothing more to do for this case.

For users wanting the X‖a* render, the new `Hex/Trig Convention`
dropdown drives both the rendering and (ideally) the auto-default
labels. Filter UI should auto-populate the three label fields from
`op->getDefaultPoleFigureNames(conv)` when the user changes the
convention dropdown — overrideable, but defaulting to the right
strings.

---

## 15. Open questions

- Confirm `QuatD` and `Matrix3X3D` work in static-init context
  (smoke test during PR 2). **Resolved during PR 2a:** they do —
  the two static `SymOps` instances build cleanly at TU init.
- Confirm `phi2` sign empirically during PR 2 (closed form vs
  empirical-from-`make_pole_figure` discrepancy described in §7).
- Confirm sym op conjugation direction during PR 2 (couples with
  the phi2 sign). **Resolved during PR 2b:** `q_30 * S * q_30Inv`
  (with `q_30 = R_z(+30°)`) maps canonical X‖a* → X‖a; verified
  by the `LaueOpsTest` convention regression suite which checks
  that the derived first-family entry is `R_z(+30°)` applied to
  the canonical first-family entry.
- Decide whether `SymOps` derivation can move to `constexpr`
  (depends on `QuatD`'s API). Optional, follow-up work.
- Decide whether `WritePoleFigure` filter UI auto-defaults the
  three plot labels from `getDefaultPoleFigureNames(conv)` when
  the user changes the convention dropdown. Strong recommendation
  yes.

---

## 16. PR 2e finding: canonical-source-of-truth direction

§5.1 originally proposed making X‖a the canonical hand-maintained
set and deriving X‖a* via the templated `SymOps::build<>` factory.
The reasoning was: X‖a is the legacy v2 form, the v2 hand-typed
sym op tables are recoverable from git history, and the templated
factory then generates the X‖a* tables algorithmically by
`q_30 * S * q_30Inv` with matching `R_z(+30°)` rotation of the
direction-family lists.

**PR 2e investigated this hand-flip and chose the opposite
direction.** Canonical = X‖a* (current v3 hand-typed values),
derived = X‖a (via the conjugation transform). This subsection
captures why.

### 16.1 — What was discovered

Before hand-flipping, the natural sanity check is: does
`R_z(-30°)` applied member-by-member to the v3 X‖a* sym op /
direction tables actually reproduce the v2 X‖a values pulled
from git history?

It does not. F1 (the {10-10}-style family) and F2 (the
{2-1-10}-style family) shift by *different* signs of 30° between
v2 and v3 in the hand-typed tables — i.e. the v2 → v3 transition
was not a uniform 30°-about-c rotation of the entire table. The
v3 author chose different orbit members as the "first" entry per
family than the v2 author did. The sym op table itself (the
twelve quaternions describing the rotation group) shows similar
per-entry mismatch when compared by index.

### 16.2 — Why this isn't a bug, and why hand-flipping doesn't help

The user supplied the missing context: the sym op ordering used
across the EbsdLib hex/trig classes originates from the
**EMsoftOO** project. The order was hand-derived for
**loop-efficiency** in EMsoftOO's inner loops, not to encode any
mathematical relationship between consecutive entries. Two
authors writing the "same" sym op table can legitimately ship
different orderings, and they will disagree by index even when
the orbits they describe are physically identical.

The same is true of the per-family direction lists: a hex 6/mmm
{10-10} family has six members (three unique up to the inversion
center). Picking which one to call "first" is a stylistic choice;
the orbit is complete and physically correct either way.

So the v3 hand-typed tables do not encode a different physical
group from the v2 tables — they encode the *same* rotation group
and the *same* plane-normal orbits, just enumerated in a different
order. Hand-flipping v3 X‖a* to v2 X‖a values would not produce a
"more correct" library; it would produce a stylistically v2-looking
library at the cost of:

- breaking the existing `LaueOpsTest::*Test` regression baselines
  that have been validated against MTEX bucket-position output
  (see §13.2 — the 1752-bucket cross-check at
  `Data/Pole_Figure_Validation/`),
- forcing every internal use of `k_QuatSym[i]` to be re-blessed
  against the new ordering, and
- introducing a numerical-output diff in the X‖a* derived path
  that's purely an enumeration artifact, not a physics change.

### 16.3 — Decision: keep canonical = X‖a*

PR 2a–2d landed with canonical = X‖a* and derived = X‖a, on the
reasoning that:

1. The X‖a* canonical tables are the ones validated by the
   1752-bucket MTEX regression. They are known-good.
2. The conjugation transform `q_30 * S * q_30Inv` plus
   `R_z(+30°) · d` for direction tables is a closed-form, easily
   audited derivation. The X‖a side is correct *by construction*
   from the validated X‖a* side.
3. The eventual X‖a output (via the derived path) lands on the
   same physical orbits as v2/OIM-Analysis, just not necessarily
   the same per-index numerical values. Because all rendering
   methods iterate the full orbit, the visible PF / IPF output is
   indistinguishable from v2 output. The only difference is which
   orbit member happens to be `sym->dirsFamily1[0]` internally,
   and that's not user-visible.

PR 2e is therefore a **documentation-only commit**: this
subsection plus §5.1 / §12 cross-references, plus comment
tightening in `HexagonalOps.cpp` / `HexagonalLowOps.cpp` /
`TrigonalOps.cpp` / `TrigonalLowOps.cpp` to reflect the chosen
direction.

### 16.4 — What this means for downstream work

- **PR 3 (default removal):** unchanged. Strip the
  `= XParallelAStar` defaults from every rendering API; force
  every caller to make the choice explicitly.
- **simplnx UI default (PR 4):** still X‖a, per §2.2. The fact
  that EbsdLib's *internal* canonical happens to be X‖a* doesn't
  change the user-facing default. simplnx filters pass
  `XParallelA` explicitly to LaueOps, which then does its
  conjugation-derive-on-the-fly via the X‖a static instance, and
  produces the legacy-OIM-style PF.
- **MTEX validation harness:** continues to use `XParallelAStar`
  explicitly, comparing against the canonical (not derived) path.
  No regression update needed.
- **v2-style auditing:** if a future user reports "the X‖a output
  doesn't match what I had in v2 at index N of the sym op table",
  the answer is "the orbit is the same, the indexing differs by
  EMsoftOO loop ordering — compare by orbit membership, not by
  index". Document this in the filter help text alongside the
  PF-rendering workflow.

### 16.5 — Lesson for future Phase-N convention work

When introducing a second convention to a Laue-class library,
**don't assume** that two existing hand-typed tables related by
"the same" basis rotation will agree member-by-member after
applying that rotation. Per-entry hand-typing inherits the
author's enumeration choice. The right validation is *orbit
equality*, not *table equality*.

---

## Cross-references

- `v3_stabilization_plan.md` — Phase 0 milestones, risk register, release notes.
- `Data/Pole_Figure_Validation/ReadMe.md` — convention background, validation methodology.
- `Docs/x_parallel_a_star_convention.svg` — geometric picture of X‖a vs X‖a*.
- `Source/Apps/make_pole_figure.cpp` — current reference for the
  in-app convention shift; PR 2 moves this logic into LaueOps.
- `Source/EbsdLib/Utilities/PoleFigureUtilities.h` — location of
  `PoleFigureConfiguration_t` to extend.
- `Source/EbsdLib/Utilities/PoleFigureCompositor.h` — location of
  `CompositePoleFigureConfiguration_t` to extend.
- `Source/EbsdLib/LaueOps/LaueOps.h` — base-class API to extend.
