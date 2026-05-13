# EbsdLib v3 API Reference (LaueOps + supporting types)

This is a hand-written reference for the public LaueOps surface as of
v3.0.0, focused on the symbols that changed in v3. The full Doxygen
header documentation is the source of truth for the per-parameter
contracts; this file gives the high-level picture and the cross-references
between symbols.

For the release-notes view (what changed, why, and migration recipes), see
[`Index.md`](Index.md).

---

## Convention and color-key enums

```cpp
#include "EbsdLib/Core/EbsdLibConstants.h"
```

### `ebsdlib::HexConvention`

```cpp
enum class HexConvention : uint8_t
{
  XParallelA = 0,
  XParallelAStar = 1,
  NotApplicable = 2
};
```

Identifies which Cartesian basis convention a hex / trig direction table is
expressed in. Three values, three audiences:

| Value | Used by | Meaning |
| ----- | ------- | ------- |
| `XParallelA` | EDAX / TSL / DREAM3D-legacy | Cartesian X is parallel to the lattice `a` vector. Older `.ang` files arrive in this basis. |
| `XParallelAStar` | MTEX / Oxford HKL / EbsdLib internal canonical | Cartesian X is parallel to the reciprocal `a*` vector (perpendicular to `a` in the basal plane, 30° rotated). |
| `NotApplicable` | Cubic, tetragonal, orthorhombic, monoclinic, triclinic | The hex/trig basis distinction doesn't apply. Pass this when the LaueOps in hand is non-hex/trig. The non-hex/trig overrides ignore the parameter; the hex/trig overrides assert if they receive it. |

Position-space validation across both bases lives in
`Data/Pole_Figure_Validation/`; the geometric picture is in
`x_parallel_a_star_convention.svg`.

### `ebsdlib::ColorKeyKind`

```cpp
enum class ColorKeyKind : uint8_t
{
  TSL = 0,
  PUCM = 1,
  NolzeHielscher = 2
};
```

Identifies which IPF coloring scheme `generateIPFColor` /
`generateIPFTriangleLegend` should use.

| Value | Reference | Notes |
| ----- | --------- | ----- |
| `TSL` | EDAX / DREAM3D-legacy | The default. Discontinuous on SST boundaries; produces the familiar red-RD / green-TD / blue-ND IPF map for cubic. |
| `PUCM` | Nolze, 2016 (perceptually uniform color mapping) | Smooth on SST boundaries; flat-shaded SST interior. Init is `std::call_once`-guarded — instances of LaueOps remain stateless and safe to share across threads. |
| `NolzeHielscher` | Nolze + Hielscher | Alternative perceptually-uniform scheme. |

Each LaueOps subclass owns a per-class singleton for each kind, lazy-initialized inside a file-local `keyForKind()` helper. The base class never sees the singletons. Callers pick a key by passing the kind enum at the call site; there's no setter API.

---

## `LaueOps` public surface that changed in v3

```cpp
#include "EbsdLib/LaueOps/LaueOps.h"
```

### IPF coloring (per-pixel)

```cpp
virtual Rgb generateIPFColor(
    double*  eulers,
    double*  refDir,
    bool     convertDegrees,
    ebsdlib::ColorKeyKind kind = ebsdlib::ColorKeyKind::TSL) const = 0;

virtual Rgb generateIPFColor(
    double e0, double e1, double e2,
    double dir0, double dir1, double dir2,
    bool   convertDegrees,
    ebsdlib::ColorKeyKind kind = ebsdlib::ColorKeyKind::TSL) const = 0;
```

- Convention-invariant. No `HexConvention` parameter — the function reduces a sample-frame reference direction into the SST and queries the color key, neither step crosses the basal basis.
- `convertDegrees=true` interprets `eulers` as degrees; `false` as radians.
- Pre-v3 3-arg callers compile unchanged and get TSL coloring.

### IPF coloring (helper, now public)

```cpp
Rgb computeIPFColor(
    double* eulers,
    double* refDir,
    bool    degToRad,
    const ebsdlib::IColorKey* key) const;
```

The FZ symmetry-reduction loop that every Laue class runs before
delegating to a color key. Public in v3 so that per-class
`CreateIPFLegend` renderers can call it directly with a (possibly
gridded-wrapped) key, without going through the kind-enum dispatch. A
null `key` falls back to a built-in TSL coloring.

### Rodrigues-space coloring

```cpp
virtual Rgb generateRodriguesColor(double r1, double r2, double r3) const = 0;
```

3-arg form. Convention-invariant for the same reason `generateIPFColor`
is — operates on a crystal-space Rodrigues vector, no basal basis is
crossed.

### IPF triangle legend

```cpp
virtual UInt8ArrayType::Pointer generateIPFTriangleLegend(
    int  imageDim,
    bool generateEntirePlane,
    ebsdlib::HexConvention conv,
    ebsdlib::ColorKeyKind  kind = ebsdlib::ColorKeyKind::TSL,
    bool gridded = false) const = 0;
```

- `imageDim` — square canvas size in pixels.
- `generateEntirePlane=true` renders the full unit circle; `false` renders just the SST (the standard view).
- `conv` — no default on the base virtual. Hex/trig overrides honor it (legend labels and basal-plane direction tables shuffle); non-hex/trig overrides ignore it.
- `kind` — picks the per-class color key. Defaults to TSL.
- `gridded=true` wraps the selected key in a `GriddedColorKey` (~1° eta × chi resolution) to produce MTEX-style flat-shaded cells. Only meaningful for legends.

Output is cropped to the SST contents plus a small margin.

### Pole figure sphere coords

```cpp
virtual void generateSphereCoordsFromEulers(
    FloatArrayType* eulers,
    FloatArrayType* c1,
    FloatArrayType* c2,
    FloatArrayType* c3,
    ebsdlib::HexConvention conv) const = 0;
```

- `c1`, `c2`, `c3` receive one normalized direction per orientation per symmetry-equivalent pole, for plane families 0, 1, 2 respectively.
- For cubic/tet/ortho/mono/triclinic, the three families are convention-invariant and `conv` is ignored.
- For hex 6/mmm and 6/m, `conv` selects whether plane families 1 and 2 are the `X‖a*` basis (`<10-10>` and `<11-20>`) or the `X‖a` basis (the same families rotated 30°).

### Default pole-figure family labels

```cpp
virtual std::array<std::string, 3> getDefaultPoleFigureNames(
    ebsdlib::HexConvention conv) const = 0;
```

Returns the labels for the three default plane families EbsdLib renders.
Hex/trig classes return *different* labels per convention; non-hex/trig
classes return the same labels regardless. Always paired with a matching
call to `generateSphereCoordsFromEulers(..., conv)` so the labels and the
data agree.

### Misorientation coloring (unchanged signature)

```cpp
virtual Rgb generateMisorientationColor(const QuatD& q, const QuatD& refFrame) const;
```

Surface unchanged; renderer internals were touched as part of the v3
SymOps refactor.

---

## Configuration structs that gained a `HexConvention` field

```cpp
#include "EbsdLib/IO/PoleFigureConfiguration.h"
```

- `ebsdlib::PoleFigureConfiguration_t::hexConvention` — picked up by
  `LaueOps::generatePoleFigure()` and by the standalone family-emission
  renderer.
- `ebsdlib::CompositePoleFigureConfiguration_t::hexConvention` — picked
  up by `PoleFigureCompositor::generateCompositeImage()`. The simplnx
  `WritePoleFigureFilter` learned to set this in v3.0; pre-v3 it was
  silently default-constructed (PR 2g, commit `9395592`).
- `ebsdlib::InversePoleFigureConfiguration_t::hexConvention` — added in
  PR 2k.

All three default to `XParallelAStar`. Callers that don't care should
leave the default; the simplnx side exposes the choice as the
`hex_convention_index` UI parameter.

---

## Per-class singleton pattern (file-local)

Inside each `LaueOps` subclass `.cpp`, there's an anonymous-namespace helper
keyed to that class's rotation point group and fundamental sector. The
CubicOps shape (rotation point group `432`, cubicHigh fundamental sector):

```cpp
ebsdlib::IColorKey::Pointer keyForKind(ebsdlib::ColorKeyKind kind)
{
  static const auto k_TSL  = std::make_shared<ebsdlib::TSLColorKey>();
  static const auto k_PUCM = std::make_shared<ebsdlib::PUCMColorKey>("432");
  static const auto k_NH   = std::make_shared<ebsdlib::NolzeHielscherColorKey>(
      ebsdlib::FundamentalSectorGeometry::cubicHigh());
  switch (kind)
  {
    case ebsdlib::ColorKeyKind::PUCM:           return k_PUCM;
    case ebsdlib::ColorKeyKind::NolzeHielscher: return k_NH;
    case ebsdlib::ColorKeyKind::TSL:            break;
  }
  return k_TSL;
}
```

The pattern is private to each subclass — the base class doesn't see it —
but listed here so users understand *why* LaueOps instances are stateless:
the kind enum routes to a `static` singleton at dispatch time, not to an
instance member. PUCM's wlenthe lookup table is built once per process
under `std::call_once`, fixing the thread race we hit before v3.

---

## Related but unchanged in v3

These are listed for completeness — they remain stable across the v3
transition:

- `LaueOps::getQuatSymOp`, `getRodSymOp`, `getMatSymOpF/D` — sym op accessors.
- `LaueOps::getRodSymOp / getQuatSymOp` count: changed for several
  low-symmetry classes (PR 2a–2d), but the accessor signatures are
  identical. See the breaking-changes note in `Index.md` about renderer
  byte-identity.
- `LaueOps::calculateMisorientation`, `getNearestQuat`, `getFZQuat`,
  `getODFFZRod`, `getMDFFZRod` — orientation-math accessors unchanged.
- `LaueOps::getSchmidFactorAndSS`, `getmPrime`, `getF1`, `getF1spt`,
  `getF7` — slip-system and bicrystal mechanics accessors unchanged.
