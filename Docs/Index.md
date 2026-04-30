# Various Bits of Documentation for EbsdLib

EbsdLib is primarily used in the [DREAM3D](https://www.dream3d.io) family of applications and libraries.

## Rotation Point Groups

The PDF is courtesy of Dr. Anthony Rollett from Carnegie Mellon University. The original
URL is [http://pajarito.materials.cmu.edu/lectures/L3-OD_symmetry-21Jan16-slide_50-operators.pdf](http://pajarito.materials.cmu.edu/lectures/L3-OD_symmetry-21Jan16-slide_50-operators.pdf)

## Hexagonal Cartesian Conventions: X‖a vs X‖a*

EbsdLib v3 aligned its hexagonal and trigonal direction conventions to `X‖a*`, matching
MTEX and Oxford Instruments / HKL acquisition systems. (EDAX/TSL/OIM Analysis use the
other convention, `X‖a`.) The 30° rotation between the two conventions is what caused
the original `(10-10)` and `(2-1-10)` pole-figure mismatches before the v3 changes.

![X parallel a-star convention](x_parallel_a_star_convention.svg)

Position-space validation across all 11 Laue classes lives in
[`Data/Pole_Figure_Validation/`](../Data/Pole_Figure_Validation/ReadMe.md).
