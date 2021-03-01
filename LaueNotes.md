# Laue Classes Notes #

There are several LaueOps classes that have incomplete implementations. This document attempts to figure that out as it relates to Synthetic Microstructure generation.

## Creating the ODF Plot ##

### Texture::CalculateODFData() ###

| | Triclinic | Monoclinic | OrthoRhombic | Tetragonal_Low | Tetragonal_High | Trigonal_Low | Trigonal_High | Hexagonal_Low | Hexagonal_Low | Cubic_Low | Cubic_High |
|---|-----------|------------|--------------|----------------|-----------------|--------------|---------------|---------------|---------------|-----------|----------|
| getODFFZRod() | O | O | O | O | O | O | O | O | O | O  | O |
| getOdfBin()  | O | O | O | O | O | O | O | O | O | O  | O |

### StatsGen::GenODFPlotData() ###

| | Triclinic | Monoclinic | OrthoRhombic | Tetragonal_Low | Tetragonal_High | Trigonal_Low | Trigonal_High | Hexagonal_Low | Hexagonal_Low | Cubic_Low | Cubic_High |
|---|-----------|------------|--------------|----------------|-----------------|--------------|---------------|---------------|---------------|-----------|----------|
| determineEulerAngles() | O | O | O | O | O | O | O | O | O | O  | O |

## Creating the MDF Plot ##

### Texture::CalculateMDFData() ###

| | Triclinic | Monoclinic | OrthoRhombic | Tetragonal_Low | Tetragonal_High | Trigonal_Low | Trigonal_High | Hexagonal_Low | Hexagonal_Low | Cubic_Low | Cubic_High |
|---|-----------|------------|--------------|----------------|-----------------|--------------|---------------|---------------|---------------|-----------|----------|
| getMDFFZRod() | X | X | X | O | O | O | O | O | O | O  | O |
| getMisoBin()  | O | O | O | O | O | O | O | O | O | O  | O |
| determineEulerAngles() | O | O | O | O | O | O | O | O | O | O  | O |
| calculateMisorientation()| O | O | O | O | O | O | O | O | O | O  | O |

### StatsGen::GenMDFPlotData() ###

| | Triclinic | Monoclinic | OrthoRhombic | Tetragonal_Low | Tetragonal_High | Trigonal_Low | Trigonal_High | Hexagonal_Low | Hexagonal_Low | Cubic_Low | Cubic_High |
|---|-----------|------------|--------------|----------------|-----------------|--------------|---------------|---------------|---------------|-----------|----------|
| determineRodriguesVector() | O | O | O | O | O | O | O | O | O | O  | O |
| getMDFFZRod() | X | X | X | O | O | O | O | O | O | O  | O |

## MatchCrystallography ##

| | Triclinic | Monoclinic | OrthoRhombic | Tetragonal_Low | Tetragonal_High | Trigonal_Low | Trigonal_High | Hexagonal_Low | Hexagonal_Low | Cubic_Low | Cubic_High |
|---|-----------|------------|--------------|----------------|-----------------|--------------|---------------|---------------|---------------|-----------|----------|
| getMisoBin()  | O | O | O | O | O | O | O | O | O | O  | O |
| calculateMisorientation()| O | O | O | O | O | O | O | O | O | O  | O |
| getOdfBin()  | O | O | O | O | O | O | O | O | O | O  | O |
| randomizeEulerAngles()  | O | O | O | O | O | O | O | O | O | O  | O |
| determineEulerAngles() | O | O | O | O | O | O | O | O | O | O  | O |
