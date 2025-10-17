# EbsdLib #

EBSDLib is a C++ Library that can read EBSD Files from OEMs and perform basic EBSD processing such as orientation conversion and IPF Color generation. Another important aspect of the 
library is to be able to convert between the seven orientation representations that
are typically used through out materials science and engineering domains.

The [DREAM.3D](https://dream3d.bluequartz.net) project and [DREAM3D-NX](https://www.dream3d.io) uses this library for all the EBSD processing.

## Supported EBSD OEM Data FilesDirectionalStatsTest

+ EDAX/AMETEK: .ang and HDF5 based file formats
+ Oxford Instruments: .ctf and .h5oina file formats
+ Bruker: HDF5 based file format

Please have a look at the unit tests for examples on using the various readers.

## Crystallographic Classes
|  # | Point Group (H–M) | Rotation Point Group | Space Group No(s). | Schoenflies   | Crystal system | Laue class  | Laue Ops         |
|---:|-------------------|----------------------|--------------------|---------------|----------------|-------------|------------------|
|  1 | 1                 | 1                    | 1                  | C₁            | Triclinic      | (\bar{1})   | TriclinicOps     |
|  2 | (\bar{1})         | 1                    | 2                  | C(_i)         | Triclinic      | (\bar{1})   |                  |
|  3 | 2                 | 2                    | 3–5                | C₂            | Monoclinic     | 2/m         |                  |
|  4 | m                 | 1                    | 6–9                | C(_s)         | Monoclinic     | 2/m         |                  |
|  5 | 2/m               | 2                    | 10–15              | C(_{2h})      | Monoclinic     | 2/m         | MonoclinicOps    |
|  6 | 222               | 222                  | 16–24              | D₂            | Orthorhombic   | mmm         |                  |
|  7 | mm2               | 2                    | 25–46              | C(_{2v})      | Orthorhombic   | mmm         |                  |
|  8 | mmm               | 222                  | 47–74              | D(_{2h})      | Orthorhombic   | mmm         | OrthorhombicOps  |
|  9 | 4                 | 4                    | 75–80              | C₄            | Tetragonal     | 4/m         |                  |
| 10 | (\bar{4})         | 2                    | 81–82              | S₄            | Tetragonal     | 4/m         |                  |
| 11 | 4/m               | 4                    | 83–88              | C(_{4h})      | Tetragonal     | 4/m         | TetragonalLowOps |
| 12 | 422               | 422                  | 89–98              | D₄            | Tetragonal     | 4/mmm       |                  |
| 13 | 4mm               | 4                    | 99–110             | C(_{4v})      | Tetragonal     | 4/mmm       |                  |
| 14 | (\bar{4}2m)       | 222                  | 111–122            | D(_{2d})      | Tetragonal     | 4/mmm       |                  |
| 15 | 4/mmm             | 422                  | 123–142            | D(_{4h})      | Tetragonal     | 4/mmm       | TetragonalOps    |
| 16 | 3                 | 3                    | 143–146            | C₃            | Trigonal       | (\bar{3})   |                  |
| 17 | (\bar{3})         | 3                    | 147–148            | C(_{3i}) (S₆) | Trigonal       | (\bar{3})   | TrigonalLowOps   |
| 18 | 32                | 32                   | 149–155            | D₃            | Trigonal       | (\bar{3}m)  |                  |
| 19 | 3m                | 3                    | 156–161            | C(_{3v})      | Trigonal       | (\bar{3}m)  |                  |
| 20 | (\bar{3}m)        | 32                   | 162–167            | D(_{3d})      | Trigonal       | (\bar{3}m)  | TrigonalOps      |
| 21 | 6                 | 6                    | 168–173            | C₆            | Hexagonal      | 6/m         |                  |
| 22 | (\bar{6})         | 3                    | 174                | C(_{3h})      | Hexagonal      | 6/m         |                  |
| 23 | 6/m               | 6                    | 175–176            | C(_{6h})      | Hexagonal      | 6/m         | HexagonalLowOps  |
| 24 | 622               | 622                  | 177–182            | D₆            | Hexagonal      | 6/mmm       |                  |
| 25 | 6mm               | 6                    | 183–186            | C(_{6v})      | Hexagonal      | 6/mmm       |                  |
| 26 | (\bar{6}m2)       | 32                   | 187–190            | D(_{3h})      | Hexagonal      | 6/mmm       |                  |
| 27 | 6/mmm             | 622                  | 191–194            | D(_{6h})      | Hexagonal      | 6/mmm       | HexagonalOps     |
| 28 | 23                | 23                   | 195–199            | T             | Cubic          | m(\bar{3})  |                  |
| 29 | m(\bar{3})        | 23                   | 200–206            | T(_h)         | Cubic          | m(\bar{3})  | CubicLowOps      |
| 30 | 432               | 432                  | 207–214            | O             | Cubic          | m(\bar{3})m |                  |
| 31 | (\bar{4}3m)       | 23                   | 215–220            | T(_d)         | Cubic          | m(\bar{3})m |                  |
| 32 | m(\bar{3})m       | 432                  | 221–230            | O(_h)         | Cubic          | m(\bar{3})m | CubicOps         |


## Orientation TransformationsDirectionalStatsTest

| From/To            | Euler | Orientation Matrix | Axis Angle | Rodrigues | Quaternion | Homochoric | Cubochoric | Stereographic |
|--------------------|-------|--------------------|------------|-----------|------------|------------|------------|---------------|
| Euler              | -     | X                  | X          | X         | X          | a          | ah         | q             |
| Orientation Matrix | X     | --                 | X          | e         | X          | a          | ah         | q             |
| Axis Angle         | o     | X                  | --         | X         | X          | X          | h          | q             |
| Rodrigues          | o     | a                  | X          | --        | a          | X          | h          | q             |
| Quaternion         | X     | X                  | X          | X         | --         | X          | h          | q             |
| Homochoric         | ao    | a                  | X          | a         | a          | --         | X          | q             |
| Cubochoric         | hao   | ha                 | h          | ha        | ha         | X          | --         | q             |
| Stereographic      | a     | a                  | X          | X         | a          | X          | hc         | --            |

**LEGEND**: X = Direct mathematical conversion between the representations
lower case letters denote the conversion uses other more basic conversions. For
example to go from Euler->Homochoric the conversion process calls the Euler->AxisAngle->OrientationMatrix->Homochoric functions.

In addition to the OrientationTransformation class there are also classes that represent
the 11 Laue classes that allow a user to perform Laue class specific calculations
including the generation of an IPF Color which is a prevalent visualization scheme within
the EBSD community. Note that each vendor has slightly different algorithms and this
library has selected to align with the AMETEK/EDAX output.

The folder Data/IPF_Legend has premade IPF Legends for all the Laue classes.

## Quaternion ConventionDirectionalStatsTest

Please also note that by default EbsdLib organizes Quaternions as Vector-Scalar (X,Y,Z,W). If your quaternions
are laid out as Scalar-Vector (w,x,y,z) there is an extra argument to some functions that you
can set to allow the orientation transformations to accept this layout.

## Dependent LibrariesDirectionalStatsTest

EbsdLib is dependent on:

+ Eigen 3.4

## Optional Libraries

+ HDF5 1.10.4 (HDF5 is optional only if you want the HDF5 functionality)
+ Qt5 5.15.x (minimum: Optional)

## Rotation ConventionDirectionalStatsTest

By convention this library uses **Passive** rotations

## CitationsDirectionalStatsTest

D Rowenhorst, A D Rollett, G S Rohrer, M Groeber, M Jackson, P J Konijnenberg and M De Graef  _et al_ 2015 _Modelling Simul. Mater. Sci. Eng._ **23** 083501

[DOI: https://doi.org/10.1088/0965-0393/23/8/083501](https://doi.org/10.1088/0965-0393/23/8/083501)

## ExamplesDirectionalStatsTest

If you want to transform an Euler angle into a Quaternion the following works:

        Quaternion<float> quat = OrientationTransformation::eu2qu(Orientation<float>(33.0f, 10.0f, 0.0f));

If you have a *lot* of angles to transform the **Orienation** class can wrap a pointer instead at which point
you can loop over the array of angles. There is also the **OrientationConverter** class that can 
mass transform from one representation into another.

Reading from an AMETEK .ang file is straightforward:

    AngReader reader;
    reader.setFileName(std::string("/path/to/ebsd_scan.ang"));
    int32_t err = reader.readFile();
    // All of the data from the .ang file is now in memory. You can access it through the pointers
    size_t numElements = reader.getNumberOfElements();
    float* ptr = reader.getPhi1Pointer();
    // The reader will clean up the memory so either tell the reader to Not clean up the pointer or keep the reader in scope.

