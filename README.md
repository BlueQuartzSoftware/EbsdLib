# EbsdLib #

EBSDLib is a C++ Library that can read EBSD Files from OEMs and perform basic EBSD processing such as orientation conversion and IPF Color generation. Another important aspect of the 
library is to be able to convert between the seven orientation representations that
are typically used through out materials science and engineering domains.

The DREAM.3D project ( [code repository](http://www.github.com/bluequartzsoftware/DREAM3D) and [web site](http://dream3d.bluequartz.net) ) uses this library for all of the EBSD processing.

## Supported EBSD OEM Data Files ##

+ EDAX/AMETEK: .ang and HDF5 based file formats
+ Oxford Instruments: .ctf file format
+ Bruker: HDF5 based file format

Please have a look at the unit tests for examples on using the various readers.

## Orientation Representations ##

| From/To |  Euler  |  Orientation Matrix   |  Axis Angle  |  Rodrigues   |  Quaternion   |  Homochoric  |  Cubochoric  |
| ------- |-------| ------- |------- |------- |-------  |-------  |------- |
|Euler|  -   |  X   |  X   |  X   |  X   |  a   | ah   |
|Orientation Matrix|  X   |  --  |  X   |  e   |  X   |  a   | ah   |
|Axis Angle|  o   |  X   | --   |  X   |  X   |  X   |  h   |
|Rodrigues|  o   |  a   |  X   | --   |  a   |  X   |  h   |
|Quaternion|  X   |  X   |  X   |  X   | --   |  X   |  h   |
|Homochoric|  ao  |  a   |  X   |  a   |  a   | --   |  X   |
|Cubochoric| hao  |  ha  |  h   |  ha  | ha   |  X   | --   |

**LEGEND**: X = Direct mathematical conversion between the representations
lower case letters denote the conversion uses other more basic conversions. For
example to go from Euler->Homochoric the conversion process calls the Euler->AxisAngle->OrientationMatrix->Homochoric functions.

In addition to the OrientationTransformation class there are also classes that represent
the 11 Laue classes that allow a user to perform Laue class specific calculations
including the generation of an IPF Color which is a prevalent visualization scheme within
the EBSD community. Note that each vendor has slightly different algorithms and this
library has selected to aligh with the AMETEK/EDAX output.

The folder Data/IPF_Legend has premade IPF Legends for most of the Laue classes.

## Quaternion Convention ##

Please also note that by default EbsdLib organizes Quaternions as Vector-Scalar. If your quaternions
are layed out as Scalar-Vector there is an extra argument to some functions that you
can set to allow the orientation transformations to accept this layout.

## Dependent Libraries ##

EbsdLib is dependent on:

+ Qt5 5.12.x (minimum)
+ Eigen 3.5
+ HDF5 1.8.20 or 1.10.4 (HDF5 is optional only if you want the HDF5 functionality)

## Rotation Convention ##

By convention this library uses **Passive** rotations

## Citations ##

D Rowenhorst, A D Rollett, G S Rohrer, M Groeber, M Jackson, P J Konijnenberg and M De Graef  _et al_ 2015 _Modelling Simul. Mater. Sci. Eng._ **23** 083501

[DOI: https://doi.org/10.1088/0965-0393/23/8/083501](https://doi.org/10.1088/0965-0393/23/8/083501)

## Examples ##

If you want to transform an Euler angle into a Quaternion the following works:

        Quaternion<float> quat = OrientationTransformation::eu2qu(Orientation<float>(33.0f, 10.0f, 0.0f));

If you have a *lot* of angles to transform the **Orienation** class can wrap a pointer instead at which point
you can loop over the array of angles. There is also the **OrientationConverter** class that can 
mass transform from one representation into another.

Reading from an AMETEK .ang file is straightforward:

    AngReader reader;
    reader.setFileName(std::string("/path/to/ebsd_scan.ang");
    int32_t err = reader.readFile();
    // All of the data from the .ang file is now in memory. You can access it through the pointers
    size_t numElements = reader.getNumberOfElements();
    float* ptr = reader.getPhi1Pointer();
    // The reader will clean up the memory so either tell the reader to Not clean up the pointer or keep the reader in scope.

