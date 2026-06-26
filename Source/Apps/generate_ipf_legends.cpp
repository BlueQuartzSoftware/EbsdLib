
#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/LaueOps/CubicLowOps.h"
#include "EbsdLib/LaueOps/CubicOps.h"
#include "EbsdLib/LaueOps/HexagonalLowOps.h"
#include "EbsdLib/LaueOps/HexagonalOps.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/LaueOps/MonoclinicOps.h"
#include "EbsdLib/LaueOps/OrthoRhombicOps.h"
#include "EbsdLib/LaueOps/TetragonalLowOps.h"
#include "EbsdLib/LaueOps/TetragonalOps.h"
#include "EbsdLib/LaueOps/TriclinicOps.h"
#include "EbsdLib/LaueOps/TrigonalLowOps.h"
#include "EbsdLib/LaueOps/TrigonalOps.h"
#include "EbsdLib/Math/Matrix3X1.hpp"
#include "EbsdLib/OrientationMath/OrientationConverter.hpp"
#include "EbsdLib/Utilities/CanvasUtilities.hpp"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"
#include "EbsdLib/Utilities/GriddedColorKey.hpp"
#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"
#include "EbsdLib/Utilities/PngWriter.h"
#include "EbsdLib/Utilities/TSLColorKey.hpp"

#include "EbsdLib/Apps/EbsdLibFileLocations.h"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace ebsdlib;

// const std::string k_Output_Dir(ebsdlib::unit_test::DataDir + "/IPF_Legend/");
const std::string k_Output_Dir(ebsdlib::unit_test::k_TestTempDir + "/IPF_Legend/");

// Per-rotation-point-group SST crop region (as fractions of imageDim). The
// per-class TSL smooth emission blocks below crop their imageDim x imageDim
// canvases to tighter rectangles that fit the SST shape (e.g. hex 6/mmm gets
// a wide-flat kite at 0.80 x 0.50). These same fractions are used by the
// NH/PUCM emission loop above so PUCM/NH legend pixel dimensions match the
// TSL legends class-by-class.
//
// Classes that omit a crop (432 cubic m-3m, 23 cubic m-3, 1 triclinic) have
// SSTs whose bounding rectangle already fills the imageDim x imageDim canvas
// at the layout the renderer uses, so no crop is necessary.
struct SstCropRect
{
  float xFrac;
  float yFrac;
  float wFrac;
  float hFrac;
};

bool sstSmoothCropFor(const std::string& rotationPointGroup, SstCropRect& out)
{
  if(rotationPointGroup == "622") // Hexagonal 6/mmm
  {
    out = {0.10F, 0.00F, 0.80F, 0.50F};
    return true;
  }
  if(rotationPointGroup == "6") // Hexagonal 6/m
  {
    out = {0.10F, 0.00F, 0.70F, 0.50F};
    return true;
  }
  if(rotationPointGroup == "32") // Trigonal -3m
  {
    out = {0.05F, 0.00F, 0.75F, 0.65F};
    return true;
  }
  if(rotationPointGroup == "3") // Trigonal -3
  {
    out = {0.00F, 0.00F, 0.90F, 0.65F};
    return true;
  }
  if(rotationPointGroup == "422") // Tetragonal 4/mmm
  {
    out = {0.10F, 0.00F, 0.78F, 0.60F};
    return true;
  }
  if(rotationPointGroup == "4") // Tetragonal 4/m
  {
    out = {0.10F, 0.00F, 0.70F, 0.60F};
    return true;
  }
  if(rotationPointGroup == "222") // OrthoRhombic mmm
  {
    out = {0.10F, 0.00F, 0.78F, 0.60F};
    return true;
  }
  if(rotationPointGroup == "2") // Monoclinic 2/m
  {
    out = {0.00F, 0.00F, 1.00F, 0.60F};
    return true;
  }
  // 432, 23, 1: no crop
  return false;
}

using EbsdDoubleArrayType = EbsdDataArray<float>;
using EbsdDoubleArrayPointerType = EbsdDoubleArrayType::Pointer;
using OCType = OrientationConverter<ebsdlib::DoubleArrayType, float>;

std::map<std::string, int32_t> k_AlgorithmIndexMap = {{"eu", 0}, {"om", 1}, {"qu", 2}, {"aa", 3}, {"ro", 4}, {"ho", 5}, {"cu", 6}, {"st", 7}};

const std::string k_QuatsFilePath(ebsdlib::unit_test::DataDir + "IPF_Legend/quats_000_1_deg.txt");

// -----------------------------------------------------------------------------
template <typename T>
std::shared_ptr<EbsdDataArray<T>> generateRepresentation(int32_t inputType, int32_t outputType, typename EbsdDataArray<T>::Pointer inputOrientations)
{
  // using ArrayType = typename EbsdDataArray<T>::Pointer;
  using OCType1 = OrientationConverter<EbsdDataArray<T>, T>;

  std::vector<typename OCType1::Pointer> converters(7);

  converters[0] = EulerConverter<EbsdDataArray<T>, T>::New();
  converters[1] = OrientationMatrixConverter<EbsdDataArray<T>, T>::New();
  converters[2] = QuaternionConverter<EbsdDataArray<T>, T>::New();
  converters[3] = AxisAngleConverter<EbsdDataArray<T>, T>::New();
  converters[4] = RodriguesConverter<EbsdDataArray<T>, T>::New();
  converters[5] = HomochoricConverter<EbsdDataArray<T>, T>::New();
  converters[6] = CubochoricConverter<EbsdDataArray<T>, T>::New();

  std::vector<ebsdlib::orientations::Type> ocTypes = OCType1::GetOrientationTypes();

  converters[inputType]->setInputData(inputOrientations);
  converters[inputType]->convertRepresentationTo(ocTypes[outputType]);

  return converters[inputType]->getOutputData();
}

// -----------------------------------------------------------------------------
class ConvertOrientations
{
public:
  ConvertOrientations() = default;
  ~ConvertOrientations() = default;
  ConvertOrientations(const ConvertOrientations&) = delete;            // Copy Constructor Not Implemented
  ConvertOrientations(ConvertOrientations&&) = delete;                 // Move Constructor Not Implemented
  ConvertOrientations& operator=(const ConvertOrientations&) = delete; // Copy Assignment Not Implemented
  ConvertOrientations& operator=(ConvertOrientations&&) = delete;      // Move Assignment Not Implemented

  /**
   * @brief execute
   * @param inputFile
   * @param outputFile
   * @param delimiter
   * @param algorithm
   */
  EbsdDoubleArrayPointerType execute(const std::string& inputFile, const std::string& outputFile, const std::string& delimiter, const std::string& algorithm, bool headerLine)
  {

    // Parse the algorithm;
    std::vector<std::string> tokens = EbsdStringUtils::split(algorithm, '2');
    int32_t fromType = k_AlgorithmIndexMap[tokens[0]];
    int32_t toType = k_AlgorithmIndexMap[tokens[1]];

    std::fstream in(inputFile, std::ios_base::in);
    if(!in.is_open())
    {
      std::cout << "Could not open input file: " << inputFile << std::endl;
      return nullptr;
    }

    std::vector<float> orientations;
    char delim = delimiter.at(0);
    std::string buf;
    // Scan the file to figure out about how many values will be in the file
    size_t lineCount = 1;
    if(headerLine)
    {
      std::getline(in, buf);
    }
    while(!in.eof())
    {
      std::getline(in, buf);
      lineCount++;
    }
    // Put the input stream back to the start
    in.clear();                 // clear fail and eof bits
    in.seekg(0, std::ios::beg); // back to the start!
    if(headerLine)
    {
      std::getline(in, buf);
    }
    orientations.reserve(lineCount * 9); // Just reserve the worst case possible.
    while(!in.eof())
    {
      std::getline(in, buf);

      tokens = EbsdStringUtils::split(buf, delim);
      double value = std::atof(tokens[0].c_str());
      orientations.push_back(value);
      value = std::atof(tokens[1].c_str());
      orientations.push_back(value);
      value = std::atof(tokens[2].c_str());
      orientations.push_back(value);
      value = std::atof(tokens[3].c_str());
      orientations.push_back(value);
    }
    in.close();

    std::vector<int> strides = OCType::GetComponentCounts<std::vector<int>>();

    size_t numTuples = orientations.size() / strides[fromType];
    std::vector<size_t> cDims = {static_cast<size_t>(strides[fromType])};
    EbsdDoubleArrayPointerType inputOrientations = EbsdDoubleArrayType::WrapPointer(orientations.data(), numTuples, cDims, "Input", false);

    EbsdDoubleArrayPointerType outputOrientations = generateRepresentation<float>(fromType, toType, inputOrientations);

    std::ofstream outFile(outputFile, std::ios_base::out);
    if(!outFile.is_open())
    {
      std::cout << "Could not open output file for writing: " << outputFile << std::endl;
      return outputOrientations;
    }

    for(size_t i = 0; i < numTuples; i++)
    {
      outputOrientations->printTuple(outFile, i, delim);
      outFile << std::endl;
    }
    outFile.close();

    return outputOrientations;
  }
};

using FloatVec3Type = std::array<float, 3>;
/**
 * @brief The GenerateIPFColorsImpl class implements a threaded algorithm that computes the IPF
 * colors for each element in a geometry
 */
class GenerateIPFColorsImpl
{
public:
  GenerateIPFColorsImpl(const FloatVec3Type& referenceDir, FloatArrayType::Pointer& eulers, Int32ArrayType::Pointer& phases, bool* goodVoxels, UInt8ArrayType::Pointer& colors)
  : m_ReferenceDir(referenceDir)
  , m_CellEulerAngles(eulers)
  , m_CellPhases(phases)
  , m_GoodVoxels(goodVoxels)
  , m_CellIPFColors(colors)
  {
  }

  virtual ~GenerateIPFColorsImpl() = default;

  void run() const
  {
    std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
    double refDir[3] = {m_ReferenceDir[0], m_ReferenceDir[1], m_ReferenceDir[2]};
    double dEuler[3] = {0.0, 0.0, 0.0};
    ebsdlib::Rgb argb = 0x00000000;
    int32_t phase = 0;
    bool calcIPF = false;
    size_t index = 0;
    int32_t numPhases = 11;

    std::vector<size_t> laueOpsIndex = {3ULL}; // This is hard coded for Cubic-Low ops

    size_t totalPoints = m_CellEulerAngles->size() / 3;
    for(size_t i = 0; i < totalPoints; i++)
    {
      phase = (*m_CellPhases)[i];
      index = i * 3;
      (*m_CellIPFColors)[index] = 0;
      (*m_CellIPFColors)[index + 1] = 0;
      (*m_CellIPFColors)[index + 2] = 0;
      dEuler[0] = (*m_CellEulerAngles)[index];
      dEuler[1] = (*m_CellEulerAngles)[index + 1];
      dEuler[2] = (*m_CellEulerAngles)[index + 2];

      // Make sure we are using a valid Euler Angles with valid crystal symmetry
      calcIPF = true;
      if(nullptr != m_GoodVoxels)
      {
        calcIPF = m_GoodVoxels[i];
      }
      // Sanity check the phase data to make sure we do not walk off the end of the array
      if(phase >= numPhases)
      {
        // m_Filter->incrementPhaseWarningCount();
        std::cout << "phase > number of phases" << std::endl;
      }

      if(phase < numPhases && calcIPF && phase < ebsdlib::CrystalStructure::LaueGroupEnd)
      {
        argb = ops[phase]->generateIPFColor(dEuler, refDir, false);
        (*m_CellIPFColors)[index] = static_cast<uint8_t>(ebsdlib::RgbColor::dRed(argb));
        (*m_CellIPFColors)[index + 1] = static_cast<uint8_t>(ebsdlib::RgbColor::dGreen(argb));
        (*m_CellIPFColors)[index + 2] = static_cast<uint8_t>(ebsdlib::RgbColor::dBlue(argb));
      }
    }
  }

private:
  FloatVec3Type m_ReferenceDir;
  FloatArrayType::Pointer m_CellEulerAngles;
  Int32ArrayType::Pointer m_CellPhases;
  // std::vector<AngPhase::Pointer> m_PhaseInfos;

  bool* m_GoodVoxels;
  UInt8ArrayType::Pointer m_CellIPFColors;
};

void GenerateTestIPFImages(const std::vector<FloatVec3Type>& referenceDirections, const std::vector<std::string>& colorNames, int32_t phase)
{
  auto ops = LaueOps::GetAllOrientationOps();
  // Read in the Quats File
  ConvertOrientations convertor;
  auto outputOrientations = convertor.execute(k_QuatsFilePath, "eulers_000_1_deg.csv", ",", "qu2eu", true);
  size_t idx = 0;
  for(const auto& referenceDir : referenceDirections)
  {
    Int32ArrayType::Pointer phases = Int32ArrayType::CreateArray(outputOrientations->getNumberOfTuples(), "Phases", true);
    phases->initializeWithValue(phase);
    UInt8ArrayType::Pointer colors = UInt8ArrayType::CreateArray(outputOrientations->getNumberOfTuples(), {3ULL}, "IPF Colors", true);
    colors->initializeWithValue(128);
    GenerateIPFColorsImpl ipfColors(referenceDir, outputOrientations, phases, nullptr, colors);
    ipfColors.run();

    std::stringstream ss;
    ss << k_Output_Dir << EbsdStringUtils::replace(ops[phase]->getSymmetryName(), "/", "_") << "/ipf_test_image_" << static_cast<int>(referenceDir[0]) << "_" << static_cast<int>(referenceDir[1])
       << "_" << static_cast<int>(referenceDir[2]) << "_" << colorNames[idx] << ".png";
    auto result = PngWriter::WriteColorImage(ss.str(), 100, 100, 3, colors->getTuplePointer(0));
    std::cout << "IPF Colors Result: " << result.first << ": " << result.second << std::endl;
    idx++;
  }
}

void GeneratePoleFigures(LaueOps& ops, int symType)
{
  std::stringstream ss;

  // Read in the Quats File
  ConvertOrientations convertor;
  auto outputOrientations = convertor.execute(k_QuatsFilePath, "eulers_000_1_deg.csv", ",", "qu2eu", true);
  auto poleFigureNames = ops.getDefaultPoleFigureNames(ebsdlib::HexConvention::XParallelA);

  PoleFigureConfiguration_t config;
  config.eulers = outputOrientations.get();
  config.hexConvention = ebsdlib::HexConvention::XParallelA; // TSL/EDAX
  config.imageDim = 512;
  config.lambertDim = 72;
  config.numColors = 32;
  config.minScale = 0.0;
  config.maxScale = 100.0;
  config.sphereRadius = 1.0F;
  config.discrete = true;
  config.discreteHeatMap = false;
  //  config.colorMap = "";
  config.labels = {poleFigureNames[0], poleFigureNames[1], poleFigureNames[2]};
  config.order = {0, 1, 2};
  config.phaseName = "Generated Quaternions";

  std::vector<ebsdlib::UInt8ArrayType::Pointer> poleFigures = ops.generatePoleFigure(config);
  size_t index = 0;
  for(auto& poleFigure : poleFigures)
  {
    // The generated pole figures are coming out assuming "screen coordinates" where the 0,0 pixel
    // is in the upper left and the +Y points DOWN. But the algorithm used real XY coordinates
    // without knowledge of which reference frame we are in.
    // So first mirror the image across the X Axis
    poleFigure = ebsdlib::MirrorImage(poleFigure.get(), config.imageDim);

    // Overlay the Standard Projection annotations onto the Image
    if(symType == 1)
    {
      poleFigure = ebsdlib::DrawStandardCubicProjection(poleFigure, config.imageDim, config.imageDim);
    }
    else if(symType == 2)
    {
      poleFigure = ebsdlib::DrawStandardHexagonalProjection(poleFigure, config.imageDim, config.imageDim);
    }
    ss.str("");

    std::string cleanedLabel = EbsdStringUtils::replace(config.labels[index], "<", "[");
    cleanedLabel = EbsdStringUtils::replace(cleanedLabel, ">", "]");
    cleanedLabel = EbsdStringUtils::replace(cleanedLabel, "|", "_");

    ss << k_Output_Dir << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << cleanedLabel << "_pole_figure.png";
    auto result = PngWriter::WriteColorImage(ss.str(), config.imageDim, config.imageDim, 3, poleFigure->getTuplePointer(0));
    std::cout << ops.getSymmetryName() << " Pole Figure Result: " << result.first << ": " << result.second << std::endl;
    index++;
  }
}

// -----------------------------------------------------------------------------
void GenerateNolzeHielscherLegends(int imageDim)
{
  std::cout << "\n=== Generating Nolze-Hielscher IPF Legends ===\n" << std::endl;

  auto allOps = LaueOps::GetAllOrientationOps();

  // Map from LaueOps index to FundamentalSectorGeometry factory
  std::vector<std::function<ebsdlib::FundamentalSectorGeometry()>> sectorFactories = {
      ebsdlib::FundamentalSectorGeometry::hexagonalHigh,  // 0: Hexagonal_High
      ebsdlib::FundamentalSectorGeometry::cubicHigh,      // 1: Cubic_High
      ebsdlib::FundamentalSectorGeometry::hexagonalLow,   // 2: Hexagonal_Low
      ebsdlib::FundamentalSectorGeometry::cubicLow,       // 3: Cubic_Low
      ebsdlib::FundamentalSectorGeometry::triclinic,      // 4: Triclinic
      ebsdlib::FundamentalSectorGeometry::monoclinic,     // 5: Monoclinic
      ebsdlib::FundamentalSectorGeometry::orthorhombic,   // 6: OrthoRhombic
      ebsdlib::FundamentalSectorGeometry::tetragonalLow,  // 7: Tetragonal_Low
      ebsdlib::FundamentalSectorGeometry::tetragonalHigh, // 8: Tetragonal_High
      ebsdlib::FundamentalSectorGeometry::trigonalLow,    // 9: Trigonal_Low
      ebsdlib::FundamentalSectorGeometry::trigonalHigh,   // 10: Trigonal_High
  };

  (void)sectorFactories; // Per-class NH sectors are now baked into each LaueOps subclass.

  for(size_t i = 0; i < allOps.size(); i++)
  {
    auto& ops = *allOps[i];
    std::string symName = EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_");

    // Generate full-circle NH legend
    auto legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA, ebsdlib::ColorKeyKind::NolzeHielscher, /*gridded=*/false);
    std::stringstream ss;
    ss << k_Output_Dir << "/" << symName << "/" << symName << "_NH_FULL.png";
    auto result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " NH Full Result: " << result.first << ": " << result.second << std::endl;

    // Generate triangle-only NH legend, cropped per-class to match the TSL
    // smooth output dimensions (see sstSmoothCropFor() at the top of this file).
    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA, ebsdlib::ColorKeyKind::NolzeHielscher, /*gridded=*/false);
    {
      SstCropRect crop{};
      int outW = imageDim;
      int outH = imageDim;
      if(sstSmoothCropFor(ops.getRotationPointGroup(), crop))
      {
        const int xStart = static_cast<int>(imageDim * crop.xFrac);
        const int yStart = static_cast<int>(imageDim * crop.yFrac);
        outW = static_cast<int>(imageDim * crop.wFrac);
        outH = static_cast<int>(imageDim * crop.hFrac);
        legend = ebsdlib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, outW, outH);
      }
      ss.str("");
      ss << k_Output_Dir << "/" << symName << "/" << symName << "_NH.png";
      result = PngWriter::WriteColorImage(ss.str(), outW, outH, 3, legend->getPointer(0));
      std::cout << ops.getSymmetryName() << " NH Triangle Result: " << result.first << ": " << result.second << std::endl;
    }

    // Generate gridded NH legends (MTEX-style flat shading, 2000x2000)
    constexpr int k_GriddedImageDim = 2000;
    legend = ops.generateIPFTriangleLegend(k_GriddedImageDim, true, ebsdlib::HexConvention::XParallelA, ebsdlib::ColorKeyKind::NolzeHielscher, /*gridded=*/true);
    ss.str("");
    ss << k_Output_Dir << "/" << symName << "/" << symName << "_NH_GRIDDED_FULL.png";
    result = PngWriter::WriteColorImage(ss.str(), k_GriddedImageDim, k_GriddedImageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " NH Gridded Full Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(k_GriddedImageDim, false, ebsdlib::HexConvention::XParallelA, ebsdlib::ColorKeyKind::NolzeHielscher, /*gridded=*/true);
    ss.str("");
    ss << k_Output_Dir << "/" << symName << "/" << symName << "_NH_GRIDDED.png";
    result = PngWriter::WriteColorImage(ss.str(), k_GriddedImageDim, k_GriddedImageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " NH Gridded Triangle Result: " << result.first << ": " << result.second << std::endl;

    // PUCM (Nolze 2017 perceptually uniform color mapping): 4 variants per
    // Laue class (full + triangle) x (smooth + gridded). Mirrors the NH
    // block above. Validation is internal-only; MTEX does not ship a PUCM
    // key, so there's no external apples-to-apples reference. The Edax
    // exemplars at /Users/Shared/Data/Edax_IPF_Test/IPF\ PUCM.bmp are the
    // only third-party PUCM reference we know of.
    legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA, ebsdlib::ColorKeyKind::PUCM, /*gridded=*/false);
    ss.str("");
    ss << k_Output_Dir << "/" << symName << "/" << symName << "_PUCM_FULL.png";
    result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " PUCM Full Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA, ebsdlib::ColorKeyKind::PUCM, /*gridded=*/false);
    {
      SstCropRect crop{};
      int outW = imageDim;
      int outH = imageDim;
      if(sstSmoothCropFor(ops.getRotationPointGroup(), crop))
      {
        const int xStart = static_cast<int>(imageDim * crop.xFrac);
        const int yStart = static_cast<int>(imageDim * crop.yFrac);
        outW = static_cast<int>(imageDim * crop.wFrac);
        outH = static_cast<int>(imageDim * crop.hFrac);
        legend = ebsdlib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, outW, outH);
      }
      ss.str("");
      ss << k_Output_Dir << "/" << symName << "/" << symName << "_PUCM.png";
      result = PngWriter::WriteColorImage(ss.str(), outW, outH, 3, legend->getPointer(0));
      std::cout << ops.getSymmetryName() << " PUCM Triangle Result: " << result.first << ": " << result.second << std::endl;
    }

    legend = ops.generateIPFTriangleLegend(k_GriddedImageDim, true, ebsdlib::HexConvention::XParallelA, ebsdlib::ColorKeyKind::PUCM, /*gridded=*/true);
    ss.str("");
    ss << k_Output_Dir << "/" << symName << "/" << symName << "_PUCM_GRIDDED_FULL.png";
    result = PngWriter::WriteColorImage(ss.str(), k_GriddedImageDim, k_GriddedImageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " PUCM Gridded Full Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(k_GriddedImageDim, false, ebsdlib::HexConvention::XParallelA, ebsdlib::ColorKeyKind::PUCM, /*gridded=*/true);
    ss.str("");
    ss << k_Output_Dir << "/" << symName << "/" << symName << "_PUCM_GRIDDED.png";
    result = PngWriter::WriteColorImage(ss.str(), k_GriddedImageDim, k_GriddedImageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " PUCM Gridded Triangle Result: " << result.first << ": " << result.second << std::endl;
  }
}

// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  // Create all the output directories needed.
  auto ops = LaueOps::GetAllOrientationOps();
  std::filesystem::create_directories(k_Output_Dir);
  for(const auto& op : ops)
  {
    std::stringstream ss;
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(op->getSymmetryName(), "/", "_");
    std::filesystem::create_directories(ss.str());
  }

  const std::string outputRoot = std::filesystem::absolute(k_Output_Dir).string();
  std::cout << "=====================================================================\n"
            << " generate_ipf_legends\n"
            << " Hex/Trig convention: X||a (TSL/EDAX)\n"
            << " Output directory (one subfolder per Laue class):\n"
            << "   " << outputRoot << "\n"
            << "=====================================================================" << std::endl;

  std::stringstream ss;
  int imageDim = 1500;
  {
    TrigonalOps ops;
    auto legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.png";
    auto result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA);
    int xStart = imageDim * 0.05F;
    int yStart = 0;
    int numCols = imageDim * 0.75F;
    int numRows = imageDim * 0.65F;
    legend = ebsdlib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".png";
    result = PngWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    std::vector<FloatVec3Type> referenceDirections = {
        {0.0F, 0.0F, 1.0F},                    // Red  DONE
        {0.0F, 1.0F, 0.0F},                    // Green
        {-0.5F, std::sqrt(3.0F) / 2.0F, 0.0F}, // Aqua
        {2.0F, 1.0F, 0.0F},                    // Blue
        {0.0F, 1.0F, -1.0F},                   // Yellow  DONE
        {1.0F, -1.0F, 1.0F},                   // Pink  DONE
        {1.0F, 0.0F, 1.0F},                    // Flesh
    };
    std::vector<std::string> colorNames{"Red", "Green", "Aqua", "Blue", "Yellow", "Pink", "Flesh"};
    GenerateTestIPFImages(referenceDirections, colorNames, 10);
    // Generate Pole Figures for the Input Test Orientations
    GeneratePoleFigures(ops, 2);
  }

  {
    TriclinicOps ops;
    auto legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.png";
    auto result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".png";
    result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    std::vector<FloatVec3Type> referenceDirections = {
        {0.0F, 0.0F, 1.0F},   // Red
        {1.0F, 0.0F, 0.0F},   // Green
        {-1.0F, 0.0F, 0.0F},  // Blue
        {0.0F, 1.0F, 0.0F},   // Aqua
        {1.0F, 1.0F, 1.0F},   // Yellow
        {-1.0F, -1.0F, 1.0F}, // Pink
        {0.0F, 1.0F, 1.0F},   // Flesh
    };
    std::vector<std::string> colorNames{"Red", "Green", "Blue", "Aqua", "Yellow", "Pink", "Flesh"};
    GenerateTestIPFImages(referenceDirections, colorNames, 4);
    // Generate Pole Figures for the Input Test Orientations
    GeneratePoleFigures(ops, 1);
  }

  {
    MonoclinicOps ops;
    auto legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.png";
    auto result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA);
    int yCropped = imageDim * 0.6F;
    legend = ebsdlib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, 0, 0, imageDim, yCropped);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".png";
    result = PngWriter::WriteColorImage(ss.str(), imageDim, yCropped, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    std::vector<FloatVec3Type> referenceDirections = {
        {0.0F, 0.0F, 1.0F},   // Red
        {1.0F, 0.0F, 0.0F},   // Green
        {0.0F, 1.0F, 0.0F},   // Aqua
        {-1.0F, 0.0F, 0.0F},  // Blue
        {1.0F, 1.0F, 1.0F},   // Yellow
        {-1.0F, -1.0F, 1.0F}, // Pink
        {0.0F, -1.0F, 0.0F},  // Aqua
    };
    std::vector<std::string> colorNames{"Red", "Green", "Aqua", "Blue", "Yellow", "Pink", "Aqua"};
    GenerateTestIPFImages(referenceDirections, colorNames, 5);
    // Generate Pole Figures for the Input Test Orientations
    GeneratePoleFigures(ops, 1);
  }

  {
    CubicLowOps ops;

    auto legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.png";
    auto result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".png";
    result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    std::vector<FloatVec3Type> referenceDirections = {
        {0.0F, 0.0F, 1.0F}, // Red
        {1.0F, 0.0F, 1.0F}, // Green
        {1.0F, 1.0F, 1.0F}, // Aqua
        {0.0F, 1.0F, 1.0F}, // Blue
        {1.0F, 0.0F, 2.0F}, // Yellow
        {0.0F, 1.0F, 2.0F}, // Pink
        {1.0F, 1.0F, 2.0F}, // Flesh
    };
    std::vector<std::string> colorNames{"Red", "Green", "Aqua", "Blue", "Yellow", "Pink", "Flesh"};
    GenerateTestIPFImages(referenceDirections, colorNames, 3);
    // Generate Pole Figures for the Input Test Orientations
    GeneratePoleFigures(ops, 1);
  }

  {
    CubicOps ops;
    auto legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA);
    ss.str("");

    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.png";
    auto result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;
    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".png";
    result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    std::vector<FloatVec3Type> referenceDirections = {
        {0.0F, 0.0F, 1.0F}, // Red
        {0.0F, 1.0F, 1.0F}, // Green
        {1.0F, 2.0F, 2.0F}, // Aqua
        {1.0F, 1.0F, 1.0F}, // Blue
        {0.0F, 2.0F, 1.0F}, // Yellow
        {1.0F, 1.0F, 2.0F}, // Pink
                            // {1.0F, 1.0F, 2.0F}, // Flesh
    };
    std::vector<std::string> colorNames{"Red", "Green", "Aqua", "Blue", "Yellow", "Pink", "Flesh"};
    GenerateTestIPFImages(referenceDirections, colorNames, 1);
    // Generate Pole Figures for the Input Test Orientations
    GeneratePoleFigures(ops, 1);
  }

  {
    OrthoRhombicOps ops;
    auto legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.png";
    auto result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    int xStart = imageDim * 0.10F;
    int yStart = 0;
    int numCols = imageDim * 0.78F;
    int numRows = imageDim * 0.6F;
    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA);
    legend = ebsdlib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".png";
    result = PngWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    std::vector<FloatVec3Type> referenceDirections = {
        {0.0F, 0.0F, 1.0F}, // Red
        {1.0F, 0.0F, 0.0F}, // Green
        {1.0F, 1.0F, 0.0F}, // Aqua
        {0.0F, 1.0F, 0.0F}, // Blue
        {1.0F, 0.0F, 1.0F}, // Yellow
        {0.0F, 1.0F, 1.0F}, // Pink
        {1.0F, 1.0F, 1.0F}, // Flesh
    };
    std::vector<std::string> colorNames{"Red", "Green", "Aqua", "Blue", "Yellow", "Pink", "Flesh"};
    GenerateTestIPFImages(referenceDirections, colorNames, 6);
    // Generate Pole Figures for the Input Test Orientations
    GeneratePoleFigures(ops, 1);
  }

  {
    TetragonalOps ops;
    auto legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA);
    ss.str("");

    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.png";
    auto result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    int xStart = imageDim * 0.10F;
    int yStart = 0;
    int numCols = imageDim * 0.78F;
    int numRows = imageDim * 0.6F;
    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA);
    legend = ebsdlib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".png";
    result = PngWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    std::vector<FloatVec3Type> referenceDirections = {
        {0.0F, 0.0F, 1.0F}, // Red
        {1.0F, 0.0F, 0.0F}, // Green
        {2.0F, 1.0F, 0.0F}, // Aqua
        {1.0F, 1.0F, 0.0F}, // Blue
        {1.0F, 0.0F, 1.0F}, // Yellow
        {1.0F, 1.0F, 1.0F}, // Pink
        {1.0F, 2.0F, 3.0F}, // Flesh
    };
    std::vector<std::string> colorNames{"Red", "Green", "Aqua", "Blue", "Yellow", "Pink", "Flesh"};
    GenerateTestIPFImages(referenceDirections, colorNames, 8);
    // Generate Pole Figures for the Input Test Orientations
    GeneratePoleFigures(ops, 1);
  }

  {
    TetragonalLowOps ops;
    auto legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.png";
    auto result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA);
    int xStart = imageDim * 0.10F;
    int yStart = 0;
    int numCols = imageDim * 0.70F;
    int numRows = imageDim * 0.6F;
    legend = ebsdlib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".png";
    result = PngWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    std::vector<FloatVec3Type> referenceDirections = {
        {0.0F, 0.0F, 1.0F}, // Red
        {1.0F, 0.0F, 0.0F}, // Green
        {1.0F, 1.0F, 0.0F}, // Aqua
        {0.0F, 1.0F, 0.0F}, // Blue
        {0.0F, 1.0F, 1.0F}, // Yellow
        {1.0F, 0.0F, 1.0F}, // Pink
        {1.0F, 1.0F, 1.0F}, // Flesh
    };
    std::vector<std::string> colorNames{"Red", "Green", "Aqua", "Blue", "Yellow", "Pink", "Flesh"};
    GenerateTestIPFImages(referenceDirections, colorNames, 7);
    // Generate Pole Figures for the Input Test Orientations
    GeneratePoleFigures(ops, 1);
  }

  {
    HexagonalOps ops;
    auto legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.png";
    auto result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA);
    int xStart = imageDim * 0.10F;
    int yStart = 0;
    int numCols = imageDim * 0.80F;
    int numRows = imageDim * 0.5F;
    legend = ebsdlib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".png";
    result = PngWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    std::vector<FloatVec3Type> referenceDirections = {
        {0.0F, 0.0F, 1.0F}, // Red
        {1.0F, 0.0F, 0.0F}, // Green
        {4.0F, 1.0F, 0.0F}, // Aqua
        {2.0F, 1.0F, 0.0F}, // Blue
        {1.0F, 0.0F, 1.0F}, // Yellow
        {2.0F, 1.0F, 2.0F}, // Pink
                            //        {1.0F, 1.0F, 2.0F}, // Flesh
    };
    std::vector<std::string> colorNames{"Red", "Green", "Aqua", "Blue", "Yellow", "Pink", "Flesh"};
    GenerateTestIPFImages(referenceDirections, colorNames, 0);
    // Generate Pole Figures for the Input Test Orientations
    GeneratePoleFigures(ops, 2);
  }

  {
    HexagonalLowOps ops;
    auto legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.png";
    auto result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA);
    int xStart = imageDim * 0.10F;
    int yStart = 0;
    int numCols = imageDim * 0.70F;
    int numRows = imageDim * 0.5F;
    legend = ebsdlib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".png";
    result = PngWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    std::vector<FloatVec3Type> referenceDirections = {
        {0.0F, 0.0F, 1.0F}, // Red
        {1.0F, 0.0F, 0.0F}, // Green
        {2.0F, 1.0F, 0.0F}, // Aqua
        {1.0F, 1.0F, 0.0F}, // Blue
        {1.0F, 0.0F, 1.0F}, // Yellow / Pink  Correct
                            //  {1.0F, 2.0F, 2.0F}, // Pink
        {2.0F, 1.0F, 2.0F}, // Flesh  Correct
    };
    std::vector<std::string> colorNames{"Red", "Green", "Aqua", "Blue", "Yellow", "Flesh"};
    GenerateTestIPFImages(referenceDirections, colorNames, 2);
    // Generate Pole Figures for the Input Test Orientations
    GeneratePoleFigures(ops, 2);
  }

  {
    TrigonalLowOps ops;
    auto legend = ops.generateIPFTriangleLegend(imageDim, true, ebsdlib::HexConvention::XParallelA);
    ss.str("");
    ss << k_Output_Dir << ops.getSymmetryName() << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.png";
    auto result = PngWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false, ebsdlib::HexConvention::XParallelA);
    int xStart = imageDim * 0.00F;
    int yStart = 0;
    int numCols = imageDim * 0.90F;
    int numRows = imageDim * 0.65F;
    legend = ebsdlib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".png";
    result = PngWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    std::vector<FloatVec3Type> referenceDirections = {
        {0.0F, 0.0F, 1.0F},   // Red
        {-1.0F, -1.0F, 0.0F}, // Green
        {1.0F, -2.0F, 0.0F},  // Aqua
        {1.0F, 0.0F, 0.0F},   // Blue
        {0.0F, -1.0F, 1.0F},  // Yellow
        {2.0F, -1.0F, 2.0F},  // Pink
        {1.0F, -2.0F, 2.0F},  // Flesh
    };
    std::vector<std::string> colorNames{"Red", "Green", "Aqua", "Blue", "Yellow", "Pink", "Flesh"};
    GenerateTestIPFImages(referenceDirections, colorNames, 9);
    // Generate Pole Figures for the Input Test Orientations
    GeneratePoleFigures(ops, 1);
  }

  GenerateNolzeHielscherLegends(imageDim);

  std::cout << "\nDone. All IPF legends and pole figures were written under:\n   " << outputRoot << std::endl;
  return 0;
}
