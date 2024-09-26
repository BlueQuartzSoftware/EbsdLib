
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
#include "EbsdLib/Utilities/TiffWriter.h"

#include "EbsdLib/Apps/EbsdLibFileLocations.h"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace EbsdLib;

// const std::string k_Output_Dir(UnitTest::DataDir + "/IPF_Legend/");
const std::string k_Output_Dir(UnitTest::TestTempDir + "/IPF_Legend/");

using EbsdDoubleArrayType = EbsdDataArray<float>;
using EbsdDoubleArrayPointerType = EbsdDoubleArrayType::Pointer;
using OCType = OrientationConverter<EbsdLib::DoubleArrayType, float>;

std::map<std::string, int32_t> k_AlgorithmIndexMap = {{"eu", 0}, {"om", 1}, {"qu", 2}, {"aa", 3}, {"ro", 4}, {"ho", 5}, {"cu", 6}, {"st", 7}};

const std::string k_QuatsFilePath(UnitTest::DataDir + "IPF_Legend/quats_000_1_deg.txt");

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

  std::vector<OrientationRepresentation::Type> ocTypes = OCType1::GetOrientationTypes();

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
    EbsdLib::Rgb argb = 0x00000000;
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

      if(phase < numPhases && calcIPF && phase < EbsdLib::CrystalStructure::LaueGroupEnd)
      {
        argb = ops[phase]->generateIPFColor(dEuler, refDir, false);
        (*m_CellIPFColors)[index] = static_cast<uint8_t>(EbsdLib::RgbColor::dRed(argb));
        (*m_CellIPFColors)[index + 1] = static_cast<uint8_t>(EbsdLib::RgbColor::dGreen(argb));
        (*m_CellIPFColors)[index + 2] = static_cast<uint8_t>(EbsdLib::RgbColor::dBlue(argb));
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
       << "_" << static_cast<int>(referenceDir[2]) << "_" << colorNames[idx] << ".tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), 100, 100, 3, colors->getTuplePointer(0));
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
  auto poleFigureNames = ops.getDefaultPoleFigureNames();

  PoleFigureConfiguration_t config;
  config.eulers = outputOrientations.get();
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

  std::vector<EbsdLib::UInt8ArrayType::Pointer> poleFigures = ops.generatePoleFigure(config);
  size_t index = 0;
  for(auto& poleFigure : poleFigures)
  {
    // The generated pole figures are coming out assuming "screen coordinates" where the 0,0 pixel
    // is in the upper left and the +Y points DOWN. But the algorithm used real XY coordinates
    // without knowledge of which reference frame we are in.
    // So first mirror the image across the X Axis
    poleFigure = EbsdLib::MirrorImage(poleFigure.get(), config.imageDim);

    // Overlay the Standard Projection annotations onto the Image
    if(symType == 1)
    {
      poleFigure = EbsdLib::DrawStandardCubicProjection(poleFigure, config.imageDim, config.imageDim);
    }
    else if(symType == 2)
    {
      poleFigure = EbsdLib::DrawStandardHexagonalProjection(poleFigure, config.imageDim, config.imageDim);
    }
    ss.str("");

    std::string cleanedLabel = EbsdStringUtils::replace(config.labels[index], "<", "[");
    cleanedLabel = EbsdStringUtils::replace(cleanedLabel, ">", "]");
    cleanedLabel = EbsdStringUtils::replace(cleanedLabel, "|", "_");

    ss << k_Output_Dir << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << cleanedLabel << "_pole_figure.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), config.imageDim, config.imageDim, 3, poleFigure->getTuplePointer(0));
    std::cout << ops.getSymmetryName() << " Pole Figure Result: " << result.first << ": " << result.second << std::endl;
    index++;
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

  std::stringstream ss;
  int imageDim = 512;
  {
    TrigonalOps ops;
    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false);
    int xStart = imageDim * 0.05F;
    int yStart = 0;
    int numCols = imageDim * 0.75F;
    int numRows = imageDim * 0.65F;
    legend = EbsdLib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".tiff";
    result = TiffWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
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
    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".tiff";
    result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
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
    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false);
    int yCropped = imageDim * 0.6F;
    legend = EbsdLib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, 0, 0, imageDim, yCropped);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".tiff";
    result = TiffWriter::WriteColorImage(ss.str(), imageDim, yCropped, 3, legend->getPointer(0));
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

    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".tiff";
    result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
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
    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");

    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;
    legend = ops.generateIPFTriangleLegend(imageDim, false);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".tiff";
    result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
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
    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    int xStart = imageDim * 0.10F;
    int yStart = 0;
    int numCols = imageDim * 0.78F;
    int numRows = imageDim * 0.6F;
    legend = ops.generateIPFTriangleLegend(imageDim, false);
    legend = EbsdLib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".tiff";
    result = TiffWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
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
    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");

    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    int xStart = imageDim * 0.10F;
    int yStart = 0;
    int numCols = imageDim * 0.78F;
    int numRows = imageDim * 0.6F;
    legend = ops.generateIPFTriangleLegend(imageDim, false);
    legend = EbsdLib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".tiff";
    result = TiffWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
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
    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false);
    int xStart = imageDim * 0.10F;
    int yStart = 0;
    int numCols = imageDim * 0.70F;
    int numRows = imageDim * 0.6F;
    legend = EbsdLib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".tiff";
    result = TiffWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
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
    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false);
    int xStart = imageDim * 0.10F;
    int yStart = 0;
    int numCols = imageDim * 0.80F;
    int numRows = imageDim * 0.5F;
    legend = EbsdLib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".tiff";
    result = TiffWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
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
    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false);
    int xStart = imageDim * 0.10F;
    int yStart = 0;
    int numCols = imageDim * 0.70F;
    int numRows = imageDim * 0.5F;
    legend = EbsdLib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".tiff";
    result = TiffWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
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
    auto legend = ops.generateIPFTriangleLegend(imageDim, true);
    ss.str("");
    ss << k_Output_Dir << ops.getSymmetryName() << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "_FULL.tiff";
    auto result = TiffWriter::WriteColorImage(ss.str(), imageDim, imageDim, 3, legend->getPointer(0));
    std::cout << ops.getSymmetryName() << " Result: " << result.first << ": " << result.second << std::endl;

    legend = ops.generateIPFTriangleLegend(imageDim, false);
    int xStart = imageDim * 0.00F;
    int yStart = 0;
    int numCols = imageDim * 0.90F;
    int numRows = imageDim * 0.65F;
    legend = EbsdLib::CropRGBImage<uint8_t>(legend, imageDim, imageDim, xStart, yStart, numCols, numRows);
    ss.str("");
    ss << k_Output_Dir << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << "/" << EbsdStringUtils::replace(ops.getSymmetryName(), "/", "_") << ".tiff";
    result = TiffWriter::WriteColorImage(ss.str(), numCols, numRows, 3, legend->getPointer(0));
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

  return 0;
}
