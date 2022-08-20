
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/OrientationMath/OrientationConverter.hpp"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

using EbsdDoubleArrayType = EbsdDataArray<double>;
using EbsdDoubleArrayPointerType = EbsdDoubleArrayType::Pointer;
using OCType = OrientationConverter<EbsdLib::DoubleArrayType, float>;

// -----------------------------------------------------------------------------
template <typename T>
std::shared_ptr<EbsdDataArray<T>> generateRepresentation(int32_t inputType, int32_t outputType, typename EbsdDataArray<T>::Pointer inputOrientations)
{
  // using ArrayType = typename EbsdDataArray<T>::Pointer;
  using OCType = OrientationConverter<EbsdDataArray<T>, T>;

  std::vector<typename OCType::Pointer> converters(7);

  converters[0] = EulerConverter<EbsdDataArray<T>, T>::New();
  converters[1] = OrientationMatrixConverter<EbsdDataArray<T>, T>::New();
  converters[2] = QuaternionConverter<EbsdDataArray<T>, T>::New();
  converters[3] = AxisAngleConverter<EbsdDataArray<T>, T>::New();
  converters[4] = RodriguesConverter<EbsdDataArray<T>, T>::New();
  converters[5] = HomochoricConverter<EbsdDataArray<T>, T>::New();
  converters[6] = CubochoricConverter<EbsdDataArray<T>, T>::New();

  std::vector<OrientationRepresentation::Type> ocTypes = OCType::GetOrientationTypes();

  converters[inputType]->setInputData(inputOrientations);
  converters[inputType]->convertRepresentationTo(ocTypes[outputType]);

  return converters[inputType]->getOutputData();
}

// -----------------------------------------------------------------------------
std::map<std::string, int32_t> k_AlgorithmIndexMap = {{"eu", 0}, {"om", 1}, {"qu", 2}, {"aa", 3}, {"ro", 4}, {"ho", 5}, {"cu", 6}, {"st", 7}};

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
  void execute(const std::string& inputFile, const std::string& outputFile, const std::string& delimiter, const std::string& algorithm, bool headerLine)
  {

    // Parse the algorithm;
    std::vector<std::string> tokens = EbsdStringUtils::split(algorithm, '2');
    int32_t fromType = k_AlgorithmIndexMap[tokens[0]];
    int32_t toType = k_AlgorithmIndexMap[tokens[1]];

    std::fstream in(inputFile, std::ios_base::in);
    if(!in.is_open())
    {
      std::cout << "Could not open input file: " << inputFile << std::endl;
      return;
    }

    std::vector<double> orientations;
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
      for(const auto& token : tokens)
      {
        double value = std::atof(token.c_str());
        orientations.push_back(value);
      }
    }
    in.close();

    std::vector<int> strides = OCType::GetComponentCounts<std::vector<int>>();

    size_t numTuples = orientations.size() / strides[fromType];
    std::vector<size_t> cDims = {static_cast<size_t>(strides[fromType])};
    EbsdDoubleArrayPointerType inputOrientations = EbsdDoubleArrayType::WrapPointer(orientations.data(), numTuples, cDims, "Input", false);

    EbsdDoubleArrayPointerType outputOrientations = generateRepresentation<double>(fromType, toType, inputOrientations);

    std::ofstream outFile(outputFile, std::ios_base::out);
    if(!outFile.is_open())
    {
      std::cout << "Could not open output file for writing: " << outputFile << std::endl;
      return;
    }

    for(size_t i = 0; i < numTuples; i++)
    {
      outputOrientations->printTuple(outFile, i, delim);
      outFile << std::endl;
    }
    outFile.close();
  }
};

// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{

  const size_t k_InputFileIndex = 0;
  const size_t k_OutputFileIndex = 1;
  const size_t k_DelimiterIndex = 2;
  const size_t k_AlgorithmIndex = 3;
  const size_t k_HeaderIndex = 4;
  const size_t k_HelpIndex = 5;

  using ArgEntry = std::vector<std::string>;
  using ArgEntries = std::vector<ArgEntry>;

  ArgEntries args;

  args.push_back({"-i", "--inputfile", "The input file to process"});
  args.push_back({"-o", "--outputfile", "The output file to write"});
  args.push_back({"-d", "--delimiter", "How are the fields separated. [SPACE|COMMA|TAB]"});
  args.push_back({"-a", "--algorithm",
                  "The orientation transformation to run. This should be in the form of \n   [eu|om|qu|aa|ro|ho|cu]2[eu|om|qu|aa|ro|ho|cu]\nExample: eu2qu to convert from Eulers to Quaternions"});
  args.push_back({"-s", "--header", "File has header line"});
  args.push_back({"-h", "--help", "Show help for this program"});

  std::string inputFile;
  std::string outputFile;
  std::string delimiter;
  std::string algorithm;
  bool header = false;

  for(int32_t i = 0; i < argc; i++)
  {
    if(argv[i] == args[k_InputFileIndex][0] || argv[i] == args[k_InputFileIndex][1])
    {
      inputFile = argv[++i];
    }
    if(argv[i] == args[k_OutputFileIndex][0] || argv[i] == args[k_OutputFileIndex][1])
    {
      outputFile = argv[++i];
    }
    if(argv[i] == args[k_DelimiterIndex][0] || argv[i] == args[k_DelimiterIndex][1])
    {
      delimiter = argv[++i];
    }
    if(argv[i] == args[k_AlgorithmIndex][0] || argv[i] == args[k_AlgorithmIndex][1])
    {
      algorithm = argv[++i];
    }
    if(argv[i] == args[k_HeaderIndex][0] || argv[i] == args[k_HeaderIndex][1])
    {
      header = true;
    }
    if(argv[i] == args[k_HelpIndex][0] || argv[i] == args[k_HelpIndex][1])
    {
      std::cout << "This program has the following arguments:" << std::endl;
      for(const auto& input : args)
      {
        std::cout << input[0] << ", " << input[1] << ": " << input[2] << std::endl;
      }
      return 0;
    }
  }
  ConvertOrientations convert;
  convert.execute(inputFile, outputFile, delimiter, algorithm, header);
  return 0;
}
