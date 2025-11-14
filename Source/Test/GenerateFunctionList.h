#pragma once

#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/OrientationMath/OrientationConverter.hpp"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

#include "UnitTestSupport.hpp"

#include <algorithm>
#include <map>
#include <string>
#include <vector>

using namespace ebsdlib;

class GenerateFunctionList
{

public:
  GenerateFunctionList() = default;
  virtual ~GenerateFunctionList() = default;

  using EntryType = std::vector<int>;

  /**
   * @brief GenerateTable
   * @param n
   * @param k
   * @return
   */
  std::vector<EntryType> GeneratePermutationsOfCombinations(int n, int k)
  {
    m_Combinations.clear();
    m_Permutations.clear();

    combinations(n, k);
    // for(size_t i = 0; i < m_Combinations.size(); i++)
    for(auto& combination : m_Combinations)
    {
      permutation(static_cast<int>(combination.size()), combination);
    }
    return m_Permutations;
  }

protected:
  /**
   * @brief combinations
   * @param n
   * @param k
   */
  void combinations(int n, int k)
  {
    std::string bitmask(k, 1); // K leading 1's
    bitmask.resize(n, 0);      // N-K trailing 0's

    do
    {
      std::vector<int> entry;
      for(std::string::size_type i = 0; i < n; ++i) // [0..N-1] integers
      {
        if(bitmask[i])
        {
          entry.push_back(static_cast<int>(i));
        }
      }
      m_Combinations.push_back(entry);

    } while(std::prev_permutation(bitmask.begin(), bitmask.end()));
  }

  /**
   * @brief permutation
   * @param n
   * @param ch
   * @return
   */
  int permutation(int n, std::vector<int> ch)
  {
    int i, j;
    int temp;
    std::vector<int>::size_type N = ch.size();

    if(n == 0)
    {
      std::vector<int> entry;

      for(j = static_cast<int>(N - 1); j >= 0; j--)
      {
        entry.push_back(ch[j]);
        // std::cout<<ch[j];
      }
      //        std::cout<<std::endl;
      m_Permutations.push_back(entry);
      return 0;
    }
    for(i = 0; i < n; i++)
    {
      temp = ch[i];
      for(j = i + 1; j < n; j++)
      {
        ch[j - 1] = ch[j];
      }
      ch[n - 1] = temp;
      // shift
      permutation(n - 1, ch);
      for(j = n - 1; j > i; j--)
      {
        ch[j] = ch[j - 1];
      }
      ch[i] = temp;
      // and shift back agian
    }
    return 1;
  }

private:
  std::vector<EntryType> m_Combinations;
  std::vector<EntryType> m_Permutations;

public:
  GenerateFunctionList(const GenerateFunctionList&) = delete;            // Copy Constructor Not Implemented
  GenerateFunctionList(GenerateFunctionList&&) = delete;                 // Move Constructor Not Implemented
  GenerateFunctionList& operator=(const GenerateFunctionList&) = delete; // Copy Assignment Not Implemented
  GenerateFunctionList& operator=(GenerateFunctionList&&) = delete;      // Move Assignment Not Implemented
};

namespace generate_test_data
{

// -----------------------------------------------------------------------------
template <typename T>
std::shared_ptr<EbsdDataArray<T>> generateRepresentation(int32_t inputType, int32_t outputType, typename EbsdDataArray<T>::Pointer inputOrientations)
{
  // using ArrayType = typename EbsdDataArray<T>::Pointer;
  using OCType = OrientationConverter<EbsdDataArray<T>, T>;

  std::vector<typename OCType::Pointer> converters(ebsdlib::s_NumReps);

  converters[0] = EulerConverter<EbsdDataArray<T>, T>::New();
  converters[1] = OrientationMatrixConverter<EbsdDataArray<T>, T>::New();
  converters[2] = QuaternionConverter<EbsdDataArray<T>, T>::New();
  converters[3] = AxisAngleConverter<EbsdDataArray<T>, T>::New();
  converters[4] = RodriguesConverter<EbsdDataArray<T>, T>::New();
  converters[5] = HomochoricConverter<EbsdDataArray<T>, T>::New();
  converters[6] = CubochoricConverter<EbsdDataArray<T>, T>::New();
  converters[7] = StereographicConverter<EbsdDataArray<T>, T>::New();

  std::vector<ebsdlib::orientations::Type> ocTypes = OCType::GetOrientationTypes();

  converters[inputType]->setInputData(inputOrientations);
  converters[inputType]->convertRepresentationTo(ocTypes[outputType]);

  return converters[inputType]->getOutputData();
}

// -----------------------------------------------------------------------------
std::string ExecuteConvertFilter(std::map<std::string, EbsdDataArray<double>::Pointer>& attrMat, GenerateFunctionList::EntryType& entry, int e, const std::string& outputName)
{
  std::string inputName = outputName;

  if(e == 0)
  {
    inputName = k_InputNames[entry[e]];
  }
  EbsdDataArray<double>::Pointer inputData = attrMat[inputName];
  EbsdDataArray<double>::Pointer outputData = generateRepresentation<double>(entry[e], entry[e + 1], inputData);
  std::string nextOutputName = EbsdStringUtils::number(e) + std::string("_") + k_InputNames[entry[e]] + std::string("2") + k_InputNames[entry[e + 1]];
  attrMat[nextOutputName] = outputData;

  return nextOutputName;
}

// -----------------------------------------------------------------------------
template <typename T>
void GenerateEulers(size_t nSteps, std::map<std::string, typename EbsdDataArray<T>::Pointer>& attrMat)
{
  std::vector<size_t> cDims = {3};

  T phi1_min = static_cast<T>(0.0);
  T phi1_max = ebsdlib::constants::k_2PiD;
  T phi1_delta = (phi1_max - phi1_min) / static_cast<T>(nSteps);

  T phi_min = static_cast<T>(0.0);
  T phi_max = ebsdlib::constants::k_PiD;
  T phi_delta = (phi_max - phi_min) / static_cast<T>(nSteps);

  T phi2_min = static_cast<T>(0.0);
  T phi2_max = ebsdlib::constants::k_2PiD;
  T phi2_delta = (phi2_max - phi2_min) / static_cast<T>(nSteps);

  size_t nStepsCubed = (nSteps + 1) * (nSteps + 1) * (nSteps + 1);
  typename EbsdDataArray<T>::Pointer eulers = EbsdDataArray<T>::CreateArray(nStepsCubed, cDims, k_InputNames[0], true);

  size_t counter = 0;
  for(size_t i = 0; i <= nSteps; i++)
  {
    for(size_t j = 0; j <= nSteps; j++)
    {
      for(size_t k = 0; k <= nSteps; k++)
      {
        //        std::cout << "Euler[" << counter << "]: "
        //                  << (phi1_min+i*phi1_delta)*DConst::k_180OverPi << ", "
        //                   << (phi_min+j*phi_delta)*DConst::k_180OverPi  << ", "
        //                      << (phi2_min+k*phi2_delta)*DConst::k_180OverPi  << std::endl;

        eulers->setComponent(counter, 0, phi1_min + i * phi1_delta);
        eulers->setComponent(counter, 1, phi_min + j * phi_delta);
        eulers->setComponent(counter, 2, phi2_min + k * phi2_delta);

        T one80Check = phi1_min + i * phi1_delta + phi2_min + k * phi2_delta;
        if(ebsdlib::math::closeEnough(static_cast<T>(ebsdlib::constants::k_PiD), one80Check, static_cast<T>(1.0E-6)))
        {
          eulers->setComponent(counter, 0, phi1_min + i * phi1_delta + .1);
          eulers->setComponent(counter, 2, phi2_min + k * phi2_delta + .1);
        }

        one80Check = fmod(one80Check, ebsdlib::constants::k_2PiD);
        if(ebsdlib::math::closeEnough(static_cast<T>(ebsdlib::constants::k_PiD), one80Check, static_cast<T>(1.0E-6)))
        {
          eulers->setComponent(counter, 0, phi1_min + i * phi1_delta + .1);
          eulers->setComponent(counter, 2, phi2_min + k * phi2_delta + .1);
        }

        counter++;
      }
    }
  }

  typename EulerConverter<EbsdDataArray<T>, T>::Pointer euConv = EulerConverter<EbsdDataArray<T>, T>::New();
  euConv->setInputData(eulers);

  euConv->toOrientationMatrix();
  typename EbsdDataArray<T>::Pointer om = euConv->getOutputData();
  om->setName(k_InputNames[1]);
  attrMat[k_InputNames[1]] = om;

  // Create an Orientation matrix from the Eulers and then transform BACK to Eulers to transform
  // the values of the Eulers into the convention set forth in the Rotations Paper.
  typename OrientationMatrixConverter<EbsdDataArray<T>, T>::Pointer omConv = OrientationMatrixConverter<EbsdDataArray<T>, T>::New();
  omConv->setInputData(om);
  omConv->toEulers();
  eulers = omConv->getOutputData();
  eulers->setName(k_InputNames[0]);
  euConv->setInputData(eulers);

  attrMat[k_InputNames[0]] = eulers;

  euConv->toQuaternion();
  typename EbsdDataArray<T>::Pointer q = euConv->getOutputData();
  q->setName(k_InputNames[2]);
  attrMat[k_InputNames[2]] = q;

  euConv->toAxisAngle();
  typename EbsdDataArray<T>::Pointer ax = euConv->getOutputData();
  ax->setName(k_InputNames[3]);
  attrMat[k_InputNames[3]] = ax;

  euConv->toRodrigues();
  typename EbsdDataArray<T>::Pointer ro = euConv->getOutputData();
  ro->setName(k_InputNames[4]);
  attrMat[k_InputNames[4]] = ro;

  euConv->toHomochoric();
  typename EbsdDataArray<T>::Pointer ho = euConv->getOutputData();
  ho->setName(k_InputNames[5]);
  attrMat[k_InputNames[5]] = ho;

  euConv->toCubochoric();
  typename EbsdDataArray<T>::Pointer cu = euConv->getOutputData();
  cu->setName(k_InputNames[6]);
  attrMat[k_InputNames[6]] = cu;

  euConv->toStereographic();
  typename EbsdDataArray<T>::Pointer st = euConv->getOutputData();
  st->setName(k_InputNames[7]);
  attrMat[k_InputNames[7]] = st;
}
} // namespace generate_test_data
