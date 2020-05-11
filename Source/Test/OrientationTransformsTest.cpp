/* ============================================================================
 * Copyright (c) 2009-2016 BlueQuartz Software, LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of BlueQuartz Software, the US Air Force, nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The code contained herein was partially funded by the followig contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include <iostream>
#include <limits>
#include <map>
#include <vector>
#include <string>

#include <QtCore/QString>
#include <QtCore/QVector>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/OrientationMath/OrientationArray.hpp"
#include "EbsdLib/OrientationMath/OrientationConverter.hpp"
#include "EbsdLib/OrientationMath/OrientationTransforms.hpp"

#include "GenerateFunctionList.h"
#include "UnitTestSupport.hpp"
#include "EbsdLibTestFileLocations.h"
#include "TestPrintFunctions.h"

/*
 *

 DREAM.3D Testing

 | From/To |  e   |  o   |  a   |  r   |  q   |  h   |  c   |
 |  -      |  -   |  -   |  -   |  -   |  -   |  -   |  -   |
 |  e      |  #   |  X   |  X   |  X   |  X   |  X   |      |
 |  o      |  X   |  #   |  X   |  X   |  X   |  X   |      |
 |  a      |  X   |  X   |  #   |  X   |  X   |  X   |      |
 |  r      |  X   |  X   |  X   |  #   |  X   |  X   |      |
 |  q      |  X   |  X   |  X   |  X   |  #   |  X   |      |
 |  h      |  X   |  X   |  X   |  X   |  X   |  #   |      |
 |  c      |      |      |      |      |      |      |  #   |


 */

class OrientationTransformsTest
{
public:
  OrientationTransformsTest() = default;
  virtual ~OrientationTransformsTest() = default;

  std::vector<QString> DataSetNames;
  std::vector<int32_t> DataSetTypes;

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void RemoveTestFiles()
  {
#if REMOVE_TEST_FILES
// QFile::remove();
#endif
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T>
  void GenerateEulers(size_t nSteps, std::map<QString, typename DataArray<T>::Pointer>& attrMat)
  {
    std::vector<size_t> cDims(1, 3);

    T phi1_min = static_cast<T>(0.0);
    T phi1_max = DConst::k_2Pi;
    T phi1_delta = (phi1_max - phi1_min) / static_cast<T>(nSteps);

    T phi_min = static_cast<T>(0.0);
    T phi_max = DConst::k_Pi;
    T phi_delta = (phi_max - phi_min) / static_cast<T>(nSteps);

    T phi2_min = static_cast<T>(0.0);
    T phi2_max = DConst::k_2Pi;
    T phi2_delta = (phi2_max - phi2_min) / static_cast<T>(nSteps);

    size_t nStepsCubed = (nSteps + 1) * (nSteps + 1) * (nSteps + 1);
    typename DataArray<T>::Pointer eulers = DataArray<T>::CreateArray(nStepsCubed, cDims, k_InputNames[0]);

    size_t counter = 0;
    for(int i = 0; i <= nSteps; i++)
    {
      for(int j = 0; j <= nSteps; j++)
      {
        for(int k = 0; k <= nSteps; k++)
        {
          //        std::cout << "Euler[" << counter << "]: "
          //                  << (phi1_min+i*phi1_delta)*DConst::k_180OverPi << ", "
          //                   << (phi_min+j*phi_delta)*DConst::k_180OverPi  << ", "
          //                      << (phi2_min+k*phi2_delta)*DConst::k_180OverPi  << std::endl;

          eulers->setComponent(counter, 0, phi1_min + i * phi1_delta);
          eulers->setComponent(counter, 1, phi_min + j * phi_delta);
          eulers->setComponent(counter, 2, phi2_min + k * phi2_delta);

          T one80Check = phi1_min + i * phi1_delta + phi2_min + k * phi2_delta;
          if(EbsdLibMath::closeEnough(static_cast<T>(EbsdLib::Constants::k_Pi), one80Check, static_cast<T>(1.0E-6)))
          {
            eulers->setComponent(counter, 0, phi1_min + i * phi1_delta + .1);
            eulers->setComponent(counter, 2, phi2_min + k * phi2_delta + .1);
          }

          one80Check = fmod(one80Check, EbsdLib::Constants::k_2Pi);
          if(EbsdLibMath::closeEnough(static_cast<T>(EbsdLib::Constants::k_Pi), one80Check, static_cast<T>(1.0E-6)))
          {
            eulers->setComponent(counter, 0, phi1_min + i * phi1_delta + .1);
            eulers->setComponent(counter, 2, phi2_min + k * phi2_delta + .1);
          }

          counter++;
        }
      }
    }

    typename EulerConverter<T>::Pointer euConv = EulerConverter<T>::New();
    euConv->setInputData(eulers);

    euConv->toOrientationMatrix();
    typename DataArray<T>::Pointer om = euConv->getOutputData();
    om->setName(k_InputNames[1]);
    attrMat[k_InputNames[1]] = om;

    // Create an Orientation matrix from the Eulers and then transform BACK to Eulers to transform
    // the values of the Eulers into the convention set forth in the Rotations Paper.
    typename OrientationMatrixConverter<T>::Pointer omConv = OrientationMatrixConverter<T>::New();
    omConv->setInputData(om);
    omConv->toEulers();
    eulers = omConv->getOutputData();
    eulers->setName(k_InputNames[0]);
    euConv->setInputData(eulers);

    attrMat[k_InputNames[0]] = eulers;

    euConv->toQuaternion();
    typename DataArray<T>::Pointer q = euConv->getOutputData();
    q->setName(k_InputNames[2]);
    attrMat[k_InputNames[2]] = q;

    euConv->toAxisAngle();
    typename DataArray<T>::Pointer ax = euConv->getOutputData();
    ax->setName(k_InputNames[3]);
    attrMat[k_InputNames[3]] = ax;

    euConv->toRodrigues();
    typename DataArray<T>::Pointer ro = euConv->getOutputData();
    ro->setName(k_InputNames[4]);
    attrMat[k_InputNames[4]] = ro;

    euConv->toHomochoric();
    typename DataArray<T>::Pointer ho = euConv->getOutputData();
    ho->setName(k_InputNames[5]);
    attrMat[k_InputNames[5]] = ho;

    euConv->toCubochoric();
    typename DataArray<T>::Pointer cu = euConv->getOutputData();
    cu->setName(k_InputNames[6]);
    attrMat[k_InputNames[6]] = cu;
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename T>
  std::shared_ptr<DataArray<T>> generateRepresentation(int32_t inputType, int32_t outputType, typename DataArray<T>::Pointer inputOrientations)
  {
    using ArrayType = typename DataArray<T>::Pointer;
    using OCType = OrientationConverter<T>;

    QVector<typename OCType::Pointer> converters(7);

    converters[0] = EulerConverter<T>::New();
    converters[1] = OrientationMatrixConverter<T>::New();
    converters[2] = QuaternionConverter<T>::New();
    converters[3] = AxisAngleConverter<T>::New();
    converters[4] = RodriguesConverter<T>::New();
    converters[5] = HomochoricConverter<T>::New();
    converters[6] = CubochoricConverter<T>::New();

    QVector<OrientationRepresentation::Type> ocTypes = OCType::GetOrientationTypes();

    converters[inputType]->setInputData(inputOrientations);
    converters[inputType]->convertRepresentationTo(ocTypes[outputType]);

    return converters[inputType]->getOutputData();
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  QString ExecuteConvertFilter(std::map<QString, DataArray<double>::Pointer>& attrMat, GenerateFunctionList::EntryType& entry, int e, const QString& outputName)
  {
    QString inputName = outputName;

    if(e == 0)
    {
      inputName = k_InputNames[entry[e]];
    }

    DataArray<double>::Pointer inputData = attrMat[inputName];
    DataArray<double>::Pointer outputData = generateRepresentation<double>(entry[e], entry[e + 1], inputData);
    QString nextOutputName = QString::number(e) + QString("_") + k_InputNames[entry[e]] + QString("2") + k_InputNames[entry[e + 1]];
    attrMat[nextOutputName] = outputData;

    return nextOutputName;
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename K>
  void CheckRepresentation(K* data, int repType)
  {
    using OrientationArrayType = OrientationArray<K>;
    using OrTr_Type = OrientationTransforms<OrientationArray<K>, K>;

    typename OrTr_Type::ResultType res;

    OrientationArrayType wrapper(data, k_CompDims[repType]);
    switch(repType)
    {
    case 0:
      res = OrTr_Type::eu_check(wrapper);
      break;
    case 1:
      res = OrTr_Type::om_check(wrapper);
      break;
    case 2:
      res = OrTr_Type::qu_check(wrapper);
      break;
    case 3:
      res = OrTr_Type::ax_check(wrapper);
      break;
    case 4:
      res = OrTr_Type::ro_check(wrapper);
      break;
    case 5:
      res = OrTr_Type::ho_check(wrapper);
      break;
    case 6:
      res = OrTr_Type::cu_check(wrapper);
      break;
    default:
      break;
    }

    if(res.result <= 0)
    {
      std::cout << res.msg << std::endl;
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  template <typename K>
  void RunTestCase(GenerateFunctionList::EntryType& entryRef, size_t nSteps)
  {

    using DataArrayType = DataArray<K>;
    using DataArrayPointerType = typename DataArrayType::Pointer;

    std::map<QString, DataArrayPointerType> attrMat;

    try
    {
      DataSetNames.clear();
      DataSetTypes.clear();

      GenerateFunctionList::EntryType entry = entryRef;
      // QVector<QString> funcNames = EulerConverter<K>::GetOrientationTypeStrings();

      std::stringstream ss;
      for(int e = 0; e < entry.size() - 1; e++)
      {
        ss << k_InputNames[entry[e]].toStdString() << "2" << k_InputNames[entry[e + 1]].toStdString();
        if(e != entry.size() - 1)
        {
          ss << "\t";
        }
      }
      // std::cout << "####################################################################" << std::endl;
      // std::cout << ss.str() << std::endl;
      std::string testName = ss.str();

      SIMPL::unittest::CurrentMethod = ss.str();
      SIMPL::unittest::numTests++;
      std::cout << "Starting Test " << ss.str() << " -----------------------------------------------------" << std::endl;

      size_t nStepsCubed = (nSteps + 1) * (nSteps + 1) * (nSteps + 1);

      // Make all the starting data
      GenerateEulers<K>(nSteps, attrMat);

      bool euCheck = false;
      //  QString outputName;

      if(entry[0] == 0)
      {
        // std::cout << "CHECK EULERS!!" << std::endl;
        euCheck = true;
        entry.push_back(1); // Add an extra conversion to OM for Eulers since there can be ambiguous cases
      }
      else
      {

        for(int e = 0; e < entry.size(); e++)
        {
          if(entry[e] == 0)
          {
            GenerateFunctionList::EntryType::iterator iter = entry.begin() + e + 1;
            entry.insert(iter, 1);
            iter = entry.begin() + e + 2;
            entry.insert(iter, 0);
            e = e + 2;
          }
        }
      }

      QString outputName; // We need this a bit further down;
      for(int e = 0; e < entry.size() - 1; e++)
      {
        outputName = ExecuteConvertFilter(attrMat, entry, e, outputName);
      }

      // If we started with Eulers, then we need to convert the original Eulers and
      // the final eulers to an Orientation Matrix and back due to ambiguities when
      // transforming Eulers. Going to an Orientation Matrix with 4 degrees of freedom
      // will give us unique Eulers back which will be numerically equivalent.
      QString inputName;

      // Find Difference Map between originals and finals
      {
        size_t cDim = entry[0];
        if(euCheck)
        {
          inputName = k_InputNames[1]; // We converted the ending eulers to an Orientation Matrix so compare against the original OM
          cDim = 1;                    // Use the CompDim from the OM instead of what is coming in
        }
        else
        {
          inputName = k_InputNames[entry[0]];
        }

        DataArrayType& inputArray = *(attrMat[inputName]);
        DataArrayType& outputArray = *(attrMat[outputName]);

        DataArrayPointerType diffArray = inputArray.createNewArray(inputArray.getNumberOfTuples(), inputArray.getComponentDimensions(), "Difference");
        DataArrayType& diff = *(diffArray);
        diff.initializeWithZeros();

        for(size_t i = 0; i < inputArray.getSize(); i++)
        {

          diff[i] = fabs(inputArray[i] - outputArray[i]);
        }

        size_t tuples = diff.getNumberOfTuples();
        // printf("Total Tuples: %lu\n", tuples);
        size_t numErrors = 0;
        K thr = 1.0E-3;
        for(size_t t = 0; t < tuples; t++)
        {
          int nComp = diff.getNumberOfComponents();
          if(entry[0] == 4)
          {
            nComp--;
          } // for Rodrigues vectors we only want to compare the first 3 components.
          for(int c = 0; c < nComp; c++)
          {
            K delta = fabs(diff.getComponent(t, c));
            if(delta > thr)
            {
              numErrors++;
              std::cout << "Delta Failed: " << delta << " DataArray: '" << diff.getName().toStdString() << "' Tuple[" << t << "] Comp[" << c << "] Value:" << diff.getComponent(t, c) << std::endl;

              // Get the AttributeMatrix:
              //              dap = DataArrayPath(DCName, AMName, k_InputNames[0]);
              //              AttributeMatrix::Pointer attrMat = dca->getAttributeMatrix(dap);

              // Print the Euler Angle that we Started with
              // cDims[0] = k_CompDims[0];
              DataArrayPointerType data = attrMat[k_InputNames[0]];
              OrientationPrinters::PrintTuple<K>(data, t);
              CheckRepresentation<K>(data->getPointer(t), 0);

              // Print the starting representation
              data = attrMat[k_InputNames[entry[0]]];
              OrientationPrinters::PrintTuple<K>(data, t);
              CheckRepresentation<K>(data->getPointer(t), entry[0]);

              // Now print all the intermediate Representations
              for(int q = 0; q < DataSetNames.size(); q++)
              {
                data = attrMat[DataSetNames[q]];
                OrientationPrinters::PrintTuple<K>(data, t);
                CheckRepresentation<K>(data->getPointer(t), DataSetTypes[q]);
              }

              DREAM3D_REQUIRED(delta, <=, thr)
              break;
            }
          }
        }
        //   printf("numErrors: %llu\n", numErrors)
      }

      if(euCheck)
      {
        // Use original OM when we first generated the Euler Angles
        entry.pop_back();
      }

      typename DataArray<K>::Pointer junk = DataArray<K>::CreateArray(1, "Junk");
      QString typeName = junk->getTypeAsString();
#if 0
      {

        AbstractFilter::Pointer writer = InstantiateFilter("DataContainerWriter");
        DREAM3D_REQUIRE_VALID_POINTER(writer.get())
        writer->setDataContainerArray(dca);

        QString outputFile;
        QTextStream out(&outputFile);

        out << UnitTest::TestTempDir << "/OrientationTransformsTest_";

        for(int e = 0; e < entry.size(); e++)
        {
          out << k_InputNames[entry[e]];
          if(e < entry.size())
          {
            out << "_";
          }
        }
        out << typeName << ".dream3d";
        var.setValue(outputFile);
        propWasSet = writer->setProperty("OutputFile", var);
        if(!propWasSet)
        {
          qDebug() << "Unable to set property OutputFile";
        }
        writer->execute();
        int err = writer->getErrorCode();
        DREAM3D_REQUIRED(err, >=, 0)
      }
#endif

      {
        ss.str("");
        ss << testName << "Type: " << typeName.toStdString();
        TestPassed(ss.str());
        SIMPL::unittest::CurrentMethod = "";
      }
    } catch(TestException& e)
    {
      TestFailed(SIMPL::unittest::CurrentMethod);
      std::cout << e.what() << std::endl;
    }
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void StartTest()
  {
    //  QVector<QString> functionNames = OrientationConverter<float>::GetOrientationTypeStrings();

    GenerateFunctionList generator;
    std::vector<GenerateFunctionList::EntryType> entries = generator.GeneratePermutationsOfCombinations(7, 2);

    // This outer loop will group the tests based on the first orientation representation
    for(int t = 0; t < 7; t++)
    {
      // Start looping on each entry in the function table.
      for(auto& entry : entries)
      {
        if(entry[0] != t)
        {
          continue;
        }
        entry.push_back(entry[0]);
        //         if (entry[0] == 2
        //              && entry[1] == 1
        //             )
        {
          // RunTestCase<float>(entry, 16);
          RunTestCase<double>(entry, 16);
        }
      }
    }

    entries = generator.GeneratePermutationsOfCombinations(7, 3);
    // This outer loop will group the tests based on the first orientation representation
    for(int t = 0; t < 7; t++)
    {
      for(auto& entry : entries)
      {
        if(entry[0] != t)
        {
          continue;
        }
        entry.push_back(entry[0]);
        //         if (entry[0] == 2
        //              && entry[1] == 4
        //              && entry[2] == 1
        //             )
        {
          //  RunTestCase<float>(entry, 16);
          RunTestCase<double>(entry, 16);
        }
      }
    }
  }

  void operator()()
  {
    int err = EXIT_SUCCESS;

    StartTest();

    DREAM3D_REGISTER_TEST(RemoveTestFiles());
  }

public:
  OrientationTransformsTest(const OrientationTransformsTest&) = delete;            // Copy Constructor Not Implemented
  OrientationTransformsTest(OrientationTransformsTest&&) = delete;                 // Move Constructor Not Implemented
  OrientationTransformsTest& operator=(const OrientationTransformsTest&) = delete; // Copy Assignment Not Implemented
  OrientationTransformsTest& operator=(OrientationTransformsTest&&) = delete;      // Move Assignment Not Implemented
};
