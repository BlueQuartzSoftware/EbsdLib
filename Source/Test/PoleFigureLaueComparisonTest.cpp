/* ============================================================================
 * Pole Figure comparison tool — loops over every unique Laue class, generates
 * a tight cluster of orientations near a reference Euler, writes the Eulers
 * to a CSV (for import into MTEX) and the EbsdLib-rendered composite pole
 * figure to a TIFF. The companion MATLAB script in
 *   Code_Review/compare_pole_figures_all_laue.m
 * reads the same CSVs and writes MTEX pole figures as PNGs for visual
 * side-by-side comparison.
 * ============================================================================ */
#include <catch2/catch.hpp>

#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"
#include "EbsdLib/Utilities/PngWriter.h"
#include "EbsdLib/Utilities/PoleFigureCompositor.h"

#include <fmt/format.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace ebsdlib;

namespace
{
// Reference Bunge Euler angle in degrees. Chosen generic (no singularity).
constexpr double k_RefPhi1Deg = 45.0;
constexpr double k_RefPhiDeg = 60.0;
constexpr double k_RefPhi2Deg = 30.0;
// Gaussian 1-sigma spread per Euler component (degrees)
constexpr double k_SpreadDeg = 3.0;
constexpr size_t k_NumSamples = 500;

std::string SafePointGroup(const std::string& rpg)
{
  std::string safe = rpg;
  // Replace any character awkward in filesystem paths
  for(char& c : safe)
  {
    if(c == '/' || c == ' ')
    {
      c = '_';
    }
  }
  return safe;
}
} // namespace

TEST_CASE("ebsdlib::PoleFigureLaueComparisonTest::GenerateAllLaueClasses", "[EbsdLib][PoleFigureLaueComparisonTest]")
{
  const std::string baseDir = fmt::format("{}PoleFigureComparison", ebsdlib::unit_test::k_TestTempDir);
  std::filesystem::create_directories(baseDir);

  std::mt19937_64 rng(static_cast<std::mt19937_64::result_type>(12345));
  std::normal_distribution<double> noise(0.0, k_SpreadDeg * ebsdlib::constants::k_PiOver180D);

  const double refPhi1 = k_RefPhi1Deg * ebsdlib::constants::k_PiOver180D;
  const double refPhi = k_RefPhiDeg * ebsdlib::constants::k_PiOver180D;
  const double refPhi2 = k_RefPhi2Deg * ebsdlib::constants::k_PiOver180D;

  auto ops = LaueOps::GetAllOrientationOps();

  // Master index file
  std::ofstream master(fmt::format("{}/manifest.txt", baseDir));
  master << "# Pole Figure Laue-class comparison\n";
  master << fmt::format("# reference Euler (deg): {}, {}, {}\n", k_RefPhi1Deg, k_RefPhiDeg, k_RefPhi2Deg);
  master << fmt::format("# per-component noise sigma (deg): {}\n", k_SpreadDeg);
  master << fmt::format("# samples per class: {}\n", k_NumSamples);
  master << "# columns: opsIndex, rotationPointGroup, symmetryName, pole_figure_names\n";

  std::set<std::string> seen;
  for(size_t opsIndex = 0; opsIndex < ops.size(); ++opsIndex)
  {
    LaueOps::Pointer op = ops[opsIndex];
    const std::string rpg = op->getRotationPointGroup();
    if(seen.count(rpg) > 0)
    {
      continue;
    }
    seen.insert(rpg);

    const std::string safe = SafePointGroup(rpg);
    const std::string dir = fmt::format("{}/{}", baseDir, safe);
    std::filesystem::create_directories(dir);

    // Generate Euler samples with small Gaussian noise around the reference
    std::vector<float> eulers;
    eulers.reserve(k_NumSamples * 3);
    for(size_t i = 0; i < k_NumSamples; ++i)
    {
      eulers.push_back(static_cast<float>(refPhi1 + noise(rng)));
      eulers.push_back(static_cast<float>(refPhi + noise(rng)));
      eulers.push_back(static_cast<float>(refPhi2 + noise(rng)));
    }

    // Write CSV (phi1, Phi, phi2 in degrees)
    {
      std::ofstream csv(fmt::format("{}/pole_figure_input_eulers.csv", dir));
      csv << "phi1,Phi,phi2\n";
      for(size_t i = 0; i < k_NumSamples; ++i)
      {
        csv << eulers[3 * i + 0] * 180.0 / M_PI << "," << eulers[3 * i + 1] * 180.0 / M_PI << "," << eulers[3 * i + 2] * 180.0 / M_PI << "\n";
      }
    }

    // Build EbsdLib composite pole figure
    ebsdlib::FloatArrayType::Pointer eulersArr = ebsdlib::FloatArrayType::FromStdVector(eulers, k_NumSamples, 3ULL, "Eulers");
    ebsdlib::CompositePoleFigureConfiguration_t config;
    config.eulers = eulersArr.get();
    config.imageDim = 512;
    config.lambertDim = 128;
    config.numColors = 16;
    config.discrete = true;
    config.discreteHeatMap = false;
    // This test is compared side-by-side against MTEX (compare_pole_figures_all_laue.m),
    // which uses the X||a* basis, so pin X||a* here even though the library default is X||a.
    config.hexConvention = ebsdlib::HexConvention::XParallelAStar;
    config.laueOpsIndex = static_cast<uint32_t>(opsIndex);
    config.layoutType = ebsdlib::PoleFigureLayoutType::Horizontal;
    config.phaseName = rpg;
    config.phaseNumber = 1;
    config.title = fmt::format("{} <{}, {}, {}>", op->getSymmetryName(), k_RefPhi1Deg, k_RefPhiDeg, k_RefPhi2Deg);

    // Use the routing entry point so discrete (non-heatmap) figures go through the
    // vector marker renderer, matching WritePoleFigure / PoleFigureCompositorTest.
    CompositePoleFigureResult result = ebsdlib::GeneratePoleFigureComposite(config);
    REQUIRE(result.image != nullptr);

    const std::string tifPath = fmt::format("{}/ebsdlib_pole_figure.png", dir);
    auto writeResult = PngWriter::WriteColorImage(tifPath, result.width, result.height, 4, result.image->data());
    REQUIRE(writeResult.first == 0);

    auto pfNames = op->getDefaultPoleFigureNames(ebsdlib::HexConvention::XParallelAStar);
    master << fmt::format("{},{},{},\"{} / {} / {}\"\n", opsIndex, safe, op->getSymmetryName(), pfNames[0], pfNames[1], pfNames[2]);
    std::cout << fmt::format("Wrote {} -> {}/ (sym={}, PFs={}/{}/{})\n", rpg, dir, op->getSymmetryName(), pfNames[0], pfNames[1], pfNames[2]);
  }

  std::cout << "Manifest: " << baseDir << "/manifest.txt" << std::endl;
}
