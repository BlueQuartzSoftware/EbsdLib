/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * CLI entry for render_ebsd. Parses command-line flags and dispatches to
 * ebsdlib::render_ebsd::run(). The actual logic lives in render_ebsd.cpp so
 * the same translation unit can be linked into the EbsdLibUnitTest binary.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "Apps/render_ebsd.h"

#include "EbsdLib/Core/EbsdLibConstants.h"

#include <array>
#include <iostream>
#include <string>
#include <vector>

namespace
{
void printUsage()
{
  std::cout << "Usage: render_ebsd <input.ang|input.ctf> <output_dir>" << std::endl;
  std::cout << "                   [--convention {x_a, x_astar}]    (default x_astar)" << std::endl;
  std::cout << "                   [--color-key  {tsl, pucm, nh}]    (default tsl)" << std::endl;
  std::cout << "                   [--phase N]                       (default: all phases)" << std::endl;
  std::cout << "                   [--ref-dir x,y,z]                 (default 0,0,1)" << std::endl;
  std::cout << "                   [--image-dim N] [--lambert-dim N] [--legend-dim N]" << std::endl;
}

bool parseRefDir(const std::string& s, std::array<float, 3>& out)
{
  std::vector<std::string> parts;
  std::string cur;
  for(char c : s)
  {
    if(c == ',')
    {
      parts.push_back(cur);
      cur.clear();
    }
    else
    {
      cur += c;
    }
  }
  parts.push_back(cur);
  if(parts.size() != 3)
  {
    return false;
  }
  try
  {
    out[0] = std::stof(parts[0]);
    out[1] = std::stof(parts[1]);
    out[2] = std::stof(parts[2]);
  } catch(...)
  {
    return false;
  }
  return true;
}
} // namespace

int main(int argc, char* argv[])
{
  // Allow `render_ebsd --help` / `-h` with no positional args.
  if(argc >= 2)
  {
    std::string a1 = argv[1];
    if(a1 == "--help" || a1 == "-h")
    {
      printUsage();
      return 0;
    }
  }

  if(argc < 3)
  {
    std::cerr << "ERROR: missing positional arguments. Need <input.ang|input.ctf> <output_dir>." << std::endl;
    printUsage();
    return 1;
  }

  // Catch the common mistake of forgetting <output_dir>: argv[2] then ends up
  // being a flag like "--convention", and the rest of argv would be parsed
  // out of position. Reject up front with a clear message.
  auto looksLikeFlag = [](const char* s) { return s != nullptr && s[0] == '-' && s[1] == '-'; };
  if(looksLikeFlag(argv[1]))
  {
    std::cerr << "ERROR: first positional argument must be the input .ang/.ctf file, got flag '" << argv[1] << "'." << std::endl;
    printUsage();
    return 1;
  }
  if(looksLikeFlag(argv[2]))
  {
    std::cerr << "ERROR: second positional argument must be the output directory, got flag '" << argv[2] << "'." << std::endl;
    std::cerr << "       It looks like <output_dir> was omitted. Insert it before any --flag." << std::endl;
    printUsage();
    return 1;
  }

  ebsdlib::render_ebsd::Options opts;
  opts.inputFile = argv[1];
  opts.outputDir = argv[2];

  for(int i = 3; i < argc; i++)
  {
    std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if(i + 1 >= argc)
      {
        return std::string{};
      }
      return std::string{argv[++i]};
    };
    if(arg == "--convention")
    {
      std::string v = next();
      if(v == "x_a")
      {
        opts.convention = ebsdlib::HexConvention::XParallelA;
      }
      else if(v == "x_astar")
      {
        opts.convention = ebsdlib::HexConvention::XParallelAStar;
      }
      else
      {
        std::cerr << "ERROR: --convention must be x_a or x_astar (got '" << v << "')" << std::endl;
        return 1;
      }
    }
    else if(arg == "--color-key")
    {
      std::string v = next();
      if(v == "tsl")
      {
        opts.colorKey = ebsdlib::ColorKeyKind::TSL;
      }
      else if(v == "pucm")
      {
        opts.colorKey = ebsdlib::ColorKeyKind::PUCM;
      }
      else if(v == "nh")
      {
        opts.colorKey = ebsdlib::ColorKeyKind::NolzeHielscher;
      }
      else
      {
        std::cerr << "ERROR: --color-key must be tsl, pucm, or nh (got '" << v << "')" << std::endl;
        return 1;
      }
    }
    else if(arg == "--phase")
    {
      try
      {
        opts.phaseFilter = std::stoi(next());
      } catch(...)
      {
        std::cerr << "ERROR: --phase requires an integer" << std::endl;
        return 1;
      }
    }
    else if(arg == "--ref-dir")
    {
      if(!parseRefDir(next(), opts.refDir))
      {
        std::cerr << "ERROR: --ref-dir must be x,y,z (three comma-separated floats)" << std::endl;
        return 1;
      }
    }
    else if(arg == "--image-dim")
    {
      opts.imageDim = std::stoi(next());
    }
    else if(arg == "--lambert-dim")
    {
      opts.lambertDim = std::stoi(next());
    }
    else if(arg == "--legend-dim")
    {
      opts.legendImageDim = std::stoi(next());
    }
    else if(arg == "--help" || arg == "-h")
    {
      printUsage();
      return 0;
    }
    else
    {
      std::cerr << "ERROR: Unknown argument '" << arg << "'" << std::endl;
      printUsage();
      return 1;
    }
  }

  std::cout << "render_ebsd" << std::endl;
  std::cout << "  Input:      " << opts.inputFile << std::endl;
  std::cout << "  Output dir: " << opts.outputDir << std::endl;
  std::cout << "  Convention: " << (opts.convention == ebsdlib::HexConvention::XParallelA ? "X||a" : "X||a*") << std::endl;
  std::cout << "  Color key:  ";
  switch(opts.colorKey)
  {
  case ebsdlib::ColorKeyKind::TSL:
    std::cout << "TSL" << std::endl;
    break;
  case ebsdlib::ColorKeyKind::PUCM:
    std::cout << "PUCM" << std::endl;
    break;
  case ebsdlib::ColorKeyKind::NolzeHielscher:
    std::cout << "Nolze-Hielscher" << std::endl;
    break;
  }

  auto result = ebsdlib::render_ebsd::run(opts);
  if(!result.ok)
  {
    std::cerr << "render_ebsd: completed with errors (" << result.phases.size() << " phases attempted)" << std::endl;
    return 2;
  }
  std::cout << "render_ebsd: wrote outputs for " << result.phases.size() << " phase(s)" << std::endl;
  for(const auto& p : result.phases)
  {
    std::cout << "  Phase " << p.phaseIndex << " (" << p.phaseName << "): " << std::endl;
    std::cout << "    PF:     " << p.poleFigurePath << std::endl;
    std::cout << "    IPF:    " << p.ipfMapPath << std::endl;
    std::cout << "    LEGEND: " << p.legendPath << std::endl;
  }
  return 0;
}
