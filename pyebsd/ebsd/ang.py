from dataclasses import dataclass
from typing import Any, Dict, Final, Generator, List, Optional, Type

from ._utils import file_line_generator

__all__ = ['HKLFamily', 'AngPhase', 'AngHeader', 'parse_header', 'parse_header_as_dict']

ANG_HEADER_CHAR: Final[str] = '#'
ANG_PROPERTY_SEP: Final[str] = ':'

ANG_PHASE: Final[str] = 'Phase'
ANG_MATERIAL_NAME: Final[str] = 'MaterialName'
ANG_FORMULA: Final[str] = 'Formula'
ANG_INFO: Final[str] = 'Info'
ANG_SYMMETRY: Final[str] = 'Symmetry'
ANG_LATTICE_CONSTANTS: Final[str] = 'LatticeConstants'
ANG_NUMBER_FAMILIES: Final[str] = 'NumberFamilies'
ANG_HKL_FAMILIES: Final[str] = 'hklFamilies'
ANG_CATEGORIES: Final[str] = 'Categories'
ANG_POINT_GROUP_ID: Final[str] = 'PointGroupID'

ANG_HEADER: Final[str] = 'HEADER'
ANG_NOTES: Final[str] = 'NOTES'
ANG_COLUMN_NOTES: Final[str] = 'COLUMN_NOTES'
ANG_START: Final[str] = 'Start'
ANG_END: Final[str] = 'End'
ANG_START_NOTES: Final[str] = '# NOTES: Start'
ANG_END_NOTES: Final[str] = '# NOTES: End'
ANG_START_COLUMN_NOTES: Final[str] = '# COLUMN_NOTES: Start'
ANG_END_COLUMN_NOTES: Final[str] = '# COLUMN_NOTES: End'

ANG_TEM_PIX_PER_UM: Final[str] = 'TEM_PIXperUM'
ANG_X_STAR: Final[str] = 'x-star'
ANG_Y_STAR: Final[str] = 'y-star'
ANG_Z_STAR: Final[str] = 'z-star'
ANG_WORKING_DISTANCE: Final[str] = 'WorkingDistance'
ANG_SAMPLE_TILT_ANGLE: Final[str] = 'SampleTiltAngle'
ANG_CAMERA_ELEVATION_ANGLE: Final[str] = 'CameraElevationAngle'
ANG_CAMERA_AZIMUTHAL_ANGLE: Final[str] = 'CameraAzimuthalAngle'
ANG_GRID: Final[str] = 'GRID'
ANG_XSTEP: Final[str] = 'XSTEP'
ANG_YSTEP: Final[str] = 'YSTEP'
ANG_NCOLS_ODD: Final[str] = 'NCOLS_ODD'
ANG_NCOLS_EVEN: Final[str] = 'NCOLS_EVEN'
ANG_NROWS: Final[str] = 'NROWS'
ANG_OPERATOR: Final[str] = 'OPERATOR'
ANG_SAMPLEID: Final[str] = 'SAMPLEID'
ANG_SCANID: Final[str] = 'SCANID'
ANG_VERSION: Final[str] = 'VERSION'
ANG_COLUMN_COUNT: Final[str] = 'COLUMN_COUNT'
ANG_COLUMN_HEADERS: Final[str] = 'COLUMN_HEADERS'
ANG_COLUMN_UNITS: Final[str] = 'COLUMN_UNITS'

ANG_HEADER_PARSE_MAP: Final[Dict[str, Type]] = {
  ANG_TEM_PIX_PER_UM : float,
  ANG_X_STAR : float,
  ANG_Y_STAR : float,
  ANG_Z_STAR : float,
  ANG_WORKING_DISTANCE : float,
  ANG_SAMPLE_TILT_ANGLE: float,
  ANG_CAMERA_ELEVATION_ANGLE : float,
  ANG_CAMERA_AZIMUTHAL_ANGLE : float,
  ANG_GRID : str,
  ANG_XSTEP : float,
  ANG_YSTEP : float,
  ANG_NCOLS_ODD : int,
  ANG_NCOLS_EVEN : int,
  ANG_NROWS : int,
  ANG_OPERATOR : str,
  ANG_SAMPLEID : str,
  ANG_SCANID : str,
  ANG_VERSION : str,
  ANG_COLUMN_COUNT : int,
  ANG_COLUMN_HEADERS : str,
  ANG_COLUMN_UNITS : str,
}

@dataclass
class HKLFamily:
  h: int = 0
  k: int = 0
  l: int = 0
  s1: int = 0
  diffraction_intensity: float = 0.0

class AngPhase:
    def __init__(self) -> None:
      self.phase: int = 0
      self.material_name: str = ''
      self.formula: str = ''
      self.symmetry: int = 0
      self.lattice_constants: List[float] = []
      self.hkl_families: List[HKLFamily] = []
      self.categories: List[int] = []
      self.info: str = ''
      self.point_group_id: int = 0

class AngHeader:
  def __init__(self) -> None:
    self.phases: Dict[int, AngPhase] = {}
    self.entries: Dict[str, Any] = {}
    self.notes: str = ''
    self.col_notes: str = ''
    self.unknown_entries: Dict[str, str] = {}

def parse_hkl_family(tokens: List[str]) -> HKLFamily:
  h = int(tokens[0])
  k = int(tokens[1])
  l = int(tokens[2])
  s1 = int(tokens[3])
  diffraction_intensity = float(tokens[4])
  return HKLFamily(h, k, l, s1, diffraction_intensity)

def parse_families(file: Generator[str, None, None], num_families: int) -> List[HKLFamily]:
  families = []
  for _ in range(num_families):
    line = next(file)
    tokens = line.strip().split()
    if tokens[0] != ANG_HEADER_CHAR and tokens[1] != ANG_HKL_FAMILIES:
      raise RuntimeError(f'{ANG_HKL_FAMILIES} not found')
    families.append(parse_hkl_family(tokens[2:]))
  return families

def parse_categories(tokens: List[str]) -> List[int]:
  tokens[0] = tokens[0][len(ANG_CATEGORIES):]
  if not tokens[0]:
    tokens.pop(0)
  return [int(value) for value in tokens]

def join_lines_until(file: Generator[str, None, None], sentinel: str) -> str:
  return '\n'.join([line for line in iter(lambda: next(file).rstrip('\n'), sentinel)])

def parse_notes(file: Generator[str, None, None]) -> str:
  return join_lines_until(file, ANG_END_NOTES)

def parse_column_notes(file: Generator[str, None, None]) -> str:
  return join_lines_until(file, ANG_END_COLUMN_NOTES)

def parse_header(filepath: str) -> AngHeader:
  file_gen = file_line_generator(filepath)

  ang_header = AngHeader()
  current_phase: Optional[AngPhase] = None
  for line in file_gen:
    if not line.startswith(ANG_HEADER_CHAR):
      break
    else:
      tokens = line[1:].strip().split()
      if not tokens:
        continue
      keyword = tokens[0]
      if keyword[-1] == ANG_PROPERTY_SEP:
        keyword = keyword[:-1]
      if keyword == ANG_HEADER:
        continue
      elif keyword == ANG_NOTES and tokens[1] == ANG_START:
        ang_header.notes = parse_notes(file_gen)
      elif keyword == ANG_COLUMN_NOTES and tokens[1] == ANG_START:
        ang_header.col_notes = parse_column_notes(file_gen)
      elif keyword == ANG_PHASE:
        index = int(tokens[1])
        current_phase = AngPhase()
        current_phase.phase = index
        ang_header.phases[index] = current_phase
      elif keyword == ANG_MATERIAL_NAME:
        current_phase.material_name = tokens[1]
      elif keyword == ANG_FORMULA:
        current_phase.formula = tokens[1]
      elif keyword == ANG_INFO:
        current_phase.info = ''.join(tokens[1:])
      elif keyword == ANG_SYMMETRY:
        current_phase.symmetry = int(tokens[1])
      elif keyword == ANG_LATTICE_CONSTANTS:
        current_phase.lattice_constants = [float(value) for value in tokens[1:]]
      elif keyword == ANG_NUMBER_FAMILIES:
        num_families = int(tokens[1])
        current_phase.hkl_families = parse_families(file_gen, num_families)
      elif keyword.startswith(ANG_CATEGORIES):
        current_phase.categories = parse_categories(tokens)
      elif keyword == ANG_POINT_GROUP_ID:
        current_phase.point_group_id = int(tokens[1])
      elif keyword in ANG_HEADER_PARSE_MAP:
        value = None
        if len(tokens) > 1:
          value = ANG_HEADER_PARSE_MAP[keyword](' '.join(tokens[1:]))
        ang_header.entries[keyword] = value
      else:
        ang_header.unknown_entries[keyword] = ' '.join(tokens[1:])
  return ang_header

def parse_header_as_dict(filepath: str) -> dict:
  header = parse_header(filepath)
  entries = header.entries
  phases = header.phases

  phases_dict = _get_phases_as_dict(phases)
  entries["Phases"] = phases_dict

  return entries

def _get_phases_as_dict(phases: Dict[int, AngPhase]) -> dict:
  phases_dict: dict = {}
  for x in phases:
    phase = phases[x]

    phase_dict: dict = {}
    phase_dict['MaterialName'] = phase.material_name
    phase_dict['Formula'] = phase.formula
    phase_dict['Symmetry'] = phase.symmetry
    phase_dict['PointGroupID'] = phase.point_group_id
    phase_dict['LatticeConstants (ABC)'] = [phase.lattice_constants[0], phase.lattice_constants[1], phase.lattice_constants[2]]
    phase_dict['LatticeConstants (Alpha, Beta, Gamma)'] = [phase.lattice_constants[3], phase.lattice_constants[4], phase.lattice_constants[5]]
    phase_dict['NumberFamilies'] = len(phase.hkl_families)

    phases_dict[f"Phase {x}"] = phase_dict

  return phases_dict