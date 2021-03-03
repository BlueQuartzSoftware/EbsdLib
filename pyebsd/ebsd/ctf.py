from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Callable, Dict, Final, Generator, List, Tuple

from ._utils import file_line_generator

__all__ = ['CtfPhase', 'CtfHeader', 'parse_header', 'parse_header_as_dict']

CTF_DELIMITER: Final[str] = '\t'

CTF_CHANNEL_TEXT_FILE: Final[str] = 'Channel Text File'
CTF_COLON_CHANNEL_TEXT_FILE: Final[str] = ':Channel Text File'
CTF_PRJ: Final[str] = 'Prj'
CTF_PHASES: Final[str] = 'Phases'

CTF_LONG_TEXT: Final[str] = 'Euler angles refer to Sample Coordinate system (CS0)!'
CTF_MAG: Final[str] = 'Mag'
CTF_COVERAGE: Final[str] = 'Coverage'
CTF_DEVICE: Final[str] = 'Device'
CTF_KV: Final[str] = 'KV'
CTF_TILT_ANGLE: Final[str] = 'TiltAngle'
CTF_TILT_AXIS: Final[str] = 'TiltAxis'

CTF_AUTHOR: Final[str] = 'Author'
CTF_JOB_MODE: Final[str] = 'JobMode'
CTF_X_CELLS: Final[str] = 'XCells'
CTF_Y_CELLS: Final[str] = 'YCells'
CTF_Z_CELLS: Final[str] = 'ZCells'
CTF_X_STEP: Final[str] = 'XStep'
CTF_Y_STEP: Final[str] = 'YStep'
CTF_Z_STEP: Final[str] = 'ZStep'
CTF_ACQ_E1: Final[str] = 'AcqE1'
CTF_ACQ_E2: Final[str] = 'AcqE2'
CTF_ACQ_E3: Final[str] = 'AcqE3'
CTF_EULER: Final[str] = 'Euler'

def euro_to_us_dec_string(text: str) -> str:
  return text.replace(',', '.')

def parse_float(text: str) -> float:
  return float(euro_to_us_dec_string(text))

CTF_HEADER_PARSE_MAP: Final[Dict[str, Callable[[str], Any]]] = {
  CTF_AUTHOR : str,
  CTF_JOB_MODE : str,
  CTF_X_CELLS : int,
  CTF_Y_CELLS : int,
  CTF_Z_CELLS : int,
  CTF_X_STEP : parse_float,
  CTF_Y_STEP : parse_float,
  CTF_Z_STEP : parse_float,
  CTF_ACQ_E1 : parse_float,
  CTF_ACQ_E2 : parse_float,
  CTF_ACQ_E3 : parse_float,
  CTF_EULER : str,
}

@dataclass
class CtfPhase:
  index: int = 0
  lattice_constants: Tuple[float, float, float] = (0.0, 0.0, 0.0)
  lattice_angles: Tuple[float, float, float] = (0.0, 0.0, 0.0)
  name: str = ''
  group: int = 0
  space_group: int = 0
  comment: str = ''
  internal1: str = ''
  internal2: str = ''

class LaueGroup(IntEnum):
  TRICLINIC = 1
  MONOCLINIC = 2
  ORTHORHOMBIC = 3
  TETRAGONAL_LOW = 4
  TETRAGONAL_HIGH = 5
  TRIGONAL_LOW = 6
  TRIGONAL_HIGH = 7
  HEXAGONAL_LOW = 8
  HEXAGONAL_HIGH = 9
  CUBIC_LOW = 10
  CUBIC_HIGH = 11
  UNKNOWN_SYMMETRY = 12

class CtfHeader:
  def __init__(self) -> None:
    self.phases: Dict[int, CtfPhase] = {}
    self.entries: Dict[str, Any] = {}
    self.unknown_entries: Dict[str, str] = {}

def parse_float_triplet(text: str) -> Tuple[float, float, float]:
  tokens = euro_to_us_dec_string(text).split(';')
  return (float(tokens[0]), float(tokens[1]), float(tokens[2]))

def parse_phases(file: Generator[str, None, None], num_phases: int) -> List[CtfPhase]:
  phases: List[CtfPhase] = []

  for i in range(num_phases):
    line = next(file).strip()
    tokens = line.split('\t')
    lattice_constants = parse_float_triplet(tokens[0])
    lattice_angles = parse_float_triplet(tokens[1])
    name = tokens[2]
    group = LaueGroup(int(tokens[3]))

    comment = ''
    space_group = 0
    internal1 = ''
    internal2 = ''

    if len(tokens) == 5:
      comment = tokens[4]
    elif len(tokens) == 8:
      space_group = int(tokens[4])
      internal1 = tokens[5]
      internal2 = tokens[6]
      comment = tokens[7]

    phase = CtfPhase(i, lattice_constants, lattice_angles, name, group, space_group, comment, internal1, internal2)
    phases.append(phase)

  return phases

def parse_ctf_long_text(tokens: List[str]) -> Dict[str, Any]:
  mag = int(tokens[2])
  coverage = int(tokens[4])
  device = int(tokens[6])
  kv = int(tokens[8])
  tilt_angle = float(tokens[10])
  tilt_axis = float(tokens[12])
  entries = {
    CTF_MAG : mag,
    CTF_COVERAGE : coverage,
    CTF_DEVICE : device,
    CTF_KV : kv,
    CTF_TILT_ANGLE : tilt_angle,
    CTF_TILT_AXIS : tilt_axis,
  }
  return entries

def parse_header(filepath: str) -> CtfHeader:
  file_gen = file_line_generator(filepath)
  ctf_header = CtfHeader()
  for line in file_gen:
    line = line.strip()
    tokens = line.split(CTF_DELIMITER)
    keyword = tokens[0]
    if line.startswith(CTF_CHANNEL_TEXT_FILE) or line.startswith(CTF_COLON_CHANNEL_TEXT_FILE):
      continue
    elif line.startswith(CTF_PRJ):
      ctf_header.entries[CTF_PRJ] = line[len(CTF_PRJ) + 1:]
    elif line.startswith(CTF_LONG_TEXT):
      entries = parse_ctf_long_text(tokens)
      ctf_header.entries.update(entries)
    elif line.startswith(CTF_PHASES):
      phase_num = int(tokens[1])
      ctf_header.phases = parse_phases(file_gen, phase_num)
      break
    elif keyword in CTF_HEADER_PARSE_MAP:
      value = None
      if len(tokens) > 1:
        value = CTF_HEADER_PARSE_MAP[keyword](CTF_DELIMITER.join(tokens[1:]))
      ctf_header.entries[keyword] = value
    else:
      ctf_header.unknown_entries[keyword] = CTF_DELIMITER.join(tokens[1:])
  return ctf_header

def parse_header_as_dict(filepath: str) -> dict:
  header = parse_header(filepath)
  entries = header.entries
  phases = header.phases

  phases_dict = _get_phases_as_dict(phases)
  entries["Phases"] = phases_dict

  return entries

def _get_phases_as_dict(phases: Dict[int, CtfPhase]) -> dict:
  phases_dict: dict = {}
  for x in range(len(phases)):
    phase = phases[x]

    phase_dict: dict = {}
    phase_dict['LaueGroup'] = str(phase.group.name)
    phase_dict['Internal1'] = phase.internal1
    phase_dict['Internal2'] = phase.internal2
    phase_dict['LatticeAngles'] = phase.lattice_angles
    phase_dict['LatticeConstants'] = phase.lattice_constants
    phase_dict['Name'] = phase.name
    phase_dict['SpaceGroup'] = phase.space_group
    phase_dict['Comment'] = phase.comment
    phase_num = x + 1
    phases_dict[f"Phase {phase_num}"] = phase_dict

  return phases_dict
