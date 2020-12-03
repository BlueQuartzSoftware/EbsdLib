from typing import Generator

def file_line_generator(file_path: str) -> Generator[str, None, None]:
  with open(file_path, 'r') as file:
    line: str
    for line in file:
      yield line
