# pyebsd #

*pyebsd* reads .ang and .ctf files. Currently only supports reading header information.

## Requirements ##

+ Python 3.8+

## Examples ##

```
import ebsd

header = ebsd.ang.parse_header('/path/to/file.ang')

for key, value in header.entries.items():
  print(f'{key}, {value}')
```
