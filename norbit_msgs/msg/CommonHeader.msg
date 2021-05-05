uint32 NORBIT_PREAMBLE_KEY = 3735928559 # 0xDEADBEEF

uint32 preable   # 0xDEADBEEF

uint32 TYPE_BATHYMETRIC = 1
uint32 TYPE_WATERCOLUMN = 2
uint32 TYPE_SNIPPET     = 4

uint32 type      # Set to 1 for bathymetric data
uint32 size      # Size of entire package in bytes
uint32 version
uint32 reserved
uint32 crc       # CRC32 of the size-24 bytes following the header (crc32, zlib-style SEED=0x00000000, POLY=0xEDB88320)
