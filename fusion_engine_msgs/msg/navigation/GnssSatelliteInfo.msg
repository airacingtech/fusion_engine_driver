# GNSS constellation / system constants
uint8 UNKNOWN  = 0
uint8 GPS      = 1
uint8 GLONASS  = 2
uint8 LEO      = 3
uint8 GALILEO  = 4
uint8 BEIDOU   = 5
uint8 QZSS     = 6
uint8 MIXED   = 7
uint8 SBAS    = 8
uint8 IRNSS   = 9

uint8 system

# PRN or slot number
uint8 prn

# Status flags
# bit 0: used
# bit 1: unhealthy
# bit 2: NLOS
# bit 4: has ephemeris
# bit 5: has SBAS
uint8 usage

# Elevation in centi-degrees (-9000 to 9000), INVALID = 0x7FFF
float64 elevation

# Azimuth in centi-degrees (0–35999), INVALID = 0xFFFF
float64 azimuth