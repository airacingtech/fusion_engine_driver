# Signal type (L1, L2, E5, etc.)
uint8 signal_type

# PRN / slot
uint8 prn

# C/N0 in 0.25 dB-Hz units
uint8 cn0

# Status flags
# bit 0: used pseudorange
# bit 1: used doppler
# bit 2: used carrier
# bit 3: carrier ambiguity resolved
# bit 4: valid pseudorange
# bit 5: valid doppler
# bit 6: carrier locked
# bit 8: has RTK
# bit 9: has SBAS
# bit 10: has ephemeris
uint16 status_flags