uint8 GNSS_MEASX_MAX_SATELLITES = 24

uint64 timestamp		# time since system start (microseconds)
uint8  version			# Message version, currently 0x01
uint32 gpstow			# gpsTOW; GPS measurement reference time
uint32 glotow			# gloTOW; GLONASS measurement reference time
uint32 bdstow			# bdsTOW; BeiDou measurement reference time
uint32 qzsstow			# qzssTOW; QZSS measurement reference time
uint8  count			# Number of satellites visible to the receiver
uint8  flags			# Flags, TOW set (0 = no, 1 or 2 = yes)

uint8[24] gnssid		# GNSS ID
uint8[24] svid			# Satellite ID
uint8[24] cno			# carrier noise ratio (0..63)
int32[24] dopplerms		# Doppler Measurement [*0.04 m/s]
int32[24] dopplerhz		# Doppler Measurement [*0.2  Hz]
uint16[24] wholechips   # whole value of the code phase measurement (0..1022 for GPS)
uint16[24] fracchips    # fractional value of the code phase measurement (0..1023)
