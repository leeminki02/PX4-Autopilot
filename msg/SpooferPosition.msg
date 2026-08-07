# Minimal position/velocity used by the GPS spoofing simulation for the emitter
# (physical) and target (claimed) roles. Deliberately separate from
# VehicleGlobalPosition/VehicleLocalPosition (rather than aliased via TOPICS onto
# them) to avoid growing those messages' already near-limit uORB format definitions.

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample

float64 lat			# Latitude, (degrees)
float64 lon			# Longitude, (degrees)
float32 alt			# Altitude AMSL, (meters)

float32 vx			# North velocity in NED local frame, (m/s)
float32 vy			# East velocity in NED local frame, (m/s)
float32 vz			# Down velocity in NED local frame, (m/s)

# TOPICS spoofer_position emitter_position target_position
