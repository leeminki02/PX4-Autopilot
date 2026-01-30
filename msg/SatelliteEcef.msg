# topic for receiving satellite's velocity and position data from ephemeris translator

uint64 timestamp		# time since system start (microseconds)
uint8 SAT_INFO_MAX_SATELLITES = 18

uint8 count			# Number of satellites visible to the receiver
uint8[18] svid	 		# Space vehicle ID [1..255], see scheme below

# ECEF Position of Satellites (m)
float32[18] p_x
float32[18] p_y
float32[18] p_z

# ECEF Velocity of Satellites (m/s)
float32[18] v_x
float32[18] v_y
float32[18] v_z

# TOPICS satellite_ecef
