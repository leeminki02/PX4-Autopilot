# topic for receiving satellite's velocity and position data from ephemeris translator

uint64 timestamp		# time since system start (microseconds)
uint8 SAT_INFO_MAX_SATELLITES = 24

uint8 count			# Number of satellites visible to the receiver
uint8[20] svid	 		# Space vehicle ID [1..255], see scheme below

# ECEF Position of Satellites (m)
float64[20] p_x
float64[20] p_y
float64[20] p_z

# ECEF Velocity of Satellites (m/s)
float64[20] v_x
float64[20] v_y
float64[20] v_z

# TOPICS satellite_ecef
