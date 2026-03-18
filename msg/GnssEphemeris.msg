uint8 GNSS_SFRBX_MAX_WORDS = 10

uint64 timestamp                # time since system start (microseconds)

uint8 gnssid                    # GNSS identifier
uint8 svid                      # Satellite identifier
uint8 sigid                     # Signal identifier
uint8 freqid                    # Only used for GLONASS: frequency slot + 7 (0 to 13)
uint8 numwords                  # The number of data words contained in this message (up to 10)
uint8 chn                       # The tracking channel number the message was received on
uint8 version                   # Message version (0x02 for this version)
uint8 reserved0                 # Reserved (Padding)

uint32[10] dwrd                 # The data words (Subframe data)

# TOPICS gnss_ephemeris gnss_ephemeris_in gnss_ephemeris_out