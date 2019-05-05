float32 snd_velocity     # Sound velocity in m/s
float32 sample_rate      # Sampling rate in Hz
uint32  N                # Number of beams
uint32  ping_number      # Sequence number of ping
float64 time             # Ping timestamp, unix time at TX
float64 time_net         # Timestamp Unix time as fract (send on network time)
float32 ping_rate        # In Hz
uint16  type             # Bathy data type (4)
uint8   beam_dist_mode   # 1:512EA 2:256EA

uint8 SONAR_MODE_FLS = 0
uint8 SONAR_MODE_BATHY = 1
uint8 SONAR_MODE_BATHY_AMP = 2
uint8 SONAR_MODE_BF_PASSTHROUGH = 4
uint8 SONAR_MODE_BATHY_EDGE_ENHANCE = 5
uint8 SONAR_MODE_SUPER_RES_BATHY = 6

uint8   sonar_mode       # see: sonar_mode_t

uint16[4]  reserved1
float32 tx_angle         # Tx elevation steering in radians
float32 gain             # Intensity value gain
float32 tx_freq          # Tx frequency in Hz
float32 tx_bw            # Tx bandwidth in Hz
float32 tx_len           # Tx pulse length in sec
float32 reserved2
float32 tx_voltage       # Tx peak-voltage over elements, NaN on non-STX sonars.
float32 swath_dir        # Center pointing direction of swath, in radians
float32 swath_open       # Opening angle of swath, in radians
float32 gate_tilt        # Gate tilt in radians, for depth gates

