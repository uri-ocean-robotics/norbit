float32 snd_velocity     # Sound velocity in m/s
float32 sample_rate      # Sampling rate in Hz
uint32  N                # Number of beams
uint32  M                # Number of samples in each beam
float64 time             # Ping timestamp, unix time at


uint32  DTYPE_UINT8 = 0
uint32  DTYPE_INT8 = 1
uint32  DTYPE_INT16 = 2
uint32  DTYPE_UINT32 = 4
uint32  DTYPE_INT32 = 5
uint32  DTYPE_UINT64 = 6
uint32  DTYPE_INT64 = 7
uint32  DTYPE_FLOAT32 = 21
uint32  DTYPE_FLOAT64 = 23

uint32  dtype

int32   t0              # Sample number of first sample in data
float32 gain            # Total processing gain
uint32  reserved1       # reserved
float32 swath_dir       # Swath center direction in radians
float32 swath_open      # Swath opening angle in radians
float32 tx_freq         # Tx frequency in kHz
float32 tx_bw           # Tx Bandwidth in kHz
float32 tx_len          # Tx pulse length in sec
uint32  tx_amp          # Tx amplitude 0-15
uint32[3] reserved2     # reserved
float32 ping_rate       # In Hz
float32 reserved3       # RESERVED
uint32  ping_number     # Sequence number of ping
float64 time_net        # Timestamp Unix time as fract (send on network time)
uint32  beams           # Total number of beams in beamformer (before decimation)
int32   vga_t1          # Sample number for first vga value
float32 vga_g1          # vga gain in dB for first vga value
int32   vga_t2          # Sample number for second vga value
float32 vga_g2          # vga gain in dB for second vga value
uint16[2] reserved4     # RESERVED
float32 tx_angle        # Tx elevation steering in radians
float32 tx_voltage      # Tx voltage - peak-voltage signal over ceramics. NaN for sonars without measurement

uint8   BEAM_DIST_MODE_512EA = 1
uint8   BEAM_DIST_MODE_256EA = 2
uint8   BEAM_DIST_MODE_256ED = 3
uint8   BEAM_DIST_MODE_512EAX = 4
uint8   BEAM_DIST_MODE_256EAX = 5

uint8   beam_dist_mode

uint8   SONAR_MODE_FLS = 0
uint8   SONAR_MODE_BATHY = 1
uint8   SONAR_MODE_BATHY_AMP = 2
uint8   SONAR_MODE_BF_PASSTHROUGH = 4
uint8   SONAR_MODE_BATHY_EDGE_ENHANCE = 5
uint8   SONAR_MODE_SUPER_RES_BATHY = 6

uint8   sonar_mode

uint16  reserved5        # RESERVED
float32 gate_tilt        # Gate tilt in radians, for depth gates
float32 reserved6        # RESERVED
uint8[28] reserved7      # RESERVED
