uint32  sample_number     # Sample number for first beam
float32 angle             # Angle in radians for first beam
uint16  upper_gate        # Sample number for upper adaptive gate
uint16  lower_gate        # Sample number for lower adaptive gate
uint32  intensity         # Intensity for first beam
uint16  flags             # Info flags for first beam
uint8   QUALITY_FLAG_SNR_PASS      = 1  #0b001
uint8   QUALITY_FLAG_FLYER_PASS    = 2  #0b010
uint8   quality_flag      # Quality flags for first beam
uint8   quality_val       # Quality value for first beam
