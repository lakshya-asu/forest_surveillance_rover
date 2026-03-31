# LoRa communication status and diagnostic info
builtin_interfaces/Time timestamp

# Radio link quality
uint8 rssi                          # Receive Signal Strength Indicator (-120 to -80 dBm, mapped to 0-255)
float32 snr                         # Signal-to-Noise Ratio (dB, -20 to +10)
uint32 packets_transmitted          # Total transmitted packets (this session)
uint32 packets_ack_received         # Acknowledged packets from base station
uint32 packets_lost                 # Estimate of lost packets

# LoRa configuration
string lora_mode                    # "rx", "tx", or "idle"
float32 frequency_mhz               # Operating frequency (e.g., 868.0)
uint8 spreading_factor              # SF7-SF12 (7=fast+short_range, 12=slow+long_range)
uint8 bandwidth_khz                 # Bandwidth code (125=BW_125KHZ, 250=BW_250KHZ, etc)

# Distance estimate (based on RSSI + known TX power)
float32 estimated_distance_m        # Estimated distance to base station (meters)
