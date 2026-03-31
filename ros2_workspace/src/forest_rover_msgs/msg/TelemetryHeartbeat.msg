# Heartbeat telemetry message sent over LoRa periodically
# Includes position, battery, detection count, and operational status

builtin_interfaces/Time timestamp
uint32 sequence_number              # Monotonic counter for packet loss detection

# Rover state
float32 position_x                  # Odometry X (meters)
float32 position_y                  # Odometry Y (meters)
float32 yaw                         # Heading (radians)
float32 velocity_cmd                # Current velocity command (m/s)

# Environmental & Detection
uint8 detection_count               # Total animals detected this mission
float32 battery_voltage             # Battery voltage (volts)
uint8 battery_percentage            # Battery % (0-100)

# Smoke/Environmental Alerts
bool smoke_alert                    # True if smoke detected
float32 gas_sensor_ppm              # CO concentration estimate (ppm)

# Autonomy state
string autonomy_state               # "idle", "patrolling", "investigating_fire", "returning"
uint8 waypoint_index                # Current waypoint in active route

# Signal quality indicator
uint8 rssi_estimate                 # RSSI quality (0-255, higher = better)
