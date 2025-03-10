import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import time

ROBOT_HOST = '169.254.56.120'
ROBOT_PORT = 30004
CONFIG_FILENAME = 'rtde_configuration.xml'

# Initialize the RTDE connection
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# Get controller version
con.get_controller_version()

# Load the configuration file
conf = rtde_config.ConfigFile(CONFIG_FILENAME)
state_names, state_types = conf.get_recipe('state')
setp_names, setp_types = conf.get_recipe('setp')

# Setup the output recipe
if not con.send_output_setup(state_names, state_types):
    print("Unable to configure output")
    con.disconnect()
    exit()

# Setup the input recipe
setp = con.send_input_setup(setp_names, setp_types)

# Start data synchronization
if not con.send_start():
    print("Unable to start synchronization")
    con.disconnect()
    exit()

# Setpoints to move the robot to
setp1 = [0.241351875460487, -0.1331365857518004, 0.2710225282773757, 1.2024845484135271, -1.247178824251458, 0.5641463050688574]

setp2 = [0.174, -0.733, 0.187, 1.202, -1.247, 0.564]

# Function to send setpoints to the robot
def send_setpoint(setpoint):
    setp.input_double_register_0 = setpoint[0]
    setp.input_double_register_1 = setpoint[1]
    setp.input_double_register_2 = setpoint[2]
    setp.input_double_register_3 = setpoint[3]
    setp.input_double_register_4 = setpoint[4]
    setp.input_double_register_5 = setpoint[5]
    con.send(setp)

# Move the robot between setpoints
try:
    for _ in range(5):
        send_setpoint(setp1)
        print("Moving to setpoint 1:", setp1)
        time.sleep(5)  # Wait for the robot to reach the setpoint

        send_setpoint(setp2)
        print("Moving to setpoint 2:", setp2)
        time.sleep(5)  # Wait for the robot to reach the setpoint

except KeyboardInterrupt:
    pass

# Stop data synchronization and disconnect
con.send_pause()
con.disconnect()