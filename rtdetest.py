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

# Setup the output recipe
if not con.send_output_setup(state_names, state_types):
    print("Unable to configure output")
    con.disconnect()
    exit()

# Start data synchronization
if not con.send_start():
    print("Unable to start synchronization")
    con.disconnect()
    exit()

# Read and print the current TCP pose
try:
    for _ in range(10):
        state = con.receive()
        if state is not None:
            print("Current TCP Pose:", state.actual_TCP_pose)
        time.sleep(0.1)
except KeyboardInterrupt:
    pass

# Stop data synchronization and disconnect
con.send_pause()
con.disconnect()