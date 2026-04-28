from px4_msgs.msg import VehicleCommand

def arm_command(timestamp):
    '''
        Create a VehicleCommand message to arm the drone.
        - command = 400 (MAV_CMD_COMPONENT_ARM_DISARM)
        - param1 = 1.0 (arm)
    '''
    return _base_command(timestamp, command=400, param1=1.0)

def disarm_command(timestamp):
    '''
        Create a VehicleCommand message to disarm the drone.
        - command = 400 (MAV_CMD_COMPONENT_ARM_DISARM)
        - param1 = 0.0 (disarm)
    '''
    return _base_command(timestamp, command=400, param1=0.0)

def offboard_mode_command(timestamp):
    '''
        Create a VehicleCommand message to switch the drone to OFFBOARD mode.
        - command = 176 (MAV_CMD_DO_SET_MODE)
        - param1 = 1.0 (custom mode)
        - param2 = 6.0 (OFFBOARD mode)
    '''
    return _base_command(timestamp, command=176, param1=1.0, param2=6.0)

def land_command(timestamp):
    '''
        Create a VehicleCommand message to land the drone.
        - command = 21 (MAV_CMD_NAV_LAND)
    '''
    return _base_command(timestamp, command=21)

# --------------------- Helper Function ---------------------
def _base_command(timestamp, command, param1=0.0, param2=0.0):
    '''
        Helper function to create a base VehicleCommand message.
    '''
    msg = VehicleCommand()
    msg.timestamp = timestamp
    msg.command = command
    msg.param1 = param1
    msg.param2 = param2
    msg.target_system = 1
    msg.target_component = 1
    msg.source_system = 1
    msg.source_component = 1
    msg.from_external = True
    return msg
    
    