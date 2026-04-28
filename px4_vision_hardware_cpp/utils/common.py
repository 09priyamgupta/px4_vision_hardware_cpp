import math

def now_us(node):
    '''
        Get the current time in microseconds.
    '''
    return int(node.get_clock().now().nanoseconds / 1000)


def now_sec(node):
    '''
        Get the current time in seconds.
    '''
    return node.get_clock().now().nanoseconds / 1e9


def yaw_to_quaternion(yaw):
    '''
        Convert a yaw angle (in radians) to a quaternion representation (ENU convention).
    '''
    qz = math.sin(-yaw / 2.0)
    qw = math.cos(-yaw / 2.0)
    return (0.0, 0.0, qz, qw)


def flu_to_frd(x_flu, y_flu, z_flu):
    '''
        Convert coordinates from FLU (Forward-Left-Up) (ROS Standard) to FRD (Forward-Right-Down) (PX4 Standard) convention.
        PX4 uses NED (North-East-Down), while ROS uses NWU (North-West-Up) for local maps.
            X (North) remains the same.
            Y (West)  must be negated to become East.
            Z (Up)    must be negated to become Down.
    '''
    x_frd = x_flu
    y_frd = -y_flu
    z_frd = -z_flu
    return (x_frd, y_frd, z_frd)


def reached_position(current_xy, target_xy, threshold):
    '''
        Check if the current 2D position/waypoint is within a certain threshold of the target position.
    '''
    dx = current_xy[0] - target_xy[0]
    dy = current_xy[1] - target_xy[1]
    distance = math.sqrt(dx * dx + dy * dy)
    return distance <= threshold


def wrap_angle(angle):
    '''
        Wrap an angle in radians to the range [-pi, pi].
    '''
    return math.atan2(math.sin(angle), math.cos(angle))