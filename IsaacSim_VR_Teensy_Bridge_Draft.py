"""
GLiDAR Quest 3 VR Control
Description: Use Quest 3 controls to move the scope model in Isaac Sim
             Send the same movement commands to the Teensy over USB
             Still a work in progress; needs full setup testing and matching Teensy code
Components: Quest 3 headset/controllers, Isaac Sim XR scene, Teensy, pyserial
"""

# Stop the old run first if this script is started again
if callable(globals().get("stop_controller")):
    stop_controller()

import time
import math
import omni.usd
import omni.kit.app
import omni.timeline

from pxr import UsdPhysics, Sdf

from omni.kit.xr.core import XRCore

# The model can still run if pyserial is not installed
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


# Leave this blank to look for a Teensy, or type its port, like COM5
TEENSY_PORT = ""

# Use the same baud rate as the Teensy code that reads the VR commands
TEENSY_BAUD = 115200

# Send commands every 20 ms, or 50 times each second
SERIAL_SEND_INTERVAL = 0.020

# Set this to False to try the model without sending commands to the robot
ENABLE_TEENSY_SERIAL = True

# Print the messages sent back by the Teensy
PRINT_TEENSY_LINES = True


# These names need to match the joints in the Isaac Sim scene
linear_joint_path = Sdf.Path("/World/URDF_full_Assembly_ver5/LA_Motor/LA_Slider_Prismatic")

rotation_joint_path = Sdf.Path("/World/URDF_full_Assembly_ver5/Rotatng_bearng_outer/Rotatng_bearng_inner_rev")

# The model uses seven joints to make the tip bend
twisting_joint_paths = [
    Sdf.Path(f"/World/URDF_full_Assembly_ver5/Endoscope_tip/Black/mesh{i + 1}/midJoint")
    for i in range(7)
]


# The model's -0.15 to 0 range stands for 0 to 177 mm of real scope travel
LINEAR_MIN = -0.15
LINEAR_MAX = 0.0

# How far the real scope can move, in mm
TEENSY_LINEAR_MIN_MM = 0.0
TEENSY_LINEAR_MAX_MM = 177.0

# How far the scope can rotate and its tip can bend, in degrees
ROTATION_MIN = -180.0
ROTATION_MAX = 180.0
TWISTING_MIN = -270.0
TWISTING_MAX = 270.0

# Use these speeds and speed changes in both this script and the Teensy code
MAXIMUM_LINEAR_SPEED_MM_PER_SEC = 2.0
MAXIMUM_ROTATION_SPEED_DEG_PER_SEC = 30.0
MAXIMUM_FLEXION_TARGET_SPEED_DEG_PER_SEC = 35.0

LINEAR_ACCELERATION_MM_PER_SEC2 = 6.0
LINEAR_DECELERATION_MM_PER_SEC2 = 10.0
ROTATION_ACCELERATION_DEG_PER_SEC2 = 90.0
ROTATION_DECELERATION_DEG_PER_SEC2 = 140.0
FLEXION_ACCELERATION_DEG_PER_SEC2 = 120.0
FLEXION_DECELERATION_DEG_PER_SEC2 = 180.0

# Treat very small speeds as stopped
STOPPED_LINEAR_SPEED_TOLERANCE_MM_PER_SEC = 0.005
STOPPED_ROTATION_SPEED_TOLERANCE_DEG_PER_SEC = 0.05
STOPPED_FLEXION_SPEED_TOLERANCE_DEG_PER_SEC = 0.05

# Convert the real scope speed to the model's smaller movement range
LINEAR_SPEED_M_PER_SEC = (
    MAXIMUM_LINEAR_SPEED_MM_PER_SEC
    * (LINEAR_MAX - LINEAR_MIN)
    / (TEENSY_LINEAR_MAX_MM - TEENSY_LINEAR_MIN_MM)
)

# Bend more slowly at large angles and close to the bend limits
FLEXION_SLOWDOWN_START_TIP_DEG = 100.0
FLEXION_SPEED_REDUCTION_PER_TIP_DEG = 2.5
FLEXION_SERVO_SPEED = 500.0
FLEXION_MINIMUM_SERVO_SPEED = 50.0
FLEXION_SOFT_LIMIT_ZONE_DEG = 10.0

# Ignore small stick movements near the center so drift does not move the scope
DEADZONE = 0.12

# Leave the controls alone for this long so the script can learn their resting values
CONTROLLER_NEUTRAL_LEARNING_SECONDS = 1.0

# Some buttons read low at rest, while others read high
BUTTON_PRESS_THRESHOLD = 0.65

BUTTON_INVERTED_PRESS_THRESHOLD = 0.35

# Show the controller readings to help check which input is which
INPUT_DEBUG_PRINT_INTERVAL = 0.5

PRINT_RAW_CONTROLLER_INPUTS = True


# Start with the model pulled back, no rotation, and the tip straight
linear_position = LINEAR_MIN

rotation_position = 0.0
twisting_position = 0.0

# Change these speeds a little at a time to make movement smoother
current_linear_speed_mm_per_sec = 0.0
current_rotation_speed_deg_per_sec = 0.0
current_flexion_speed_deg_per_sec = 0.0

# Keep track of the time between updates and between serial messages
_last_time = time.monotonic()
_last_serial_send_time = 0.0
_serial_read_buffer = b""

teensy_serial = None

_update_sub = None

# Save the average readings while the controls are released at startup
input_learning_complete = False
input_learning_start_time = None
input_learning_sample_count = 0
left_rotation_rest_sum = 0.0
right_flexion_rest_sum = 0.0
x_button_rest_sum = 0.0
y_button_rest_sum = 0.0
left_rotation_rest_value = 0.0
right_flexion_rest_value = 0.0
x_button_rest_value = 0.0
y_button_rest_value = 0.0
_last_input_debug_time = 0.0


# Set up the drives that move the model's joints
def setup_drives():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Open the GLiDAR USD scene before running this script")

    # Make sure all the joints are there and have the right type before changing them
    joints = [(linear_joint_path, UsdPhysics.PrismaticJoint)]
    joints += [(rotation_joint_path, UsdPhysics.RevoluteJoint)]
    joints += [(path, UsdPhysics.RevoluteJoint) for path in twisting_joint_paths]
    for path, joint_type in joints:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsA(joint_type):
            raise RuntimeError(f"Missing or wrong joint type at {path}; check the joint paths")

    linear_prim = stage.GetPrimAtPath(linear_joint_path)
    rotation_prim = stage.GetPrimAtPath(rotation_joint_path)
    twisting_prims = [stage.GetPrimAtPath(path) for path in twisting_joint_paths]

    linear_drive = UsdPhysics.DriveAPI.Get(linear_prim, "linear")
    if not linear_drive:
        linear_drive = UsdPhysics.DriveAPI.Apply(linear_prim, "linear")
    # Move toward a position target and keep the target velocity at zero
    linear_drive.CreateTargetVelocityAttr().Set(0.0)
    linear_drive.CreateStiffnessAttr().Set(100000.0)
    linear_drive.CreateDampingAttr().Set(10.0)
    linear_drive.CreateMaxForceAttr().Set(1000000.0)

    rotation_drive = UsdPhysics.DriveAPI.Get(rotation_prim, "angular")
    if not rotation_drive:
        rotation_drive = UsdPhysics.DriveAPI.Apply(rotation_prim, "angular")
    rotation_drive.CreateTargetVelocityAttr().Set(0.0)
    rotation_drive.CreateStiffnessAttr().Set(10000000.0)
    rotation_drive.CreateDampingAttr().Set(10.0)
    rotation_drive.CreateMaxForceAttr().Set(1000000.0)

    twisting_drives = []
    for twisting_prim in twisting_prims:
        twisting_drive = UsdPhysics.DriveAPI.Get(twisting_prim, "angular")
        if not twisting_drive:
            twisting_drive = UsdPhysics.DriveAPI.Apply(twisting_prim, "angular")
        twisting_drive.CreateTargetVelocityAttr().Set(0.0)
        twisting_drive.CreateStiffnessAttr().Set(100000.0)
        twisting_drive.CreateDampingAttr().Set(100.0)
        twisting_drive.CreateMaxForceAttr().Set(1000000.0)
        twisting_drives.append(twisting_drive)

    return linear_drive, rotation_drive, twisting_drives


# Tell the model where to move; this does not read where the joints actually are
def apply_positions(linear_drive, rotation_drive, twisting_drives):
    linear_drive.CreateTargetPositionAttr().Set(linear_position)

    rotation_drive.CreateTargetPositionAttr().Set(rotation_position)

    for twisting_drive in twisting_drives:
        twisting_drive.CreateTargetPositionAttr().Set(twisting_position / 7.0)


# Ignore the center area, then spread the rest of the stick travel over the full range
def apply_deadzone(value):
    if abs(value) < DEADZONE:
        return 0.0

    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - DEADZONE) / (1.0 - DEADZONE)

def clamp_unit(value):
    return max(-1.0, min(1.0, value))

# Do not let a missing or bad reading turn into movement
def safe_input_value(device, input_name, gesture_name):
    try:
        if not device.has_input_gesture(input_name, gesture_name):
            return None
        value = device.get_input_gesture_value(input_name, gesture_name)
    except Exception:
        return None

    if value is None:
        return None

    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None

    if not math.isfinite(value):
        return None

    return value

# Read each control once so the rest of this update uses the same values
def read_controller_inputs(left, right):
    if left is None or right is None:
        return None
    values = (
        safe_input_value(left, "x", "click"),
        safe_input_value(left, "y", "click"),
        safe_input_value(left, "thumbstick", "y"),
        safe_input_value(right, "thumbstick", "x"),
    )
    for value, minimum in zip(values, (0.0, 0.0, -1.0, -1.0)):
        if value is None or not minimum <= value <= 1.0:
            return None
    return values

# Start over if the readings stop before the resting values have been learned
def reset_controller_neutral_values():
    global input_learning_complete, input_learning_start_time, input_learning_sample_count
    global left_rotation_rest_sum, right_flexion_rest_sum
    global x_button_rest_sum, y_button_rest_sum

    input_learning_complete = False
    input_learning_start_time = None
    input_learning_sample_count = 0
    left_rotation_rest_sum = right_flexion_rest_sum = 0.0
    x_button_rest_sum = y_button_rest_sum = 0.0

# A high reading can mean released on some controllers
def is_button_pressed(raw_value, resting_value):
    if resting_value >= 0.5:
        return raw_value < BUTTON_INVERTED_PRESS_THRESHOLD

    return raw_value > BUTTON_PRESS_THRESHOLD

# Subtract the stick's resting value before checking how far it moved
def centered_axis_value(raw_value, resting_value):
    return apply_deadzone(clamp_unit(raw_value - resting_value))

# Learn the average readings with the controls released before allowing movement
def learn_controller_neutral_values(values, now):
    global input_learning_complete
    global input_learning_start_time
    global input_learning_sample_count
    global left_rotation_rest_sum
    global right_flexion_rest_sum
    global x_button_rest_sum
    global y_button_rest_sum
    global left_rotation_rest_value
    global right_flexion_rest_value
    global x_button_rest_value
    global y_button_rest_value

    if input_learning_complete:
        return True

    # Wait for the sticks to be centered so a held stick is not mistaken for its resting point
    if any(abs(value) > DEADZONE for value in values[2:]):
        reset_controller_neutral_values()
        return False

    if input_learning_start_time is None:
        input_learning_start_time = now
        print("[VR] Keep Quest controls released while neutral input is learned")

    x_raw, y_raw, left_rotation_raw, right_flexion_raw = values

    x_button_rest_sum += x_raw
    y_button_rest_sum += y_raw
    left_rotation_rest_sum += left_rotation_raw
    right_flexion_rest_sum += right_flexion_raw
    input_learning_sample_count += 1

    if now - input_learning_start_time < CONTROLLER_NEUTRAL_LEARNING_SECONDS:
        return False

    sample_count = max(1, input_learning_sample_count)
    x_button_rest_value = x_button_rest_sum / sample_count
    y_button_rest_value = y_button_rest_sum / sample_count
    left_rotation_rest_value = left_rotation_rest_sum / sample_count
    right_flexion_rest_value = right_flexion_rest_sum / sample_count
    input_learning_complete = True

    print(
        "[VR] Neutral learned "
        f"x={x_button_rest_value:.3f} "
        f"y={y_button_rest_value:.3f} "
        f"leftY={left_rotation_rest_value:.3f} "
        f"rightX={right_flexion_rest_value:.3f}"
    )

    return True

def print_raw_input_debug_if_ready(now, x_raw, y_raw, left_rotation_raw, right_flexion_raw, linear_input, rotation_input, flexion_input):
    global _last_input_debug_time

    if not PRINT_RAW_CONTROLLER_INPUTS:
        return

    if now - _last_input_debug_time < INPUT_DEBUG_PRINT_INTERVAL:
        return

    _last_input_debug_time = now
    print(
        "[VR input] "
        f"raw x={x_raw:.3f} "
        f"raw y={y_raw:.3f} "
        f"raw leftY={left_rotation_raw:.3f} "
        f"raw rightX={right_flexion_raw:.3f} "
        f"cmd L={linear_input:.3f} "
        f"R={rotation_input:.3f} "
        f"F={flexion_input:.3f}"
    )

# Make small stick movements slower, using the same curve as the Teensy
def apply_controller_response_curve(value):
    cubic_value = value * value * value
    return 0.35 * value + 0.65 * cubic_value

# Smooth out the change from 0 to 1
def smooth_step_01(value):
    if value <= 0.0:
        return 0.0
    if value >= 1.0:
        return 1.0
    return value * value * (3.0 - 2.0 * value)

# Slow down near a bend limit but still let the tip move back away from it
def apply_flexion_soft_limit_to_speed(requested_speed):
    if requested_speed == 0.0:
        return 0.0

    distance_to_limit = (
        TWISTING_MAX - twisting_position
        if requested_speed > 0.0
        else twisting_position - TWISTING_MIN
    )

    if distance_to_limit <= 0.0:
        return 0.0

    if distance_to_limit >= FLEXION_SOFT_LIMIT_ZONE_DEG:
        return requested_speed

    blend_amount = smooth_step_01(distance_to_limit / FLEXION_SOFT_LIMIT_ZONE_DEG)
    return requested_speed * blend_amount

# Work out the expected servo speed as the tip bends farther from straight
def calculate_flexion_servo_speed(tip_deg):
    absolute_tip_deg = abs(tip_deg)
    if absolute_tip_deg <= FLEXION_SLOWDOWN_START_TIP_DEG:
        return FLEXION_SERVO_SPEED

    degrees_beyond_slowdown = absolute_tip_deg - FLEXION_SLOWDOWN_START_TIP_DEG
    # Round .5 up, the same way Arduino does for these positive values
    speed_reduction = math.floor(
        degrees_beyond_slowdown * FLEXION_SPEED_REDUCTION_PER_TIP_DEG + 0.5
    )
    calculated_speed = FLEXION_SERVO_SPEED - speed_reduction

    if calculated_speed < FLEXION_MINIMUM_SERVO_SPEED:
        calculated_speed = FLEXION_MINIMUM_SERVO_SPEED

    return calculated_speed

# Slow the model's bending by the same fraction as the servo speed
def calculate_flexion_target_speed_scale(tip_deg):
    return calculate_flexion_servo_speed(tip_deg) / FLEXION_SERVO_SPEED

# Move closer to the target without passing it
def move_value_toward(current_value, target_value, maximum_change):
    if current_value < target_value:
        current_value += maximum_change
        if current_value > target_value:
            current_value = target_value
    elif current_value > target_value:
        current_value -= maximum_change
        if current_value < target_value:
            current_value = target_value

    return current_value

# Use the slowdown rate when reducing speed or changing direction
def choose_ramp_rate(current_value, target_value, acceleration, deceleration):
    same_direction = (
        (target_value >= 0.0 and current_value >= 0.0)
        or (target_value <= 0.0 and current_value <= 0.0)
    )
    increasing_magnitude = abs(target_value) > abs(current_value)

    if same_direction and increasing_magnitude:
        return acceleration

    return deceleration

# Limit how quickly each movement speeds up or slows down
def update_smoothed_speeds(target_linear_speed, target_rotation_speed, target_flexion_speed, dt):
    global current_linear_speed_mm_per_sec
    global current_rotation_speed_deg_per_sec
    global current_flexion_speed_deg_per_sec

    current_linear_speed_mm_per_sec = move_value_toward(
        current_linear_speed_mm_per_sec,
        target_linear_speed,
        choose_ramp_rate(
            current_linear_speed_mm_per_sec,
            target_linear_speed,
            LINEAR_ACCELERATION_MM_PER_SEC2,
            LINEAR_DECELERATION_MM_PER_SEC2,
        )
        * dt,
    )

    current_rotation_speed_deg_per_sec = move_value_toward(
        current_rotation_speed_deg_per_sec,
        target_rotation_speed,
        choose_ramp_rate(
            current_rotation_speed_deg_per_sec,
            target_rotation_speed,
            ROTATION_ACCELERATION_DEG_PER_SEC2,
            ROTATION_DECELERATION_DEG_PER_SEC2,
        )
        * dt,
    )

    current_flexion_speed_deg_per_sec = move_value_toward(
        current_flexion_speed_deg_per_sec,
        target_flexion_speed,
        choose_ramp_rate(
            current_flexion_speed_deg_per_sec,
            target_flexion_speed,
            FLEXION_ACCELERATION_DEG_PER_SEC2,
            FLEXION_DECELERATION_DEG_PER_SEC2,
        )
        * dt,
    )

    if (
        abs(current_linear_speed_mm_per_sec) < STOPPED_LINEAR_SPEED_TOLERANCE_MM_PER_SEC
        and abs(target_linear_speed) < STOPPED_LINEAR_SPEED_TOLERANCE_MM_PER_SEC
    ):
        current_linear_speed_mm_per_sec = 0.0

    if (
        abs(current_rotation_speed_deg_per_sec) < STOPPED_ROTATION_SPEED_TOLERANCE_DEG_PER_SEC
        and abs(target_rotation_speed) < STOPPED_ROTATION_SPEED_TOLERANCE_DEG_PER_SEC
    ):
        current_rotation_speed_deg_per_sec = 0.0

    if (
        abs(current_flexion_speed_deg_per_sec) < STOPPED_FLEXION_SPEED_TOLERANCE_DEG_PER_SEC
        and abs(target_flexion_speed) < STOPPED_FLEXION_SPEED_TOLERANCE_DEG_PER_SEC
    ):
        current_flexion_speed_deg_per_sec = 0.0

# Keep the targets within their limits and stop adding movement past them
def clamp_positions_and_stop_at_limits():
    global linear_position, rotation_position, twisting_position
    global current_linear_speed_mm_per_sec
    global current_rotation_speed_deg_per_sec
    global current_flexion_speed_deg_per_sec

    if linear_position <= LINEAR_MIN:
        linear_position = LINEAR_MIN
        if current_linear_speed_mm_per_sec < 0.0:
            current_linear_speed_mm_per_sec = 0.0

    if linear_position >= LINEAR_MAX:
        linear_position = LINEAR_MAX
        if current_linear_speed_mm_per_sec > 0.0:
            current_linear_speed_mm_per_sec = 0.0

    if rotation_position <= ROTATION_MIN:
        rotation_position = ROTATION_MIN
        if current_rotation_speed_deg_per_sec < 0.0:
            current_rotation_speed_deg_per_sec = 0.0

    if rotation_position >= ROTATION_MAX:
        rotation_position = ROTATION_MAX
        if current_rotation_speed_deg_per_sec > 0.0:
            current_rotation_speed_deg_per_sec = 0.0

    if twisting_position <= TWISTING_MIN:
        twisting_position = TWISTING_MIN
        if current_flexion_speed_deg_per_sec < 0.0:
            current_flexion_speed_deg_per_sec = 0.0

    if twisting_position >= TWISTING_MAX:
        twisting_position = TWISTING_MAX
        if current_flexion_speed_deg_per_sec > 0.0:
            current_flexion_speed_deg_per_sec = 0.0


# Only pick a port automatically if it is clearly labeled as a Teensy
def find_teensy_port():
    if TEENSY_PORT:
        return TEENSY_PORT

    candidates = set()
    if list_ports is not None:
        for port in list_ports.comports():
            description = f"{port.description} {port.manufacturer} {port.product}".lower()
            if "teensy" in description:
                candidates.add(port.device)

    if len(candidates) == 1:
        return candidates.pop()
    if len(candidates) > 1:
        print("[VR] More than one Teensy found; set TEENSY_PORT to the intended board")

    return None

# Try opening the USB connection, or keep running just the model
def open_teensy_serial():
    if not ENABLE_TEENSY_SERIAL:
        print("[VR] Teensy serial bridge disabled")
        return None

    if serial is None:
        print("[VR] pyserial is not installed, so Isaac Sim will run without Teensy output")
        return None

    connection = None
    try:
        port = find_teensy_port()
        if port is None:
            print("[VR] No unique Teensy port found; set TEENSY_PORT manually")
            return None
        connection = serial.Serial(port, TEENSY_BAUD, timeout=0, write_timeout=0.01)
        # End any leftover message and ask the Teensy to stop before starting
        packet = b"\nVR,0.000,0.000,0.000\n"
        if connection.write(packet) != len(packet):
            raise IOError("Could not send the startup stop packet")
        print(f"[VR] Teensy serial connected on {port}")
        return connection
    except Exception as error:
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        print(f"[VR] Teensy serial connection failed: {error}")
        return None

# Close the USB connection after stopping or if serial communication fails
def close_teensy_serial():
    global teensy_serial, _serial_read_buffer
    connection = teensy_serial
    teensy_serial = None
    _serial_read_buffer = b""
    if connection is not None:
        try:
            connection.close()
        except Exception as error:
            print(f"[VR] Teensy serial close failed: {error}")

# Send the three movement values from -1 to +1, then end the line
def send_teensy_command(linear_input, rotation_input, flexion_input):
    global _last_serial_send_time

    if teensy_serial is None:
        return

    now = time.monotonic()
    if now - _last_serial_send_time < SERIAL_SEND_INTERVAL:
        return

    packet = f"VR,{linear_input:.3f},{rotation_input:.3f},{flexion_input:.3f}\n"

    try:
        data = packet.encode("ascii")
        if teensy_serial.write(data) != len(data):
            raise IOError("Incomplete VR packet; rerun the script to reconnect")
        _last_serial_send_time = now
    except Exception as error:
        print(f"[VR] Teensy serial write failed: {error}")
        close_teensy_serial()

# Read any Teensy messages that are ready without waiting for more
def read_teensy_messages():
    global _serial_read_buffer
    if teensy_serial is None:
        return

    try:
        # Read a limited amount each update and save unfinished lines for next time
        data = teensy_serial.read(min(teensy_serial.in_waiting, 4096))
        lines = (_serial_read_buffer + data).split(b"\n")
        _serial_read_buffer = lines.pop()
        if len(_serial_read_buffer) > 4096:
            _serial_read_buffer = b""
        if PRINT_TEENSY_LINES:
            for raw_line in lines:
                line = raw_line.decode("ascii", errors="replace").strip()
                if line:
                    print(f"[Teensy] {line}")
    except Exception as error:
        print(f"[VR] Teensy serial read failed: {error}")
        close_teensy_serial()

# Stop updates, ask the Teensy to stop, and close the USB connection
def stop_controller():
    global _update_sub
    global current_linear_speed_mm_per_sec
    global current_rotation_speed_deg_per_sec, current_flexion_speed_deg_per_sec

    _update_sub = None
    current_linear_speed_mm_per_sec = 0.0
    current_rotation_speed_deg_per_sec = current_flexion_speed_deg_per_sec = 0.0
    if teensy_serial is not None:
        try:
            # Start a new line in case the Teensy still has part of an old message
            packet = b"\nVR,0.000,0.000,0.000\n"
            if teensy_serial.write(packet) != len(packet):
                raise IOError("Incomplete stop packet; the receiver timeout must stop motion")
        except Exception as error:
            print(f"[VR] Could not send the stop packet: {error}")
        finally:
            close_teensy_serial()
    print("[VR] Controller stopped")

# Keep updating the position while slowing down, even while learning the resting values
def advance_model_positions(dt):
    global linear_position, rotation_position, twisting_position
    linear_position += (
        current_linear_speed_mm_per_sec
        * (LINEAR_MAX - LINEAR_MIN)
        / (TEENSY_LINEAR_MAX_MM - TEENSY_LINEAR_MIN_MM)
        * dt
    )
    rotation_position += current_rotation_speed_deg_per_sec * dt
    twisting_position += current_flexion_speed_deg_per_sec * dt
    clamp_positions_and_stop_at_limits()

linear_drive, rotation_drive, twisting_drives = setup_drives()
apply_positions(linear_drive, rotation_drive, twisting_drives)

controlled_stage = omni.usd.get_context().get_stage()
timeline = omni.timeline.get_timeline_interface()
xr_core = XRCore.get_singleton()
if xr_core is None:
    raise RuntimeError("Enable the Isaac Sim XR extension before running this script")
teensy_serial = open_teensy_serial()

# Read the Quest controls and update the model each frame
def on_update(event):
    global linear_position, rotation_position, twisting_position, _last_time
    global current_linear_speed_mm_per_sec
    global current_rotation_speed_deg_per_sec, current_flexion_speed_deg_per_sec

    now = time.monotonic()
    dt = max(0.0, min(now - _last_time, 0.05))
    _last_time = now

    # Stop this run if a different scene is opened or the current one closes
    if omni.usd.get_context().get_stage() != controlled_stage:
        stop_controller()
        return
    if not all(drive.GetPrim() for drive in [linear_drive, rotation_drive, *twisting_drives]):
        raise RuntimeError("A controlled joint was removed; rerun after checking the scene")

    # When the simulation is paused, ask the Teensy to stop too
    if not timeline.is_playing():
        current_linear_speed_mm_per_sec = 0.0
        current_rotation_speed_deg_per_sec = current_flexion_speed_deg_per_sec = 0.0
        if not input_learning_complete:
            reset_controller_neutral_values()
        send_teensy_command(0.0, 0.0, 0.0)
        read_teensy_messages()
        return

    left = xr_core.get_input_device("/user/hand/left")
    right = xr_core.get_input_device("/user/hand/right")
    values = read_controller_inputs(left, right)

    # Send no movement request if a controller reading is missing or bad
    if values is None:
        if not input_learning_complete:
            reset_controller_neutral_values()
        update_smoothed_speeds(0.0, 0.0, 0.0, dt)
        advance_model_positions(dt)
        apply_positions(linear_drive, rotation_drive, twisting_drives)
        send_teensy_command(0.0, 0.0, 0.0)
        read_teensy_messages()
        return

    # Wait until the resting values are learned before using the controls
    if not learn_controller_neutral_values(values, now):
        update_smoothed_speeds(0.0, 0.0, 0.0, dt)
        advance_model_positions(dt)
        apply_positions(linear_drive, rotation_drive, twisting_drives)
        send_teensy_command(0.0, 0.0, 0.0)
        read_teensy_messages()
        return

    x_button, y_button, left_rotation_raw, right_flexion_raw = values
    x_pressed = is_button_pressed(x_button, x_button_rest_value)
    y_pressed = is_button_pressed(y_button, y_button_rest_value)

    # X moves back, Y moves forward, and pressing both sends no movement request
    if x_pressed and not y_pressed:
        linear_input = -1.0
    elif y_pressed and not x_pressed:
        linear_input = 1.0
    else:
        linear_input = 0.0

    # Use the left stick up/down to rotate and the right stick left/right to bend
    rotation_input = centered_axis_value(left_rotation_raw, left_rotation_rest_value)

    flexion_input = centered_axis_value(right_flexion_raw, right_flexion_rest_value)

    print_raw_input_debug_if_ready(
        now,
        x_button,
        y_button,
        left_rotation_raw,
        right_flexion_raw,
        linear_input,
        rotation_input,
        flexion_input,
    )

    # Use the Teensy's response curve before working out the requested speeds
    shaped_linear = apply_controller_response_curve(linear_input)
    shaped_rotation = apply_controller_response_curve(rotation_input)
    shaped_flexion = apply_controller_response_curve(flexion_input)

    target_linear_speed_mm_per_sec = shaped_linear * MAXIMUM_LINEAR_SPEED_MM_PER_SEC
    target_rotation_speed_deg_per_sec = shaped_rotation * MAXIMUM_ROTATION_SPEED_DEG_PER_SEC

    # Do not request more in/out movement if the model is already at that end
    if linear_position <= LINEAR_MIN and target_linear_speed_mm_per_sec < 0.0:
        target_linear_speed_mm_per_sec = 0.0
    if linear_position >= LINEAR_MAX and target_linear_speed_mm_per_sec > 0.0:
        target_linear_speed_mm_per_sec = 0.0

    # Slow the bending down at large angles and close to its limits
    flexion_speed_scale = calculate_flexion_target_speed_scale(twisting_position)
    target_flexion_speed_deg_per_sec = (
        shaped_flexion * MAXIMUM_FLEXION_TARGET_SPEED_DEG_PER_SEC * flexion_speed_scale
    )
    target_flexion_speed_deg_per_sec = apply_flexion_soft_limit_to_speed(
        target_flexion_speed_deg_per_sec
    )

    update_smoothed_speeds(
        target_linear_speed_mm_per_sec,
        target_rotation_speed_deg_per_sec,
        target_flexion_speed_deg_per_sec,
        dt,
    )

    advance_model_positions(dt)

    apply_positions(linear_drive, rotation_drive, twisting_drives)
    send_teensy_command(linear_input, rotation_input, flexion_input)
    read_teensy_messages()

# Stop the script and connection if something goes wrong during an update
def guarded_on_update(event):
    try:
        on_update(event)
    except Exception as error:
        print(f"[VR] Update failed: {error}")
        stop_controller()

_update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_push(
    guarded_on_update
)

print("[VR] Quest 3 to Teensy mirror controller active")
print(f"[VR] Keep all controls released for {CONTROLLER_NEUTRAL_LEARNING_SECONDS:.1f} second while neutral is learned")
print(f"[VR] X button sends linear -1 backward")
print(f"[VR] Y button sends linear +1 forward")
print(f"[VR] Left joystick Y sends rotation -1 to +1")
print(f"[VR] Right joystick X sends flexion -1 to +1")
print(f"[VR] Serial packet format is VR,linear,rotation,flexion")
print("[VR] Press Play to begin; keep controls released while neutral is learned")
print("[VR] Run this in Script Editor to stop: stop_controller()")
