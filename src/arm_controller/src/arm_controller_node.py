#!/system/bin/env python3
"""
Copyright (c) 2025, BlackBerry Limited. All rights reserved.
 
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
 
    http://www.apache.org/licenses/LICENSE-2.0
 
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
 
@file arm_controller_node.py
@brief ROS2 node for controlling a 5-DOF robotic arm with a joystick.
 
This node manages all hardware-level servo control for a 5-DOF robotic arm
(plus gripper) using joystick input and/or inverse kinematics (IK) commands.
It subscribes to joystick messages(/joy) and IK solver output(/mov), converts commands
to PWM signals, and sends them to the servos via a PCA9685 PWM driver over I2C.
 
Key Features:
- Dual Control Modes: Supports both direct joystick control and IK-based
  Cartesian control, toggled via gamepad button B.
- Inverse Kinematics Integration: Subscribes to /mov topic for joint angles
  from the IK solver node, converts radians to servo percentages using
  asymmetric joint limit mapping where 0 rad always equals 50% (neutral).
- Cartesian Command Publishing: In IK mode, publishes joystick axes as
  Cartesian velocity commands to /CartesianCmd for the IK solver.
- Position Feedback: Publishes current servo positions as radians to
  /CurrentPositions for IK solver state synchronization.
- Smoothing Factor: Implements an interpolation (easing) function to ensure
  all arm movements are smooth/controlled.
- Individual Servo Speed Tuning: Allows each of the 6 servos to have a unique
  speed setting.
- Home Position: A dedicated joystick button returns the arm to a neutral,
  centered position.
- Deadzone and MIN/MAX: Includes configurable deadzone and gripper open/close
  percentages to prevent servo drift and stalling.
- Inactivity timer to release servos, and state management for
  Homing and Screensaver modes
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from PCA9685 import PCA9685
import time
import math
import numpy as np

 
class ArmControllerNode(Node):
    """
    Manages arm hardware, joystick input, and controls all arm states.
    """
    def __init__(self):
        """
        @brief Initializes the ArmControllerNode.
        """
        super().__init__('arm_controller_node')
        self.get_logger().info("Arm Controller Node has started (Merged Version).")
        ## --------------------------------------------------------------------------
        ## Tunable Parameters
        ## --------------------------------------------------------------------------
        
        self.SMOOTHING_FACTOR = 0.1
        self.DEADZONE = 0.08
        self.NUM_SERVOS = 6
        
        # Speeds are matched to the NEW servo layout (Swapped 3 and 4).
        self.SERVO_SPEEDS = [
            0.5,  # Servo 0: Base
            0.4,  # Servo 1: Shoulder
            0.4,  # Servo 2: Elbow
            0.7,  # Servo 3: Wrist Pitch (Swapped: Was Roll, now Pitch)
            1.5,  # Servo 4: Wrist Roll  (Swapped: Was Pitch, now Roll)
            1.5   # Servo 5: Gripper
        ]

# These are the full range of all the servos present in the arm. It is used to map the IK solver's radian outputs to percentage values for the servos.
        self.MAX_JOINT_LIMITS = [
            (-2.356,  2.356),
            (-2.356,  2.356),
            (-2.356,  2.356),
            (-1.5708, 1.5708),
            (-1.5708, 1.5708),
            (-1.5708, 1.5708),
        ]
        
        self.GRIPPER_CLOSED_PERCENT = 15
        self.GRIPPER_OPEN_PERCENT = 65
        self.SERVO_RELEASE_TIMEOUT_SEC = 5.0 # Seconds of inactivity before servos relax

        ## --------------------------------------------------------------------------
        ## Drawing Pose & Screensaver Parameters
        ## --------------------------------------------------------------------------
        ELBOW_DOWNWARD_BEND = 85.0
        SHOULDER_FORWARD_REACH = 40.0
        WRIST_CORRECTION = -5.0
        self.SHOULDER_COMPENSATION = 0.5
        self.WRIST_COMPENSATION = 1.0
        self.DRAWING_POSE = [
            50.0,                               # Servo 0: Base
            SHOULDER_FORWARD_REACH,             # Servo 1: Shoulder
            ELBOW_DOWNWARD_BEND,                # Servo 2: Elbow
            (100 - ELBOW_DOWNWARD_BEND) + WRIST_CORRECTION, # Servo 3: Wrist Pitch (Auto-calculated)
            50.0,                               # Servo 4: Wrist Roll (Keep centered)
            50.0                                # Servo 5: Gripper
        ]
        
        self.SCREENSAVER_SPEED = 0.05
        self.SCREENSAVER_WIDTH = 15.0
        self.SCREENSAVER_HEIGHT = 10.0
        self.SCREENSAVER_PAUSE_SEC = 1.0

        ## --------------------------------------------------------------------------
        ## Servo Limits (ROS 2 Parameters)
        ## --------------------------------------------------------------------------
        # Declare parameters with default 0.0 to 100.0 boundaries
        self.declare_parameter('servo_min_limits', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('servo_max_limits', [100.0, 100.0, 100.0, 100.0, 100.0, 100.0])

        # Fetch limits provided by the launch script
        self.SERVO_MIN = self.get_parameter('servo_min_limits').value
        self.SERVO_MAX = self.get_parameter('servo_max_limits').value
        
        # Calculate a safe "Center" position that respects the new limits
        self.CENTER_POSITIONS = [
            max(self.SERVO_MIN[i], min(self.SERVO_MAX[i], 50.0)) for i in range(self.NUM_SERVOS)
        ]
        self.get_logger().info(f"Loaded MIN limits: {self.SERVO_MIN}")
        self.get_logger().info(f"Loaded MAX limits: {self.SERVO_MAX}")

        ## --------------------------------------------------------------------------
        ## State Storage
        ## --------------------------------------------------------------------------

        self.ik_mode_active = True
        self.ik_mode_button_was_pressed = False
        self.ik_raw_data = []

        self.ik_target_positions = [50.0] * self.NUM_SERVOS
        self.ik_data_updated     = False
        self.target_positions    = list(self.CENTER_POSITIONS)
        self.current_positions   = list(self.CENTER_POSITIONS)

        # Gamepad-specific states
        self.home_button_was_pressed = False
        self.screensaver_active = False
        self.screensaver_time = 0.0
        self.screensaver_button_was_pressed = False
        self.screensaver_is_paused = False
        self.pause_start_time = 0.0
        
        # Inactivity Timer State
        self.last_input_time = time.time()
        self.servos_are_released = False
        
        ## --------------------------------------------------------------------------
        ## Hardware and ROS Initialization
        ## --------------------------------------------------------------------------
        try:
            self.pwm = PCA9685()
            self.pwm.set_pwm_freq(50)
            self.get_logger().info("PCA9685 Initialized at 50Hz.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PCA9685: {e}")
            self.get_logger().error("Is the I2C bus enabled and the device connected? Try running as root.")
            rclpy.shutdown()
            return
        # Subscription to the joystick controller
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        ## @brief Publisher for Cartesian velocity commands sent to
        ## the IK solver when in IK mode.
        ## Message format: [X_vel, Y_vel, Z_vel, home_flag]
        self.cartesian_pub = self.create_publisher(
            Float64MultiArray, 
            '/CartesianCmd',
            10
        )

        ## @brief Publisher for current joint positions in radians,
        ## used by the IK solver for state synchronization.
        self.curr_pos_pub = self.create_publisher(
            Float64MultiArray, 
            '/CurrentPositions',
            10
        )

        ## @brief Subscription to IK solver output containing joint
        ## angles in radians for joints 0-4.
        self.mov_subscription = self.create_subscription(JointState, '/mov', self.mov_callback, 10)
        
        # The Main Control Loop for smoothing and PWM, runs at 50Hz (0.02s)
        self.smoothing_timer = self.create_timer(0.02, self.smoothing_loop)
        
        self.get_logger().info("Centering arm on startup...")
        self.center_all_servos()
        self.get_logger().info("Arm centered. Waiting for commands...")

    ## --------------------------------------------------------------------------
    ## Main Hardware Control and Smoothing Loop
    ## --------------------------------------------------------------------------

    def smoothing_loop(self):
        """
        @brief Main hardware control loop running at 50Hz.

        This is the core servo control function that runs every 20ms.
        It performs three operations in order:
            1. Checks for inactivity timeout and releases servos if needed
            2. Applies incoming IK data to target positions (if IK mode active)
            3. Smooths current positions toward targets using exponential easing
               and sends PWM commands to all servos

        The smoothing uses a first-order low-pass filter:
            current += (target - current) * SMOOTHING_FACTOR

        All target positions are clamped to configured SERVO_MIN/MAX limits
        before smoothing to protect hardware from out-of-range commands.

        """
        # Check for inactivity timeout (only if screensaver is off)
        has_timed_out = (time.time() - self.last_input_time) > self.SERVO_RELEASE_TIMEOUT_SEC
        if has_timed_out and not self.screensaver_active and not self.servos_are_released:
            self.get_logger().info("Inactivity detected. Releasing servos.")
            self.release_all_servos()
            return

        if self.ik_mode_active and self.ik_data_updated and self.ik_raw_data and not self.screensaver_active:
                self.ik_data_updated = False

                # Convert radians to percentages here
                percentages = self.ik_to_percentage(self.ik_raw_data)

                for i in range(min(len(percentages), self.NUM_SERVOS - 1 )):
                    self.target_positions[i] = percentages[i]

        # Apply smoothing and send final commands to servos
        for i in range(self.NUM_SERVOS):
            # Enforce dynamic boundaries here instead of hardcoded 0.0/100.0
            if not self.ik_mode_active:
                self.target_positions[i] = max(self.SERVO_MIN[i], min(self.SERVO_MAX[i], self.target_positions[i]))
            
            error = self.target_positions[i] - self.current_positions[i]
            self.current_positions[i] += error * self.SMOOTHING_FACTOR
            
            # Only send PWM commands if servos are not supposed to be released
            if not self.servos_are_released:
                self.setPercent(i, self.current_positions[i])


    def joy_callback(self, msg: Joy):
        """
        @brief Callback for joystick input, handling mode toggles, home, screensaver, and control commands.
        @param msg (Joy): The joystick message containing axes and button states.
        Control flow:
        1. Reset inactivity timer on any input
        2. Handle mode toggle (B button)
        3. Handle home position (Y button)
        4. Handle screensaver toggle (Start button)
        5. Execute active mode logic:
            - Screensaver mode: Move in a pattern and pause periodically
            - IK mode: Publish Cartesian commands to /CartesianCmd
            - Direct mode: Update target_positions from joystick axes
        6. Handle gripper (LB/RB or stick clicks) in all modes
        """
        # Check for gamepad input to reset inactivity timer
        axes = [0.0] * len(msg.axes)
        for i, value in enumerate(msg.axes):
            if abs(value) > self.DEADZONE:
                axes[i] = value
        
        has_input = any(axes) or any(msg.buttons)
        if has_input:
            self.last_input_time = time.time()
            if self.servos_are_released:
                self.get_logger().info("Gamepad input detected. Re-engaging servos.")
                self.current_positions = list(self.target_positions)
                self.servos_are_released = False

        if msg.buttons[1] == 1 and not self.ik_mode_button_was_pressed:
            self.ik_mode_active = not self.ik_mode_active
            mode = "IK Mode" if self.ik_mode_active else "Joystick Mode"
            self.get_logger().info(f"Toggled {mode} with button B.")
            if self.ik_mode_active:
                pos = self.percentage_to_radians(self.current_positions)
                self.curr_pos_pub.publish(Float64MultiArray(data=pos))
        self.ik_mode_button_was_pressed = (msg.buttons[1] == 1)

        # Home button ('Y') is a master override
        if msg.buttons[3] == 1 and not self.home_button_was_pressed:
            self.get_logger().info("Home button pressed. Setting target to safe center.")
            if self.screensaver_active:
                self.screensaver_active = False
                self.get_logger().info("Screensaver cancelled by home button.")
            self.target_positions = list(self.CENTER_POSITIONS)
        self.home_button_was_pressed = (msg.buttons[3] == 1)
        # Screensaver cancel logic
        is_joystick_moved = any(axes)
        # Check for shoulder buttons (4,5) OR stick clicks (10,11)
        is_action_button_pressed = any(msg.buttons[i] == 1 for i in [4, 5, 10, 11])
        if self.screensaver_active and (is_joystick_moved or is_action_button_pressed):
            if self.ik_mode_active:
                pos = self.percentage_to_radians(self.target_positions)
                self.curr_pos_pub.publish(Float64MultiArray(data=pos))
            self.screensaver_active = False
            self.get_logger().info("Screensaver deactivated by user input.")
        # Screensaver toggle logic ('START' button, index 9)
        if msg.buttons[9] == 1 and not self.screensaver_button_was_pressed:
            self.screensaver_active = not self.screensaver_active
            if self.screensaver_active:
                self.get_logger().info("Drawing screensaver activated.")
                self.screensaver_time = 0.0
                self.screensaver_is_paused = False
            else:
                if self.ik_mode_active:
                    pos = self.percentage_to_radians(self.target_positions)
                    self.curr_pos_pub.publish(Float64MultiArray(data=pos))
                self.get_logger().info("Screensaver deactivated.")
        self.screensaver_button_was_pressed = (msg.buttons[9] == 1)
        # Execute the correct logic
        if self.screensaver_active:
            # --- Screensaver Motion Logic with Pause ---
            if self.screensaver_is_paused:
                if (time.time() - self.pause_start_time) >= self.SCREENSAVER_PAUSE_SEC:
                    self.screensaver_is_paused = False
                    self.screensaver_time = 0.0
            else:
                self.screensaver_time += self.SCREENSAVER_SPEED
                if self.screensaver_time >= (2 * math.pi):
                    self.screensaver_is_paused = True
                    self.pause_start_time = time.time()
                else:
                    base_pose = self.DRAWING_POSE
                    offset_x = self.SCREENSAVER_WIDTH * math.cos(self.screensaver_time)
                    offset_z = self.SCREENSAVER_HEIGHT * math.sin(2 * self.screensaver_time)
                    self.target_positions[0] = base_pose[0] + offset_x
                    self.target_positions[2] = base_pose[2] + offset_z
                    
                    # UPDATED FOR NEW MAPPING: 3 is Pitch, 4 is Roll
                    self.target_positions[3] = base_pose[3] - (offset_z * self.WRIST_COMPENSATION) # Pitch
                    self.target_positions[4] = base_pose[4] # Roll
                    
                    self.target_positions[1] = base_pose[1] - (offset_z * self.SHOULDER_COMPENSATION)
                    self.target_positions[5] = self.DRAWING_POSE[5]
            
        elif not self.servos_are_released:
            if not self.ik_mode_active:
            # --- Normal Joystick Control Logic ---
                self.target_positions[0] += axes[0] * -1 * self.SERVO_SPEEDS[0]  # Base (Left Stick L/R)
                self.target_positions[1] += axes[1] * self.SERVO_SPEEDS[1]       # Shoulder (Left Stick U/D)
                self.target_positions[2] += axes[3] * self.SERVO_SPEEDS[2]       # Elbow (Right Stick U/D)
                self.target_positions[3] += axes[4] * self.SERVO_SPEEDS[3]       # Servo 3 now on Left Side (D-Pad L/R)
                self.target_positions[4] += axes[2] * -1 * self.SERVO_SPEEDS[4]  # Servo 4 now on Right Stick L/R
            
            elif is_joystick_moved or self.home_button_was_pressed: # Allow IK commands if joystick is moved or Home button is pressed
                cmd = Float64MultiArray()
                cmd.data = [
                    float(-axes[0]),
                    float( axes[1]),
                    float( axes[3]),
                    1.0 if self.home_button_was_pressed else 0.0 # data[3]: home button
                ]
                self.cartesian_pub.publish(cmd)
     
            # Gripper Logic: Shoulders (4/5) OR Stick Clicks (10/11)
            if msg.buttons[4] == 1 or msg.buttons[10] == 1: 
                self.target_positions[5] = self.GRIPPER_CLOSED_PERCENT
                self.get_logger().info("Closing gripper.")
            elif msg.buttons[5] == 1 or msg.buttons[11] == 1: 
                self.target_positions[5] = self.GRIPPER_OPEN_PERCENT
                self.get_logger().info("Opening gripper.")

    def mov_callback(self, msg: JointState):
        """
        @brief Receives joint angles from the IK solver node.

        Stores raw radian values for processing by smoothing_loop.
        Kept lightweight to avoid blocking the 50Hz control loop.
        Only fires when the IK solver publishes new data (event-driven,
        not polled).

        @param msg: Float64MultiArray containing joint angles in radians.
            data[0]: Joint 0 angle (Base)
            data[1]: Joint 1 angle (Shoulder)
            data[2]: Joint 2 angle (Elbow)
            data[3]: Joint 3 angle (Wrist Pitch)
            data[4]: Joint 4 angle (Wrist Roll)
        """        
        if len(msg.position) < 1:
            return 

    # Just store the raw radian values — no processing here
        self.ik_raw_data = list(msg.position)
        self.ik_data_updated = True
        
   
    ## --------------------------------------------------------------------------
    ## Hardware Helper Functions
    ## --------------------------------------------------------------------------
    def ik_to_percentage(self, joint_angles_rad):
        """
        @brief Converts joint angles in radians to servo percentages (0-100).

        Maps radians to percentages using asymmetric joint limit handling:
            0 rad = 50% = 1500us = physical neutral [DS3218/MG90S Datasheets]
            Positive angles map from 50% toward 100%
            Negative angles map from 50% toward 0%

        Each direction uses its own limit for mapping, allowing asymmetric
        joint ranges (e.g. shoulder can only move in one direction from neutral).

        @param joint_angles_rad: List of joint angles in radians from IK solver.
        @return: List of servo percentages (0.0 to 100.0) for each joint.
        """
        
        percentages = []

        for i, angle_rad in enumerate(joint_angles_rad[:self.NUM_SERVOS]):
            min_angle  = self.MAX_JOINT_LIMITS[i][0]
            max_angle  = self.MAX_JOINT_LIMITS[i][1]

            if angle_rad >= 0.0:
                if angle_rad == 0.0:
                    percentage = 50.0
                else:
                    percentage = 50.0 + (angle_rad / max_angle) * 50.0

            else:
                percentage = 50.0 + (angle_rad / abs(min_angle)) * 50.0

            percentages.append(percentage)

        return percentages

    def percentage_to_radians(self, percentages):
        """
        @brief Converts servo percentages (0-100) to joint angles in radians.

        Inverse of ik_to_percentage. Uses asymmetric mapping:
            50% = 0 rad = 1500us neutral [DS3218/MG90S Datasheets]
            Above 50%: maps toward positive max_angle
            Below 50%: maps toward negative min_angle

        Appends two extra values to the output:
            - A zero placeholder for the gripper joint
            - A flag indicating whether this is min limits (1),
              max limits (2), or regular position data (0)

        @param percentages: List of servo percentages (0-100) for each joint.
        @return: List of joint angles in radians with appended flag values.
        """
        radians = []
        for i, percentage in enumerate(percentages):
            min_angle = self.MAX_JOINT_LIMITS[i][0]
            max_angle = self.MAX_JOINT_LIMITS[i][1]
            if percentage >= 50:
                angle_rad = ((percentage - 50) / 50) * max_angle
            else:
                angle_rad = ((percentage - 50) / 50) * abs(min_angle)
            radians.append(angle_rad)
        
        radians.append(float(0))
        if percentages == self.SERVO_MAX:
            radians.append(float(2)) 
        elif percentages == self.SERVO_MIN:
            radians.append(float(1))
    

        return radians


    def setPercent(self, servo, percentage):
        """
        @brief Converts a percentage (0-100) to a PWM value and sends it to a servo.
        """
        if not 0 <= servo <= self.NUM_SERVOS: return
        percentage = max(0, min(100, percentage))

        if (servo == 0): max_pulse, min_pulse = 1012, 602
        elif (servo == 1): max_pulse, min_pulse = 1012, 602
        elif (servo == 2): max_pulse, min_pulse = 1012, 602
        elif (servo == 3): max_pulse, min_pulse = 910, 705
        elif (servo == 4): max_pulse, min_pulse = 910, 705
        elif (servo == 5): max_pulse, min_pulse = 930, 620
        else: return
        
        pwm_val = int(min_pulse + (percentage / 100.0) * (max_pulse - min_pulse))
        self.pwm.set_pwm(servo, 500, pwm_val)

    def center_all_servos(self):
        """
        @brief Instantly centers all servos, respecting the safe limits.
        """
        self.target_positions = list(self.CENTER_POSITIONS)
        self.current_positions = list(self.CENTER_POSITIONS)
        for i in range(self.NUM_SERVOS):
            self.setPercent(i, self.CENTER_POSITIONS[i])
            
    def release_all_servos(self):
        """
        @brief Turns off PWM signals to all servos, allowing them to go limp.
        """
        self.get_logger().info("Releasing all servos (turning off PWM).")
        for i in range(self.NUM_SERVOS):
            self.pwm.set_pwm(i, 0, 0)
        self.servos_are_released = True

## --------------------------------------------------------------------------
## Main Function
## --------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    arm_controller_node = ArmControllerNode()
    try:
        rclpy.spin(arm_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        # shut down the node.
        arm_controller_node.get_logger().info("Shutting down...")
        # Center the arm before exiting.
        for i in range(arm_controller_node.NUM_SERVOS):
            arm_controller_node.setPercent(i, arm_controller_node.CENTER_POSITIONS[i])
        arm_controller_node.get_logger().info("Arm centered.")
        time.sleep(1)
        arm_controller_node.release_all_servos()
        arm_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
