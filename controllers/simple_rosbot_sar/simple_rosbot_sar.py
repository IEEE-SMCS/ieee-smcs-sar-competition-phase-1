"""
Basic Rosbot Controller for Search and Rescue Simulation
This controller implements a simple exploration strategy with human-readable explanations.
"""

from controller import Robot
import json
import random
import math
from enum import Enum
from typing import Tuple, List


class RobotState(Enum):
    """Robot operational states"""

    EXPLORING = "exploring"
    WAITING_APPROVAL = "waiting_approval"


class BasicRosbotController:
    def __init__(self):
        """Initialize the robot controller"""
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Get robot name for identification
        self.robot_id = self.robot.getName()

        # Initialize sensors and actuators
        self._init_devices()

        # Robot state and navigation
        self.state = RobotState.EXPLORING
        self.last_decision_time = 0
        self.decision_interval = 5.0  # seconds between decisions
        self.action_pending = False

        # Exploration variables
        self.proposed_action_fun = None
        self.turn_direction = 1  # 1 for left, -1 for right

        # Victim detection
        self.victim_confidence_threshold = 0.7

        # Navigation constants
        self.max_speed = 5.0
        self.obstacle_threshold = 0.5  # distance sensor threshold

        print(f"[{self.robot_id}] Initialized - Ready for search and rescue mission")

    def _init_devices(self):
        """Initialize robot sensors and actuators"""
        # Motors
        self.front_left_motor = self.robot.getDevice("fl_wheel_joint")
        self.front_right_motor = self.robot.getDevice("fr_wheel_joint")
        self.rear_left_motor = self.robot.getDevice("rl_wheel_joint")
        self.rear_right_motor = self.robot.getDevice("rr_wheel_joint")
        self.front_left_motor.setPosition(float("inf"))
        self.front_right_motor.setPosition(float("inf"))
        self.rear_left_motor.setPosition(float("inf"))
        self.rear_right_motor.setPosition(float("inf"))
        self.front_left_motor.setVelocity(0)
        self.front_right_motor.setVelocity(0)
        self.rear_left_motor.setVelocity(0)
        self.rear_right_motor.setVelocity(0)

        # Wheel position sensors
        self.front_left_position_sensor = self.robot.getDevice(
            "front left wheel motor sensor"
        )
        self.front_right_position_sensor = self.robot.getDevice(
            "front right wheel motor sensor"
        )
        self.rear_left_position_sensor = self.robot.getDevice(
            "rear left wheel motor sensor"
        )
        self.rear_right_position_sensor = self.robot.getDevice(
            "rear right wheel motor sensor"
        )
        self.front_left_position_sensor.enable(self.timestep)
        self.front_right_position_sensor.enable(self.timestep)
        self.rear_left_position_sensor.enable(self.timestep)
        self.rear_right_position_sensor.enable(self.timestep)

        # RGB camera
        try:
            self.camera_rgb = self.robot.getDevice("camera rgb")
            self.camera_rgb.enable(self.timestep)
        except:
            self.camera_rgb = None
            print(f"[{self.robot_id}] Warning: No RGB camera found")

        # Depth camera
        try:
            self.camera_depth = self.robot.getDevice("camera depth")
            self.camera_depth.enable(self.timestep)
        except:
            self.camera_depth = None
            print(f"[{self.robot_id}] Warning: No depth camera found")

        # Lidar sensor
        try:
            self.lidar = self.robot.getDevice("laser")
            self.lidar.enable(self.timestep)
        except:
            self.lidar = None
            print(f"[{self.robot_id}] Warning: No lidar found")

        # GPS
        try:
            self.gps = self.robot.getDevice("gps")
            self.gps.enable(self.timestep)
        except:
            print(f"[{self.robot_id}] Warning: No gps found")
            self.gps = None

        # Accelerometer
        try:
            self.accelerometer = self.robot.getDevice("imu accelerometer")
            self.accelerometer.enable(self.timestep)
        except:
            print(f"[{self.robot_id}] Warning: No accelerometer found")
            self.accelerometer = None

        # Gyro
        try:
            self.gyro = self.robot.getDevice("imu gyro")
            self.gyro.enable(self.timestep)
        except:
            print(f"[{self.robot_id}] Warning: No gyro found")
            self.gyro = None

        # Compass
        try:
            self.compass = self.robot.getDevice("imu compass")
            self.compass.enable(self.timestep)
        except:
            print(f"[{self.robot_id}] Warning: No compass found")
            self.compass = None

        # Distance sensors
        self.distance_sensors = []
        sensor_names = ["fl_range", "fr_range", "rl_range", "rr_range"]
        for name in sensor_names:
            try:
                sensor = self.robot.getDevice(name)
                sensor.enable(self.timestep)
                self.distance_sensors.append(sensor)
            except:
                # If sensor doesn't exist, create a dummy
                print(f"[{self.robot_id}] Warning: No {name} sensor found")

        # Communication devices to supervisor
        try:
            self.supervisor_receiver = self.robot.getDevice("supervisor receiver")
            self.supervisor_receiver.enable(self.timestep)

            self.supervisor_emitter = self.robot.getDevice("supervisor emitter")
        except:
            print(
                f"[{self.robot_id}] Warning: Supervisor communication devices not found"
            )
            self.supervisor_receiver = None
            self.supervisor_emitter = None

        # Communication devices for robot to robot communication
        try:
            self.squad_receiver = self.robot.getDevice("robot to robot receiver")
            self.squad_receiver.enable(self.timestep)

            self.squad_emitter = self.robot.getDevice("robot to robot emitter")
        except:
            print(
                f"[{self.robot_id}] Warning: Robot to robot communication devices not found"
            )
            self.squad_receiver = None
            self.squad_emitter = None

    def get_position(self) -> Tuple[float, float, float]:
        """Get current robot position"""
        if self.gps:
            return self.gps.getValues()

        # Return dummy position if no GPS
        return (0.0, 0.0, 0.0)

    def get_orientation(self) -> float:
        """Get current robot orientation in radians"""
        if self.compass:
            north = self.compass.getValues()
            return math.atan2(north[0], north[1])

        return 0.0

    def get_distance_readings(self) -> List[float]:
        """Get distance sensor readings"""
        readings = []
        for sensor in self.distance_sensors:
            readings.append(sensor.getValue())
        return readings

    def detect_obstacles(self) -> Tuple[bool, str]:
        """
        Detect obstacles around the robot
        Returns: (obstacle_detected, description)
        """
        distances = self.get_distance_readings()

        # Check front sensors
        if (
            distances[0] < self.obstacle_threshold
            and distances[1] < self.obstacle_threshold
        ):
            return True, "obstacle ahead"
        elif distances[0] < self.obstacle_threshold:
            return True, "obstacle front left"
        elif distances[1] < self.obstacle_threshold:
            return True, "obstacle front right"

        return False, "clear path"

    def detect_victim(self) -> Tuple[bool, float, str]:
        """
        Attempt to detect victims using camera and sensors
        Returns: (victim_detected, confidence, description)
        """

        position = self.get_position()

        # Random chance of detecting something (including false positives)
        if random.random() < 0.001:  # small chance per check
            # Generate confidence based on "sensor quality"
            base_confidence = random.uniform(0.3, 0.95)

            if base_confidence > self.victim_confidence_threshold:
                confidence = base_confidence
                description = f"Potential victim detected at position {position[0]:.1f}, {position[1]:.1f}"
                return True, confidence, description
            else:
                confidence = random.uniform(0.1, 0.6)
                description = f"Uncertain detection - possible debris or false positive"
                return True, confidence, description

        return False, 0.0, "No victims detected in current area"

    def generate_explanation(self, action: str) -> str:
        """Generate human-readable explanation for the intended action"""

        position = self.get_position()

        explanations = {
            "explore_forward": [
                f"Moving forward to explore uncharted territory at coordinates {position[0]:.1f}, {position[1]:.1f}",
                "Proceeding ahead to systematically search for victims in this area",
                "Continuing forward exploration to maximize coverage of the search zone",
            ],
            "turn_left": [
                "Turning left to avoid obstacle and continue search pattern",
                f"Changing direction left to explore new areas",
                "Executing left turn to maintain systematic room coverage",
            ],
            "turn_right": [
                "Turning right to navigate around detected obstacle",
                f"Adjusting course right due to obstacles",
                "Making right turn to access unexplored sections",
            ],
            "investigate_victim": [
                f"Moving closer to investigate potential victim",
                f"Approaching suspected victim location for detailed assessment",
                f"Investigating anomaly that may be a victim",
            ],
        }

        # Select appropriate explanation based on action
        if action in explanations:
            explanation = random.choice(explanations[action])
        else:
            explanation = f"Executing {action} to continue search and rescue mission"

        return explanation

    def send_decision_request(
        self,
        action: str,
        reason: str,
        victim_detected: bool = False,
        confidence: float = 0.0,
    ):
        """Send decision request to supervisor"""
        if not self.supervisor_emitter:
            print(f"[{self.robot_id}] Warning: Cannot send request - no emitter")
            return

        request = {
            "timestamp": self.robot.getTime(),
            "robot_id": self.robot_id,
            "position": self.gps.getValues(),
            "intended_action": action,
            "reason": reason,
            "victim_found": victim_detected,
            "victim_confidence": confidence,
        }

        message = json.dumps(request)
        self.supervisor_emitter.send(message.encode())

        self.action_pending = True
        self.state = RobotState.WAITING_APPROVAL

        print(f"[{self.robot_id}] REQUEST: {action} - {reason}")
        if victim_detected:
            print(f"[{self.robot_id}] VICTIM ALERT: Confidence {confidence:.1%}")

    def handle_supervisor_response(self):
        """Handle response from supervisor"""

        while self.supervisor_receiver.getQueueLength() > 0:
            message = self.supervisor_receiver.getString()
            self.supervisor_receiver.nextPacket()

            try:
                response = json.loads(message)
                approved = response.get("approved", False)
                message_target = response.get("robot_id", None)
                # Ignore messages that are not for this particular robot
                if message_target != self.robot_id:
                    print(
                        f"Ignoring message for a different robot {message_target, self.robot_id}"
                    )
                    continue

                if approved:
                    self.action_pending = False
                    self.state = RobotState.EXPLORING
                    return True
                else:
                    self.action_pending = False
                    self.state = RobotState.EXPLORING
                    return False

            except json.JSONDecodeError:
                print(f"[{self.robot_id}] Error decoding supervisor response")

        return False  # No response yet

    def set_wheel_speeds(self, left_speed: float, right_speed: float):
        """Set wheel motor speeds"""
        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))

        self.front_left_motor.setVelocity(left_speed)
        self.rear_left_motor.setVelocity(left_speed)
        self.front_right_motor.setVelocity(right_speed)
        self.rear_right_motor.setVelocity(right_speed)

    def move_forward(self, speed: float = None):
        """Move robot forward"""
        if speed is None:
            speed = self.max_speed * 0.7
        self.set_wheel_speeds(speed, speed)

    def turn_left(self, speed: float = None):
        """Turn robot left"""
        if speed is None:
            speed = self.max_speed * 0.5
        self.set_wheel_speeds(-speed, speed)

    def turn_right(self, speed: float = None):
        """Turn robot right"""
        if speed is None:
            speed = self.max_speed * 0.5
        self.set_wheel_speeds(speed, -speed)

    def stop(self):
        """Stop robot movement"""
        self.set_wheel_speeds(0, 0)

    def explore_behavior(self, force_new_action):
        """Main exploration behavior logic"""
        obstacle_detected, obstacle_desc = self.detect_obstacles()
        victim_detected, victim_confidence, victim_desc = self.detect_victim()
        current_time = self.robot.getTime()

        # Priority 1: Investigate potential victims
        if victim_detected:
            reason = self.generate_explanation("investigate_victim")
            self.send_decision_request(
                "investigate_victim", reason, True, victim_confidence
            )
            self.last_decision_time = current_time
            self.proposed_action_fun = self.move_forward
            return

        # Priority 2: Handle obstacles
        elif obstacle_detected:
            if "front right" in obstacle_desc:
                reason = self.generate_explanation("turn_left")
                self.send_decision_request("turn_left", reason)
                self.proposed_action_fun = self.turn_left
            else:
                reason = self.generate_explanation("turn_right")
                self.send_decision_request("turn_right", reason)
                self.proposed_action_fun = self.turn_right

            self.last_decision_time = current_time
            return

        # Check if it's time to make a decision
        if (force_new_action and not self.action_pending) or (
            current_time - self.last_decision_time > self.decision_interval
            and not self.action_pending
        ):
            exploration_actions = [
                ("explore_forward", 0.4, self.move_forward),
                ("turn_left", 0.3, self.turn_left),
                ("turn_right", 0.3, self.turn_right),
            ]

            # Weighted random selection
            rand_val = random.random()
            cumulative = 0
            selected_action = "explore_forward"

            for action, weight, cb in exploration_actions:
                cumulative += weight
                if rand_val <= cumulative:
                    selected_action = action
                    self.proposed_action_fun = cb
                    break

            reason = self.generate_explanation(selected_action)
            self.send_decision_request(
                selected_action, reason, victim_detected, victim_confidence
            )
            self.last_decision_time = current_time

    def run(self):
        """Main robot control loop"""
        print(f"[{self.robot_id}] Starting search and rescue mission")

        while self.robot.step(self.timestep) != -1:

            # Handle supervisor communication
            force_replan = False
            if self.action_pending:
                approval = self.handle_supervisor_response()
                if not approval:
                    # Action was rejected, change behavior
                    self.state = RobotState.EXPLORING
                    force_replan = True
                else:
                    # Execute the proposed action
                    self.proposed_action_fun()

            # Execute behavior based on current state
            if self.state == RobotState.EXPLORING:
                self.explore_behavior(force_replan)

            elif self.state == RobotState.WAITING_APPROVAL:
                # Stop and wait for approval
                self.stop()


def main():
    """Main function to run the robot controller"""
    controller = BasicRosbotController()
    controller.run()


if __name__ == "__main__":
    main()
