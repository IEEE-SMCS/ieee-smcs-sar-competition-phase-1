"""
Human-in-the-Loop Supervisor for Search and Rescue Simulation
This supervisor manages robot decisions and logs their actions.
"""

from controller import Supervisor
import random
import json
from datetime import datetime
import os
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional
import requests
import warnings


@dataclass
class SupervisorDecisionLogEntry:
    """A log entry by the supervisor"""

    robot_id: str  # Identifier of robot affected by decision
    decision_timestamp: float  # Time at which the supervisor made the decision
    proposal_timestamp: float  # Time at which the robot proposed the action
    position: List  # Position of the robot at decision time
    reported_position: (
        List  # Position reported by robot at the time of proposing the action
    )
    intended_action: str  # Action proposed by robot
    reason: str  # Explanation for intended action
    victim_found: bool  # Robot report of finding a victim
    victim_confidence: float  # Confidence on victim found report
    victim_found_verdict: (
        bool  # True if robot was correct, False if robot was incorrect
    )
    approved: bool  # Approval/rejection of proposed action
    supervisor_reason: str = ""  # Reason for approval/rejection


class HumanSupervisor:
    def __init__(
        self,
        rejection_rate: float = 0.2,
        log_file: str = None,
        max_duration: int = 180,
    ):
        """
        Initialize the Human-in-the-Loop Supervisor

        Args:
            rejection_rate: Probability of rejecting a robot's action (0.0 to 1.0)
            log_file: Path to save mission logs
            max_duration: Mission duration in seconds
        """
        self.supervisor = Supervisor()
        self.rejection_rate = rejection_rate
        self.log_file = log_file
        self.max_duration = max_duration
        self.decisions_log: List[SupervisorDecisionLogEntry] = []
        self.robots = {}
        self.victims_found = set()

        # Communication channels
        self.receiver = self.supervisor.getDevice("receiver")
        self.receiver.enable(int(self.supervisor.getBasicTimeStep()))

        self.emitter = self.supervisor.getDevice("emitter")

        # Get all robots in the simulation
        self._discover_robots()

        # Get all victims in the simulation
        self._discover_victims()

        # Create logs file
        self.logs_dir = "../../log"
        if not os.path.exists(self.logs_dir):
            os.mkdir(self.logs_dir)
        print(
            f"[SUPERVISOR] Found {len(self.robots)} robots and {len(self.victims)} victims"
        )

    def _discover_robots(self):
        """Find all Rosbot robots in the simulation"""
        root = self.supervisor.getRoot()
        children_field = root.getField("children")

        for i in range(children_field.getCount()):
            node = children_field.getMFNode(i)
            if node and node.getTypeName() == "Rosbot":
                robot_name = node.getField("name").getSFString()
                self.robots[robot_name] = {
                    "node": node,
                    "decisions": [],
                    "victims_found": 0,
                    "distance_travelled": 0.0,
                    "last_position": None,
                }
                print(f"[SUPERVISOR] Discovered robot: {robot_name}")

    def _discover_victims(self):
        """Find all victims in the simulation"""
        self.victims = {}
        root = self.supervisor.getRoot()
        children_field = root.getField("children")

        for i in range(children_field.getCount()):
            node = children_field.getMFNode(i)
            if node and node.getTypeName() == "Victim":
                victim_name = node.getField("name").getSFString()
                position = node.getField("translation").getSFVec3f()
                self.victims[victim_name] = {
                    "node": node,
                    "position": position,
                    "found": False,
                    "found_by": None,
                    "found_time": None,
                }
                print(f"[SUPERVISOR] Discovered victim: {victim_name} at {position}")

    def _get_robot_position(self, robot_id: str) -> Optional[List]:
        """Get current position of a robot"""
        if robot_id in self.robots:
            node = self.robots[robot_id]["node"]
            return node.getField("translation").getSFVec3f()
        return None

    def _check_victim_proximity(
        self, robot_pos: List, detection_radius: float = 1.0
    ) -> tuple:
        """
        Check if robot is near any victims
        Returns: (victim_found, victim_id)
        """
        if not robot_pos:
            return False, None

        for victim_id, victim_data in self.victims.items():
            if victim_data["found"]:
                continue

            victim_pos = victim_data["position"]
            distance = (
                (robot_pos[0] - victim_pos[0]) ** 2
                + (robot_pos[1] - victim_pos[1]) ** 2
            ) ** 0.5

            if distance < detection_radius:
                return True, victim_id

        return False, None

    def process_robot_request(self, data: Dict) -> Dict:
        """
        Process a decision request from a robot

        Expected data format:
        {
            "timestamp": float
            "robot_id": str,
            "position": list,
            "intended_action": str,
            "reason": str,
            "victim_found": bool,
            "victim_confidence": float,
        }
        """

        # Get robot position
        robot_id = data.get("robot_id", "undefined")
        robot_pos = self._get_robot_position(robot_id)

        # Check for nearby victims
        super_victim_found, super_victim_id = self._check_victim_proximity(robot_pos)

        # Make approval decision
        approved = (
            random.random() > self.rejection_rate
            if not data.get("victim_found", False)
            else True
        )

        # Generate supervisor reason
        if not approved:
            supervisor_reason = "Action rejected - please re-plan"
        else:
            supervisor_reason = "Action approved - proceed with caution"

        # Log the supervisors decision
        decision = SupervisorDecisionLogEntry(
            robot_id=robot_id,
            decision_timestamp=self.supervisor.getTime(),
            proposal_timestamp=data.get("timestamp", -1.0),
            position=robot_pos if robot_pos else [-1000.0, -1000.0, -1000.0],
            reported_position=data.get("position", [-1000.0, -1000.0, -1000.0]),
            intended_action=data.get("intended_action", "undefined"),
            reason=data.get("reason", "undefined"),
            victim_found=data.get("victim_found", False),
            victim_confidence=data.get("victim_confidence", 0.0),
            victim_found_verdict=super_victim_found == data.get("victim_found", False),
            approved=approved,
            supervisor_reason=supervisor_reason,
        )
        self.decisions_log.append(decision)

        # Update robot stats
        if robot_id in self.robots:
            self.robots[robot_id]["decisions"].append(decision)

            # Mark victim as found if applicable
            if (
                decision.victim_found_verdict
                and super_victim_id
                and not self.victims[super_victim_id]["found"]
            ):
                self.victims[super_victim_id]["found"] = True
                self.victims[super_victim_id]["found_by"] = robot_id
                self.victims[super_victim_id][
                    "found_time"
                ] = decision.proposal_timestamp
                self.robots[robot_id]["victims_found"] += 1
                print(
                    f"[SUPERVISOR] VICTIM FOUND: {super_victim_id} by {robot_id} (confidence: {decision.victim_confidence:.2f})"
                )

        # Print decision
        status = "APPROVED" if approved else "REJECTED"
        print(
            f"[SUPERVISOR] {status} - Robot: {robot_id}, Action: {decision.intended_action}"
        )
        if not approved:
            print(f"              Reason: {supervisor_reason}")

        # Return response
        return {
            "timestamp": self.supervisor.getTime(),
            "robot_id": robot_id,
            "approved": approved,
            "supervisor_reason": supervisor_reason,
        }

    def handle_communications(self):
        """Check for incoming messages from robots"""
        while self.receiver.getQueueLength() > 0:
            message = self.receiver.getString()
            self.receiver.nextPacket()

            try:
                # Parse JSON message from robot
                data = json.loads(message)

                # Process the request
                response = self.process_robot_request(data)

                # Send response back
                response_json = json.dumps(response)
                self.emitter.send(response_json.encode())

            except json.JSONDecodeError:
                print(f"[SUPERVISOR] Error decoding message: {message}")
            except Exception as e:
                print(f"[SUPERVISOR] Error processing request: |{e}| ")

    def save_logs(self):
        """Save mission logs to file"""
        mission_data = {
            "mission_duration": self.supervisor.getTime(),
            "total_decisions": len(self.decisions_log),
            "total_victims": len(self.victims),
            "victims_found": sum(1 for v in self.victims.values() if v["found"]),
            "robots": {},
            "victims": {},
            "decisions": [asdict(d) for d in self.decisions_log],
        }

        # Add robot statistics
        for robot_id, robot_data in self.robots.items():
            mission_data["robots"][robot_id] = {
                "victims_found": robot_data["victims_found"],
                "distance_travelled": robot_data["distance_travelled"],
                "decisions_made": len(robot_data["decisions"]),
                "decisions_approved": sum(
                    1 for d in robot_data["decisions"] if d.approved
                ),
            }

        # Add victim statistics
        for victim_id, victim_data in self.victims.items():
            mission_data["victims"][victim_id] = {
                "position": victim_data["position"],
                "found": victim_data["found"],
                "found_by": victim_data["found_by"],
                "found_time": victim_data["found_time"],
            }

        # Save to file
        if self.log_file:
            log_file = self.log_file
        else:
            fname = datetime.now().strftime("SupervisorLog_%Y_%h_%d___%H_%M_%S")
            log_file = f"{self.logs_dir}/{fname}.json"

        with open(log_file, "w") as f:
            json.dump(mission_data, f, indent=2)

        print(f"[SUPERVISOR] Mission logs saved to {log_file}")

        # Send request to marking server
        url = "https://ieee-sar-competition.ts.r.appspot.com/mark-log"
        response = requests.post(url, json=mission_data)
        if response.status_code != 200:
            warnings.warn(
                "Error sending log to marking server! Please double check log structure, make sure there are no missing fields"
            )
            warnings.warn("Server returned status code", response.status_code)
            warnings.warn("Server error reason: ", response.reason)
            warnings.warn("Server error text: ", response.text)
        else:
            print(response.text)

    def print_mission_summary(self):
        """Print summary of the mission"""
        duration = self.supervisor.getTime()
        victims_found = sum(1 for v in self.victims.values() if v["found"])

        print("\n" + "=" * 60)
        print("MISSION SUMMARY")
        print("=" * 60)
        print(f"Duration: {duration:.3f} seconds")
        print(f"Victims Found: {victims_found}/{len(self.victims)}")
        print(f"Total Decisions: {len(self.decisions_log)}")
        print(f"Decisions Approved: {sum(1 for d in self.decisions_log if d.approved)}")

        print("\nRobot Performance:")
        for robot_id, robot_data in self.robots.items():
            approved = sum(1 for d in robot_data["decisions"] if d.approved)
            total = len(robot_data["decisions"])
            print(f"  {robot_id}:")
            print(f"    - Victims Found: {robot_data['victims_found']}")
            print(f"    - Distance Travelled: {robot_data['distance_travelled']:.1f}m")
            print(f"    - Decisions: {approved}/{total} approved")

        print("\nVictim Status:")
        for victim_id, victim_data in self.victims.items():
            if victim_data["found"]:
                print(
                    f"  {victim_id}: FOUND by {victim_data['found_by']} at t={victim_data['found_time']:.1f}s"
                )
            else:
                print(f"  {victim_id}: NOT FOUND")
        print("=" * 60)

    def update_distance_travelled(self):
        """Updates distance travelled for all Rosbots"""
        for robot_id in self.robots:
            robot_pos = self._get_robot_position(robot_id)
            if self.robots[robot_id]["last_position"] and robot_pos:
                last_pos = self.robots[robot_id]["last_position"]
                dist = (
                    (robot_pos[0] - last_pos[0]) ** 2
                    + (robot_pos[1] - last_pos[1]) ** 2
                ) ** 0.5
                self.robots[robot_id]["distance_travelled"] += dist
            self.robots[robot_id]["last_position"] = robot_pos

    def run(self):
        """Main supervisor loop"""
        timestep = int(self.supervisor.getBasicTimeStep())
        self.last_update_time = 0
        print("[SUPERVISOR] Starting mission supervision...")

        while self.supervisor.step(timestep) != -1:
            current_time = self.supervisor.getTime()

            # Handle incoming communications
            self.handle_communications()
            self.update_distance_travelled()

            # Periodic status update (every second)
            if int(current_time) > self.last_update_time and int(current_time) % 1 == 0:
                self.last_update_time = int(current_time)
                victims_found = sum(1 for v in self.victims.values() if v["found"])
                print(
                    f"[SUPERVISOR] Status - Time: {current_time:.3f}s, "
                    f"Victims: {victims_found}/{len(self.victims)}, "
                    f"Decisions: {len(self.decisions_log)}"
                )

            # Stop if all victims found
            if sum(1 for v in self.victims.values() if v["found"]) == len(self.victims):
                print("[SUPERVISOR] All victims found! Mission complete.")
                break

            # Stop if timer is done
            if current_time > self.max_duration:
                print("[SUPERVISOR] Mission timeout reached.")
                break

        # Mission complete
        self.supervisor.simulationSetMode(0)
        self.print_mission_summary()
        self.save_logs()
        self.supervisor.simulationReset()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Human-in-the-loop supervisor for SAR simulation"
    )
    parser.add_argument(
        "--rejection-rate",
        type=float,
        default=0.2,
        help="Probability of rejecting robot actions [0.0, 1.0]",
    )
    parser.add_argument(
        "--log-file",
        type=str,
        help="Path to save mission logs",
    )

    args = parser.parse_args()

    supervisor = HumanSupervisor(
        rejection_rate=args.rejection_rate, log_file=args.log_file
    )
    supervisor.run()
