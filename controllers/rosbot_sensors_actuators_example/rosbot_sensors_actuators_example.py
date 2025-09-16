from controller import Robot
import numpy as np
from PIL import Image
import os

class Rosbot:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Directory for debug outputs
        self.debug_dir = os.path.join(os.getcwd(), "example_camera_outputs")
        os.makedirs(self.debug_dir, exist_ok=True)

        # --- Motors ---
        self.motors = {}
        for k, name in (
            ("fl", "fl_wheel_joint"),
            ("fr", "fr_wheel_joint"),
            ("rl", "rl_wheel_joint"),
            ("rr", "rr_wheel_joint"),
        ):
            try:
                m = self.robot.getDevice(name)
                m.setPosition(float("inf"))
                m.setVelocity(0.0)
                self.motors[k] = m
            except Exception as ex:
                self.motors[k] = None
                print(f"[Rosbot] Warning: Motor '{name}' not found", ex)

        # --- Wheel position sensors ---
        self.wheel_sensors = {}
        for k, name in (
            ("fl", "front left wheel motor sensor"),
            ("fr", "front right wheel motor sensor"),
            ("rl", "rear left wheel motor sensor"),
            ("rr", "rear right wheel motor sensor"),
        ):
            try:
                s = self.robot.getDevice(name)
                s.enable(self.timestep)
                self.wheel_sensors[k] = s
            except Exception as ex:
                self.wheel_sensors[k] = None
                print(f"[Rosbot] Warning: Wheel sensor '{name}' not found", ex)

        # --- RGB + Depth Camera (Orbbec Astra) ---
        try:
            self.camera_rgb = self.robot.getDevice("camera rgb")
            self.camera_rgb.enable(self.timestep)
            print(
                f"[Rosbot] RGB Camera: {self.camera_rgb.getWidth()}x{self.camera_rgb.getHeight()}"
            )
        except Exception as ex:
            self.camera_rgb = None
            print("[Rosbot] Warning: No RGB camera found", ex)

        try:
            self.camera_depth = self.robot.getDevice("camera depth")
            self.camera_depth.enable(self.timestep)
            print(
                f"[Rosbot] Depth Camera: {self.camera_depth.getWidth()}x{self.camera_depth.getHeight()}"
            )
            print(
                f"[Rosbot] Depth range: {self.camera_depth.getMinRange()} - {self.camera_depth.getMaxRange()} meters"
            )
        except Exception as ex:
            self.camera_depth = None
            print("[Rosbot] Warning: No depth camera found", ex)

        # --- Lidar ---
        try:
            self.lidar = self.robot.getDevice("laser")
            self.lidar.enable(self.timestep)
            print(
                f"[Rosbot] Lidar resolution: {self.lidar.getHorizontalResolution()} points"
            )
            print(f"[Rosbot] Lidar FOV: {self.lidar.getFov()} rad")
            print(
                f"[Rosbot] Lidar range: {self.lidar.getMinRange()} - {self.lidar.getMaxRange()} meters"
            )
            # point cloud is optional; some lidar devices support it
            try:
                self.lidar.enablePointCloud()
                self.lidar_pointcloud_supported = True
            except Exception:
                self.lidar_pointcloud_supported = False
                print("[Rosbot] Lidar does not support point cloud")
        except Exception as ex:
            self.lidar = None
            self.lidar_pointcloud_supported = False
            print("[Rosbot] Warning: No lidar found", ex)

        # --- GPS ---
        try:
            self.gps = self.robot.getDevice("gps")
            self.gps.enable(self.timestep)
        except Exception as ex:
            self.gps = None
            print("[Rosbot] Warning: No gps found", ex)

        # --- IMU components ---
        try:
            self.accelerometer = self.robot.getDevice("imu accelerometer")
            self.accelerometer.enable(self.timestep)
        except Exception as ex:
            self.accelerometer = None
            print("[Rosbot] Warning: No accelerometer found", ex)

        try:
            self.gyro = self.robot.getDevice("imu gyro")
            self.gyro.enable(self.timestep)
        except Exception as ex:
            self.gyro = None
            print("[Rosbot] Warning: No gyro found", ex)

        try:
            self.compass = self.robot.getDevice("imu compass")
            self.compass.enable(self.timestep)
        except Exception as ex:
            self.compass = None
            print("[Rosbot] Warning: No compass found", ex)

        # inertial unit (quaternion / orientation)
        try:
            self.inertial_unit = self.robot.getDevice("imu inertial_unit")
            self.inertial_unit.enable(self.timestep)
        except Exception as ex:
            self.inertial_unit = None
            print("[Rosbot] Warning: No inertial unit found", ex)

        # --- Range Finders (short-range IR / distance sensors) ---
        self.range_sensors = {}
        for name in ("fl_range", "fr_range", "rl_range", "rr_range"):
            try:
                s = self.robot.getDevice(name)
                s.enable(self.timestep)
                print(
                    f"[Rosbot] RangeFinder {name}: {s.getMinValue()} - {s.getMaxValue()} meters"
                )
                self.range_sensors[name] = s
            except Exception as ex:
                self.range_sensors[name] = None
                print(f"[Rosbot] Warning: No {name} sensor found", ex)

        # --- Communication Devices ---
        try:
            self.supervisor_receiver = self.robot.getDevice("supervisor receiver")
            self.supervisor_receiver.enable(self.timestep)
            self.supervisor_emitter = self.robot.getDevice("supervisor emitter")
        except Exception as ex:
            self.supervisor_receiver = None
            self.supervisor_emitter = None
            print("[Rosbot] Warning: Supervisor communication devices not found", ex)

        try:
            self.squad_receiver = self.robot.getDevice("robot to robot receiver")
            self.squad_receiver.enable(self.timestep)
            self.squad_emitter = self.robot.getDevice("robot to robot emitter")
        except Exception as ex:
            self.squad_receiver = None
            self.squad_emitter = None
            print(
                "[Rosbot] Warning: Robot-to-robot communication devices not found", ex
            )

    # ----------------------
    # Actuator helper methods
    # ----------------------
    def set_wheel_speeds(self, fl=0.0, fr=0.0, rl=0.0, rr=0.0):
        """Set wheel velocities (rad/s). None motors are ignored."""
        if self.motors.get("fl"):
            self.motors["fl"].setVelocity(fl)
        if self.motors.get("fr"):
            self.motors["fr"].setVelocity(fr)
        if self.motors.get("rl"):
            self.motors["rl"].setVelocity(rl)
        if self.motors.get("rr"):
            self.motors["rr"].setVelocity(rr)

    # ----------------------
    # Sensor access helpers
    # ----------------------
    def get_rgb_image(self):
        if not self.camera_rgb:
            return None
        buf = self.camera_rgb.getImage()
        w, h = self.camera_rgb.getWidth(), self.camera_rgb.getHeight()
        # Webots stores camera images as BGRA
        img = np.frombuffer(buf, dtype=np.uint8).reshape((h, w, 4))
        rgb = img[:, :, :3][:, :, ::-1]  # convert BGRA->RGB
        return rgb

    def get_depth_image(self):
        if not self.camera_depth:
            return None
        w, h = self.camera_depth.getWidth(), self.camera_depth.getHeight()
        depth = np.array(self.camera_depth.getRangeImage(), dtype=np.float32).reshape(
            (h, w)
        )
        return depth

    def get_lidar_ranges(self):
        """Return lidar range image (list of distances) or None."""
        if not self.lidar:
            return None
        try:
            return self.lidar.getRangeImage()
        except Exception:
            return None

    def get_lidar_point_cloud(self):
        """Return lidar point cloud if supported, otherwise None."""
        if not self.lidar or not self.lidar_pointcloud_supported:
            return None
        try:
            return self.lidar.getPointCloud()
        except Exception:
            return None

    def get_inertial_quaternion(self):
        """Return quaternion from the inertial unit or None."""
        if not self.inertial_unit:
            return None
        try:
            return self.inertial_unit.getQuaternion()
        except Exception:
            return None

    # ----------------------
    # Debug image saving
    # ----------------------
    def save_rgb_frame(self, step_count):
        rgb = self.get_rgb_image()
        if rgb is None:
            return
        img = Image.fromarray(rgb)
        path = os.path.join(self.debug_dir, f"rgb_step{step_count}.png")
        img.save(path)
        print(f"[Rosbot] Saved RGB frame -> {path}")

    def save_depth_frame(self, step_count):
        depth = self.get_depth_image()
        if depth is None:
            return
        dmin, dmax = self.camera_depth.getMinRange(), self.camera_depth.getMaxRange()
        depth_clipped = np.clip(depth, dmin, dmax)
        norm = ((depth_clipped - dmin) / (dmax - dmin) * 255).astype(np.uint8)
        img = Image.fromarray(norm)
        path = os.path.join(self.debug_dir, f"depth_step{step_count}.png")
        img.save(path)
        print(f"[Rosbot] Saved Depth frame -> {path}")

    # ----------------------
    # High level read method
    # ----------------------
    def read_sensors(self):
        """Take a snapshot of all enabled sensors and print a concise summary."""
        data = {}

        # wheel encoder positions
        wheel_positions = {}
        for k, s in self.wheel_sensors.items():
            wheel_positions[k] = s.getValue() if s else None
        data["wheel_positions"] = wheel_positions

        # GPS
        data["gps"] = self.gps.getValues() if self.gps else None

        # IMU parts
        data["accelerometer"] = (
            self.accelerometer.getValues() if self.accelerometer else None
        )
        data["gyro"] = self.gyro.getValues() if self.gyro else None
        data["compass"] = self.compass.getValues() if self.compass else None
        data["inertial_quaternion"] = self.get_inertial_quaternion()

        # Range sensors (short-range IR)
        ranges = {}
        for name, s in self.range_sensors.items():
            ranges[name] = s.getValue() if s else None
        data["range_sensors"] = ranges

        # Cameras
        rgb = self.get_rgb_image()
        data["rgb"] = {"width": rgb.shape[1], "height": rgb.shape[0]}

        depth = self.get_depth_image()
        data["depth"] = (
            {"width": depth.shape[1], "height": depth.shape[0]}
        )

        # Lidar
        lidar_ranges = self.get_lidar_ranges()
        data["lidar_ranges_length"] = (
            len(lidar_ranges) if lidar_ranges is not None else None
        )
        lidar_pc = self.get_lidar_point_cloud()
        data["lidar_pointcloud_length"] = (
            len(lidar_pc) if lidar_pc is not None else None
        )

        # Print a concise summary (useful for debugging / demo)
        print("=== Sensor snapshot ===")
        print("Wheels (pos):", data["wheel_positions"])
        print("GPS:", data["gps"])
        print("Accelerometer:", data["accelerometer"])
        print("Gyro:", data["gyro"])
        print("Compass:", data["compass"])
        print("Inertial (quat):", data["inertial_quaternion"])
        print("Range sensors:", data["range_sensors"])
        print(f"RGB camera: {data['rgb']['width']}x{data['rgb']['height']}")
        print(f"Depth camera: {data['depth']['width']}x{data['depth']['height']}")
        print("Lidar ranges length:", data["lidar_ranges_length"])
        print("Lidar point cloud length:", data["lidar_pointcloud_length"])
        print("========================")

        return data

    # ----------------------
    # Communication helpers
    # ----------------------
    def send_squad_message(self, payload: bytes):
        if self.squad_emitter:
            self.squad_emitter.send(payload)

    def poll_squad_messages(self):
        msgs = []
        if not self.squad_receiver:
            return msgs
        while self.squad_receiver.getQueueLength() > 0:
            msgs.append(self.squad_receiver.getData())
            self.squad_receiver.nextPacket()
        return msgs


if __name__ == "__main__":
    rosbot = Rosbot()
    step_count = 0

    # Example main loop: drive forward and print sensor snapshots every 10 steps
    while rosbot.robot.step(rosbot.timestep) != -1:
        # simple forward velocity (rad/s)
        rosbot.set_wheel_speeds(2.0, 2.0, 2.0, 2.0)

        # poll messages quickly
        messages = rosbot.poll_squad_messages()
        for m in messages:
            print("Received squad message:", m)

        # read sensors periodically (reduce spam)
        if step_count % 10 == 0:
            rosbot.read_sensors()

        # optionally broadcast (example)
        if step_count % 100 == 0 and rosbot.squad_emitter:
            rosbot.send_squad_message(b"hello_from_rosbot")

        # Save example images once
        if step_count == 0:
            rosbot.save_rgb_frame(step_count)
            rosbot.save_depth_frame(step_count)
        
        step_count += 1
