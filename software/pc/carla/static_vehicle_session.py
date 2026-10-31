"""Independent, stationary CARLA 0.9.16 vehicle feedback demonstration.

Owns only the vehicle and two cameras it creates. The existing ego vehicle,
world settings, weather, navigation and board perception are not modified.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path

import carla
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets


ROOT = Path(__file__).resolve().parent


def distance(a, b):
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)


class DemoSession:
    def __init__(self, host, port, output):
        self.output = output
        output.mkdir(parents=True, exist_ok=True)
        self.client = carla.Client(host, port)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.owned = []
        self.cameras = []
        self.vehicle = None
        self.anchor = None
        self.closed = False
        self.frames = {}
        self.lock = threading.Lock()
        self.turn = "off"
        self.brake = False
        self.wheel_deg = 0.0
        self.max_drift = 0.0
        self.corrected_drifts = 0
        self.first_snapshot = self.world.get_snapshot()
        self.last_snapshot = self.first_snapshot
        self.started = time.monotonic()
        self.expected_lights = 0
        self.physics_disabled = False
        self.start_versions = {
            "client": self.client.get_client_version(),
            "server": self.client.get_server_version(),
        }

    def start(self):
        if self.world.get_settings().no_rendering_mode:
            raise RuntimeError("CARLA当前关闭渲染；请启用正常渲染后再启动演示。")
        actors = list(self.world.get_actors().filter("vehicle.*"))
        existing = [a for a in actors if a.attributes.get("role_name", "").startswith("static_feedback_demo_")]
        if existing:
            raise RuntimeError("已有静止演示车辆，请先关闭原演示窗口，避免重复启动。")
        bp = self.world.get_blueprint_library().find("vehicle.tesla.model3")
        bp.set_attribute("role_name", "static_feedback_demo_20261024")
        if bp.has_attribute("color"):
            bp.set_attribute("color", "35,80,160")
        candidates = self.world.get_map().get_spawn_points()
        ego = next((a for a in actors if a.attributes.get("role_name") == "ego_vehicle"), None)
        if ego is not None:
            # Keep the demonstration near the current scene, with independent actors.
            loc = ego.get_location()
            candidates.sort(key=lambda t: distance(t.location, loc))
        for transform in candidates:
            if any(distance(transform.location, a.get_location()) < 10 for a in actors):
                continue
            self.vehicle = self.world.try_spawn_actor(bp, transform)
            if self.vehicle is not None:
                break
        if self.vehicle is None:
            raise RuntimeError("没有空闲的车辆出生点。")
        self.owned.append(self.vehicle)
        self.vehicle.set_autopilot(False)
        self.vehicle.apply_control(carla.VehicleControl(throttle=0, brake=1, hand_brake=True))
        # Let the new vehicle settle onto the road before locking its body.
        settle_until = time.monotonic() + 1.2
        while time.monotonic() < settle_until:
            self.world.wait_for_tick(3)
        self.anchor = self.vehicle.get_transform()
        self.vehicle.set_target_velocity(carla.Vector3D())
        self.vehicle.set_target_angular_velocity(carla.Vector3D())
        self.vehicle.set_simulate_physics(False)
        self.physics_disabled = True
        self.vehicle.set_transform(self.anchor)
        camera_bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        for key, value in {"image_size_x": "640", "image_size_y": "400", "fov": "58", "sensor_tick": "0.05"}.items():
            camera_bp.set_attribute(key, value)
        views = {
            "rear": carla.Transform(carla.Location(x=-5.6, y=-2.5, z=1.9), carla.Rotation(pitch=-10, yaw=24)),
            "front": carla.Transform(carla.Location(x=4.1, y=-2.8, z=1.15), carla.Rotation(pitch=-6, yaw=145)),
        }
        for name, transform in views.items():
            sensor = self.world.spawn_actor(camera_bp, transform, attach_to=self.vehicle)
            self.cameras.append(sensor)
            self.owned.append(sensor)
            sensor.listen(lambda image, name=name: self.on_image(name, image))
        self.set_feedback("off", False)
        self.event("started", vehicle_id=self.vehicle.id, versions=self.start_versions,
                   world=self.world.get_map().name, anchor=self.anchor_dict())

    def on_image(self, name, image):
        rgb = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)[:, :, 2::-1].copy()
        with self.lock:
            self.frames[name] = (rgb, image.frame, time.monotonic())

    def anchor_dict(self):
        return {"x": self.anchor.location.x, "y": self.anchor.location.y, "z": self.anchor.location.z,
                "yaw": self.anchor.rotation.yaw}

    def event(self, name, **details):
        record = {"event": name, "wall_time": datetime.now().astimezone().isoformat(), **details}
        with (self.output / "events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    def set_feedback(self, turn=None, brake=None):
        if turn is not None:
            if turn not in ("off", "left", "right", "hazard"):
                raise ValueError(turn)
            self.turn = turn
        if brake is not None:
            self.brake = bool(brake)
        self.wheel_deg = {"left": -28.0, "right": 28.0}.get(self.turn, 0.0)
        # Position lamps stay steady. Brake lamps are an independent bit, and
        # turn indicators are used only for an actual turn or explicit self-test.
        flags = int(carla.VehicleLightState.Position)
        if self.turn in ("left", "hazard"):
            flags |= int(carla.VehicleLightState.LeftBlinker)
        if self.turn in ("right", "hazard"):
            flags |= int(carla.VehicleLightState.RightBlinker)
        if self.brake:
            flags |= int(carla.VehicleLightState.Brake)
        self.expected_lights = flags
        self.apply_feedback()
        self.event("feedback", turn=self.turn, brake=self.brake,
                   visual_wheel_degrees=self.wheel_deg, light_flags=flags)

    def apply_feedback(self):
        # Physics is disabled. Brake/handbrake are also applied defensively;
        # brake *lamp* is explicitly controlled independently for this demo.
        self.vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1, hand_brake=True))
        visible=self.expected_lights
        if self.turn!='off' and int(time.monotonic()*2)%2==0:
            visible &= ~(int(carla.VehicleLightState.LeftBlinker)|int(carla.VehicleLightState.RightBlinker))
        self.visible_light_flags=visible
        self.vehicle.set_light_state(carla.VehicleLightState(visible))
        for wheel in (carla.VehicleWheelLocation.FL_Wheel, carla.VehicleWheelLocation.FR_Wheel):
            self.vehicle.set_wheel_steer_direction(wheel, self.wheel_deg)

    def status(self):
        snap = self.world.get_snapshot()
        tf = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()
        drift = distance(tf.location, self.anchor.location)
        self.max_drift = max(self.max_drift, drift)
        if drift > 0.02:
            self.corrected_drifts += 1
            self.vehicle.set_transform(self.anchor)
        self.last_snapshot = snap
        with self.lock:
            frames = {name: {"frame": item[1], "age_ms": round((time.monotonic()-item[2])*1000, 1)}
                      for name, item in self.frames.items()}
        ctrl = self.vehicle.get_control()
        return {
            "vehicle_id": self.vehicle.id, "turn": self.turn, "brake_lamp": self.brake,
            "visual_wheel_command_deg": self.wheel_deg,
            "drift_m": drift, "max_drift_m": self.max_drift,
            "lock_corrections": self.corrected_drifts,
            "speed_kmh": 3.6*math.sqrt(vel.x**2+vel.y**2+vel.z**2),
            "physics_disable_command_sent": self.physics_disabled,
            "expected_light_flags": self.expected_lights,
            "position_lamp": bool(self.expected_lights & int(carla.VehicleLightState.Position)),
            "actual_light_flags": int(self.vehicle.get_light_state()),
            "visible_light_flags": getattr(self,"visible_light_flags",self.expected_lights),
            "throttle": ctrl.throttle, "world_frame": snap.frame,
            "world_frames_advanced": snap.frame-self.first_snapshot.frame,
            "world_sim_seconds_advanced": snap.timestamp.elapsed_seconds-self.first_snapshot.timestamp.elapsed_seconds,
            "camera_frames": frames,
            "note": "Wheel value is the requested VISUAL angle; physical wheel readback is not a visual verification.",
        }

    def save_evidence(self, label):
        state = self.status()
        with self.lock:
            frames = {k: v[0].copy() for k, v in self.frames.items()}
        for name, rgb in frames.items():
            image = QtGui.QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
            image.save(str(self.output / (label + "_" + name + ".png")))
        (self.output / (label + ".json")).write_text(json.dumps(state, indent=2), encoding="utf-8")
        return state

    def close(self):
        if self.closed:
            return
        self.closed = True
        for sensor in self.cameras:
            try:
                sensor.stop()
            except RuntimeError:
                pass
        for actor in reversed(self.owned):
            try:
                if actor.is_alive:
                    actor.destroy()
            except RuntimeError:
                pass
        self.event("closed", destroyed_owned_ids=[a.id for a in self.owned])


