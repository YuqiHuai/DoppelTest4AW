import logging
import math
import time
import uuid
from logging import Logger
from typing import List, Optional, Sequence, Tuple, Union

import requests

from DoppelAutoware_previousoneDoppelAutoware_previous.framework.scenario.PedestrianManager import PedestrianManager
from DoppelAutoware_previousoneDoppelAutoware_previous.framework.scenario.TrafficControlManager import (
    TrafficControlManager,
)
from DoppelAutoware_previousoneDoppelAutoware_previous.framework.scenario.ad_agents import ADAgent
from DoppelAutoware_previousoneDoppelAutoware_previous.framework.scenario.tc_config import SCENARIO_UPPER_LIMIT


class AutoModeUnavailableError(RuntimeError):
    pass


class VehicleEndpoint:
    """
    Thin helper that wraps a single Autoware container's HTTP API.
    """

    def __init__(
        self,
        name: str,
        base_url: str,
        timeout: float = 5.0,
        analysis_timeout: float = 60.0,
    ):
        self.name = name
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.analysis_timeout = analysis_timeout
        self.session = requests.Session()

    def close(self):
        """Close the HTTP session to release resources."""
        if self.session:
            self.session.close()

    def _post(
        self,
        path: str,
        json_body: Optional[dict] = None,
        timeout: Optional[float] = None,
    ):
        url = f"{self.base_url}{path}"
        resp = self.session.post(url, json=json_body, timeout=timeout or self.timeout)
        try:
            resp.raise_for_status()
        except requests.HTTPError as exc:
            body = resp.text
            raise RuntimeError(
                f"{self.name} POST {path} failed ({resp.status_code}): {body}"
            ) from exc
        if resp.content:
            return resp.json()
        return {}

    def reset(self):
        try:
            self._post("/logging/stop")
            time.sleep(3.0)
        except Exception as exc:
            if "logging/stop" in str(exc) and "404" in str(exc):
                pass
            else:
                print(f"[{self.name}] reset logging/stop failed: {exc}")
        try:
            self._post("/change_operation_stop_mode")
            time.sleep(3.0)
        except Exception as exc:
            print(f"[{self.name}] reset stop mode failed: {exc}")
        try:
            self._post("/clear_routes")
            time.sleep(3.0)
        except Exception as exc:
            print(f"[{self.name}] reset clear routes failed: {exc}")

    def initialize_localization(self, pose_payload: dict):
        payload = {"pose": pose_payload}
        return self._post("/initialize_localization", payload)

    def set_route(self, start_pose: dict, goal_pose: dict):
        current_time = time.time()
        payload = {
            "header": {
                "stamp": [
                    int(current_time),
                    int((current_time - int(current_time)) * 1e9),
                ],
                "frame_id": "map",
            },
            "goal": goal_pose,
            "waypoints": [start_pose, goal_pose],
        }
        return self._post("/set_route_points", payload)

    def start_auto_mode(self):
        return self._post("/change_operation_auto_mode")

    def stop(self):
        return self._post("/change_operation_stop_mode")

    def restart_autoware(self):
        return self._post("/autoware/restart")

    def start_logging(self, filename: str):
        return self._post("/logging/start", {"filename": filename})

    def stop_logging(self):
        return self._post("/logging/stop")

    def start_autoware(self):
        return self._post("/autoware/start")

    def start_sender(self):
        return self._post("/sender/start")

    def restart_sender(self):
        return self._post("/sender/restart")

    def publish_pedestrian(self, payload: dict):
        return self._post("/publish_pedestrain", payload)

    def publish_pedestrians(self, payload: dict):
        return self._post("/publish_pedestrians", payload)

    def publish_traffic_signal(self, payload: dict):
        return self._post("/traffic_signal", payload)

    def publish_traffic_signals(self, payload: dict):
        return self._post("/traffic_signals", payload)

    def fetch_violations(
        self, route_lanelet_ids: Optional[Sequence[int]] = None
    ) -> dict:
        payload = None
        if route_lanelet_ids:
            payload = {"route_lanelet_ids": [int(v) for v in route_lanelet_ids]}
        return self._post(
            "/violations/calculate", json_body=payload, timeout=self.analysis_timeout
        )

    def fetch_decisions(self) -> dict:
        return self._post("/decisions/calculate", timeout=self.analysis_timeout)


def _pose_payload(pose) -> dict:
    orientation = pose.orientation
    return {
        "position": [float(pose.x), float(pose.y), float(pose.z)],
        "orientation": [
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ],
    }


def _build_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.propagate = False
    if logger.handlers:
        return logger
    logger.setLevel(logging.INFO)
    handler = logging.StreamHandler()
    handler.setLevel(logging.INFO)
    formatter = logging.Formatter("%(asctime)s %(name)s %(levelname)s: %(message)s")
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger


def _get_scenario_logger() -> logging.Logger:
    return _build_logger("Scenario")


class ScenarioRunner:
    """
    Executes a scenario by coordinating Autoware endpoints.
    """

    logger: Logger
    vehicles: List[VehicleEndpoint]
    curr_scenario: Optional[object]
    pm: Optional[PedestrianManager]
    tm: Optional[TrafficControlManager]
    is_initialized: bool
    __instance = None

    def __init__(self, endpoints: Sequence[Union[VehicleEndpoint, str]]) -> None:
        self.logger = _build_logger("ScenarioRunner")
        self.vehicles = []
        for idx, endpoint in enumerate(endpoints):
            if isinstance(endpoint, VehicleEndpoint):
                self.vehicles.append(endpoint)
            elif isinstance(endpoint, str):
                self.vehicles.append(VehicleEndpoint(f"vehicle_{idx}", endpoint))
            else:
                raise TypeError(
                    "ScenarioRunner expects VehicleEndpoint instances or base URL strings."
                )
        if not self.vehicles:
            raise ValueError("ScenarioRunner requires at least one VehicleEndpoint")
        self.curr_scenario = None
        self.is_initialized = False
        ScenarioRunner.__instance = self
        self._pedestrian_ids: List[str] = []
        self._restart_wait_s = 120.0
        self._autoware_started = False
        self._sender_started = False

    def configure_recovery(
        self, restart_wait_s: float = 120.0
    ) -> None:
        self._restart_wait_s = float(restart_wait_s)

    def _is_autonomous_mode_unavailable(self, exc: Exception) -> bool:
        message = str(exc)
        return "Failed to change to AUTONOMOUS mode" in message and "not available" in message

    def _is_autoware_already_running(self, exc: Exception) -> bool:
        message = str(exc)
        return "409" in message and "already running" in message

    def _is_sender_already_running(self, exc: Exception) -> bool:
        message = str(exc)
        return "409" in message and "Sender" in message and "already running" in message

    def _start_auto_mode_with_recovery(
        self, 
        vehicle: VehicleEndpoint, 
        vehicle_idx: int,
        active_runs: List[Tuple[VehicleEndpoint, ADAgent]],
        start_flags: List[bool]
    ) -> Tuple[bool, bool]:
        """
        Start auto mode with recovery.
        
        Args:
            vehicle: The vehicle endpoint to start
            vehicle_idx: Index of the vehicle in active_runs
            active_runs: List of all active vehicle runs
            start_flags: List of flags indicating which vehicles have already started
            
        Returns:
            Tuple of (restart_needed, other_vehicles_were_stopped)
            - restart_needed: True if restart was needed, False otherwise
            - other_vehicles_were_stopped: True if other vehicles were stopped for sync
        """
        try:
            vehicle.start_auto_mode()
            return (False, False)  # No restart needed
        except Exception as exc:
            if not self._is_autonomous_mode_unavailable(exc):
                raise
            
            # Check if other vehicles have already started
            other_vehicles_started = any(
                start_flags[i] for i in range(len(start_flags)) if i != vehicle_idx
            )
            
            if other_vehicles_started:
                self.logger.warning(
                    "[%s] auto mode unavailable; other vehicles already started. "
                    "Stopping all vehicles for synchronized restart.",
                    vehicle.name,
                )
                # Stop all vehicles that have started
                for idx, (v, _adc) in enumerate(active_runs):
                    if start_flags[idx]:
                        try:
                            self.logger.info("[%s] Stopping vehicle for synchronized restart.", v.name)
                            v.stop()
                        except Exception as stop_exc:
                            self.logger.warning("[%s] stop failed: %s", v.name, stop_exc)
                # Reset start flags for all vehicles
                for idx in range(len(start_flags)):
                    start_flags[idx] = False
                self.logger.info("All vehicles stopped. Proceeding with restart and reconfiguration.")
            else:
                self.logger.warning(
                    "[%s] auto mode unavailable; restarting Autoware (no other vehicles started yet).",
                    vehicle.name,
                )
            
            # Restart the failed vehicle's Autoware
            try:
                vehicle.restart_autoware()
            except Exception as restart_exc:
                self.logger.warning("[%s] restart_autoware failed: %s", vehicle.name, restart_exc)
            
            self._autoware_started = True
            self.logger.info(
                "[%s] restart requested; sleeping %ss before restarting senders.",
                vehicle.name,
                self._restart_wait_s,
            )
            time.sleep(self._restart_wait_s)
            self.logger.info("[%s] restart wait complete.", vehicle.name)
            
            # Restart sender processes for all vehicles
            self.logger.info("Restarting sender processes after Autoware restart.")
            for v in self.vehicles:
                try:
                    v.restart_sender()
                except Exception as sender_exc:
                    if self._is_sender_already_running(sender_exc):
                        continue
                    self.logger.warning("[%s] sender restart failed: %s", v.name, sender_exc)
            self.logger.info("Sender restart requested; waiting 10s for startup.")
            time.sleep(10.0)
            self._sender_started = True
            
            # If other vehicles were stopped, we need to reconfigure them
            if other_vehicles_started:
                self.logger.info("Reconfiguring all vehicles after restart...")
                for idx, (v, adc) in enumerate(active_runs):
                    self.logger.info("[%s] Reconfiguring vehicle after restart.", v.name)
                    self._configure_vehicle(v, adc)
                self.logger.info("All vehicles reconfigured. Ready to restart auto mode.")
            
            # Retry auto mode after recovery
            self.logger.info("[%s] Retrying auto mode after recovery.", vehicle.name)
            vehicle.start_auto_mode()
            return (True, other_vehicles_started)  # Restart was needed

    @staticmethod
    def get_instance() -> "ScenarioRunner":
        return ScenarioRunner.__instance

    def set_scenario(self, s: object):
        self.curr_scenario = s
        self.is_initialized = False

    def init_scenario(self):
        if self.curr_scenario is None:
            raise ValueError("Scenario is not set.")
        
        # Validate that no vehicles are initialized too close together
        ad_section = self.curr_scenario.ad_section
        if hasattr(ad_section, 'adcs') and len(ad_section.adcs) > 1:
            from DoppelAutoware_previousoneDoppelAutoware_previous.framework.scenario.ad_agents import (
                MIN_VEHICLE_SPACING,
                generate_vehicle_polygon,
            )
            for i, adc1 in enumerate(ad_section.adcs):
                for j, adc2 in enumerate(ad_section.adcs[i+1:], start=i+1):
                    try:
                        poly1 = generate_vehicle_polygon(adc1.start_pose)
                        poly2 = generate_vehicle_polygon(adc2.start_pose)
                        distance = poly1.distance(poly2)
                        if distance < MIN_VEHICLE_SPACING:
                            self.logger.warning(
                                f"Vehicles {i} and {j} are too close at initialization: "
                                f"distance={distance:.2f}m < {MIN_VEHICLE_SPACING}m. "
                                f"Vehicle {i} at ({adc1.start_pose.x:.2f}, {adc1.start_pose.y:.2f}), "
                                f"Vehicle {j} at ({adc2.start_pose.x:.2f}, {adc2.start_pose.y:.2f})"
                            )
                            # Raise an error to prevent running invalid scenarios
                            raise ValueError(
                                f"Scenario initialization failed: Vehicles {i} and {j} are too close "
                                f"(distance={distance:.2f}m < {MIN_VEHICLE_SPACING}m). "
                                "This would result in an immediate collision."
                            )
                    except Exception as e:
                        # If polygon generation fails, log warning but continue
                        self.logger.warning(
                            f"Could not validate distance between vehicles {i} and {j}: {e}"
                        )
        
        map_path = getattr(self.curr_scenario, "map_path", None)
        if map_path:
            self.pm = PedestrianManager(
                self.curr_scenario.pd_section, map_path=map_path
            )
        else:
            self.pm = PedestrianManager(self.curr_scenario.pd_section)
        self.tm = TrafficControlManager(self.curr_scenario.tc_section)
        self._pedestrian_ids = [
            str(uuid.uuid5(uuid.NAMESPACE_URL, f"pd-{idx}-{pd.cw_id}"))
            for idx, pd in enumerate(self.curr_scenario.pd_section.pds)
        ]
        self.is_initialized = True

    def _publish_traffic_signals(self, curr_time: float):
        config = self.tm.get_traffic_configuration(curr_time)
        signals_payload = []
        for signal in config.get("signals", []):
            color = str(signal["color"]).upper()
            if color == "GREEN":
                out_color = 3
            elif color == "YELLOW":
                out_color = 2
            else:
                out_color = 1
            signals_payload.append(
                {
                    "map_primitive_id": int(signal["id"]),
                    "color": out_color,
                    "shape": 1,
                    "status": 1,
                    "confidence": 1.0,
                }
            )
        if not signals_payload:
            return
        payload = {"signals": signals_payload}
        for vehicle in self.vehicles:
            try:
                vehicle.publish_traffic_signals(payload)
            except Exception:
                for signal_payload in signals_payload:
                    vehicle.publish_traffic_signal(signal_payload)

    def _publish_pedestrians(self, curr_time: float):
        pedestrians = self.pm.get_pedestrians(curr_time)
        items = []
        for idx, obs in enumerate(pedestrians):
            position = obs.get("position", {})
            orientation = obs.get("orientation", {})
            speed = float(obs.get("speed", 0.0))
            if not orientation:
                heading = float(obs.get("heading", 0.0))
                orientation = {
                    "x": 0.0,
                    "y": 0.0,
                    "z": math.sin(heading * 0.5),
                    "w": math.cos(heading * 0.5),
                }
            items.append(
                {
                    "pedestrian_id": self._pedestrian_ids[idx]
                    if idx < len(self._pedestrian_ids)
                    else str(uuid.uuid4()),
                    "position": [
                        float(position.get("x", 0.0)),
                        float(position.get("y", 0.0)),
                        float(position.get("z", 0.0)),
                    ],
                    "orientation": [
                        float(orientation.get("x", 0.0)),
                        float(orientation.get("y", 0.0)),
                        float(orientation.get("z", 0.0)),
                        float(orientation.get("w", 1.0)),
                    ],
                    "speed": speed,
                }
            )
        if not items:
            return
        payload = {"pedestrians": items}
        for vehicle in self.vehicles:
            try:
                vehicle.publish_pedestrians(payload)
            except Exception as exc:
                self.logger.warning("[%s] publish_pedestrians failed: %s", vehicle.name, exc)

    def _configure_vehicle(self, vehicle: VehicleEndpoint, adc: ADAgent):
        start_pose = _pose_payload(adc.start_pose)
        goal_pose = _pose_payload(adc.goal_pose)
        self.logger.info(
            "[%s] configure start (reset/localization/route)", vehicle.name
        )
        vehicle.reset()
        self.logger.info("[%s] reset done", vehicle.name)
        vehicle.initialize_localization(start_pose)
        self.logger.info("[%s] initialize_localization done", vehicle.name)
        time.sleep(10.0)
        vehicle.set_route(start_pose, goal_pose)
        self.logger.info("[%s] set_route done", vehicle.name)
        time.sleep(10.0)

    def run_scenario(
        self, generation_name: str, scenario_name: str, save_record: bool = False
    ) -> List[Tuple[VehicleEndpoint, ADAgent]]:
        if self.curr_scenario is None or not self.is_initialized:
            print("Error: No scenario or not initialized")
            return []

        adcs = self.curr_scenario.ad_section.adcs
        if len(adcs) > len(self.vehicles):
            raise ValueError(
                f"Scenario requires {len(adcs)} vehicles but only {len(self.vehicles)} endpoints provided."
            )

        scenario_logger = _get_scenario_logger()
        scenario_logger.info(
            f"Scenario start: {generation_name} {scenario_name} "
            f"(limit={SCENARIO_UPPER_LIMIT}s)"
        )
        if not self._autoware_started:
            scenario_logger.info("Starting Autoware for all vehicles...")
            for vehicle in self.vehicles:
                try:
                    vehicle.start_autoware()
                except Exception as exc:
                    if self._is_autoware_already_running(exc):
                        scenario_logger.info(
                            "[%s] Autoware already running; continuing.",
                            vehicle.name,
                        )
                        continue
                    raise
            scenario_logger.info(
                "Autoware start requested; waiting %ss for startup.",
                self._restart_wait_s,
            )
            time.sleep(self._restart_wait_s)
            self._autoware_started = True
        if not self._sender_started:
            scenario_logger.info("Starting sender for all vehicles...")
            for vehicle in self.vehicles:
                try:
                    vehicle.start_sender()
                except Exception as exc:
                    if self._is_sender_already_running(exc):
                        scenario_logger.info("[%s] Sender already running; continuing.", vehicle.name)
                        continue
                    raise
            scenario_logger.info("Sender start requested; waiting 10s for startup.")
            time.sleep(10.0)
            self._sender_started = True
        active_runs: List[Tuple[VehicleEndpoint, ADAgent]] = []
        for idx, adc in enumerate(adcs):
            vehicle = self.vehicles[idx]
            self._configure_vehicle(vehicle, adc)
            if save_record:
                log_name = f"{generation_name}_{scenario_name}_{vehicle.name}_{int(time.time())}"
                vehicle.start_logging(log_name)
            active_runs.append((vehicle, adc))

        start_flags = [False] * len(active_runs)
        scenario_start = time.time()
        runner_time = 0.0
        next_log_time = 0.0

        while True:
            elapsed = time.time() - scenario_start
            runner_time = elapsed
            for idx, (vehicle, adc) in enumerate(active_runs):
                if not start_flags[idx] and elapsed >= adc.start_t:
                    # Record scenario time BEFORE restart (in case restart pauses scenario time)
                    scenario_time_before_restart = runner_time
                    restart_needed, other_vehicles_stopped = self._start_auto_mode_with_recovery(
                        vehicle, idx, active_runs, start_flags
                    )
                    if restart_needed:
                        if other_vehicles_stopped:
                            # If other vehicles were stopped, we paused scenario time during restart
                            # Adjust scenario_start so that elapsed time continues from where it paused
                            # This ensures the scenario still runs for exactly SCENARIO_UPPER_LIMIT seconds
                            # The restart took real time, but scenario time should continue from where it paused
                            scenario_start = time.time() - scenario_time_before_restart
                            scenario_logger.info(
                                "Scenario time paused during restart. Resuming from t=%.1fs "
                                "(restart took real time but scenario time paused)",
                                scenario_time_before_restart,
                            )
                            # Restart other vehicles that should have already started
                            # Note: All vehicles were reconfigured in _start_auto_mode_with_recovery
                            for other_idx, (other_vehicle, other_adc) in enumerate(active_runs):
                                if other_idx != idx and not start_flags[other_idx]:
                                    # Check if this vehicle's start time has passed
                                    if runner_time >= other_adc.start_t:
                                        scenario_logger.info(
                                            "[%s] Restarting auto mode after synchronized restart "
                                            "(start_t=%.1fs, scenario_time=%.1fs).",
                                            other_vehicle.name,
                                            other_adc.start_t,
                                            runner_time,
                                        )
                                        try:
                                            other_vehicle.start_auto_mode()
                                            start_flags[other_idx] = True
                                            scenario_logger.info(
                                                "[%s] Successfully restarted auto mode after recovery.",
                                                other_vehicle.name,
                                            )
                                        except Exception as restart_exc:
                                            scenario_logger.error(
                                                "[%s] Failed to restart auto mode after recovery: %s",
                                                other_vehicle.name,
                                                restart_exc,
                                            )
                                            # Don't set start_flags[other_idx] = True, so it will retry
                                            # But this could cause infinite loop, so we might want to handle this differently
                        # Mark the current vehicle as started (it was restarted in _start_auto_mode_with_recovery)
                        start_flags[idx] = True
                        scenario_logger.info(
                            "[%s] Marked as started after recovery.",
                            vehicle.name,
                        )
                    else:
                        start_flags[idx] = True

            self._publish_traffic_signals(runner_time)
            self._publish_pedestrians(runner_time)

            if runner_time >= next_log_time:
                scenario_logger.info(f"Scenario time: {round(next_log_time, 1)}.")
                next_log_time += 1.0

            if elapsed >= SCENARIO_UPPER_LIMIT:
                scenario_logger.info("\n")
                break

            time.sleep(0.01)

        for vehicle, _adc in active_runs:
            vehicle.stop()
            if save_record:
                vehicle.stop_logging()

        scenario_logger.info(
            f"Scenario end: {generation_name} {scenario_name} "
            f"elapsed={round(runner_time, 2)}s"
        )
        self.logger.debug(f"Scenario ended. Length: {round(runner_time, 2)} seconds.")
        self.is_initialized = False
        return active_runs


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description="Generate a sample scenario configuration and print it."
    )
    parser.add_argument(
        "--map",
        default=None,
        help="Lanelet2 OSM map path (optional).",
    )
    args = parser.parse_args()

    map_path = args.map
    from DoppelAutoware_previousoneDoppelAutoware_previous.framework.scenario.ad_agents import ADSection
    from DoppelAutoware_previousoneDoppelAutoware_previous.framework.scenario.pd_agents import PDSection
    from DoppelAutoware_previousoneDoppelAutoware_previous.framework.scenario.tc_config import TCSection

    try:
        if map_path:
            ad_section = ADSection.get_one(map_path=map_path)
            pd_section = PDSection.get_one(map_path=map_path)
            tc_section = TCSection.get_one(map_path=map_path)
        else:
            ad_section = ADSection.get_one()
            pd_section = PDSection.get_one()
            tc_section = TCSection.get_one()
    except Exception as exc:
        print(f"Failed to generate scenario sections: {exc}")
        return

    print("Scenario configuration:")
    print({"ad_section": [adc.to_dict() for adc in ad_section.adcs]})
    print({"pd_section": [pd.__dict__ for pd in pd_section.pds]})
    print(
        {
            "tc_section": {
                "initial": tc_section.initial,
                "final": tc_section.final,
                "duration_g": tc_section.duration_g,
                "duration_y": tc_section.duration_y,
                "duration_b": tc_section.duration_b,
            }
        }
    )


if __name__ == "__main__":
    main()
