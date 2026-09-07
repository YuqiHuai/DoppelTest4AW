import logging
import math
import os
import time
import uuid
from logging import Logger
from pathlib import Path
from typing import List, Optional, Sequence, Tuple, Union

import requests

from scenario_runner.framework.scenario.PedestrianManager import PedestrianManager
from scenario_runner.framework.scenario.TrafficControlManager import (
    TrafficControlManager,
)
from scenario_runner.framework.scenario.ad_agents import ADAgent
from scenario_runner.framework.scenario.tc_config import SCENARIO_UPPER_LIMIT


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
        timeout: float = 30.0,
        analysis_timeout: float = 120.0,
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

    def reset(self, stop_logging: bool = True):
        if stop_logging:
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

    def rebind(self, ip: str) -> None:
        """Point this endpoint at the same vehicle on a new address."""
        import re
        self.base_url = re.sub(r"//[^:/]+", f"//{ip}", self.base_url, count=1)

    def wait_healthy(self, timeout_s: float = 60.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                self.session.get(f"{self.base_url}/health", timeout=3)
                return True
            except Exception:
                time.sleep(2.0)
        return False

    def restart_autoware(self):
        return self._post("/autoware/restart")

    def stop_autoware(self):
        return self._post("/autoware/stop", timeout=60)

    def start_logging(self, filename: str, record_root: Optional[str] = None):
        payload = {"filename": filename}
        if record_root:
            payload["record_root"] = record_root
        return self._post("/logging/start", payload)

    def stop_logging(self):
        try:
            return self._post("/logging/stop")
        except Exception as exc:
            # Idempotent stop: logging might already be stopped during recovery/reset.
            msg = str(exc)
            if "logging/stop" in msg and "404" in msg:
                return {"status": "ok", "message": "logging already stopped"}
            raise

    def start_autoware(
        self,
        map_path: Optional[str] = None,
        vehicle_model: Optional[str] = None,
        sensor_model: Optional[str] = None,
    ):
        payload = {}
        if map_path:
            payload["map_path"] = map_path
        if vehicle_model:
            payload["vehicle_model"] = vehicle_model
        if sensor_model:
            payload["sensor_model"] = sensor_model
        if payload:
            return self._post("/autoware/start", json_body=payload)
        return self._post("/autoware/start")

    def start_sender(
        self,
        receiver_urls: Optional[Sequence[str]] = None,
    ):
        payload = {}
        if receiver_urls is not None:
            payload["receiver_urls"] = [str(u) for u in receiver_urls]
        return self._post("/sender/start", payload if payload else None)

    def restart_sender(
        self,
        receiver_urls: Optional[Sequence[str]] = None,
    ):
        payload = {}
        if receiver_urls is not None:
            payload["receiver_urls"] = [str(u) for u in receiver_urls]
        return self._post("/sender/restart", payload if payload else None)

    def sender_status(self):
        url = f"{self.base_url}/sender/status"
        resp = self.session.get(url, timeout=self.timeout)
        try:
            resp.raise_for_status()
        except requests.HTTPError as exc:
            body = resp.text
            raise RuntimeError(
                f"{self.name} GET /sender/status failed ({resp.status_code}): {body}"
            ) from exc
        if resp.content:
            return resp.json()
        return {}

    def stop_sender(self):
        try:
            return self._post("/sender/stop")
        except Exception as exc:
            msg = str(exc)
            if "/sender/stop" in msg and "404" in msg:
                return {"status": "ok", "message": "sender already stopped"}
            raise

    def autoware_status(self):
        url = f"{self.base_url}/autoware/status"
        resp = self.session.get(url, timeout=self.timeout)
        try:
            resp.raise_for_status()
        except requests.HTTPError as exc:
            body = resp.text
            raise RuntimeError(
                f"{self.name} GET /autoware/status failed ({resp.status_code}): {body}"
            ) from exc
        if resp.content:
            return resp.json()
        return {}

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


def _docker(method: str, path: str, timeout: float = 60.0):
    """Ask the docker daemon for something, over its unix socket.

    Not the docker CLI: this image does not carry one, and installing it into
    every generator container to issue two requests is a worse dependency than
    a few lines of http.client.
    """
    import http.client
    import json as _json
    import socket as _socket

    class _UnixHTTP(http.client.HTTPConnection):
        def __init__(self, sock_path: str, conn_timeout: float):
            super().__init__("localhost", timeout=conn_timeout)
            self._sock_path = sock_path

        def connect(self):
            sock = _socket.socket(_socket.AF_UNIX, _socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            sock.connect(self._sock_path)
            self.sock = sock

    conn = _UnixHTTP("/var/run/docker.sock", timeout)
    try:
        conn.request(method, path)
        resp = conn.getresponse()
        body = resp.read()
        if resp.status >= 400:
            raise RuntimeError(f"docker {method} {path}: {resp.status} {body[:200]!r}")
        return _json.loads(body) if body.strip() else None
    finally:
        conn.close()


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
        self._restart_wait_s = 60.0
        self._max_recovery_retries = 3
        # Where this campaign's bags go, as a repo-relative path the containers
        # can resolve. None keeps each receiver's own container_<n>/log tree.
        self._record_root: Optional[str] = None
        # Per-scenario coverage: the gcov build directory to harvest .gcda from
        # after each scenario, or None to leave coverage alone.
        self._coverage_build_dir: Optional[str] = None
        # How long to let a stack reach the state the next step needs. A
        # ceiling, not a delay: the wait ends when the state arrives.
        self._state_wait_s = 60.0
        # Container name per vehicle, in the order the endpoints were given.
        # Set when the caller wants each vehicle restarted between scenarios.
        self._vehicle_containers: List[str] = []
        self._autoware_started = False
        self._sender_started = False
        self._repo_root = Path(__file__).resolve().parents[3]

    def _resolve_autoware_map_path(self) -> Optional[str]:
        map_path = getattr(self.curr_scenario, "map_path", None)
        if not map_path:
            return None
        path = Path(map_path)
        if not path.is_absolute():
            path = (Path.cwd() / path).resolve()
        if path.is_file():
            map_dir = path.parent
        else:
            map_dir = path
        map_name = map_dir.name
        candidate = self._repo_root / "autoware_map" / map_name
        if candidate.exists():
            # Return a repo-relative path so the container can resolve it.
            return str(Path("autoware_map") / map_name)
        return str(map_dir)

    def set_record_root(self, record_root: Optional[str]) -> None:
        """Record every vehicle's bag under one run directory.

        A scenario's evidence is one bag per vehicle, and without this each one
        lands in the container that recorded it -- so reading a scenario back
        means visiting N directories and joining them by filename.
        """
        self._record_root = str(record_root) if record_root else None

    def _wait_until_ready(self, vehicles, timeout_s: float, reason: str) -> bool:
        """Wait for the stacks to say they are up, instead of sleeping.

        `ros2 launch` returns immediately and its process lives for the whole
        run, so "the launch is alive" is not readiness -- which is why this
        used to be a fixed sleep long enough to cover the worst case. The
        receiver now reports whether the ADAPI services this is about to call
        exist and whether /autoware/state is arriving, so the wait can end when
        the stack is actually ready and the fixed time becomes the timeout.

        Returns False if any vehicle never became ready; the caller carries on,
        because the scenario's own recovery path is better at judging that than
        a timeout is.
        """
        pending = list(vehicles)
        started = time.time()
        legacy = False
        while pending and (time.time() - started) < timeout_s:
            for vehicle in list(pending):
                try:
                    status = vehicle.autoware_status()
                except Exception:
                    continue
                if "ready" not in status:
                    # A receiver from before this reported readiness at all.
                    legacy = True
                    pending = []
                    break
                if status.get("ready"):
                    pending.remove(vehicle)
                    self.logger.info(
                        "[%s] Autoware ready after %.1fs (%s).",
                        vehicle.name, time.time() - started, status.get("state"),
                    )
            if pending:
                time.sleep(2.0)

        if legacy:
            remaining = timeout_s - (time.time() - started)
            self.logger.info(
                "Receiver does not report readiness; falling back to waiting %.0fs.",
                max(remaining, 0.0),
            )
            if remaining > 0:
                time.sleep(remaining)
            return True
        if pending:
            self.logger.warning(
                "%s: %s not ready after %.0fs; continuing anyway.",
                reason, ", ".join(v.name for v in pending), timeout_s,
            )
            return False
        self.logger.info("%s: all vehicles ready in %.1fs.", reason, time.time() - started)
        return True

    def _wait_for_state(
        self, vehicle, states: Sequence[str], timeout_s: float, reason: str
    ) -> bool:
        """Wait for a vehicle to reach one of `states`, or give up after timeout.

        The states are the stack's own account of what it is doing. Sleeping a
        fixed time instead assumes the slowest case is the one you guessed:
        after set_route the planner sits in PLANNING until it has a trajectory,
        and asking for autonomous mode before then is refused as "not
        available" -- which the caller reads as a broken stack and answers with
        a restart, so a slow plan costs a whole scenario instead of a few
        seconds. Measured with five vehicles: the fifth needed longer than the
        10 s it was given, every time, and the scenario looped through all
        three recovery attempts on it.

        Returns False on timeout; the caller keeps its own recovery for that.
        """
        started = time.time()
        last = None
        while (time.time() - started) < timeout_s:
            try:
                status = vehicle.autoware_status()
            except Exception:
                time.sleep(1.0)
                continue
            if "state" not in status:      # receiver from before this existed
                time.sleep(max(timeout_s - (time.time() - started), 0.0))
                return True
            last = status.get("state")
            if last in states:
                self.logger.info(
                    "[%s] %s after %.1fs (%s).", vehicle.name, reason,
                    time.time() - started, last,
                )
                return True
            time.sleep(1.0)
        self.logger.warning(
            "[%s] %s: still %s after %.0fs.", vehicle.name, reason, last, timeout_s
        )
        return False

    def set_vehicle_containers(self, names: Sequence[str]) -> None:
        """Restart each vehicle's container between scenarios.

        Stopping Autoware is enough for gcov -- measured: all 253 translation
        units write on exit and nothing writes afterwards -- but only while the
        stop succeeds. The receiver escalates to SIGKILL after 15 s, and a
        process killed that way writes no counters at all; one that outlives the
        archive writes them into the NEXT scenario's. Over the hundreds of
        scenarios in a campaign, a fresh container is the cheaper guarantee than
        being right about every teardown.

        The receiver is the container's own command, so a restart brings it back
        with nothing else alive. The IP can change across a restart, so the
        endpoint's URL is re-resolved from the container rather than assumed.
        """
        self._vehicle_containers = list(names)

    def _restart_vehicle_containers(self, vehicles, scenario_logger) -> None:
        if not self._vehicle_containers:
            return
        started = time.time()
        by_name = dict(zip(self._vehicle_containers, self.vehicles))
        for name in self._vehicle_containers:
            try:
                _docker("POST", f"/containers/{name}/restart?t=30", timeout=180)
            except Exception as exc:
                scenario_logger.warning("[%s] container restart failed: %s", name, exc)
                continue
            vehicle = by_name.get(name)
            if vehicle is None:
                continue
            # A restart can move the container's IP; the old URL would then
            # point at nothing, or -- worse -- at another vehicle.
            try:
                info = _docker("GET", f"/containers/{name}/json", timeout=30)
                nets = (info.get("NetworkSettings") or {}).get("Networks") or {}
                ip = next((n.get("IPAddress") for n in nets.values() if n.get("IPAddress")), "")
                if ip:
                    vehicle.rebind(ip)
            except Exception as exc:
                scenario_logger.warning("[%s] could not re-resolve its address: %s", name, exc)

        # The receiver comes up with the container; wait for it before the next
        # scenario asks it for anything.
        for name in self._vehicle_containers:
            vehicle = by_name.get(name)
            if vehicle is None:
                continue
            if not vehicle.wait_healthy(60.0):
                scenario_logger.warning(
                    "[%s] receiver did not answer /health after the restart", name
                )
        scenario_logger.info(
            "restarted %d vehicle containers in %.1fs",
            len(self._vehicle_containers), time.time() - started,
        )

    def set_coverage_build_dir(self, build_dir: Optional[str]) -> None:
        """Harvest gcov counters into each scenario's record directory.

        gcov writes a translation unit's .gcda when the process that owns it
        exits, and it MERGES into whatever .gcda is already there. A search
        that keeps one Autoware launch across every scenario therefore produces
        one union at the end and nothing per scenario -- so attributing
        coverage to the test case that caused it means ending the stack after
        each one, taking the counters, and clearing them before the next.

        That costs a relaunch per scenario (about a minute). Only the harvest
        happens here; turning the counters into a report is slow (minutes of
        gcovr) and belongs offline, which is why the .gcda are archived rather
        than analysed.
        """
        self._coverage_build_dir = build_dir or None

    def _harvest_coverage(
        self, active_runs, scenario_dir: str, scenario_logger
    ) -> None:
        build = self._coverage_build_dir
        if not build or not self._record_root or not active_runs:
            return
        import glob
        import subprocess

        if not os.path.isdir(build):
            scenario_logger.warning("coverage: no build directory at %s", build)
            return

        # Every vehicle shares this build tree, so all of them have to be down
        # before the counters are complete: one still driving keeps writing.
        for vehicle, _adc in active_runs:
            try:
                vehicle.stop_autoware()
            except Exception as exc:
                scenario_logger.warning(
                    "[%s] coverage: stopping Autoware failed: %s", vehicle.name, exc
                )

        # The launch parent exits before its component containers do, so wait
        # for the file count to stop moving rather than for a fixed time.
        pattern = os.path.join(build, "**", "*.gcda")
        prev = -1
        for _ in range(30):
            count = len(glob.glob(pattern, recursive=True))
            if count and count == prev:
                break
            prev = count
            time.sleep(2.0)
        if not prev:
            scenario_logger.warning(
                "coverage: no .gcda after stopping the stack -- is this the coverage build?"
            )
            return

        # The .gcno are the other half of a coverage report: gcov needs the
        # structure the compiler emitted as well as the counts. They live only
        # in the build tree, which `coverage build` wipes, and they carry the
        # compiler's version stamp, so counters outlive their .gcno and become
        # unreadable. Snapshot them ONCE per run -- they are identical for
        # every scenario in it, 388 MB raw and 36 MB compressed.
        run_dir = os.path.dirname(self._record_root.rstrip("/"))
        gcno_archive = os.path.abspath(
            os.path.join(run_dir, "coverage", "gcno.tar.gz")
        )
        if not os.path.exists(gcno_archive):
            os.makedirs(os.path.dirname(gcno_archive), exist_ok=True)
            try:
                names = subprocess.run(
                    ["find", ".", "-name", "*.gcno", "-print0"],
                    cwd=build, check=True, stdout=subprocess.PIPE,
                )
                subprocess.run(
                    ["tar", "-czf", gcno_archive, "-C", build, "--null", "-T", "-"],
                    input=names.stdout, check=True,
                )
                scenario_logger.info(
                    "coverage: build structure -> %s (%.1f MB, once per run)",
                    gcno_archive, os.path.getsize(gcno_archive) / 1048576,
                )
            except Exception as exc:
                scenario_logger.warning("coverage: archiving .gcno failed: %s", exc)

        out_dir = os.path.join(self._record_root, scenario_dir)
        os.makedirs(out_dir, exist_ok=True)
        # Absolute: tar changes directory into the build tree below, and a
        # relative archive path would then be written inside it.
        archive = os.path.abspath(os.path.join(out_dir, "coverage.tar.gz"))
        # Paths relative to the build directory, so restoring is unambiguous:
        # gcov needs each .gcda beside the .gcno it was compiled with.
        try:
            find = subprocess.run(
                ["find", ".", "-name", "*.gcda", "-print0"],
                cwd=build, check=True, stdout=subprocess.PIPE,
            )
            # -C BEFORE -T: tar applies options in order, so a -C after the
            # file list leaves those names resolved against the wrong
            # directory and every one of them is "not found".
            subprocess.run(
                ["tar", "-czf", archive, "-C", build, "--null", "-T", "-"],
                input=find.stdout, check=True,
            )
            size_mb = os.path.getsize(archive) / 1048576
            scenario_logger.info(
                "coverage: %d translation units -> %s (%.1f MB)", prev, archive, size_mb
            )
        except Exception as exc:
            scenario_logger.warning("coverage: archiving failed: %s", exc)
            return

        # Clear, so the next scenario's counters are its own. Deleting is what
        # `lcov --zerocounters` does, without needing lcov in this container.
        for path in glob.glob(pattern, recursive=True):
            try:
                os.remove(path)
            except OSError:
                pass

    def configure_recovery(
        self, restart_wait_s: float = 60.0, max_recovery_retries: int = 3
    ) -> None:
        self._restart_wait_s = float(restart_wait_s)
        self._max_recovery_retries = max(0, int(max_recovery_retries))

    def _is_autonomous_mode_unavailable(self, exc: Exception) -> bool:
        message = str(exc)
        return "Failed to change to AUTONOMOUS mode" in message and "not available" in message

    def _is_autoware_already_running(self, exc: Exception) -> bool:
        message = str(exc)
        return "409" in message and "already running" in message

    def _is_sender_already_running(self, exc: Exception) -> bool:
        message = str(exc)
        return "409" in message and "Sender" in message and "already running" in message

    @staticmethod
    def _perception_url(base_url: str) -> str:
        return f"{base_url.rstrip('/')}/perception"

    def _sender_peer_urls(
        self, active_vehicles: Sequence[VehicleEndpoint], vehicle_idx: int
    ) -> List[str]:
        return [
            self._perception_url(v.base_url)
            for idx, v in enumerate(active_vehicles)
            if idx != vehicle_idx
        ]

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
            Tuple of (restart_needed, restart_scenario_from_t0)
            - restart_needed: True if recovery/restart was performed, False otherwise
            - restart_scenario_from_t0: True when scenario timing should restart from 0
        """
        try:
            vehicle.start_auto_mode()
            return (False, False)  # No restart needed
        except Exception as exc:
            if not self._is_autonomous_mode_unavailable(exc):
                raise

            self.logger.warning(
                "[%s] auto mode unavailable; restarting Autoware and rerunning scenario from t=0.",
                vehicle.name,
            )
            # Stop any vehicles that already entered AUTO before rerun.
            for idx, (v, _adc) in enumerate(active_runs):
                if not start_flags[idx]:
                    continue
                try:
                    self.logger.info("[%s] Stopping vehicle before rerun.", v.name)
                    v.stop()
                except Exception as stop_exc:
                    self.logger.warning("[%s] stop failed: %s", v.name, stop_exc)
            for idx in range(len(start_flags)):
                start_flags[idx] = False

            # Restart the failed vehicle's Autoware
            try:
                vehicle.restart_autoware()
            except Exception as restart_exc:
                self.logger.warning("[%s] restart_autoware failed: %s", vehicle.name, restart_exc)
            
            self._autoware_started = True
            self.logger.info(
                "[%s] restart requested; waiting up to %ss before restarting senders.",
                vehicle.name,
                self._restart_wait_s,
            )
            self._wait_until_ready([vehicle], self._restart_wait_s, "Autoware recovery")
            self.logger.info("[%s] restart wait complete.", vehicle.name)
            
            # Refresh sender peer wiring after recovery.
            active_vehicles = [v for v, _ in active_runs]
            self.logger.info("Restarting sender for all active vehicles after Autoware restart.")
            for idx, v in enumerate(active_vehicles):
                try:
                    v.restart_sender(
                        receiver_urls=self._sender_peer_urls(active_vehicles, idx),
                    )
                except Exception as sender_exc:
                    self.logger.warning("[%s] sender ensure failed: %s", v.name, sender_exc)
            self.logger.info("Sender restart requested; waiting 2s for startup.")
            time.sleep(2.0)
            self._sender_started = True

            # After Autoware restart, localization/routes are lost: reconfigure all vehicles.
            self.logger.info("Reconfiguring all vehicles after restart...")
            for idx, (v, adc) in enumerate(active_runs):
                self.logger.info("[%s] Reconfiguring vehicle after restart.", v.name)
                self._configure_vehicle(v, adc, stop_logging_on_reset=False)
            self.logger.info("All vehicles reconfigured.")

            return (True, True)

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
            from scenario_runner.framework.scenario.ad_agents import (
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

    def _publish_traffic_signals(
        self, curr_time: float, target_vehicles: Optional[Sequence[VehicleEndpoint]] = None
    ):
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
        vehicles = list(target_vehicles) if target_vehicles is not None else self.vehicles
        for vehicle in vehicles:
            try:
                vehicle.publish_traffic_signals(payload)
            except Exception:
                for signal_payload in signals_payload:
                    vehicle.publish_traffic_signal(signal_payload)

    def _publish_pedestrians(
        self, curr_time: float, target_vehicles: Optional[Sequence[VehicleEndpoint]] = None
    ):
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
        vehicles = list(target_vehicles) if target_vehicles is not None else self.vehicles
        for vehicle in vehicles:
            try:
                vehicle.publish_pedestrians(payload)
            except Exception as exc:
                self.logger.warning("[%s] publish_pedestrians failed: %s", vehicle.name, exc)

    def _configure_vehicle(
        self,
        vehicle: VehicleEndpoint,
        adc: ADAgent,
        stop_logging_on_reset: bool = True,
    ):
        start_pose = _pose_payload(adc.start_pose)
        goal_pose = _pose_payload(adc.goal_pose)
        self.logger.info(
            "[%s] configure start (reset/localization/route)", vehicle.name
        )
        vehicle.reset(stop_logging=stop_logging_on_reset)
        self.logger.info("[%s] reset done", vehicle.name)
        vehicle.initialize_localization(start_pose)
        self.logger.info("[%s] initialize_localization done", vehicle.name)
        # Localized when the stack leaves INITIALIZING; it cannot accept a
        # route before that.
        self._wait_for_state(
            vehicle, ("WAITING_FOR_ROUTE", "PLANNING", "WAITING_FOR_ENGAGE"),
            self._state_wait_s, "localized",
        )
        vehicle.set_route(start_pose, goal_pose)
        self.logger.info("[%s] set_route done", vehicle.name)
        # Engageable only once the planner has a trajectory. This is the wait
        # that used to be 10 s and cost whole scenarios when it was not enough.
        self._wait_for_state(
            vehicle, ("WAITING_FOR_ENGAGE",), self._state_wait_s, "route planned",
        )

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
        active_vehicles = self.vehicles[: len(adcs)]
        inactive_vehicles = self.vehicles[len(adcs):]

        scenario_logger = _get_scenario_logger()
        scenario_logger.info(
            f"Scenario start: {generation_name} {scenario_name} "
            f"(limit={SCENARIO_UPPER_LIMIT}s)"
        )
        scenario_logger.info("Active vehicles this scenario: %s", len(active_vehicles))
        # Mixed-size runs require per-scenario startup checks.
        scenario_logger.info("Ensuring Autoware is running for active vehicles...")
        autoware_map_path = self._resolve_autoware_map_path()
        started_any_autoware = False
        for vehicle in active_vehicles:
            try:
                vehicle.start_autoware(map_path=autoware_map_path)
                scenario_logger.info("[%s] Autoware start requested.", vehicle.name)
                started_any_autoware = True
            except Exception as exc:
                if self._is_autoware_already_running(exc):
                    scenario_logger.info(
                        "[%s] Autoware already running; continuing.",
                        vehicle.name,
                    )
                    continue
                raise
        if started_any_autoware:
            scenario_logger.info(
                "Autoware start requested; waiting up to %ss for it to come up.",
                self._restart_wait_s,
            )
            self._wait_until_ready(active_vehicles, self._restart_wait_s, "Autoware startup")
        else:
            # Already up, but a previous scenario may have left it mid-teardown.
            scenario_logger.info("Autoware already running on active vehicles.")
            self._wait_until_ready(active_vehicles, 10.0, "Autoware already running")
        self._autoware_started = True

        if inactive_vehicles:
            scenario_logger.info("Stopping sender on inactive vehicles...")
            for vehicle in inactive_vehicles:
                try:
                    vehicle.stop_sender()
                except Exception as exc:
                    scenario_logger.warning("[%s] sender stop failed: %s", vehicle.name, exc)

        scenario_logger.info("Refreshing sender peer URLs for active vehicles...")
        for idx, vehicle in enumerate(active_vehicles):
            try:
                vehicle.restart_sender(
                    receiver_urls=self._sender_peer_urls(active_vehicles, idx),
                )
                scenario_logger.info("[%s] sender restarted.", vehicle.name)
            except Exception as exc:
                scenario_logger.warning("[%s] sender restart failed: %s", vehicle.name, exc)
                raise
        scenario_logger.info("Sender restart requested; waiting 2s for startup.")
        time.sleep(2.0)
        self._sender_started = True
        active_runs: List[Tuple[VehicleEndpoint, ADAgent]] = []
        for idx, adc in enumerate(adcs):
            vehicle = active_vehicles[idx]
            self._configure_vehicle(vehicle, adc)
            active_runs.append((vehicle, adc))

        def _start_logging_for_active_runs() -> None:
            if not save_record or not active_runs:
                return
            log_ts = int(time.time())
            # One directory per scenario, one bag per vehicle inside it. The
            # timestamp stays on the bag rather than the scenario directory so a
            # recovery restart within a scenario adds a bag instead of a
            # directory nothing links to the scenario.
            scenario_dir = f"{generation_name}_{scenario_name}"
            root = f"{self._record_root}/{scenario_dir}" if self._record_root else None
            for vehicle, _adc in active_runs:
                log_name = (f"{vehicle.name}_{log_ts}" if root
                            else f"{generation_name}_{scenario_name}_{vehicle.name}_{log_ts}")
                vehicle.start_logging(log_name, record_root=root)

        _start_logging_for_active_runs()

        start_flags = [False] * len(active_runs)
        scenario_start = time.time()
        runner_time = 0.0
        next_log_time = 0.0
        recovery_attempts = 0
        fatal_error: Optional[Exception] = None

        while True:
            elapsed = time.time() - scenario_start
            runner_time = elapsed
            recovery_triggered = False
            for idx, (vehicle, adc) in enumerate(active_runs):
                if not start_flags[idx] and elapsed >= adc.start_t:
                    restart_needed, restart_from_t0 = self._start_auto_mode_with_recovery(
                        vehicle, idx, active_runs, start_flags
                    )
                    if restart_needed:
                        recovery_attempts += 1
                        scenario_logger.warning(
                            "Recovery attempt %s/%s for %s %s.",
                            recovery_attempts,
                            self._max_recovery_retries,
                            generation_name,
                            scenario_name,
                        )
                        if recovery_attempts >= self._max_recovery_retries:
                            fatal_error = AutoModeUnavailableError(
                                f"Exceeded max recovery retries ({self._max_recovery_retries}) "
                                f"for {generation_name} {scenario_name}."
                            )
                            scenario_logger.error(str(fatal_error))
                            break
                        if restart_from_t0:
                            if save_record and active_runs:
                                scenario_logger.warning(
                                    "Recovery triggered; discarding partial recording and starting a fresh recording."
                                )
                                for v, _ in active_runs:
                                    try:
                                        v.stop_logging()
                                    except Exception as exc:
                                        scenario_logger.warning(
                                            "[%s] stop_logging failed during recovery: %s", v.name, exc
                                        )
                                _start_logging_for_active_runs()
                            scenario_start = time.time()
                            runner_time = 0.0
                            next_log_time = 0.0
                            scenario_logger.warning(
                                "Recovery completed; restarting scenario timing from t=0."
                            )
                        recovery_triggered = True
                        break
                    else:
                        start_flags[idx] = True

            if fatal_error is not None:
                break
            if recovery_triggered:
                continue

            self._publish_traffic_signals(runner_time, target_vehicles=active_vehicles)
            self._publish_pedestrians(runner_time, target_vehicles=active_vehicles)

            if runner_time >= next_log_time:
                scenario_logger.info(f"Scenario time: {round(next_log_time, 1)}.")
                next_log_time += 1.0

            if elapsed >= SCENARIO_UPPER_LIMIT:
                scenario_logger.info("\n")
                break

            time.sleep(0.01)

        for vehicle, _adc in active_runs:
            try:
                vehicle.stop()
            except Exception as exc:
                scenario_logger.warning("[%s] stop failed during cleanup: %s", vehicle.name, exc)
            if save_record:
                try:
                    vehicle.stop_logging()
                except Exception as exc:
                    scenario_logger.warning(
                        "[%s] stop_logging failed during cleanup: %s", vehicle.name, exc
                    )

        self._harvest_coverage(
            active_runs, f"{generation_name}_{scenario_name}", scenario_logger
        )
        # AFTER the archive: the restart is what guarantees the next scenario
        # starts with no process of this one alive, and the archive must be
        # taken while the counters this scenario produced are still there.
        self._restart_vehicle_containers(active_vehicles, scenario_logger)

        scenario_logger.info(
            f"Scenario end: {generation_name} {scenario_name} "
            f"elapsed={round(runner_time, 2)}s"
        )
        self.logger.debug(f"Scenario ended. Length: {round(runner_time, 2)} seconds.")
        self.is_initialized = False
        if fatal_error is not None:
            raise fatal_error
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
    from scenario_runner.framework.scenario.ad_agents import ADSection
    from scenario_runner.framework.scenario.pd_agents import PDSection
    from scenario_runner.framework.scenario.tc_config import TCSection

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
