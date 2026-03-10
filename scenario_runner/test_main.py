from dataclasses import dataclass, field
import os
from pathlib import Path
import subprocess
import sys
from typing import List, Dict, Any, Tuple

from copy import deepcopy
from random import randint, random, sample, shuffle
import uuid

import numpy as np
from deap import algorithms, base, tools

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scenario_runner.framework.scenario.ScenarioRunner import (
    AutoModeUnavailableError,
    ScenarioRunner,
    VehicleEndpoint,
)
from scenario_runner.framework.scenario.ad_agents import (
    ADAgent,
    ADSection,
    DEFAULT_MAP,
    MAX_ADC_COUNT,
)
from scenario_runner.framework.scenario.pd_agents import PDSection, PDAgent, MAX_PD_COUNT
from scenario_runner.framework.scenario.tc_config import TCSection
from scenario_runner.hdmap.MapParser import MapParser


class ScenarioFitness(base.Fitness):
    # (min_distance, unique_decisions, conflict, unique_violation)
    # Minimize distance, maximize decisions, maximize conflict, maximize violations
    weights = (-1.0, 1.0, 1.0, 1.0)


@dataclass
class Scenario:
    ad_section: ADSection
    pd_section: PDSection
    tc_section: TCSection
    map_path: str
    gid: int = 0
    cid: int = 0
    fitness: base.Fitness = field(default_factory=ScenarioFitness)
    uid: str = field(default_factory=lambda: uuid.uuid4().hex)

    @staticmethod
    def get_one(count: int, map_path: str, gid: int, cid: int) -> "Scenario":
        return _build_random_scenario(count, map_path, gid, cid)

    @staticmethod
    def get_conflict_one(count: int, map_path: str, gid: int, cid: int) -> "Scenario":
        return _build_random_conflict_scenario(count, map_path, gid, cid)


def _build_ad_section(count: int, map_path: str) -> ADSection:
    section = ADSection([])
    mp = MapParser.get_instance(map_path)
    trial_count = 0
    max_trials = 2000 * count  # Allow many attempts to ensure we reach count
    allow_junction_start = False
    while len(section.adcs) < count:
        candidate = ADAgent.get_one(
            map_path=map_path, must_not_start_from_junction=not allow_junction_start
        )
        if mp.is_lane_in_intersection(candidate.goal_pose.lanelet_id):
            trial_count += 1
        else:
            if section.add_agent(candidate):
                # Successfully added
                trial_count = 0
            else:
                # Failed to add (too close), increment trial count
                trial_count += 1

        if trial_count >= max_trials:
            # Relax constraints to avoid returning fewer than count agents
            allow_junction_start = True
            trial_count = 0

    # Always adjust time (count should be reached here)
    if section.adcs:
        section.adjust_time()
    return section


def _count_ad_conflicts(section: ADSection, map_path: str) -> int:
    """Count the number of conflicting ADC pairs in the section.

    Matches the original Apollo GA's has_ad_conflict() which returns
    the count of unique conflict pairs, not a boolean.
    """
    mp = MapParser.get_instance(map_path)
    conflict = set()
    for ad in section.adcs:
        for bd in section.adcs:
            # Skip same route comparison
            if str(ad.route) == str(bd.route):
                continue
            if mp.is_conflict_lanes(ad.route, bd.route):
                # Use frozenset to avoid counting same pair twice
                conflict.add(frozenset([str(ad.route), str(bd.route)]))
    return len(conflict)


def _build_conflict_ad_section(count: int, map_path: str) -> ADSection:
    max_attempts = 200
    for _ in range(max_attempts):
        section = _build_ad_section(count, map_path)
        if _count_ad_conflicts(section, map_path) > 0:
            return section
    raise RuntimeError(
        f"Unable to generate conflict scenario after {max_attempts} attempts."
    )


def _count_decision_subtypes(value: Any) -> int:
    if isinstance(value, dict):
        return sum(1 for v in value.values() if bool(v))
    if isinstance(value, list):
        return len(value)


def _discover_docker_receiver_urls(max_needed: int) -> List[str]:
    prefix = os.environ.get("AUTOWARE_RECEIVER_CONTAINER_PREFIX", "autoware_").strip()
    if not prefix:
        prefix = "autoware_"
    port = os.environ.get("AUTOWARE_RECEIVER_PORT", "5002").strip() or "5002"

    urls: List[str] = []
    for i in range(1, max_needed + 1):
        container_name = f"{prefix}{i}"
        try:
            ip = subprocess.check_output(
                [
                    "docker",
                    "inspect",
                    "-f",
                    "{{if .State.Running}}{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}{{end}}",
                    container_name,
                ],
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
        except Exception:
            ip = ""
        if ip:
            urls.append(f"http://{ip}:{port}")
    return urls


def _extract_unique_decisions(decision_payload: Dict[str, Any]) -> int:
    decisions = decision_payload.get("decisions", {})
    if not isinstance(decisions, dict):
        return 0
    for key in ("unique_decisions", "unique_decision_count"):
        if key in decisions:
            return int(decisions[key])
    return sum(_count_decision_subtypes(value) for value in decisions.values())


def _compute_fitness(
    min_distance: float, unique_decisions: int, conflict: int, unique_violation: int
) -> Tuple[float, int, int, int]:
    """Compute fitness matching the original Apollo GA.

    Returns:
        (min_distance, unique_decisions, conflict_count, unique_violation)
    """
    return (min_distance, unique_decisions, conflict, unique_violation)


def _build_random_scenario(count: int, map_path: str, gid: int, cid: int) -> Scenario:
    ad_section = _build_ad_section(count, map_path)
    pd_section = PDSection.get_one(map_path=map_path)
    tc_section = TCSection.get_one(map_path=map_path)
    return Scenario(
        ad_section=ad_section,
        pd_section=pd_section,
        tc_section=tc_section,
        map_path=map_path,
        gid=gid,
        cid=cid,
    )


def _build_random_conflict_scenario(
    count: int, map_path: str, gid: int, cid: int
) -> Scenario:
    ad_section = _build_conflict_ad_section(count, map_path)
    pd_section = PDSection.get_one(map_path=map_path)
    tc_section = TCSection.get_one(map_path=map_path)
    return Scenario(
        ad_section=ad_section,
        pd_section=pd_section,
        tc_section=tc_section,
        map_path=map_path,
        gid=gid,
        cid=cid,
    )


def _mut_ad_section(
    section: ADSection, map_path: str, max_adc_count: int = MAX_ADC_COUNT
) -> ADSection:
    mut_pb = random()
    if mut_pb < 0.1 and len(section.adcs) > 2:
        shuffle(section.adcs)
        section.adcs.pop()
        section.adjust_time()
        return section

    trial = 0
    if mut_pb < 0.4 and len(section.adcs) < max_adc_count:
        while True:
            new_ad = ADAgent.get_one(
                map_path=map_path, must_not_start_from_junction=True
            )
            if section.has_conflict(new_ad, map_path=map_path) and section.add_agent(
                new_ad
            ):
                break
            trial += 1
            # Don't force add agents that are too close - respect distance constraints
            if trial >= 50:  # Limit trials to prevent infinite loops
                break
        section.adjust_time()
        return section

    index = randint(0, len(section.adcs) - 1)
    routing = section.adcs[index].route
    original_adc = section.adcs.pop(index)
    mut_counter = 0
    while True:
        if section.add_agent(ADAgent.get_one_for_route(routing, map_path=map_path)):
            break
        mut_counter += 1
        if mut_counter == 5:
            section.add_agent(original_adc)
            break
    section.adjust_time()
    return section


def _mut_pd_section(section: PDSection, map_path: str) -> PDSection:
    if len(section.pds) == 0:
        section.add_agent(PDAgent.get_one(map_path=map_path))
        return section

    mut_pb = random()
    if mut_pb < 0.2 and len(section.pds) > 0:
        shuffle(section.pds)
        section.pds.pop()
        return section

    if mut_pb < 0.4 and len(section.pds) <= MAX_PD_COUNT:
        section.add_agent(PDAgent.get_one(map_path=map_path))
        return section

    index = randint(0, len(section.pds) - 1)
    section.pds[index] = PDAgent.get_one_for_cw(
        section.pds[index].cw_id, map_path=map_path
    )
    return section


def _mut_tc_section(section: TCSection, map_path: str) -> TCSection:
    mut_pb = random()
    if mut_pb < 0.3:
        section.initial = TCSection.generate_config(map_path=map_path)
        return section
    elif mut_pb < 0.6:
        section.final = TCSection.generate_config(map_path=map_path)
    elif mut_pb < 0.9:
        section.duration_g = TCSection.get_random_duration_g()
    return TCSection.get_one(map_path=map_path)


def mut_scenario(
    ind: Scenario, map_path: str, max_adc_count: int = MAX_ADC_COUNT
) -> Tuple[Scenario]:
    mut_pb = random()
    if mut_pb < 1 / 3:
        ind.ad_section = _mut_ad_section(
            ind.ad_section, map_path, max_adc_count=max_adc_count
        )
    elif mut_pb < 2 / 3:
        ind.pd_section = _mut_pd_section(ind.pd_section, map_path)
    else:
        ind.tc_section = _mut_tc_section(ind.tc_section, map_path)
    return (ind,)


# CROSSOVER OPERATORS (matching original Apollo GA)


def _get_route_str(adc: ADAgent) -> str:
    """Get a string representation of the route for comparison."""
    # Use route attribute - adapt based on actual ADAgent API
    route = adc.route
    if hasattr(route, "__str__"):
        return str(route)
    return repr(route)


def _cx_ad_section(
    ind1: ADSection,
    ind2: ADSection,
    map_path: str,
    max_adc_count: int = MAX_ADC_COUNT,
) -> Tuple[ADSection, ADSection]:
    """Crossover for AD sections - matching original Apollo GA."""
    # swap entire ad section with 5% probability
    cx_pb = random()
    if cx_pb < 0.05:
        return ind2, ind1

    cxed = False

    # Try to find common routes and swap attributes
    for adc1 in ind1.adcs:
        for adc2 in ind2.adcs:
            if _get_route_str(adc1) == _get_route_str(adc2):
                # same routing in both parents
                # swap start_s or start_t
                if random() < 0.5:
                    if hasattr(adc1, "start_s") and hasattr(adc2, "start_s"):
                        adc1.start_s, adc2.start_s = adc2.start_s, adc1.start_s
                else:
                    if hasattr(adc1, "start_t") and hasattr(adc2, "start_t"):
                        adc1.start_t, adc2.start_t = adc2.start_t, adc1.start_t
                cxed = True
    if cxed:
        ind1.adjust_time()
        return ind1, ind2

    # Try to add a conflicting agent from parent 2 to parent 1
    if len(ind1.adcs) < max_adc_count:
        for adc in ind2.adcs:
            if ind1.has_conflict(adc, map_path=map_path) and ind1.add_agent(
                deepcopy(adc)
            ):
                # add an agent from parent 2 to parent 1 if there exists a conflict
                ind1.adjust_time()
                return ind1, ind2

    # if none of the above happened, no common adc, no conflict in either
    # combine to make a new population
    available_adcs = ind1.adcs + ind2.adcs
    shuffle(available_adcs)
    split_index = randint(2, min(len(available_adcs), max_adc_count))

    result1 = ADSection([])
    # Only add agents that are not too close to existing ones
    for x in available_adcs[:split_index]:
        if result1.add_agent(deepcopy(x)):
            # Successfully added
            if len(result1.adcs) >= max_adc_count:
                break
        # If add_agent returns False (too close), skip this agent

    # make sure offspring adc count is valid
    while len(result1.adcs) > max_adc_count:
        result1.adcs.pop()

    trial = 0
    max_trials = 50  # Increased limit to prevent forcing invalid agents
    while len(result1.adcs) < 2 and trial < max_trials:
        new_ad = ADAgent.get_one(
            map_path=map_path, must_not_start_from_junction=(trial < 15)
        )
        if result1.has_conflict(new_ad, map_path=map_path) and result1.add_agent(
            new_ad
        ):
            break
        trial += 1
        # Don't force add agents that are too close - respect distance constraints
    result1.adjust_time()
    return result1, ind2


def _cx_pd_section(
    ind1: PDSection, ind2: PDSection, map_path: str
) -> Tuple[PDSection, PDSection]:
    """Crossover for PD sections - matching original Apollo GA."""
    cx_pb = random()
    if cx_pb < 0.1:
        return ind2, ind1

    available_pds = ind1.pds + ind2.pds

    if len(available_pds) == 0:
        return ind1, ind2

    result1 = PDSection(
        sample(available_pds, k=randint(0, min(MAX_PD_COUNT, len(available_pds))))
    )
    result2 = PDSection(
        sample(available_pds, k=randint(0, min(MAX_PD_COUNT, len(available_pds))))
    )
    return result1, result2


def _cx_tc_section(
    ind1: TCSection, ind2: TCSection, map_path: str
) -> Tuple[TCSection, TCSection]:
    """Crossover for TC sections - matching original Apollo GA."""
    cx_pb = random()
    if cx_pb < 0.1:
        return ind2, ind1
    elif cx_pb < 0.4:
        ind1.initial, ind2.initial = ind2.initial, ind1.initial
    elif cx_pb < 0.7:
        ind1.final, ind2.final = ind2.final, ind1.final
    else:
        ind1.duration_g, ind2.duration_g = ind2.duration_g, ind1.duration_g
    return ind1, ind2


def cx_scenario(
    ind1: Scenario,
    ind2: Scenario,
    map_path: str,
    max_adc_count: int = MAX_ADC_COUNT,
) -> Tuple[Scenario, Scenario]:
    """Crossover scenarios - matching original Apollo GA."""
    cx_pb = random()
    if cx_pb < 0.6:
        ind1.ad_section, ind2.ad_section = _cx_ad_section(
            ind1.ad_section, ind2.ad_section, map_path, max_adc_count=max_adc_count
        )
    elif cx_pb < 0.8:
        ind1.pd_section, ind2.pd_section = _cx_pd_section(
            ind1.pd_section, ind2.pd_section, map_path
        )
    else:
        ind1.tc_section, ind2.tc_section = _cx_tc_section(
            ind1.tc_section, ind2.tc_section, map_path
        )
    return ind1, ind2


def main() -> None:
    import argparse
    import json
    import time
    import sys

    parser = argparse.ArgumentParser(description="Run a minimal Autoware scenario.")
    parser.add_argument(
        "--url",
        action="append",
        dest="urls",
        default=[],
        help=(
            "Autoware receiver base URL (repeatable). "
            "If omitted, uses AUTOWARE_RECEIVER_URLS or Docker-bridge examples."
        ),
    )
    parser.add_argument(
        "--num-vehicles",
        type=int,
        default=None,
        help="Fixed number of vehicles per scenario (optional).",
    )
    parser.add_argument(
        "--min-vehicles",
        type=int,
        default=2,
        help="Minimum vehicles per scenario when using mixed-size generation.",
    )
    parser.add_argument(
        "--max-vehicles",
        type=int,
        default=5,
        help="Maximum vehicles per scenario when using mixed-size generation.",
    )
    parser.add_argument(
        "--map",
        default="autoware_map/BorregasAve/lanelet2_map.osm",
        help="Lanelet2 OSM map path.",
    )
    parser.add_argument(
        "--population",
        type=int,
        default=10,
        help="Number of individuals per generation.",
    )
    parser.add_argument(
        "--generations",
        type=int,
        default=0,
        help="Number of generations to run (0 = use duration instead).",
    )
    parser.add_argument(
        "--duration-hours",
        type=float,
        default=15.0,
        help="Duration to run in hours (used when generations=0).",
    )
    parser.add_argument(
        "--log-dir",
        default="scenario_runs",
        help="Directory to store scenario run metadata.",
    )
    parser.add_argument(
        "--restart-wait",
        type=float,
        default=60.0,
        help="Seconds to wait after Autoware restart before retrying auto mode.",
    )
    parser.add_argument(
        "--max-recovery-retries",
        type=int,
        default=3,
        help="Maximum per-scenario auto-mode recovery retries before marking scenario failed.",
    )
    parser.add_argument(
        "--max-discard-regenerations",
        type=int,
        default=10,
        help=(
            "Maximum times to discard and regenerate a scenario in-place when "
            "recovery retry limit is exceeded."
        ),
    )
    parser.add_argument(
        "--conflict-only",
        action="store_true",
        default=True,
        help="Generate only conflict scenarios (default: True).",
    )
    parser.add_argument(
        "--no-conflict-only",
        action="store_false",
        dest="conflict_only",
        help="Generate any scenarios, not just conflict ones.",
    )
    args = parser.parse_args()

    if args.num_vehicles is not None:
        if int(args.num_vehicles) < 2:
            raise ValueError("--num-vehicles must be >= 2.")
        min_vehicles = int(args.num_vehicles)
        max_vehicles = int(args.num_vehicles)
    else:
        min_vehicles = max(2, int(args.min_vehicles))
        max_vehicles = max(min_vehicles, int(args.max_vehicles))

    # Receiver URL source priority:
    # 1. explicit --url
    # 2. AUTOWARE_RECEIVER_URLS
    # 3. Docker auto-discovery of autoware_<n> containers
    # 4. built-in Docker bridge examples
    default_urls_env = os.environ.get("AUTOWARE_RECEIVER_URLS", "").strip()
    docker_discovered_urls: List[str] = []
    if default_urls_env:
        default_urls = [
            u.strip().rstrip("/")
            for u in default_urls_env.split(",")
            if u.strip()
        ]
    else:
        docker_discovered_urls = _discover_docker_receiver_urls(max_vehicles)
        if docker_discovered_urls:
            default_urls = docker_discovered_urls
        else:
            default_urls = [
                "http://172.17.0.2:5002",
                "http://172.17.0.3:5002",
                "http://172.17.0.4:5002",
                "http://172.17.0.5:5002",
                "http://172.17.0.6:5002",
            ]

    url_pool: List[str] = [str(u).rstrip("/") for u in (args.urls or [])]
    if len(url_pool) < max_vehicles:
        # Not enough URLs: pad from defaults while preserving given order.
        seen = set(url_pool)
        for default_url in default_urls:
            if default_url not in seen:
                url_pool.append(default_url)
                seen.add(default_url)
            if len(url_pool) >= max_vehicles:
                break
        if len(url_pool) < max_vehicles:
            raise ValueError(
                f"Need {max_vehicles} URLs but only {len(url_pool)} available. "
                "Pass more --url entries."
            )
    urls: List[str] = url_pool[:max_vehicles]
    print(
        f"[GA] Vehicle count mode: min={min_vehicles}, max={max_vehicles}",
        flush=True,
    )
    if not args.urls and not default_urls_env:
        if docker_discovered_urls:
            print(
                f"[GA] Auto-discovered receiver endpoints from Docker: {docker_discovered_urls}",
                flush=True,
            )
        else:
            print(
                "[GA][WARN] Using built-in Docker bridge example URLs "
                "(172.17.0.2..172.17.0.6). If your container IPs differ, pass "
                "--url multiple times, set AUTOWARE_RECEIVER_URLS, or run from "
                "a shell that can access Docker for auto-discovery.",
                flush=True,
            )
    print(f"[GA] Receiver endpoints: {urls}", flush=True)
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / "test_main.log"

    class _Tee:
        def __init__(self, *streams):
            self._streams = streams

        def write(self, data):
            for stream in self._streams:
                stream.write(data)
                stream.flush()

        def flush(self):
            for stream in self._streams:
                stream.flush()

    log_file = open(log_path, "a", encoding="utf-8")
    sys.stdout = _Tee(sys.stdout, log_file)
    sys.stderr = _Tee(sys.stderr, log_file)

    print(f"[GA] Log dir: {log_dir.resolve()}", flush=True)
    print(f"[GA] Log file: {log_path.resolve()}", flush=True)

    def _scenario_to_dict(scenario: Scenario) -> Dict[str, Any]:
        return {
            "ad_section": [
                {**adc.to_dict(), "start_t": adc.start_t}
                for adc in scenario.ad_section.adcs
            ],
            "pd_section": [
                {"cw_id": pd.cw_id, "speed": pd.speed, "start_t": pd.start_t}
                for pd in scenario.pd_section.pds
            ],
            "tc_section": {
                "initial": scenario.tc_section.initial,
                "final": scenario.tc_section.final,
                "duration_g": scenario.tc_section.duration_g,
                "duration_y": scenario.tc_section.duration_y,
                "duration_b": scenario.tc_section.duration_b,
            },
            "uid": scenario.uid,
        }

    # Create endpoints for the maximum configured vehicle count.
    endpoints = [VehicleEndpoint(f"vehicle_{i}", url) for i, url in enumerate(urls)]
    runner = ScenarioRunner(endpoints)
    runner.configure_recovery(
        restart_wait_s=args.restart_wait,
        max_recovery_retries=args.max_recovery_retries,
    )

    def eval_scenario(ind: Scenario) -> Tuple[float, int, int, int]:
        """Evaluate scenario fitness - matching original Apollo GA.

        Returns:
            (min_distance, unique_decisions, conflict, unique_violation)
        """
        g_name = f"Generation_{ind.gid:05d}"
        s_name = f"Scenario_{ind.cid:05d}"
        log_path = log_dir / f"{g_name}_{s_name}.json"
        discard_regen_count = 0
        max_discard_regen = 50

        while True:
            start_ts = time.time()
            print(f"[GA] Running {g_name} {s_name} uid={ind.uid}", flush=True)
            record = {
                "generation": ind.gid,
                "scenario_id": ind.cid,
                "scenario_uid": ind.uid,
                "urls": urls,
                "start_time": start_ts,
                "scenario": _scenario_to_dict(ind),
                "conflict_only": bool(args.conflict_only),
            }
            try:
                runner.set_scenario(ind)
                runner.init_scenario()
                runner.run_scenario(g_name, s_name, save_record=True)
                violations = []
                decisions = []
                min_distance = None
                unique_violation = 0
                unique_decisions = 0
                for endpoint, adc in zip(endpoints, ind.ad_section.adcs):
                    try:
                        vresp = endpoint.fetch_violations(adc.route)
                        violations.append(vresp)
                        unique_violation += int(vresp.get("violation_count", 0))
                        if vresp.get("min_distance") is not None:
                            if min_distance is None:
                                min_distance = vresp.get("min_distance")
                            else:
                                min_distance = min(
                                    min_distance, vresp.get("min_distance")
                                )
                    except Exception as exc:
                        violations.append({"status": "error", "error": str(exc)})
                    try:
                        dresp = endpoint.fetch_decisions()
                        decisions.append(dresp)
                        unique_decisions += _extract_unique_decisions(dresp)
                    except Exception as exc:
                        decisions.append({"status": "error", "error": str(exc)})

                if min_distance is None:
                    min_distance = float("inf")

                # Count AD conflict pairs (matching original GA's has_ad_conflict)
                conflict = _count_ad_conflicts(ind.ad_section, args.map)

                record["violations"] = violations
                record["decisions"] = decisions
                record["min_distance"] = min_distance
                record["unique_violation"] = unique_violation
                record["unique_decisions"] = unique_decisions
                record["conflict"] = conflict
                record["status"] = "completed"
                if discard_regen_count > 0:
                    record["discarded_recovery_limit_attempts"] = discard_regen_count

                fitness = _compute_fitness(
                    float(min_distance), unique_decisions, conflict, unique_violation
                )
                record["fitness"] = {
                    "min_distance": float(min_distance),
                    "unique_decisions": unique_decisions,
                    "conflict": conflict,
                    "unique_violation": unique_violation,
                    "score": list(fitness),
                }
                record["end_time"] = time.time()
                record["elapsed_s"] = record["end_time"] - start_ts
                with open(log_path, "w", encoding="utf-8") as fp:
                    json.dump(record, fp, indent=2)
                return fitness
            except AutoModeUnavailableError as exc:
                # Discard this attempt if recovery retry limit was exceeded.
                if "Exceeded max recovery retries" in str(exc):
                    discard_regen_count += 1
                    print(
                        (
                            f"[GA] Discarding {g_name} {s_name} due to recovery limit "
                            f"({discard_regen_count}/{max_discard_regen}); regenerating."
                        ),
                        flush=True,
                    )
                    if discard_regen_count <= max_discard_regen:
                        vehicle_count = len(ind.ad_section.adcs)
                        if args.conflict_only:
                            regenerated = Scenario.get_conflict_one(
                                vehicle_count, args.map, ind.gid, ind.cid
                            )
                        else:
                            regenerated = Scenario.get_one(
                                vehicle_count, args.map, ind.gid, ind.cid
                            )
                        ind.ad_section = regenerated.ad_section
                        ind.pd_section = regenerated.pd_section
                        ind.tc_section = regenerated.tc_section
                        ind.uid = uuid.uuid4().hex
                        if log_path.exists():
                            log_path.unlink()
                        continue

                record["status"] = "failed"
                record["error"] = str(exc)
                record["fitness"] = {
                    "min_distance": float("inf"),
                    "unique_decisions": 0,
                    "conflict": 0,
                    "unique_violation": 0,
                    "score": [float("inf"), 0, 0, 0],
                }
                record["end_time"] = time.time()
                record["elapsed_s"] = record["end_time"] - start_ts
                with open(log_path, "w", encoding="utf-8") as fp:
                    json.dump(record, fp, indent=2)
                return (float("inf"), 0, 0, 0)
            except Exception as exc:
                record["status"] = "failed"
                record["error"] = str(exc)
                fitness = (float("inf"), 0, 0, 0)
                record["fitness"] = {
                    "min_distance": float("inf"),
                    "unique_decisions": 0,
                    "conflict": 0,
                    "unique_violation": 0,
                    "score": [float("inf"), 0, 0, 0],
                }
                record["end_time"] = time.time()
                record["elapsed_s"] = record["end_time"] - start_ts
                with open(log_path, "w", encoding="utf-8") as fp:
                    json.dump(record, fp, indent=2)
                return fitness

    POP_SIZE = args.population
    OFF_SIZE = args.population
    CXPB = 0.8
    MUTPB = 0.2

    toolbox = base.Toolbox()
    toolbox.register("evaluate", eval_scenario)
    toolbox.register("mate", cx_scenario, map_path=args.map, max_adc_count=max_vehicles)
    toolbox.register(
        "mutate", mut_scenario, map_path=args.map, max_adc_count=max_vehicles
    )
    toolbox.register("select", tools.selNSGA2)
    toolbox.register("clone", deepcopy)

    population: List[Scenario] = []
    for cid in range(POP_SIZE):
        scenario_vehicle_count = randint(min_vehicles, max_vehicles)
        if args.conflict_only:
            population.append(
                Scenario.get_conflict_one(scenario_vehicle_count, args.map, 0, cid)
            )
        else:
            population.append(Scenario.get_one(scenario_vehicle_count, args.map, 0, cid))

    for index, ind in enumerate(population):
        ind.gid = 0
        ind.cid = index
        ind.uid = uuid.uuid4().hex

    invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit

    stats = tools.Statistics(key=lambda ind: ind.fitness.values)
    stats.register("avg", np.mean, axis=0)
    stats.register("max", np.max, axis=0)
    stats.register("min", np.min, axis=0)
    logbook = tools.Logbook()
    logbook.header = "gen", "avg", "max", "min"

    # Time-based or generation-based termination
    ga_start_time = time.time()
    duration_seconds = args.duration_hours * 3600.0
    use_time_limit = args.generations <= 0

    if use_time_limit:
        print(
            f"[GA] Running for {args.duration_hours} hours ({duration_seconds:.0f} seconds)",
            flush=True,
        )
    else:
        print(f"[GA] Running for {args.generations} generations", flush=True)

    curr_gen = 0
    while True:
        # Check termination condition
        elapsed = time.time() - ga_start_time
        if use_time_limit:
            if elapsed >= duration_seconds:
                print(
                    f"[GA] Time limit reached: {elapsed / 3600:.2f} hours", flush=True
                )
                break
        else:
            if curr_gen >= args.generations:
                print(f"[GA] Generation limit reached: {curr_gen}", flush=True)
                break

        # Log progress for time-based runs
        if use_time_limit and curr_gen % 5 == 0:
            remaining = duration_seconds - elapsed
            print(
                f"[GA] Progress: {elapsed / 3600:.2f}h elapsed, {remaining / 3600:.2f}h remaining",
                flush=True,
            )
        curr_gen += 1
        prev_population_uids = {ind.uid for ind in population}
        if len(population) < 2:
            offspring = [toolbox.clone(population[0]) for _ in range(OFF_SIZE)]
        else:
            offspring = algorithms.varOr(population, toolbox, OFF_SIZE, CXPB, MUTPB)

        for index, ind in enumerate(offspring):
            ind.gid = curr_gen
            ind.cid = index
            ind.uid = uuid.uuid4().hex

        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        selected = toolbox.select(population + offspring, POP_SIZE)
        population[:] = selected

        record = stats.compile(population)
        logbook.record(gen=curr_gen, **record)
        print(logbook.stream)

        selection_log = {
            "generation": curr_gen,
            "selected": [
                {
                    "uid": ind.uid,
                    "gid": ind.gid,
                    "cid": ind.cid,
                    "fitness": list(ind.fitness.values),
                    "from_previous_population": ind.uid in prev_population_uids,
                }
                for ind in population
            ],
        }
        sel_path = log_dir / f"GA_selection_gen_{curr_gen:05d}.json"
        print(f"[GA] Writing selection log: {sel_path}", flush=True)
        try:
            with open(sel_path, "w", encoding="utf-8") as fp:
                json.dump(selection_log, fp, indent=2)
            print(f"[GA] Wrote selection log: {sel_path}", flush=True)
        except Exception as exc:
            print(f"[GA] Failed to write selection log: {exc}", flush=True)


if __name__ == "__main__":
    main()
