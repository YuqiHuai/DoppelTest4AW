from dataclasses import dataclass
from random import randint, shuffle
from typing import Dict, List, Optional

from scenario_runner.hdmap.MapParser import MapParser

DEFAULT_MAP = "autoware_map/sample-map-planning/lanelet2_map.osm"
SCENARIO_UPPER_LIMIT = 30


@dataclass
class TCSection:
    """
    Genetic representation of the traffic control section.

    :param Dict[str, str] initial: the initial configuration
    :param Dict[str, str] final: the final configuration
    :param float duration_g: green signal duration
    :param float duration_y: yellow change interval duration
    :param float duration_b: red clearance interval duration
    """

    initial: Dict[str, str]
    final: Dict[str, str]
    duration_g: float
    duration_y: float
    duration_b: float

    def calculate_transition(self) -> Dict[str, str]:
        """
        Calculates the color of signals during the transition stage.
        """
        result = dict()
        for k in self.initial:
            if self.initial[k] == "GREEN" and self.final[k] == "RED":
                result[k] = "YELLOW"
            else:
                result[k] = self.initial[k]
        return result

    def get_config_with_color(self, color: str) -> Dict[str, str]:
        """
        Gets a configuration where all signals have the specified color.
        """
        result = dict()
        for k in self.initial:
            result[k] = color
        return result

    @staticmethod
    def generate_config(
        preference: Optional[List[str]] = None, map_path: str = DEFAULT_MAP
    ) -> Dict[str, str]:
        """
        Generate a configuration with certain signals being green.
        """
        if preference is None:
            preference = []
        mp = MapParser.get_instance(map_path)
        result = dict()
        signals = list(mp.get_signals())
        shuffle(signals)
        while signals:
            if preference:
                curr_sig = preference.pop()
                if curr_sig in signals:
                    signals.remove(curr_sig)
            else:
                curr_sig = signals.pop()

            result[curr_sig] = "GREEN"
            relevant = mp.get_signals_wrt(curr_sig)
            for sig, cond in relevant:
                if sig in preference:
                    preference.remove(sig)
                if sig in signals:
                    signals.remove(sig)
                    if cond == "EQ":
                        result[sig] = "GREEN"
                    else:
                        result[sig] = "RED"
        return result

    @staticmethod
    def get_one(map_path: str = DEFAULT_MAP) -> "TCSection":
        """
        Randomly generates a traffic control section.
        """
        init = TCSection.generate_config(map_path=map_path)
        final = TCSection.generate_config(
            preference=[x for x in init if init[x] == "RED"], map_path=map_path
        )
        return TCSection(
            initial=init,
            final=final,
            duration_g=randint(5, int(SCENARIO_UPPER_LIMIT / 2)),
            duration_y=3,
            duration_b=2,
        )

    @staticmethod
    def get_random_duration_g() -> int:
        """
        Generate a random duration for green light.
        """
        return randint(5, int(SCENARIO_UPPER_LIMIT / 2))


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Generate a sample traffic control config.")
    parser.add_argument(
        "--map",
        default=DEFAULT_MAP,
        help="Path to the Lanelet2 OSM map.",
    )
    args = parser.parse_args()

    tc = TCSection.get_one(map_path=args.map)
    print(
        {
            "initial": tc.initial,
            "final": tc.final,
            "duration_g": tc.duration_g,
            "duration_y": tc.duration_y,
            "duration_b": tc.duration_b,
        }
    )


if __name__ == "__main__":
    main()
