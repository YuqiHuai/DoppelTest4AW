from dataclasses import dataclass
from random import randint, uniform
from secrets import choice
from typing import List

from DoppelAutoware.hdmap.MapParser import MapParser

DEFAULT_MAP = "DoppelAutoware/data/maps/sample-map-planning/lanelet2_map.osm"
MAX_PD_COUNT = 5
SCENARIO_UPPER_LIMIT = 30


@dataclass
class PDAgent:
    """
    Genetic representation of a single pedestrian.

    :param str cw_id: the ID of the crosswalk this pedestrian is on
    :param float speed: the speed of this pedestrian
    :param float start_t: the start time of this pedestrian
    """

    cw_id: str
    speed: float
    start_t: float

    @staticmethod
    def get_one(map_path: str = DEFAULT_MAP) -> "PDAgent":
        """
        Randomly generates a pedestrian representation.
        """
        mp = MapParser.get_instance(map_path)
        cws = list(mp.get_crosswalks())
        cw = choice(cws)
        return PDAgent(
            cw_id=cw,
            speed=round(uniform(1.25, 3), 1),
            start_t=randint(0, SCENARIO_UPPER_LIMIT),
        )

    @staticmethod
    def get_one_for_cw(cw_id: str, map_path: str = DEFAULT_MAP) -> "PDAgent":
        """
        Get a pedestrian representation with the specified crosswalk ID.
        """
        mp = MapParser.get_instance(map_path)
        if cw_id not in mp.get_crosswalks():
            raise ValueError(f"Crosswalk {cw_id} not found on map.")
        return PDAgent(
            cw_id=cw_id,
            speed=round(uniform(1.25, 3), 1),
            start_t=randint(0, SCENARIO_UPPER_LIMIT),
        )


@dataclass
class PDSection:
    """
    Genetic representation of the pedestrian section.

    :param List[PDAgent] pds: list of pedestrian representations
    """

    pds: List[PDAgent]

    def add_agent(self, pd: PDAgent) -> bool:
        for p in self.pds:
            if p.cw_id == pd.cw_id:
                return False
        self.pds.append(pd)
        return True

    @staticmethod
    def get_one(map_path: str = DEFAULT_MAP) -> "PDSection":
        """
        Randomly generates a pedestrian section.
        """
        mp = MapParser.get_instance(map_path)
        num = randint(0, MAX_PD_COUNT)
        num = min(len(mp.get_crosswalks()), num)
        result = PDSection([])
        while len(result.pds) < num:
            result.add_agent(PDAgent.get_one(map_path))
        return result


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Generate a sample PDSection.")
    parser.add_argument(
        "--map",
        default=DEFAULT_MAP,
        help="Path to the Lanelet2 OSM map.",
    )
    args = parser.parse_args()

    section = PDSection.get_one(map_path=args.map)
    print(f"Generated section with {len(section.pds)} pedestrians.")
    for idx, pd in enumerate(section.pds, start=1):
        print(f"Ped {idx} cw_id={pd.cw_id} speed={pd.speed} start_t={pd.start_t}")


if __name__ == "__main__":
    main()
