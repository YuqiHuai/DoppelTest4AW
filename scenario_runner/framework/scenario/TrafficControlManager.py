from typing import Dict, List

from DoppelAutoware.framework.scenario.tc_config import TCSection

FORCE_INVALID_TRAFFIC_CONTROL = False


class TrafficControlManager:
    """
    Manages traffic signals during a scenario based on the specification.

    :param TCSection tc: traffic control section
    """

    def __init__(self, tc: TCSection) -> None:
        self.tc = tc

    def get_traffic_configuration(self, curr_t: float) -> Dict:
        """
        Based on the current time, generate a traffic signal configuration.
        """
        if FORCE_INVALID_TRAFFIC_CONTROL:
            config = self.tc.get_config_with_color("GREEN")
        elif self.tc.initial == self.tc.final:
            config = self.tc.initial
        else:
            if curr_t <= self.tc.duration_g:
                config = self.tc.initial
            elif curr_t <= self.tc.duration_g + self.tc.duration_y:
                config = self.tc.calculate_transition()
            elif curr_t <= self.tc.duration_g + self.tc.duration_y + self.tc.duration_b:
                config = self.tc.get_config_with_color("RED")
            else:
                config = self.tc.final

        signals: List[Dict[str, str]] = []
        for signal_id, color in config.items():
            signals.append({"id": signal_id, "color": color})

        return {"signals": signals}
