from kuri_api import Lights
from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track


class ObserverModeAnimations(AnimationGroup):
    """
    Animations that go with observer mode
    """

    def observer_indicator(self):
        tk = Track()
        inner = (30, 0, 124)
        outer = (37, 0, 119)
        pattern = [outer] * Lights.NUM_LEDS
        pattern[Lights.IDX_INNER_BOTTOM_LEFT] = inner
        pattern[Lights.IDX_INNER_BOTTOM_RIGHT] = inner
        pattern[Lights.IDX_INNER_UPPER_LEFT] = inner
        pattern[Lights.IDX_INNER_UPPER_RIGHT] = inner
        pattern[Lights.IDX_CENTER] = Lights.OFF
        pattern[Lights.IDX_INNER_LEFT] = Lights.OFF
        pattern[Lights.IDX_INNER_RIGHT] = Lights.OFF
        tk.add(0.0, self.lights_mot.glow_pattern(pattern, fade_length=0.0, hold_length=0.5))
        return tk

    def observer_start(self):
        tk = Track()
        tk.add(0.0, self.movies.to_animated('observer-mode-start.mov'))
        return tk

    def observer_end(self):
        tk = Track()
        tk.add(0.0, self.movies.to_animated('observer-mode-exit.mov'))
        return tk
