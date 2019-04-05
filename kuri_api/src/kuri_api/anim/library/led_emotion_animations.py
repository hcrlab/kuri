from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track
from gizmo.utils.heartbeat_utils import happiness_to_color, happiness_to_fade, excitation_to_period

class LEDEmotionAnimations(AnimationGroup):
    """
    Emotional animations run in the background on the chest light
    """

    def breath(self, excited, happy):
        tk = Track()
        one_breath = self.lights_mot.heartbeat(happiness_to_color(happy), happiness_to_fade(happy), excitation_to_period(excited))
        tk.add(0.0, one_breath)
        return tk