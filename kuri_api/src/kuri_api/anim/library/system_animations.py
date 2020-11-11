from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track


class SystemAnimations(AnimationGroup):
    """
    System-level animations.
    """

    def boot_up(self):
        tk = Track()
        tk.add(0, self.lights_mot.circle_boot(float('inf')))
        return tk

    def start_sound(self):
        tk = Track()
        tk.add(0.0, self.sound_mot.open('Startup.wav'))
        return tk

    def critical_battery(self):
        tk = Track()
        tk.add(0.0, self.movies.to_animated('critical_battery.mov'))
        return tk
