from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track


class RelocalizeAnimations(AnimationGroup):
    """
    Turn the robots through facing
    0, 315, 45, 135, 90, 180, 270, 225, 0, 360
    To do those you run through relocalize_part[0:8].
    These animations are always called sequentially in
    this order.
    """

    def relocalize_part0(self):
        keep_alive_speed = 0
        spacing = (keep_alive_speed * 0.6 + 10) * 0.1
        speed = (keep_alive_speed + 10) * 0.1
        tk = Track()
        tk.add(0, self.sound_mot.open('Wake.wav'))
        tk.add(0.0, self.head_mot.moveeyes(0, 0.08))
        tk.add(0.0, self.head_mot.pantilt(pan=0, tilt=0.4, duration=0.2 * speed))
        tk.add(0.7 * spacing, self.head_mot.pantilt(pan=0, tilt=0, duration=0.3 * speed))
        return tk

    def relocalize_part1(self):
        keep_alive_speed = 0
        spacing = (keep_alive_speed * 0.6 + 10) * 0.1
        speed = (keep_alive_speed + 10) * 0.1
        tk = Track()
        tk.add(0.7 * spacing, self.head_mot.pantilt(pan=0.78, tilt=0, duration=0.25 * speed))
        return tk

    def relocalize_part2(self):
        keep_alive_speed = 0
        spacing = (keep_alive_speed * 0.6 + 10) * 0.1
        speed = (keep_alive_speed + 10) * 0.1
        tk = Track()
        tk.add(0.9 * spacing, self.head_mot.pantilt(pan=-0.78, tilt=0, duration=0.4 * speed))
        return tk

    def relocalize_part3(self):
        keep_alive_speed = 0
        spacing = (keep_alive_speed * 0.6 + 10) * 0.1
        tk = Track()
        tk.add(0.9 * spacing,
               self.head_mot.blinkeyes(open_amplitude=1.0, close_amplitude=1.0, open_time=0.27, close_time=0.15))
        tk.add(1.0 * spacing, self.wheels_mot.rotate(-10, 1.5))
        tk.add(1.0 * spacing, self.head_mot.pantilt(pan=0, tilt=0, duration=1.5))
        return tk

    def relocalize_part4(self):
        keep_alive_speed = 0
        spacing = (keep_alive_speed * 0.6 + 10) * 0.1
        speed = (keep_alive_speed + 10) * 0.1
        tk = Track()
        tk.add(1.9 * spacing, self.head_mot.pantilt(pan=0.78, tilt=0, duration=0.3 * speed))
        return tk

    def relocalize_part5(self):
        keep_alive_speed = 0
        spacing = (keep_alive_speed * 0.6 + 10) * 0.1
        speed = (keep_alive_speed + 10) * 0.1
        tk = Track()
        tk.add(0.5 * spacing, self.head_mot.pantilt(pan=0.78, tilt=-0.4, duration=0.25 * speed))
        tk.add(1.1 * spacing, self.head_mot.pantilt(pan=-0.78, tilt=0, duration=0.4 * speed))
        return tk

    def relocalize_part6(self):
        keep_alive_speed = 0
        spacing = (keep_alive_speed * 0.6 + 10) * 0.1
        tk = Track()
        tk.add(0.6 * spacing,
               self.head_mot.blinkeyes(open_amplitude=1.0, close_amplitude=1.0, open_time=0.27, close_time=0.15))
        tk.add(0.7 * spacing, self.wheels_mot.rotate(-10, 1.5))
        tk.add(0.7 * spacing, self.head_mot.pantilt(pan=0, tilt=0, duration=1.5))
        return tk

    def relocalize_part7(self):
        keep_alive_speed = 0
        spacing = (keep_alive_speed * 0.6 + 10) * 0.1
        speed = (keep_alive_speed + 10) * 0.1
        tk = Track()
        tk.add(1.9 * spacing, self.head_mot.pantilt(pan=0.78, tilt=0, duration=0.25 * speed))
        return tk

    def relocalize_part8(self):
        keep_alive_speed = 0
        spacing = (keep_alive_speed * 0.6 + 10) * 0.1
        speed = (keep_alive_speed + 10) * 0.1
        tk = Track()
        tk.add(0.7 * spacing, self.head_mot.pantilt(pan=-0.78, tilt=0.3, duration=0.32 * speed))
        tk.add(1.25 * spacing,
               self.head_mot.blinkeyes(open_amplitude=1.0, close_amplitude=1.0, open_time=0.27, close_time=0.15))
        tk.add(1.3 * spacing, self.head_mot.pantilt(pan=-0.78, tilt=0, duration=0.2 * speed))
        tk.add(1.5 * spacing, self.wheels_mot.rotate(-9, 1))
        tk.add(1.4 * spacing,
               self.head_mot.blinkeyes(open_amplitude=1.0, close_amplitude=1.0, open_time=0.27, close_time=0.15))
        tk.add(1.4 * spacing, self.head_mot.pantilt(pan=0, tilt=0, duration=1))
        tk.add(2.5 * spacing, self.head_mot.pantilt(pan=-0.4, tilt=-0.5, duration=0.4 * speed))
        tk.add(3.7 * spacing, self.head_mot.pantilt(pan=0, tilt=0, duration=0.4 * speed))
        return tk
