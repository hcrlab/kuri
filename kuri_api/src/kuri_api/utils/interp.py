def round_val(v):
    return min(max(round(v), 0), 255)


def round_color(color):
    r, g, b = color
    return (
        round_val(r), round_val(g), round_val(b))


def mul_color(color, s):
    r, g, b = color
    return (
        r * s, g * s, b * s)


def add_color(c1, c2):
    r1, g1, b1 = c1
    r2, g2, b2 = c2
    return (
        r1 + r2, g1 + g2, b1 + b2)


def add_pat(p1, p2):
    return [add_color(c1, c2) for c1, c2 in zip(p1, p2)]


def mul_pat(pat, s):
    return [mul_color(color, s) for color in pat]


def round_pat(pat):
    return [round_color(color) for color in pat]


def linear(t):
    return t


def quad_in(t):
    return t * t


def quad_out(t):
    return -(t - 2) * t


def quad_inout(t):
    if t < 0.5:
        return 0.5 * quad_in(t * 2.0)
    return 0.5 * (quad_out(2 * t - 1) + 1)


def cubic_in(t):
    return pow(t, 3)


def cubic_out(t):
    return pow(t - 1, 3) + 1


def cubic_inout(t):
    if t < 0.5:
        return 0.5 * cubic_in(2 * t)
    return 0.5 * (cubic_out(2 * t - 1) + 1)


def quart_in(t):
    return pow(t, 4)


def quart_out(t):
    return -(pow(t - 1, 4) - 1)


def quart_inout(t):
    if t < 0.5:
        return 0.5 * quart_in(t * 2)
    return 0.5 * (quart_out(2 * t - 1) + 1)


def quint_in(t):
    return pow(t, 5)


def quint_out(t):
    return pow(t - 1, 5) + 1


def quint_inout(t):
    if t < 0.5:
        return 0.5 * quint_in(2 * t)
    return 0.5 * (quint_out(2 * t - 1) + 1)


def interp_pat(start_pat, end_pat, func, nframes):
    """
    Given a start pattern, end pattern and a linear interpolation function
    return a series of patterns that interpolates between the two.

    Parameters
    ----------
    start_pat (list of n 3 tuples): pattern to start with
    end_pat (list of n 3 tuples): pattern to end at
    func (f(float) -> float): interpolation func
    nframes: number of frames to interpolate over
    """
    patterns = []
    for frame_idx in range(nframes):
        t = func(frame_idx / float(nframes))
        pat = add_pat(mul_pat(start_pat, 1 - t), mul_pat(end_pat, t))
        patterns.append(round_pat(pat))

    return patterns


class KeyFrame(object):
    """
    This class represents keyframes to build an interpolation sequence
    frame_idx: Frame index of this keyframe
    pattern: The pattern to hit on this keyframe
    func: The interpolation function to hit this keyframe
    """

    def __init__(self, frame_idx, pattern, func):
        self.frame_idx = frame_idx
        self.pattern = pattern
        self.func = func


def interp_keyframes(keyframes):
    """
    This function takes an array of keyframes and builds an interpolated
    series of frames
    """
    frames = []
    assert len(keyframes) > 1
    last_keyframe = keyframes[0]
    for keyframe in keyframes[1:]:
        frames += interp_pat(last_keyframe.pattern, keyframe.pattern, keyframe.func,
                             keyframe.frame_idx - last_keyframe.frame_idx)
        last_keyframe = keyframe

    return frames