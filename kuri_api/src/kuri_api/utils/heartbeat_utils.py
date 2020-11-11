import numpy as np


def happiness_to_fade(happiness):
    """
    Currently just rounds to the nearest bin.
    """
    HAPPINESS_RANGE = 6.0
    HAPPINESS_COLORS = [
        (94, 31, 0),
        (101, 57, 26),
        (81, 71, 65),
        (63, 61, 61),
        (73, 73, 73),
        (91, 91, 91),
        (166, 166, 166)]
    num_colors = len(HAPPINESS_COLORS) - 1
    nearest_index = np.clip(int(round(happiness + HAPPINESS_RANGE / 2.0)), 0, num_colors)
    return HAPPINESS_COLORS[nearest_index]


def happiness_to_color(happiness):
    """
    Currently just rounds to the nearest bin.
    """
    HAPPINESS_RANGE = 6.0
    HAPPINESS_COLORS = [
        (255, 91, 0),
        (255, 158, 92),
        (244, 218, 201),
        (244, 234, 232),
        (178, 178, 178),
        (219, 219, 219),
        (255, 255, 255)]
    num_colors = len(HAPPINESS_COLORS) - 1
    nearest_index = np.clip(int(round(happiness + HAPPINESS_RANGE / 2.0)), 0, num_colors)
    return HAPPINESS_COLORS[nearest_index]


def excitation_to_period(excitation):
    """
    Maps a value in the excitation range [-3 : 3] to the pulse range
    [4.825 : 1.075]
    """
    MOOD_RANGE = (-3, 3)
    EXCITATION_CYCLE_RANGE_SECS = (4.825, 1.075)
    return map_range(excitation, MOOD_RANGE, EXCITATION_CYCLE_RANGE_SECS)


def map_range(value, in_range, out_range):
    """
    Maps a value in one range to another.
    :param: in_range A tuple of the range to map from
    :param: out_range A tuple of the range to map to
    """
    return (value - in_range[0]) * (out_range[1] - out_range[0]) / (in_range[1] - in_range[0]) + out_range[0]
