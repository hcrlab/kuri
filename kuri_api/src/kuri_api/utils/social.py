import random
from math import radians
MIN_ADULT_TILT = radians(40) * -1
MAX_ADULT_TILT = radians(25) * -1
MIN_CHILD_TILT = radians(27) * -1
MAX_CHILD_TILT = 0
MIN_NAV_TILT = radians(40) * -1
MAX_NAV_TILT = radians(5) * -1

def random_photo_tilt():
    """
    Calculates a random head tilt appropriate for a photo.
    :return: tilt in radians
    """
    seed = random.random()
    if seed < 0.5:
        return random_adult_tilt()
    return random_child_tilt()


def random_adult_tilt():
    return random.uniform(MIN_ADULT_TILT, MAX_ADULT_TILT)


def random_child_tilt():
    return random.uniform(MIN_CHILD_TILT, MAX_CHILD_TILT)


def random_nav_tilt():
    return random.uniform(MIN_NAV_TILT, MAX_NAV_TILT)


def random_turn():
    """
    Calculates a random turn amount.
    :return: angle in radians to turn
    """
    seed = random.random()
    TURN_90_THRESH = 0.45
    TURN_45_THRESH = 0.25
    TURN_135_THRESH = 0.18
    turn_direction = random.choice([-1, 1])
    if seed < TURN_90_THRESH:
        turn_angle = 90
    else:
        if seed < TURN_90_THRESH + TURN_45_THRESH:
            turn_angle = 45
        else:
            if seed < TURN_90_THRESH + TURN_45_THRESH + TURN_135_THRESH:
                turn_angle = 135
            else:
                turn_angle = 180
                turn_direction = 1
    return radians(turn_angle * turn_direction)