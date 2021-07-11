import logging

from kuri_api.anim.library.blink_animations import BlinkAnimations
from kuri_api.anim.library.common_animations import CommonAnimations
from kuri_api.anim.library.docking_animations import DockingAnimations
from kuri_api.anim.library.docking_animations import DockingLEDAnimations
from kuri_api.anim.library.docking_animations import DockingSupportAnimations
from kuri_api.anim.library.idle_animations import IdleAnimations
from kuri_api.anim.library.led_emotion_animations import LEDEmotionAnimations
from kuri_api.anim.library.observer_animations import ObserverModeAnimations
from kuri_api.anim.library.onboarding_animations import OnboardingAnimations
from kuri_api.anim.library.reaction_animations import ReactionAnimations
from kuri_api.anim.library.relocalize_animations import RelocalizeAnimations
from kuri_api.anim.library.romoji_animations import RomojiAnimations
from kuri_api.anim.library.sleep_wake_animations import SleepAndWakeAnimations
from kuri_api.anim.library.social_animations import SocialAnimations
from kuri_api.anim.library.system_animations import SystemAnimations
from kuri_api.anim.library.test_animations import TestAnimations

logger = logging.getLogger(__name__)
PRODUCTION_ANIMATIONS = [
    BlinkAnimations,
    CommonAnimations,
    DockingSupportAnimations,
    IdleAnimations,
    LEDEmotionAnimations,
    ObserverModeAnimations,
    OnboardingAnimations,
    ReactionAnimations,
    RelocalizeAnimations,
    RomojiAnimations,
    SleepAndWakeAnimations,
    SocialAnimations,
    SystemAnimations,
    DockingAnimations,
    DockingLEDAnimations,
    TestAnimations]


def generate_all_animations(head_mux=None, wheel_mux=None, light_mux=None, sound_mux=None):
    """
    Returns all tracks from all user-facing animation groups.
    :return: List of the relevant animation groups.
    """
    anims = []
    for anim_class in PRODUCTION_ANIMATIONS:
        anims.append(anim_class(head_mux, wheel_mux, light_mux, sound_mux))

    return anims
