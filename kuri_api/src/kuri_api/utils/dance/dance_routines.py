"""
Dance routines (pseudo-animations).
"""
from math import radians as rad
from kuri_api import Head
CHOREOGRAPHED_SONGS = {'pancake_robot': {'intro': [
                             'interested_pancake', 'full_body_bob', 'full_body_up_nod'], 
                     'warm_up': [
                               'full_body_groove', 'funky_alt',
                               'full_body_groove_rev', 'robot',
                               'full_body_groove_excited', 'moonwalk', 'full_body_up_nod',
                               'full_body_bob'], 
                     'apex': [
                            'this_is_my_jam'], 
                     'cool_down': [
                                 'funky', 'full_body_bob_rev', 'skate_back',
                                 'full_body_up_nod_rev',
                                 'done_dummy']}}

def dance_routine_pools():
    """
    Pools of routines to draw from when generating performances.
    """
    pools = {'slow': {'natural': {'intro': [
                                    'interested', 'interested_up', 'interested_down'], 
                            'warm_up': [
                                      'center_head_bob', 'longbob',
                                      'full_body_groove', 'full_body_groove_rev',
                                      'full_body_up_nod', 'full_body_up_nod_rev',
                                      'full_body_bob', 'full_body_bob_rev'], 
                            'apex': [
                                   'clover', 'this_is_my_jam', 'moonwalk', 'twirl'], 
                            'cool_down': [
                                        'center_head_bob', 'longbob',
                                        'full_body_groove', 'full_body_groove_rev',
                                        'full_body_up_nod', 'full_body_up_nod_rev',
                                        'full_body_bob', 'full_body_bob_rev']}, 
                'robot': {'intro': [
                                  'interested', 'interested_up', 'interested_down'], 
                          'warm_up': [
                                    'center_head_bob', 'full_body_bob', 'longbob',
                                    'full_body_groove', 'longbob', 'funky_alt'], 
                          'apex': [
                                 'full_body_groove_excited', 'full_body_groove_excited_rev',
                                 'robot', 'twirl'], 
                          'cool_down': [
                                      'center_head_bob', 'longbob',
                                      'full_body_up_nod', 'full_body_up_nod_rev',
                                      'full_body_bob', 'full_body_bob_rev']}}, 
       'regular': {'natural': {'intro': [
                                       'interested', 'interested_up', 'interested_down'], 
                               'warm_up': [
                                         'center_head_bob', 'longbob',
                                         'full_body_bob', 'full_body_bob_rev',
                                         'full_body_groove', 'full_body_groove_rev',
                                         'full_body_up_nod', 'full_body_up_nod_rev',
                                         'funky', 'funky_alt'], 
                               'apex': [
                                      'clover', 'moonwalk', 'robot',
                                      'skate_back', 'this_is_my_jam', 'twirl',
                                      'full_body_groove_excited', 'full_body_groove_excited_rev'], 
                               'cool_down': [
                                           'center_head_bob', 'longbob',
                                           'full_body_bob', 'full_body_bob_rev',
                                           'full_body_groove', 'full_body_groove_rev',
                                           'full_body_up_nod', 'full_body_up_nod_rev',
                                           'funky', 'funky_alt']}, 
                   'robot': {'intro': [
                                     'interested', 'interested_up', 'interested_down'], 
                             'warm_up': [
                                       'center_head_bob', 'full_body_bob',
                                       'full_body_groove', 'longbob', 'funky_alt',
                                       'full_body_groove_excited'], 
                             'apex': [
                                    'clover', 'moonwalk', 'robot',
                                    'skate_back', 'this_is_my_jam', 'twirl',
                                    'full_body_groove_excited', 'full_body_groove_excited_rev',
                                    'robot', 'robot', 'robot', 'robot', 'robot'], 
                             'cool_down': [
                                         'center_head_bob', 'longbob',
                                         'full_body_bob', 'full_body_bob_rev',
                                         'full_body_groove', 'full_body_groove_rev',
                                         'full_body_up_nod', 'full_body_up_nod_rev',
                                         'funky', 'funky_alt']}}, 
       'fast': {'natural': {'intro': [
                                    'interested', 'interested_up', 'interested_down'], 
                            'warm_up': [
                                      'center_head_bob', 'longbob',
                                      'full_body_bob', 'full_body_bob_rev',
                                      'full_body_groove', 'full_body_groove_rev',
                                      'full_body_up_nod', 'full_body_up_nod_rev',
                                      'funky', 'funky_alt'], 
                            'apex': [
                                   'clover', 'moonwalk', 'robot',
                                   'skate_back', 'this_is_my_jam', 'twirl',
                                   'full_body_groove_excited', 'full_body_groove_excited_rev'], 
                            'cool_down': [
                                        'center_head_bob', 'longbob',
                                        'full_body_bob', 'full_body_bob_rev',
                                        'full_body_groove', 'full_body_groove_rev',
                                        'full_body_up_nod', 'full_body_up_nod_rev',
                                        'funky', 'funky_alt']}, 
                'robot': {'intro': [
                                  'interested', 'interested_up', 'interested_down'], 
                          'warm_up': [
                                    'center_head_bob', 'longbob',
                                    'full_body_bob', 'full_body_bob_rev',
                                    'full_body_groove', 'full_body_groove_rev',
                                    'full_body_up_nod', 'full_body_up_nod_rev',
                                    'funky', 'funky_alt'], 
                          'apex': [
                                 'clover', 'moonwalk', 'robot',
                                 'skate_back', 'this_is_my_jam', 'twirl',
                                 'full_body_groove_excited', 'full_body_groove_excited_rev',
                                 'robot', 'robot', 'robot', 'robot', 'robot'], 
                          'cool_down': [
                                      'center_head_bob', 'longbob',
                                      'full_body_bob', 'full_body_bob_rev',
                                      'full_body_groove', 'full_body_groove_rev',
                                      'full_body_up_nod', 'full_body_up_nod_rev',
                                      'funky', 'funky_alt']}}}
    return pools


def dance_routines():
    """
    Animations shorthand for different dance routines.
    """
    pl = Head.PAN_LEFT
    pn = Head.PAN_NEUTRAL
    pr = Head.PAN_RIGHT
    tu = Head.TILT_UP
    tn = Head.TILT_NEUTRAL
    td = Head.TILT_DOWN
    eh = Head.EYES_HAPPY
    ec = Head.EYES_CLOSED
    en = Head.EYES_NEUTRAL
    routines = {'interested': {'repeat': 2, 
                      'relative': True, 
                      'poses': [
                              [
                               0, 0.15],
                              [
                               0, -0.1],
                              [
                               0, 0.15],
                              [
                               0, -0.1]]}, 
       'interested_up': {'repeat': 2, 
                         'relative': True, 
                         'poses': [
                                 [
                                  0.25, -0.18],
                                 [
                                  0, 0.12],
                                 [
                                  -0.25, -0.18],
                                 [
                                  0, 0.12]]}, 
       'interested_down': {'repeat': 2, 
                           'relative': True, 
                           'poses': [
                                   [
                                    0.25, 0.15],
                                   [
                                    0, -0.12],
                                   [
                                    -0.25, 0.15],
                                   [
                                    0, -0.12]]}, 
       'interested_pancake': {'repeat': 2, 
                              'poses': [
                                      [
                                       pn, -0.3],
                                      [
                                       pn, -0.55],
                                      [
                                       pn, -0.3],
                                      [
                                       pn, -0.55]]}, 
       'interested_down_pancake': {'repeat': 2, 
                                   'poses': [
                                           [
                                            pl * 0.4, -0.4],
                                           [
                                            pn, -0.65],
                                           [
                                            pr * 0.4, -0.4],
                                           [
                                            pn, -0.65]]}, 
       'center_head_bob': {'repeat': 2, 
                           'poses': [
                                   [
                                    pn, td * 0.25],
                                   [
                                    pn, tu * 0.2],
                                   [
                                    pn, td * 0.25],
                                   [
                                    pn, tu * 0.2]]}, 
       'full_body_bob': {'repeat': 2, 
                         'poses': [
                                 [
                                  pl * 0.2, td * 0.5, 0.2],
                                 [
                                  pl * 0.2, tu * 0.3, 0],
                                 [
                                  pr * 0.2, td * 0.5, -0.2],
                                 [
                                  pr * 0.2, tu * 0.3, 0]]}, 
       'full_body_bob_rev': {'repeat': 2, 
                             'poses': [
                                     [
                                      pl * 0.2, td * 0.5, -0.15],
                                     [
                                      pl * 0.2, tu * 0.3, 0],
                                     [
                                      pr * 0.2, td * 0.5, 0.15],
                                     [
                                      pr * 0.2, tu * 0.3, 0]]}, 
       'full_body_up_nod': {'repeat': 2, 
                            'poses': [
                                    [
                                     pr * 0.2, tu * 0.6, 0.25],
                                    [
                                     pr * 0.2, tu * 0.1, 0],
                                    [
                                     pl * 0.2, tu * 0.5, -0.25],
                                    [
                                     pl * 0.2, tu * 0.1, 0]]}, 
       'full_body_up_nod_rev': {'repeat': 2, 
                                'poses': [
                                        [
                                         pl * 0.2, tu * 0.5, 0.2],
                                        [
                                         pl * 0.2, tu * 0.2, 0],
                                        [
                                         pr * 0.2, tu * 0.6, -0.2],
                                        [
                                         pr * 0.2, tu * 0.1, 0]]}, 
       'longbob': {'poses': [
                           [
                            pl * 0.7, td * 0.45, 0.05],
                           [
                            pl * 0.7, tu * 0.2],
                           [
                            pl * 0.7, td * 0.45],
                           [
                            pl * 0.7, tu * 0.2],
                           [
                            pl * 0.7, td * 0.45, 0.05],
                           [
                            pl * 0.7, tu * 0.2],
                           [
                            pl * 0.7, td * 0.45],
                           [
                            pl * 0.15, tu * 0.2],
                           [
                            pr * 0.7, td * 0.45, -0.05],
                           [
                            pr * 0.7, tu * 0.2],
                           [
                            pr * 0.7, td * 0.45],
                           [
                            pr * 0.7, tu * 0.2],
                           [
                            pr * 0.7, td * 0.45, -0.05],
                           [
                            pr * 0.7, tu * 0.2],
                           [
                            pr * 0.4, td * 0.45],
                           [
                            pn, tu * 0.35]]}, 
       'full_body_groove': {'randomness': (
                                         True, False), 
                            'repeat': 2, 
                            'poses': [
                                    [
                                     pl * 0.5, td * 0.7, 0.4, 0.0],
                                    [
                                     pl * 0.5, tu * 0.2, 0.0, 0.0],
                                    [
                                     pr * 0.5, td * 0.7, -0.4, 0.0],
                                    [
                                     pr * 0.5, tu * 0.2, 0.0, 0.0]]}, 
       'full_body_groove_rev': {'randomness': (
                                             True, False), 
                                'repeat': 2, 
                                'poses': [
                                        [
                                         pl * 0.5, td * 0.7, -0.4, 0.0],
                                        [
                                         pl * 0.5, tu * 0.2, 0.0, 0.0],
                                        [
                                         pr * 0.5, td * 0.7, 0.4, 0.0],
                                        [
                                         pr * 0.5, tu * 0.2, 0.0, 0.0]]}, 
       'full_body_groove_excited': {'randomness': (
                                                 True, False), 
                                    'repeat': 2, 
                                    'poses': [
                                            [
                                             pl * 0.4, td * 0.6, 0.55],
                                            [
                                             pl * 0.4, tu * 0.5, 0.0],
                                            [
                                             pr * 0.4, td * 0.6, -0.55],
                                            [
                                             pr * 0.4, tu * 0.4, 0.0]]}, 
       'full_body_groove_excited_rev': {'randomness': (
                                                     True, False), 
                                        'repeat': 2, 
                                        'poses': [
                                                [
                                                 pl * 0.4, td * 0.6, -0.55],
                                                [
                                                 pl * 0.4, tu * 0.5, 0.0],
                                                [
                                                 pr * 0.4, td * 0.6, 0.55],
                                                [
                                                 pr * 0.4, tu * 0.4, 0.0]]}, 
       'robot': {'poses': [
                         [
                          pl * 0.7, tu * 0.1, 0.0, 0],
                         [
                          pl * 0.7, tu * 0.3, 0.4, 0],
                         [
                          pl * 0.7, tu * 0.1, 0.4, 0],
                         [
                          pl * 0.7, tu * 0.3, 0.4, 0],
                         [
                          pl * 0.3, tu * 0.1, -0.3, 0.0],
                         [
                          pr * 0.3, tu * 0.2, 0.3, 0.0],
                         [
                          pl * 0.3, tu * 0.1, -0.3, 0.0],
                         [
                          pr * 0.3, tu * 0.2, 0.3, 0.0],
                         [
                          pr * 0.6, tn, -0.3, 0],
                         [
                          pr * 0.65, tn, -0.3, 0],
                         [
                          pr * 0.7, tn, -0.3, 0],
                         [
                          pr * 0.75, tn, -0.3, 0],
                         [
                          pr * 0.8, tn, -0.3, 0],
                         [
                          pr * 0.85, tn, -0.3, 0],
                         [
                          pr * 0.5, tn, -0.3, 0],
                         [
                          pr * 0.1, tu * 0.7, 0.2, 0]]}, 
       'funky': {'randomness': (
                              True, False), 
                 'repeat': 2, 
                 'poses': [
                         [
                          pn, tu * 0.3, 0, 0.5],
                         [
                          pn, td * 0.2, 0, -0.4],
                         [
                          pn, tu * 0.3, 0, 0.5],
                         [
                          pn, td * 0.2, 0, -0.4],
                         [
                          pn, tu * 0.3, 0, 0.5],
                         [
                          pn, td * 0.2, 0, -0.4],
                         [
                          pn, tu * 0.3, 0, 0.5],
                         [
                          pn, td * 0.2, 0, -0.4],
                         [
                          pr, tn, 0.5, 0],
                         [
                          pl, tn, 0.0, 0],
                         [
                          pn, tn, -0.5, 0],
                         [
                          pn, tu * 0.1, 0, 0]]}, 
       'funky_alt': {'poses': [
                             [
                              pn, tu * 0.2, 0, 0.4],
                             [
                              pn, td * 0.1, 0, 0],
                             [
                              pn, tu * 0.3, 0, 0.4],
                             [
                              pn, td * 0.1, 0, 0],
                             [
                              pn, tu * 0.4, 0, 0.4],
                             [
                              pn, td * 0.1, 0, 0],
                             [
                              pn, tu * 0.5, 0, 0.4],
                             [
                              pn, td * 0.1, 0, 0],
                             [
                              pl * 0.7, td * 0.3, 0, -0.2],
                             [
                              pl * 0.7, tu * 0.1, 0, -0.2],
                             [
                              pr * 0.7, td * 0.5, 0, -0.2],
                             [
                              pr * 0.7, tu * 0.2, 0, -0.2],
                             [
                              pn, td * 0.8, 0, -0.2],
                             [
                              pn, tu * 0.6, 0, -0.2],
                             [
                              pn, tn, 0, -0.2],
                             [
                              pn, tn, 0, 0]]}, 
       'twirl': {'poses': [
                         [
                          pr * 0.3, tu * 0.3, -1.0],
                         [
                          pr * 0.4, tu * 0.4, -1.0],
                         [
                          pr * 0.5, tu * 0.5, -1.0],
                         [
                          pn, td * 0.1, -1.0],
                         [
                          pr * 0.3, tu * 0.5, -1.0],
                         [
                          pr * 0.4, tu * 0.4, -1.0],
                         [
                          pr * 0.5, tu * 0.3, -1.0],
                         [
                          pn, tn, -1.0],
                         [
                          pl * 0.3, tu * 0.3, 1.0],
                         [
                          pl * 0.4, tu * 0.4, 1.0],
                         [
                          pl * 0.5, tu * 0.5, 1.0],
                         [
                          pn, td * 0.1, 1.0],
                         [
                          pl * 0.3, tu * 0.5, 1.0],
                         [
                          pl * 0.4, tu * 0.4, 1.0],
                         [
                          pl * 0.5, tu * 0.3, 1.0],
                         [
                          pn, tn, 1.0]]}, 
       'this_is_my_jam': {'poses': [
                                  [
                                   pl * 0.7, tu * 0.6, 0, 0, (0, 0), eh],
                                  [
                                   pr * 0.7, tu * 0.6, 0, 0, (0, 0), eh],
                                  [
                                   pl * 0.7, tu * 0.6, 0, 0, (0, 0), eh],
                                  [
                                   pr * 0.7, tu * 0.6, 0, 0, (0, 0), eh],
                                  [
                                   pl * 0.7, tu * 0.6, 0, 0, (0, 0), ec],
                                  [
                                   pr * 0.7, tu * 0.6, 0, 0, (0, 0), ec],
                                  [
                                   pl * 0.7, tu * 0.6, 0, 0, (0, 0), ec],
                                  [
                                   pr * 0.7, tu * 0.6, 0, 0, (0, 0), ec],
                                  [
                                   pl * 0.7, tu * 0.4, 0, 0, (rad(-60), -0.1), eh],
                                  [
                                   pl * 0.5, tn, 0, 0, (rad(-60), -0.1), eh],
                                  [
                                   pl * 0.3, tu * 0.2, 0, 0, (rad(-60), -0.1), eh],
                                  [
                                   pn, tu * 0.7, 0, 0, (rad(-60), -0.1), ec],
                                  [
                                   pl * 0.5, tu * 0.7, 0.4, 0, (0, 0), ec],
                                  [
                                   pl * 0.5, tu * 0.7, 0.0, 0, (0, 0), ec],
                                  [
                                   pr * 0.5, tu * 0.7, -0.4, 0, (0, 0), ec],
                                  [
                                   pr * 0.5, tu * 0.7, 0.0, 0, (0, 0), ec],
                                  [
                                   pl * 0.5, tu * 0.7, 0.4, 0, (0, 0), ec],
                                  [
                                   pl * 0.5, tu * 0.7, 0.0, 0, (0, 0), ec],
                                  [
                                   pr * 0.5, tu * 0.7, -0.4, 0, (0, 0), ec],
                                  [
                                   pn, tu * 0.1, 0.0, 0, (0, 0), en]]}, 
       'moonwalk': {'repeat': 2, 
                    'poses': [
                            [
                             pr * 0.5, td * 0.7, 0, -0.5],
                            [
                             pr * 0.1, td * 0.8, 0, 0],
                            [
                             pl * 0.4, td * 0.6, 0, -0.5],
                            [
                             pn, td * 0.3, 0, 0],
                            [
                             pl * 0.6, tu * 0.5, 0, -0.5],
                            [
                             pl * 0.1, tu * 0.7, 0, 0],
                            [
                             pr * 0.5, tu * 0.4, 0, -0.5],
                            [
                             pn, tu * 0.3, 0, 0]]}, 
       'skate_back': {'poses': [
                              [
                               pl * 0.2, tu * 0.1, 0, 0, (rad(-10), -0.04), eh],
                              [
                               pr * 0.2, tu * 0.1, 0, 0, (rad(10), -0.04), eh],
                              [
                               pl * 0.2, tu * 0.2, 0, 0, (rad(-10), -0.04), eh],
                              [
                               pr * 0.2, tu * 0.2, 0, 0, (rad(10), -0.04), eh],
                              [
                               pl * 0.2, tu * 0.1, 0, 0, (rad(-10), -0.04)],
                              [
                               pr * 0.2, tu * 0.1, 0, 0, (rad(10), -0.04)],
                              [
                               pr * 0.2, tu * 0.2, 0, 0, (rad(-10), -0.04)],
                              [
                               pl * 0.2, tu * 0.2, 0, 0, (rad(10), -0.04), en]]}, 
       'clover': {'poses': [
                          [
                           pr * 0.2, tu * 0.3, 0, 0, (rad(-90), 0.1), eh],
                          [
                           pr * 0.3, tu * 0.4, 0, 0, (rad(-90), 0.1), eh],
                          [
                           pr * 0.4, tu * 0.5, 0, 0, (rad(-90), 0.1), eh],
                          [
                           pr * 0.5, tu * 0.5, 0, 0, (rad(-90), 0.1), eh],
                          [
                           pr * 0.5, tu * 0.4, 0, 0, (rad(-90), -0.1), eh],
                          [
                           pr * 0.3, tu * 0.4, 0, 0, (rad(-90), -0.1), eh],
                          [
                           pr * 0.2, tu * 0.3, 0, 0, (rad(-90), -0.1), eh],
                          [
                           pr * 0.1, tu * 0.2, 0, 0, (rad(-90), -0.1), en],
                          [
                           pl * 0.1, tu * 0.2, 0, 0, (rad(90), 0.1)],
                          [
                           pl * 0.2, tu * 0.3, 0, 0, (rad(90), 0.1)],
                          [
                           pl * 0.3, tu * 0.5, 0, 0, (rad(90), 0.1)],
                          [
                           pl * 0.4, tu * 0.6, 0, 0, (rad(90), 0.1)],
                          [
                           pl * 0.5, tu * 0.7, 0, 0, (rad(90), -0.1)],
                          [
                           pl * 0.4, tu * 0.6, 0, 0, (rad(90), -0.1)],
                          [
                           pl * 0.3, tu * 0.5, 0, 0, (rad(90), -0.1)],
                          [
                           pl * 0.2, tu * 0.4, 0, 0, (rad(90), -0.1)],
                          [
                           pr * 0.2, tu * 0.3, 0, 0, (rad(-90), -0.1)],
                          [
                           pr * 0.4, tu * 0.5, 0, 0, (rad(-90), -0.1)],
                          [
                           pr * 0.5, tu * 0.6, 0, 0, (rad(-90), -0.1)],
                          [
                           pr * 0.6, tu * 0.7, 0, 0, (rad(-90), -0.1)],
                          [
                           pr * 0.7, tu * 0.8, 0, 0, (rad(-90), 0.1)],
                          [
                           pr * 0.6, tu * 0.6, 0, 0, (rad(-90), 0.1)],
                          [
                           pr * 0.3, tu * 0.4, 0, 0, (rad(-90), 0.1)],
                          [
                           pr * 0.2, tu * 0.3, 0, 0, (rad(-90), 0.1)],
                          [
                           pl * 0.1, tu * 0.2, 0, 0, (rad(90), -0.1)],
                          [
                           pl * 0.2, tu * 0.1, 0, 0, (rad(90), -0.1)],
                          [
                           pl * 0.3, tn, 0, 0, (rad(90), -0.1)],
                          [
                           pl * 0.4, td * 0.2, 0, 0, (rad(90), -0.1), eh],
                          [
                           pl * 0.3, td * 0.3, 0, 0, (rad(90), 0.1), eh],
                          [
                           pl * 0.2, td * 0.5, 0, 0, (rad(90), 0.1), eh],
                          [
                           pl * 0.1, td * 0.3, 0, 0, (rad(90), 0.1), eh],
                          [
                           pn, td * 0.2, 0, 0, (rad(90), 0.1), en]]}, 
       'done_dummy': {'repeat': 8, 
                      'poses': [
                              [
                               pn, tn],
                              [
                               pn, tn],
                              [
                               pn, tn],
                              [
                               pn, tn]]}}
    return routines


def dance_routine_names():
    """
    Returns a list of valid dance routine names.
    """
    return list(dance_routines().keys())


def choreographed_performances():
    """
    Get the intro order of dance routines
    """
    return CHOREOGRAPHED_SONGS