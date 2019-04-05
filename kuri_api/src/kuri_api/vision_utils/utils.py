import numpy as np, operator
from functools import reduce

def cast_nan(item, cast_to):
    try:
        if np.isnan(item):
            return cast_to
        return item
    except Exception:
        return item


def get_key_ref_by_path(d, key_path):
    try:
        return reduce(operator.getitem, key_path, d)
    except KeyError:
        return

    return


def set_key_by_path(d, key_path, val):
    get_key_ref_by_path(d, key_path[:-1])[key_path[-1]] = val
