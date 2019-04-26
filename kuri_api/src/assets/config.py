import os

def get_data_path():
    """
    Return the data path for either the GBZ or a development workspace.
    """
    if os.path.isdir("/opt/gizmo"):
        return os.path.join('/opt/gizmo', 'share/assets')
    return "/tmp"


def get_assets_path():
    """
    Path to assets
    """
    return get_data_path()


def get_movies_path():
    return os.path.join(get_assets_path(), 'movies')


def get_sounds_path():
    return os.path.join(get_assets_path(), 'sounds')


def get_music_path():
    return os.path.join(get_assets_path(), 'music')


def get_anims_path():
    return os.path.join(get_assets_path(), 'anims')
