import functools, hashlib, os, cPickle as pickle

def _sha1(s):
    return hashlib.sha1(s).hexdigest()


def _keyhash(key):
    return _sha1(key)


def _store(cache_dir, key, val):
    hash = _keyhash(key)
    objd = os.path.join(cache_dir, hash[:2])
    if not os.path.isdir(objd):
        os.makedirs(objd)
    with open(os.path.join(objd, hash[2:]), 'w') as (f):
        pickle.dump(val, f)


def _load(cache_dir, key):
    """ :raise: IOError if the key is not in the store
                EOFError if the cache is corrupted
    """
    hash = _keyhash(key)
    objd = os.path.join(cache_dir, hash[:2])
    with open(os.path.join(objd, hash[2:])) as (f):
        return pickle.load(f)


def memoize(root_dir):
    """ A better memoize:
         - pickle objects instead of yaml
         - one file per object instead of a big dict
         - objects are lazily loaded from disk when first used
    """
    rtd = root_dir() if callable(root_dir) else root_dir
    cache_dir = os.path.join(rtd, 'cache')

    def memoize_helper(f):
        cache = {}

        @functools.wraps(f)
        def cached(*args, **kwargs):
            key = ('{}{}{}').format(f.func_name, pickle.dumps(args), pickle.dumps(kwargs))
            if key not in cache:
                try:
                    cache[key] = _load(cache_dir, key)
                except (IOError, EOFError):
                    val = f(*args, **kwargs)
                    cache[key] = val
                    _store(cache_dir, key, val)

            return cache[key]

        return cached

    return memoize_helper
