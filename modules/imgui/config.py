def can_build(env, platform):
    if not env.debug_features:
        return False
    return platform == "windows" or platform == "linuxbsd"


def configure(env):
    pass
