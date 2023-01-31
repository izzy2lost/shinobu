def can_build(env, platform):
    if env.debug_features:
        env.module_add_dependencies("game", ["imgui"])
    return platform == "windows" or platform == "linuxbsd"


def configure(env):
    pass
