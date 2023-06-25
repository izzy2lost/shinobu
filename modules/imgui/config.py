def can_build(env, platform):
    if not env.debug_features:
        return False
    env.module_add_dependencies("imgui", ["steamworks"])
    return platform == "windows" or platform == "linuxbsd"


def configure(env):
    pass
