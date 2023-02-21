def can_build(env, platform):
    if env.debug_features:
        env.module_add_dependencies("game", ["imgui"])
    return platform == "windows" or platform == "linuxbsd"


def configure(env):
    pass


def get_doc_path():
    return "doc_classes"


def get_doc_classes():
    return [
        "HBActor",
        "HBPlayerActor",
        "HBPlayerCameraArm",
        "EPASPose",
        "EPASNode",
        "EPASBlendNode",
        "EPASAddNode",
        "EPASPoseNode",
        "EPASController",
        "EPASWheelLocomotion",
        "EPASKeyframe",
        "EPASAnimation",
        "HBUtils",
        "EPASIKNode",
        "HBGameMainLoop",
    ]
