def can_build(env, platform):
    if env.debug_features:
        env.module_add_dependencies("game", ["imgui"])
    env.module_add_dependencies("game", ["tracy"])
    env.module_add_dependencies("game", ["steamworks"])
    env.module_add_dependencies("game", ["tbloader"])
    return platform == "windows" or platform == "linuxbsd"


def configure(env):
    pass


def get_doc_path():
    return "doc_classes"


def get_doc_classes():
    return [
        "HBAgent",
        "HBPlayerAgent",
        "HBPlayerAgentController",
        "HBPlayerCameraArm",
        "EPASPose",
        "EPASNode",
        "EPASBlendNode",
        "EPASAddNode",
        "EPASPoseNode",
        "EPASController",
        "EPASOneshotAnimationNode",
        "EPASAnimationNode",
        "EPASWheelLocomotion",
        "EPASInertializationNode",
        "EPASTransitionNode",
        "EPASKeyframe",
        "EPASAnimation",
        "HBUtils",
        "EPASIKNode",
        "HBGameMainLoop",
        "HBAgentConstants",
        "HBStateMachine",
        "FABRIKSolver",
        "HBAgentState",
    ]
