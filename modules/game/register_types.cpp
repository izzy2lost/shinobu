#include "register_types.h"

#include "core/config/project_settings.h"

#include "agent.h"
#include "agent_constants.h"
#include "agent_parkour.h"
#include "agent_state.h"
#include "core/object/class_db.h"
#include "fabrik/fabrik.h"
#include "game_main_loop.h"
#include "modules/game/animation_system/epas_animation.h"
#include "modules/game/animation_system/epas_ik_node.h"
#include "player_agent.h"
#include "player_camera_arm.h"
#include "scene/main/scene_tree.h"
#include "scene/main/window.h"
#include "state_machine.h"
#ifdef DEBUG_ENABLED
#include "animation_system/epas_editor_camera.h"
#include "animation_system/epas_editor_grid.h"
#endif

#include "animation_system/epas_add_node.h"
#include "animation_system/epas_animation_editor.h"
#include "animation_system/epas_animation_node.h"
#include "animation_system/epas_blend_node.h"
#include "animation_system/epas_controller.h"
#include "animation_system/epas_inertialization_node.h"
#include "animation_system/epas_node.h"
#include "animation_system/epas_oneshot_animation_node.h"
#include "animation_system/epas_pose.h"
#include "animation_system/epas_pose_node.h"
#include "animation_system/epas_transition_node.h"
#include "animation_system/epas_wheel_locomotion.h"

void initialize_game_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GLOBAL_DEF("game/mouse_sensitivity", 175.0f);
		GLOBAL_DEF("game/player/graphics_rotation_speed", 45.0f);
		// Agent stuff
		GDREGISTER_CLASS(HBAgent);
		GDREGISTER_CLASS(HBAgentConstants);
		GDREGISTER_CLASS(HBPlayerAgent);
		GDREGISTER_CLASS(HBPlayerAgentController);
		GDREGISTER_CLASS(HBAgentParkourPoint);
		// Agent states
		GDREGISTER_ABSTRACT_CLASS(HBAgentState);
		GDREGISTER_CLASS(HBAgentMoveState);
		GDREGISTER_CLASS(HBAgentVaultState);
		GDREGISTER_CLASS(HBAgentTurnState);
		GDREGISTER_CLASS(HBAgentWallrunState);
		GDREGISTER_CLASS(HBAgentWallGrabbedState);
		GDREGISTER_CLASS(HBAgentFallState);
		GDREGISTER_CLASS(HBAgentLedgeGetUpState);
		GDREGISTER_CLASS(HBAgentWallParkourState);
		// State machine stuff
		GDREGISTER_CLASS(HBStateMachine);
		GDREGISTER_ABSTRACT_CLASS(HBStateMachineState);

		GDREGISTER_CLASS(HBPlayerCameraArm);
		GDREGISTER_CLASS(HBGameMainLoop);
		// EPAS
		GDREGISTER_CLASS(EPASPose);
		GDREGISTER_ABSTRACT_CLASS(EPASNode);
		GDREGISTER_CLASS(EPASBlendNode);
		GDREGISTER_CLASS(EPASAddNode);
		GDREGISTER_CLASS(EPASInertializationNode);
		GDREGISTER_CLASS(EPASTransitionNode);
		GDREGISTER_CLASS(EPASPoseNode);
		GDREGISTER_CLASS(EPASWheelLocomotion);
		GDREGISTER_CLASS(EPASWheelVisualizer);
		GDREGISTER_CLASS(EPASController);
		GDREGISTER_CLASS(EPASAnimation);
		GDREGISTER_CLASS(EPASWarpPoint);
		GDREGISTER_CLASS(EPASKeyframe);
		GDREGISTER_CLASS(EPASAnimationNode);
		GDREGISTER_CLASS(EPASOneshotAnimationNode);
		GDREGISTER_CLASS(EPASOneshotAnimationNodeDebug);
		GDREGISTER_CLASS(EPASIKNode);
		GDREGISTER_CLASS(FABRIKSolver);
#ifdef DEBUG_ENABLED
		GDREGISTER_CLASS(EPASAnimationEditor);
		GDREGISTER_CLASS(EPASEditorGrid);
		GDREGISTER_CLASS(EPASEditorCamera);
#endif
	}
}

void uninitialize_game_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}
