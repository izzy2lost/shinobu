#include "register_types.h"

#include "agent_string_names.h"
#include "core/config/project_settings.h"

#include "agent.h"
#include "agent_constants.h"
#include "agent_parkour.h"
#include "agent_state.h"
#include "core/object/class_db.h"
#include "fabrik/fabrik.h"
#include "game_main_loop.h"
#include "jolt_character_body.h"
#include "ledge_traversal_controller.h"
#include "modules/game/animation_system/epas_animation.h"
#include "modules/game/animation_system/epas_ik_node.h"
#include "modules/game/animation_system/epas_lookat_node.h"
#include "modules/game/level_preprocessor.h"
#include "modules/game/map_compile_hooks.h"
#include "modules/tbloader/src/tb_loader_singleton.h"
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
#include "animation_system/epas_softness_node.h"
#include "animation_system/epas_transition_node.h"
#include "animation_system/epas_wheel_locomotion.h"

AgentStringNames *agent_string_names = nullptr;

void initialize_game_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	GLOBAL_DEF("game/mouse_sensitivity", 175.0f);
	GLOBAL_DEF("game/player/graphics_rotation_speed", 45.0f);
	// Agent stuff
	GDREGISTER_CLASS(HBAgent);
	GDREGISTER_CLASS(HBAgentConstants);
	GDREGISTER_CLASS(HBPlayerAgent);
	GDREGISTER_CLASS(HBPlayerAgentController);
	GDREGISTER_CLASS(HBAgentParkourPoint);
	GDREGISTER_CLASS(HBAgentParkourBeam);
	// Agent states
	GDREGISTER_ABSTRACT_CLASS(HBAgentState);
	GDREGISTER_CLASS(HBAgentMoveState);
	GDREGISTER_CLASS(HBAgentTurnState);
	GDREGISTER_CLASS(HBAgentLedgeGrabbedStateNew);
	GDREGISTER_CLASS(HBAgentFallState);
	GDREGISTER_CLASS(HBAgentWallParkourState);
	GDREGISTER_CLASS(HBAgentWallParkourStateNew);
	GDREGISTER_CLASS(HBAgentParkourBeamWalk);
	GDREGISTER_CLASS(HBAgentRootMotionState);
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
	GDREGISTER_CLASS(EPASSoftnessNode);
	GDREGISTER_CLASS(EPASLookatNode);
	GDREGISTER_CLASS(EPASIKNode);
	GDREGISTER_CLASS(FABRIKSolver);
	GDREGISTER_CLASS(HBDebugGeometry);
	GDREGISTER_CLASS(HBInfoPlayerStart);

	// Tests...
	GDREGISTER_CLASS(JoltCharacterBody3D);
	GDREGISTER_CLASS(HBLedgeTraversalController);
	GDREGISTER_CLASS(HBLevelPreprocessor);
	GDREGISTER_CLASS(HBAgentParkourLedge);
	GDREGISTER_CLASS(EPASEditorAnimation);
	GDREGISTER_CLASS(RotationInertializer);

	TBLoaderSingleton::register_entity_type<HBAgentParkourPoint>();
	TBLoaderSingleton::register_entity_type<HBAgentParkourBeam>();
	TBLoaderSingleton::register_entity_type<HBInfoPlayerStart>();
	TBLoaderSingleton::register_compile_hook(memnew(MapCompileHooks));
#ifdef DEBUG_ENABLED
	GDREGISTER_CLASS(EPASAnimationEditor);
	GDREGISTER_CLASS(EPASEditorGrid);
	GDREGISTER_CLASS(EPASEditorCamera);
#endif
	agent_string_names = memnew(AgentStringNames);
}

void uninitialize_game_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	if (agent_string_names) {
		memdelete(agent_string_names);
	}
}
