#include "register_types.h"

#include "core/config/project_settings.h"

#include "actor.h"
#include "game_main_loop.h"
#include "player_actor.h"
#include "player_camera_arm.h"
#include "scene/main/scene_tree.h"
#include "scene/main/window.h"

void initialize_game_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GLOBAL_DEF("game/mouse_sensitivity", 175.0f);
		GLOBAL_DEF("game/player/graphics_rotation_speed", 45.0f);
		GDREGISTER_CLASS(HBActor);
		GDREGISTER_CLASS(HBPlayerActor);
		GDREGISTER_CLASS(HBPlayerCameraArm);
		GDREGISTER_CLASS(HBGameMainLoop);
	}
}

void uninitialize_game_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}
