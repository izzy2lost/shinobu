#include "game_main_loop.h"

#include "modules/imgui/register_types.h"

HBGameMainLoop::HBGameMainLoop() :
		SceneTree() {
	imgui_module_post_init();
}

void HBGameMainLoop::initialize() {
	SceneTree::initialize();
}