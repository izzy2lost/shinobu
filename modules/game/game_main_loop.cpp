#include "game_main_loop.h"

#include "modules/imgui/register_types.h"

#ifdef DEBUG_ENABLED
#include "animation_system/epas_animation_editor.h"
#include "animation_system/epas_pose_editor.h"
#endif

HBGameMainLoop::HBGameMainLoop() :
		SceneTree() {
	imgui_module_post_init();
}

void HBGameMainLoop::change_scene(Node *p_new_scene) {
	ERR_FAIL_COND(!p_new_scene);
	if (get_current_scene()) {
		memdelete(get_current_scene());
		set_current_scene(nullptr);
	}
	get_root()->add_child(p_new_scene);
	set_current_scene(p_new_scene);
}

void HBGameMainLoop::initialize() {
	SceneTree::initialize();
#ifdef DEBUG_ENABLED
	List<String> args = OS::get_singleton()->get_cmdline_args();
	for (int i = 0; i < args.size() - 1; i++) {
		if (args[i] == "--hb-tool") {
			String tool_name = args[i + 1];
			Node *tool = nullptr;
			if (tool_name == "pose_editor") {
				tool = memnew(EPASPoseEditor);
			} else if (tool_name == "animation_editor") {
				tool = memnew(EPASAnimationEditor);
			}

			if (tool) {
				change_scene(tool);
			}
		}
	}
#endif
}