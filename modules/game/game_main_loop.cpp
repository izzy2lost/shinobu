#include "game_main_loop.h"

#include "modules/imgui/register_types.h"

#ifdef DEBUG_ENABLED
#include "animation_system/epas_animation_editor.h"
#endif

#ifdef LINUXBSD_ENABLED
#define _GNU_SOURCE 1
#include <fenv.h>
#endif

#include "modules/tracy/tracy.gen.h"

#include "modules/modules_enabled.gen.h"

#ifdef MODULE_STEAMWORKS_ENABLED
#include "modules/steamworks/input_glyphs_steamworks.h"
#include "modules/steamworks/steamworks.h"
#endif

#include "modules/input_glyphs/input_glyphs_singleton.h"
#include "modules/input_glyphs/input_glyphs_source.h"

HBGameMainLoop::HBGameMainLoop() {
#ifdef DEBUG_ENABLED
	imgui_module_post_init();
#endif
#ifdef MODULE_STEAMWORKS_ENABLED
	Steamworks::get_singleton()->init(1216230);
	if (Steamworks::get_singleton()->is_valid()) {
		Steamworks::get_singleton()->get_input()->init();
		HBSteamworksInputGlyphsSource::make_current();
		print_line("MAKING SOURCE CURRENT!");
	}
	InputGlyphsSingleton::get_singleton()->init();
#endif
}

void HBGameMainLoop::change_scene(Node *p_new_scene) {
	ERR_FAIL_COND(!p_new_scene);
	if (get_current_scene()) {
		get_current_scene()->queue_free();
		set_current_scene(nullptr);
	}
	add_current_scene(p_new_scene);
}

bool HBGameMainLoop::process(double p_time) {
	ZoneScopedN("Process Frame");
	bool result = SceneTree::process(p_time);
	if (Steamworks::get_singleton()->get_input()) {
		Steamworks::get_singleton()->get_input()->run_frame();
		Steamworks::get_singleton()->run_callbacks();
	}
	return result;
}

bool HBGameMainLoop::physics_process(double p_time) {
	ZoneScopedN("Physics Frame");
	bool result = SceneTree::physics_process(p_time);
	return result;
}

void HBGameMainLoop::finalize() {
	SceneTree::finalize();
}

void HBGameMainLoop::enable_fp_exceptions() {
#ifdef LINUXBSD_ENABLED
	feenableexcept(FE_INVALID);
#endif
}

void HBGameMainLoop::initialize() {
	SceneTree::initialize();
#ifdef DEBUG_ENABLED
	List<String> args = OS::get_singleton()->get_cmdline_args();
	for (int i = 0; i < args.size() - 1; i++) {
		if (args[i] == "--hb-tool") {
			String tool_name = args[i + 1];
			Node *tool = nullptr;
			if (tool_name == "animation_editor") {
				tool = memnew(EPASAnimationEditor);
			}

			if (tool) {
				callable_mp(this, &HBGameMainLoop::change_scene).bind(tool).call_deferred();
			}
		} else if (args[i] == "--dump-steamworks-input-glyphs") {
#ifdef MODULE_STEAMWORKS_ENABLED
			callable_mp_static(HBSteamworksInputGlyphDumpTool::dump).call_deferred(args[i + 1]);
#endif // MODULE_STEAMWORKS_ENABLED
		};
	}
#endif
}