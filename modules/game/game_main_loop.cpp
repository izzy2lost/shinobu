/**************************************************************************/
/*  game_main_loop.cpp                                                    */
/**************************************************************************/
/*                         This file is part of:                          */
/*                               SWANSONG                                 */
/*                          https://eirteam.moe                           */
/**************************************************************************/
/* Copyright (c) 2023-present Álex Román Núñez (EIRTeam).                 */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "game_main_loop.h"

#include "modules/game/console.h"
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

#include "modules/game/game_goap.h"
#include "modules/input_glyphs/input_glyphs_singleton.h"
#include "modules/input_glyphs/input_glyphs_source.h"
#include "servers/navigation_server_3d.h"

HBConsole *console = nullptr;
HBGameMainLoop *HBGameMainLoop::singleton = nullptr;
CCommand HBGameMainLoop::quit_command = CCommand("quit");

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
	singleton = this;
}

HBGameWorld *HBGameMainLoop::get_game_world() const {
	return game_world;
}

void HBGameMainLoop::change_scene(Node *p_new_scene) {
	ERR_FAIL_COND(!p_new_scene);
	if (get_current_scene()) {
		unload_current_scene();
		set_current_scene(nullptr);
	}
	get_root()->add_child(p_new_scene);
	set_current_scene(p_new_scene);
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
	console = nullptr;
}

void HBGameMainLoop::enable_fp_exceptions() {
#ifdef LINUXBSD_ENABLED
	feenableexcept(FE_INVALID);
#endif
}

HBGameMainLoop *HBGameMainLoop::get_singleton() {
	return singleton;
}

void HBGameMainLoop::initialize() {
	game_world = memnew(HBGameWorld);
	get_root()->add_child(game_world);
	SceneTree::initialize();
	if (get_current_scene() && get_current_scene()->get_scene_file_path().begins_with("res://maps")) {
		game_world->spawn_player();
	} else {
		memdelete(game_world);
	}
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

	console = memnew(HBConsole);
	get_root()->add_child(console);

	/*Node *current_scene = get_current_scene();
	// Maps have to be remapped
	if (current_scene) {
		String path = current_scene->get_scene_file_path();
		if (path.begins_with("res://maps/")) {
			HBGameWorld *world = memnew(HBGameWorld);
			get_root()->remove_child(current_scene);
			world->set_scene_file_path("WORLD:" + path);
			change_scene(world);
			world->add_child(current_scene);
		}
	}*/

	quit_command.data->get_signaler()->connect("executed", callable_mp((SceneTree *)this, &SceneTree::quit).bind(EXIT_SUCCESS));
}