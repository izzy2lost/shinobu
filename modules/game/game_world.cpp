#include "game_world.h"
#include "core/config/project_settings.h"
#include "modules/game/player_agent.h"
#include "scene/main/canvas_layer.h"
#include "scene/resources/packed_scene.h"

#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui_macros.h"
#endif

CCommand GameWorldState::trigger_alert_cc = CCommand("trigger_alert");

void HBGameWorld::_on_node_added(Node *p_node) {
	p_node->notification(NOTIFICATION_HB_ENTER_GAME_WORLD);
}

void HBGameWorld::_on_node_removed(Node *p_node) {
	p_node->notification(NOTIFICATION_HB_EXIT_GAME_WORLD);
}

void HBGameWorld::set_player(HBPlayerAgent *p_player) {
	DEV_ASSERT(p_player != nullptr);
	player = p_player;
	world_state->set_player(player);
	game_ui->set_player_agent(player);
}

void HBGameWorld::set_player_start_transform(const Transform3D &p_transform) {
	player_start_transform = p_transform;
}

void HBGameWorld::spawn_player() {
	float starting_heading = 0.0f;
	Vector3 starting_position;

	// Find info_player_start, or use defaults if we can't find it
	HBInfoPlayerStart *ips = Object::cast_to<HBInfoPlayerStart>(get_tree()->get_first_node_in_group("info_player_start"));

	if (ips) {
		starting_position = ips->get_global_position();
		starting_heading = ips->get_global_basis().get_euler().y;
	}

	String player_scene_path = GLOBAL_GET("game/player/player_scene");
	DEV_ASSERT(!player_scene_path.is_empty());
	Ref<PackedScene> packed_scene = ResourceLoader::load(player_scene_path);
	DEV_ASSERT(packed_scene.is_valid());
	Node *player_node = packed_scene->instantiate();
	HBPlayerAgent *player_agent_node = Object::cast_to<HBPlayerAgent>(player_node);
	DEV_ASSERT(player_agent_node);

	player_agent_node->set_starting_heading(starting_heading);
	player_agent_node->set_position(starting_position);
	add_child(player_agent_node);
}

Ref<GameWorldState> HBGameWorld::get_game_world_state() const {
	return world_state;
}

void HBGameWorld::_notification(int p_what) {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

#ifdef DEBUG_ENABLED
	_debug_notification(p_what);
#endif

	switch (p_what) {
		case NOTIFICATION_READY: {
		} break;
		case NOTIFICATION_ENTER_TREE: {
			SceneTree::get_singleton()->connect("node_added", callable_mp(this, &HBGameWorld::_on_node_added));
			SceneTree::get_singleton()->connect("node_removed", callable_mp(this, &HBGameWorld::_on_node_removed));
		} break;
		case NOTIFICATION_EXIT_TREE: {
			SceneTree::get_singleton()->disconnect("node_added", callable_mp(this, &HBGameWorld::_on_node_added));
			SceneTree::get_singleton()->disconnect("node_removed", callable_mp(this, &HBGameWorld::_on_node_removed));
		} break;
	}
}
#ifdef DEBUG_ENABLED
void HBGameWorld::_debug_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			REGISTER_DEBUG(this);
		} break;
		case NOTIFICATION_EXIT_TREE: {
			UNREGISTER_DEBUG(this);
		} break;
		case NOTIFICATION_PROCESS: {
			GodotImGui *gim = GodotImGui::get_singleton();
			if (gim && gim->is_debug_enabled(this)) {
				if (gim->begin_debug_window(this)) {
					String alert_strname;
					switch (world_state->get_alert_status()) {
						case GameWorldState::ALERT_CLEAR: {
							alert_strname = "Clear";
						} break;
						case GameWorldState::ALERT_LOST: {
							alert_strname = "Lost";
						} break;
						case GameWorldState::ALERT_SEEN: {
							alert_strname = "Seen";
						} break;
					}
					ImGui::Text("Alert status %s", alert_strname.utf8().get_data());
					if (ImGui::Button("Toggle alert")) {
						if (world_state->get_alert_status() == GameWorldState::ALERT_CLEAR) {
							world_state->set_alert_status(GameWorldState::ALERT_SEEN);
						} else {
							world_state->set_alert_status(GameWorldState::ALERT_CLEAR);
						}
					}
				}
				ImGui::End();
			}
		} break;
	}
}
#endif

HBGameWorld::HBGameWorld() {
	world_state.instantiate();
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_process(true);
	}
	String ingame_ui_scene_path = GLOBAL_GET("game/ingame_ui_scene");
	Ref<PackedScene> scene = ResourceLoader::load(ingame_ui_scene_path);
	DEV_ASSERT(scene.is_valid());
	Node *scene_node = scene->instantiate();
	DEV_ASSERT(scene_node);
	HBInGameUI *game_ui_candidate = Object::cast_to<HBInGameUI>(scene_node);
	DEV_ASSERT(game_ui_candidate);
	ui_canvas_layer = memnew(CanvasLayer);
	add_child(ui_canvas_layer);
	ui_canvas_layer->add_child(game_ui_candidate);
	game_ui = game_ui_candidate;
}

void GameWorldState::_bind_methods() {
	ADD_SIGNAL(MethodInfo("agent_entered_combat", PropertyInfo(Variant::OBJECT, "agent", PROPERTY_HINT_NODE_TYPE, "HBAgent")));
	ADD_SIGNAL(MethodInfo("agent_exited_combat", PropertyInfo(Variant::OBJECT, "agent", PROPERTY_HINT_NODE_TYPE, "HBAgent")));
}

void GameWorldState::_on_agent_entered_combat(HBAgent *p_agent) {
	agents_in_combat.insert(p_agent->get_instance_id());
	emit_signal(SNAME("agent_entered_combat"), p_agent);
}

void GameWorldState::_on_agent_exited_combat(HBAgent *p_agent) {
	if (agents_in_combat.has(p_agent->get_instance_id())) {
		agents_in_combat.erase(p_agent->get_instance_id());
	}
	emit_signal(SNAME("agent_exited_combat"), p_agent);
}

void GameWorldState::agent_entered_tree(HBAgent *p_agent) {
	p_agent->connect("entered_combat", callable_mp(this, &GameWorldState::_on_agent_entered_combat).bind(p_agent));
	p_agent->connect("exited_combat", callable_mp(this, &GameWorldState::_on_agent_exited_combat).bind(p_agent));
}

void GameWorldState::agent_exited_tree(HBAgent *p_agent) {
	p_agent->disconnect("entered_combat", callable_mp(this, &GameWorldState::_on_agent_entered_combat).bind(p_agent));
	p_agent->disconnect("exited_combat", callable_mp(this, &GameWorldState::_on_agent_exited_combat).bind(p_agent));
	if (agents_in_combat.has(p_agent->get_instance_id())) {
		agents_in_combat.erase(p_agent->get_instance_id());
	}
}

void GameWorldState::set_player(HBPlayerAgent *p_player) {
	ERR_FAIL_COND(p_player == nullptr);
	player = p_player;
};

GameWorldState::GameWorldState() {
	trigger_alert_cc.data->get_signaler()->connect("executed", callable_mp(this, &GameWorldState::set_alert_status).bind(ALERT_SEEN));
}
