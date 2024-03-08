#include "npc_approach_player_action.h"
#include "modules/game/player_agent.h"

void GOAPApproachPlayerAction::_update_target_point(const HBAgent *p_player) {
	const HBAgent *us = get_brains()->get_agent();
	Vector3 dir_to_player = us->get_global_position().direction_to(p_player->get_global_position());
	target_point = p_player->get_global_position() + (-dir_to_player) * action_settings.target_distance;
	get_brains()->get_agent()->set_navigation_target_position(target_point);
	// TODO: Check if we can fit in target point
}

void GOAPApproachPlayerAction::set_prerequisites(actionplanner_t *p_action_planner) const {
	goap_set_pre(p_action_planner, get_action_name(), action_settings.atom_name, false);
}

void GOAPApproachPlayerAction::set_effects(actionplanner_t *p_action_planner) const {
	goap_set_pst(p_action_planner, get_action_name(), action_settings.atom_name, true);
}

const char *GOAPApproachPlayerAction::get_action_name() const {
	return action_settings.action_name;
}

bool GOAPApproachPlayerAction::is_valid(const Ref<GameWorldState> &p_world_state) const {
	return true;
}

bool GOAPApproachPlayerAction::execute(const Ref<GameWorldState> &p_world_state) {
	HBNPCAgent *agent = get_brains()->get_agent();

	HBPlayerAgent *player = p_world_state->get_player();
	if (target_point.distance_to(player->get_global_position()) > action_settings.player_max_target_deviation) {
		_update_target_point(player);
	}

	if (agent->move_to_target(agent->get_process_delta_time(), true)) {
		return true;
	}

	return false;
}

GOAPApproachPlayerAction::GOAPApproachPlayerAction(const ApproachPlayerActionSettings &p_settings, NPCBrains *p_brains) :
		GOAPActionNPC(p_brains) {
	action_settings = p_settings;
}

void GOAPApproachPlayerAction::enter(const Ref<GameWorldState> &p_world_state) {
	_update_target_point(p_world_state->get_player());
	if (action_settings.should_target_player) {
		get_brains()->get_agent()->set_target(p_world_state->get_player());
	} else {
		get_brains()->get_agent()->set_target(nullptr);
	}
}

void GOAPApproachPlayerAction::exit(const Ref<GameWorldState> &p_world_state) {
	get_brains()->get_agent()->set_target(nullptr);
}
