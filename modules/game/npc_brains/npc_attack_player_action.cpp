/**************************************************************************/
/*  npc_attack_player_action.cpp                                          */
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

#include "npc_attack_player_action.h"
#include "../player_agent.h"
#include "npc_brain_constants.h"

bool NPCAttackPlayerAction::is_valid(const Ref<GameWorldState> &p_world_state) const {
	bool is_close_to_player = p_world_state->get_player()->get_position().distance_to(get_brains()->get_agent()->get_global_position()) <= NPCBrainConstants::ATTACK_RANGE_TARGET_DISTANCE;
	return !attack_aborted && (is_close_to_player || delay_timer.is_valid());
}

void NPCAttackPlayerAction::set_prerequisites(actionplanner_t *p_action_planner) const {
	goap_set_pre(p_action_planner, get_action_name(), NPCBrainConstants::IN_ATTACK_RANGE_TO_PLAYER, true);
	goap_set_pre(p_action_planner, get_action_name(), NPCBrainConstants::PLAYER_DEAD_ATOM, false);
}

void NPCAttackPlayerAction::set_effects(actionplanner_t *p_action_planner) const {
	goap_set_pst(p_action_planner, get_action_name(), NPCBrainConstants::PLAYER_DEAD_ATOM, true);
}

const char *NPCAttackPlayerAction::get_action_name() const {
	return NPCBrainConstants::ATTACK_PLAYER_ACTION_NAME;
}

void NPCAttackPlayerAction::on_attack_connected() {
	attack_connected = true;
	HBAgent *agent = get_brains()->get_agent();
	delay_timer = agent->get_tree()->create_timer(2, false);
	agent->set_input_action_state(HBAgent::INPUT_ACTION_ATTACK, false);
}

void NPCAttackPlayerAction::on_attack_aborted() {
	attack_aborted = true;
}

bool NPCAttackPlayerAction::execute(const Ref<GameWorldState> &p_world_state) {
	if (delay_timer.is_valid() && delay_timer->get_time_left() == 0.0) {
		delay_timer.unref();
	}
	if (!delay_timer.is_valid() && attack_connected) {
		return true;
	}
	return false;
}

void NPCAttackPlayerAction::enter(const Ref<GameWorldState> &p_world_state) {
	attack_aborted = false;
	attack_connected = false;
	HBAgent *agent = get_brains()->get_agent();
	agent->set_target(Object::cast_to<HBAgent>(p_world_state->get_player()));
	// TODO: Perhaps for these things we should just be switching states manually, who knows, this is what FEAR did with its animations
	agent->set_input_action_state(HBAgent::INPUT_ACTION_ATTACK, true);
	agent->connect("attack_connected", callable_mp(this, &NPCAttackPlayerAction::on_attack_connected));
	agent->connect("attack_aborted", callable_mp(this, &NPCAttackPlayerAction::on_attack_aborted));
}

void NPCAttackPlayerAction::exit(const Ref<GameWorldState> &p_world_state) {
	HBAgent *agent = get_brains()->get_agent();
	agent->set_target(nullptr);
	agent->set_input_action_state(HBAgent::INPUT_ACTION_ATTACK, false);
	agent->disconnect("attack_connected", callable_mp(this, &NPCAttackPlayerAction::on_attack_connected));
	agent->disconnect("attack_aborted", callable_mp(this, &NPCAttackPlayerAction::on_attack_aborted));
}
