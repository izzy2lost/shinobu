/**************************************************************************/
/*  game_goap.cpp                                                         */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
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

#include "game_goap.h"
#include "modules/game/player_agent.h"
#include "modules/imgui/godot_imgui.h"
#include "npc_brains/npc_brain_constants.h"
#include "thirdparty/goap/astar.h"

CVar GOAPActionPlanner::goap_debugger_enabled = CVar("goap_debugger_enabled", Variant::BOOL, false);

void GOAPActionPlanner::get_goap_state_from_game_state(worldstate_t *p_state) {
	goap_worldstate_clear(p_state);
	// A bit of a hack, but patrol is never done
	goap_worldstate_set(&action_planner, p_state, PATROL_DONE_ATOM_NAME, false);

	Vector3 player_pos = world_state->get_player()->get_global_position();
	Vector3 agent_pos = agent->get_global_position();

	bool is_in_attack_range = player_pos.distance_to(agent_pos) < NPCBrainConstants::ATTACK_RANGE_TARGET_DISTANCE;
	goap_worldstate_set(&action_planner, p_state, NPCBrainConstants::IN_ATTACK_RANGE_TO_PLAYER, is_in_attack_range);
	bool is_in_combat_range = player_pos.distance_to(agent_pos) < NPCBrainConstants::COMBAT_RANGE_TARGET_DISTANCE;
	goap_worldstate_set(&action_planner, p_state, NPCBrainConstants::IN_COMBAT_RANGE_TO_PLAYER, is_in_combat_range);
	bool is_player_dead = world_state->get_player()->is_dead();
	goap_worldstate_set(&action_planner, p_state, NPCBrainConstants::PLAYER_DEAD_ATOM, is_player_dead);
}

void GOAPActionPlanner::replan(const Ref<GOAPGoal> &p_goal) {
	if (current_plan.size() > 0) {
		current_plan[current_plan_action]->exit(world_state);
		current_plan.clear();
	}

	worldstate_t states[16];
	const char *plan[16];

	worldstate_t goap_current_world_state;
	worldstate_t goap_target_world_state;
	get_goap_state_from_game_state(&goap_current_world_state);
	p_goal->get_desired_world_state(&action_planner, &goap_target_world_state);

	int plan_size = 16;

	char c[2048];
	goap_worldstate_description(&action_planner, &goap_current_world_state, c, sizeof(c));
	goap_worldstate_description(&action_planner, &goap_current_world_state, c, sizeof(c));

	goap_worldstate_description(&action_planner, &goap_target_world_state, c, sizeof(c));

	int plan_cost = astar_plan(&action_planner, goap_current_world_state, goap_target_world_state, plan, states, &plan_size);
	// Something went wrong
	if (plan_size == 0 || plan_cost < 0) {
		is_plan_valid = false;
		return;
	}

	goap_description(&action_planner, c, sizeof(c));

	Vector<Ref<GOAPAction>> plan_actions;

	for (int i = 0; i < plan_size; i++) {
		const char *action_name = plan[i];
		for (int j = 0; j < actions.size(); j++) {
			if (String(action_name) == actions[j]->get_action_name()) {
				plan_actions.push_back(actions[j]);
				break;
			}
		}
	}

	is_plan_valid = true;
	current_plan_action = 0;
	current_plan = plan_actions;
	current_plan[0]->enter(world_state);
	current_goal = p_goal;

	DEV_ASSERT(plan_actions.size() == plan_size);
}

void GOAPActionPlanner::update_goal_priorities() {
	// Update the priorities of each goal
	for (Ref<GOAPGoal> goal : goals) {
		goal->update_priority(world_state);
	}
	goals.sort_custom<GoalCompare>();
}

void GOAPActionPlanner::update() {
	// Update and sort goals by priority
	update_goal_priorities();

	// Check if the highest priority valid goal is different from our current goal
	for (int i = 0; i < goals.size(); i++) {
		if (goals[i]->is_valid(world_state)) {
			if (goals[i] != current_goal) {
				is_plan_valid = false;
			}
			break;
		}
	}

	// Validate if the plan is valid
	if (is_plan_valid) {
		// We don't need to validate already done actions, what is in the past is in the past
		for (int i = current_plan_action; i < current_plan.size(); i++) {
			if (!current_plan[i]->is_valid(world_state)) {
				// Plan wasn't valid, bail out and hope we can find another goal
				is_plan_valid = false;
				break;
			}
		}
	}

	// Validate if the goal is still possible
	if (!is_plan_valid || !current_goal->is_valid(world_state)) {
		is_plan_valid = false;
		for (Ref<GOAPGoal> goal : goals) {
			if (goal->is_valid(world_state)) {
				replan(goal);
				break;
			}
		}
	}

	// try to run a plan, if we have one
	if (is_plan_valid) {
		if (current_plan[current_plan_action]->execute(world_state)) {
			current_plan_action++;
			current_plan[current_plan_action - 1]->exit(world_state);
			if (current_plan_action == current_plan.size()) {
				// Plan done!
				is_plan_valid = false;
				current_plan.clear();
			} else {
				current_plan[current_plan_action]->enter(world_state);
			}
		}
	}
#ifdef DEBUG_ENABLED
	if (goap_debugger_enabled.get()) {
		if (ImGui::Begin("GOAP")) {
			if (ImGui::BeginTabBar("##Planners")) {
				String tab_name = to_string();
				if (ImGui::BeginTabItem(tab_name.utf8().get_data())) {
					ImGui::Text("Goal: %s", String(current_goal->get_class_name()).utf8().get_data());
					for (int i = 0; i < current_plan.size(); i++) {
						String action_name = current_plan[i]->get_action_name();
						if (current_plan_action == i) {
							action_name = "->" + action_name;
						}
						ImGui::TextUnformatted(action_name.utf8().get_data());
					}

					ImGui::SeparatorText("System info");

					char desc[1024];
					goap_description(&action_planner, desc, 1024);
					ImGui::TextUnformatted(desc);

					if (ImGui::BeginTabBar("")) {
						for (int i = 0; i < goals.size(); i++) {
							String cname = goals[i]->get_class_name();
							if (!goals[i]->is_valid(world_state)) {
								cname += " (Disabled)";
							}
							cname += String(vformat(" %d", goals[i]->get_priority()));
							if (ImGui::BeginTabItem(cname.utf8().get_data())) {
								worldstate_t desired_world_state;
								char out[1024];
								goals[i]->get_desired_world_state(&action_planner, &desired_world_state);
								goap_worldstate_description(&action_planner, &desired_world_state, out, 1024);
								ImGui::SeparatorText("Desired world state");
								ImGui::TextUnformatted(out);

								ImGui::EndTabItem();
							}
						}
						ImGui::EndTabBar();
					}

					ImGui::EndTabItem();
				}
				ImGui::EndTabBar();
			}
		}
		ImGui::End();
	}
#endif
}

void GOAPActionPlanner::register_action(const Ref<GOAPAction> &p_goap_action) {
	actions.push_back(p_goap_action);
	p_goap_action->set_effects(&action_planner);
	p_goap_action->set_prerequisites(&action_planner);
}

void GOAPActionPlanner::register_goal(const Ref<GOAPGoal> &p_goap_goal) {
	DEV_ASSERT(p_goap_goal.is_valid());
	goals.push_back(p_goap_goal);
}

GOAPActionPlanner::GOAPActionPlanner(const Ref<GameWorldState> &p_world_state, HBAgent *p_agent) {
	goap_actionplanner_clear(&action_planner);
	world_state = p_world_state;
	agent = p_agent;
}

GOAPActionPlanner::~GOAPActionPlanner() {
}

_FORCE_INLINE_ bool GOAPActionPlanner::GoalCompare::operator()(const Ref<GOAPGoal> &l, const Ref<GOAPGoal> &r) const {
	return l->get_priority() >= r->get_priority();
}
