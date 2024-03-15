/**************************************************************************/
/*  npc_brains.cpp                                                        */
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

#include "npc_brains.h"
#include "modules/game/game_main_loop.h"
#include "modules/game/npc_brains/npc_approach_player_action.h"
#include "modules/game/npc_brains/npc_attack_player_action.h"
#include "modules/game/npc_brains/npc_kill_player_goal.h"
#include "modules/game/npc_brains/npc_patrol_action.h"
#include "modules/game/npc_brains/npc_patrol_goal.h"
#include "npc_brains/npc_brain_constants.h"

void NPCBrains::_bind_methods() {
}

void NPCBrains::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PROCESS: {
			action_planner->update();
		} break;
		case NOTIFICATION_READY: {
			if (!Engine::get_singleton()->is_editor_hint()) {
				agent = Object::cast_to<HBNPCAgent>(get_parent());
				HBGameWorld *gw = Object::cast_to<HBGameMainLoop>(get_tree())->get_game_world();
				action_planner = Ref<GOAPActionPlanner>(memnew(GOAPActionPlanner(gw->get_game_world_state(), agent)));
				register_actions();
			}
		} break;
	}
}

void NPCBrains::register_actions() {
	action_planner->register_action(memnew(GOAPPatrolAction(this)));
	action_planner->register_goal(memnew(GOAPPatrolGoalNPC(this)));

	action_planner->register_action(memnew(GOAPApproachPlayerAction(NPCBrainConstants::in_attack_range_approach_settings, this)));

	action_planner->register_action(memnew(GOAPApproachPlayerAction(NPCBrainConstants::in_combat_range_approach_settings, this)));
	action_planner->register_action(memnew(NPCAttackPlayerAction(this)));

	action_planner->register_goal(memnew(GOAPNPCPlayerDeadGoal(this)));
}

HBNPCAgent *NPCBrains::get_agent() const {
	return agent;
}

NPCBrains::NPCBrains() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_process(true);
	}
}
