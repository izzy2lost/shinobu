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
