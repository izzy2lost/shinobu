#include "npc_kill_player_goal.h"
#include "modules/game/npc_brains/npc_brain_constants.h"

void GOAPNPCPlayerDeadGoal::get_desired_world_state(actionplanner_t *p_action_planner, worldstate_t *p_world_state) const {
	goap_worldstate_clear(p_world_state);
	goap_worldstate_set(p_action_planner, p_world_state, NPCBrainConstants::PLAYER_DEAD_ATOM, true);
}
