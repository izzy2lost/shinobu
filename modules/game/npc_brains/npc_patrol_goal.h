#ifndef NPC_PATROL_GOAL_H
#define NPC_PATROL_GOAL_H

#include "modules/game/npc_brains.h"

class GOAPPatrolGoalNPC : public GOAPGoalNPC {
	GDCLASS(GOAPPatrolGoalNPC, GOAPGoalNPC);

public:
	virtual void get_desired_world_state(actionplanner_t *p_action_planner, worldstate_t *p_world_state) const override {
		goap_worldstate_clear(p_world_state);
		goap_worldstate_set(p_action_planner, p_world_state, PATROL_DONE_ATOM_NAME, true);
	}

	virtual bool is_valid(const Ref<GameWorldState> &p_world_state) const override {
		// Patrolling is always valid
		return true;
	}

	virtual int calculate_priority(const Ref<GameWorldState> &p_world_state) const override {
		// basically anything can stop a patrol, so keep it at -1
		return -1;
	}
	using GOAPGoalNPC::GOAPGoalNPC;
};

#endif // NPC_PATROL_GOAL_H
