#ifndef NPC_KILL_PLAYER_GOAL_H
#define NPC_KILL_PLAYER_GOAL_H

#include "modules/game/npc_brains.h"

class GOAPNPCPlayerDeadGoal : public GOAPGoalNPC {
	GDCLASS(GOAPNPCPlayerDeadGoal, GOAPGoalNPC);

public:
	virtual void get_desired_world_state(actionplanner_t *p_action_planner, worldstate_t *p_world_state) const override;

	virtual bool is_valid(const Ref<GameWorldState> &p_world_state) const override {
		// You can only kill the player if you know where he is
		return p_world_state->get_alert_status() == GameWorldState::ALERT_SEEN;
	}

	virtual int calculate_priority(const Ref<GameWorldState> &p_world_state) const override {
		return 100;
	}
	using GOAPGoalNPC::GOAPGoalNPC;
};

#endif // NPC_KILL_PLAYER_GOAL_H
