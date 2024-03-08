#ifndef NPC_PATROL_ACTION_H
#define NPC_PATROL_ACTION_H

#include "modules/game/npc_brains.h"

static constexpr const char *PATROL_ACTION_NAME = "patrol";

class GOAPPatrolAction : public GOAPActionNPC {
	int target_point = -1;

public:
	virtual void set_prerequisites(actionplanner_t *p_action_planner) const override;
	virtual void set_effects(actionplanner_t *p_action_planner) const override;
	virtual const char *get_action_name() const override;

	virtual bool is_valid(const Ref<GameWorldState> &p_world_state) const override;
	virtual int get_cost() const override;

	virtual bool execute(const Ref<GameWorldState> &p_world_state) override;
	virtual void enter(const Ref<GameWorldState> &p_world_state) override;
	using GOAPActionNPC::GOAPActionNPC;
};

#endif // NPC_PATROL_ACTION_H
