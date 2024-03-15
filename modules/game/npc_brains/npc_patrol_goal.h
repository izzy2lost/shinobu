/**************************************************************************/
/*  npc_patrol_goal.h                                                     */
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
