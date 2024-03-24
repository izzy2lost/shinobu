/**************************************************************************/
/*  npc_patrol_action.h                                                   */
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
	virtual void exit(const Ref<GameWorldState> &p_world_state) override;
	using GOAPActionNPC::GOAPActionNPC;
};

#endif // NPC_PATROL_ACTION_H
