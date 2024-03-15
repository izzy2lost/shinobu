/**************************************************************************/
/*  npc_approach_player_action.h                                          */
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

#ifndef NPC_APPROACH_PLAYER_ACTION_H
#define NPC_APPROACH_PLAYER_ACTION_H

#include "modules/game/npc_brains.h"

class GOAPApproachPlayerAction : public GOAPActionNPC {
	GDCLASS(GOAPApproachPlayerAction, GOAPActionNPC);
	Vector3 target_point;

public:
	struct ApproachPlayerActionSettings {
		// Object distance from the player
		float target_distance = 0.0f;
		// Max deviation from the player to the targetted position that we can tolerate
		// before we trigger a repath
		float player_max_target_deviation = 0.0f;
		char *action_name = nullptr;
		char *atom_name = nullptr;
		bool should_target_player;
	};

private:
	ApproachPlayerActionSettings action_settings;

public:
	void _update_target_point(const HBAgent *p_player);
	virtual void set_prerequisites(actionplanner_t *p_action_planner) const override;
	virtual void set_effects(actionplanner_t *p_action_planner) const override;
	virtual const char *get_action_name() const override;

	virtual bool is_valid(const Ref<GameWorldState> &p_world_state) const override;

	virtual bool execute(const Ref<GameWorldState> &p_world_state) override;
	virtual void enter(const Ref<GameWorldState> &p_world_state) override;
	virtual void exit(const Ref<GameWorldState> &p_world_state) override;

	GOAPApproachPlayerAction(const ApproachPlayerActionSettings &p_action_settings, NPCBrains *p_brains);
	using GOAPActionNPC::GOAPActionNPC;
};

#endif // NPC_APPROACH_PLAYER_ACTION_H
