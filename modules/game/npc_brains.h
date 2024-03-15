/**************************************************************************/
/*  npc_brains.h                                                          */
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

#ifndef NPC_BRAINS_H
#define NPC_BRAINS_H

#include "core/object/object.h"
#include "modules/game/game_goap.h"
#include "modules/game/npc_agent.h"
#include "safe_godot_obj_id_ref.h"
#include "state_machine.h"

class NPCBrains : public Node3D {
	GDCLASS(NPCBrains, Node3D);
	Ref<GOAPActionPlanner> action_planner;
	HBNPCAgent *agent = nullptr;

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	void register_actions();

	HBNPCAgent *get_agent() const;

	NPCBrains();
};

class GOAPActionNPC : public GOAPAction {
	SafeGodotObjIDRef<NPCBrains> brains;

protected:
	NPCBrains *get_brains() const {
		return *brains;
	}

public:
	GOAPActionNPC(NPCBrains *p_brains) :
			brains(SafeGodotObjIDRef<NPCBrains>::from_object(p_brains)) {
	}
};

class GOAPGoalNPC : public GOAPGoal {
	SafeGodotObjIDRef<NPCBrains> brains;

protected:
	NPCBrains *get_brains() const {
		return *brains;
	}

public:
	GOAPGoalNPC(NPCBrains *p_brains) :
			brains(SafeGodotObjIDRef<NPCBrains>::from_object(p_brains)) {
	}
};

#endif // NPC_BRAINS_H
