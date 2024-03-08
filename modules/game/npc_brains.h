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
