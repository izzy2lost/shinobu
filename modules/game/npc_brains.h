#ifndef NPC_BRAINS_H
#define NPC_BRAINS_H

#include "modules/game/game_world.h"
#include "modules/game/npc_agent.h"
#include "state_machine.h"
#include "safe_godot_obj_id_ref.h"
#include "thirdparty/goap/goap.h"
#include "thirdparty/goap/astar.h"

static constexpr const char *PATROL_DONE_ATOM_NAME = "patrol_done";

class GOAPAction : public RefCounted {
    GDCLASS(GOAPAction, RefCounted);
public:
    virtual void set_prerequisites(actionplanner_t *p_action_planner) const = 0;
    virtual void set_effects(actionplanner_t *p_action_planner) const = 0;
    virtual const char *get_action_name() const = 0;
    virtual bool is_valid(const Ref<GameWorldState> &p_world_state) const {
        return false;
    }
    virtual void set_cost(actionplanner_t *p_action_planner) const {
        goap_set_cost(p_action_planner, get_action_name(), get_cost());
    };
    virtual int get_cost() const { return 0; };
    virtual bool execute(const Ref<GameWorldState> &p_world_state) {
        return false;
    };
    virtual void enter() {};
    virtual void exit() {};
};

class GOAPGoal : public RefCounted {
    GDCLASS(GOAPGoal, RefCounted);
    int priority = -1;
public:
    virtual void get_desired_world_state(actionplanner_t *p_action_planner, worldstate_t *p_world_state) const {

    }
    virtual bool is_valid(const Ref<GameWorldState> &p_world_state) const {
        return false;
    }
    virtual int get_priority() const {
        return priority;
    }
    virtual int calculate_priority(const Ref<GameWorldState> &p_world_state) const {
        return -1;
    }
    virtual void update_priority(const Ref<GameWorldState> &p_world_state) {
        priority = calculate_priority(Ref<GameWorldState>());
    }
};

class GOAPActionPlanner : public RefCounted {
    Vector<Ref<GOAPAction>> actions;
    Vector<Ref<GOAPGoal>> goals;
    bool is_plan_valid = false;

    Ref<GOAPGoal> current_goal;
    Vector<Ref<GOAPAction>> current_plan;
    int current_plan_action = -1;
    Ref<GameWorldState> world_state;

    actionplanner_t action_planner;
public:
    void get_goap_state_from_game_state(worldstate_t *p_state) {
        goap_worldstate_clear(p_state);
        // A bit of a hack, but patrol is never done
        goap_worldstate_set(&action_planner, p_state, PATROL_DONE_ATOM_NAME, false);
    }

    void replan(const Ref<GOAPGoal> &p_goal);

	struct GoalCompare {
		_FORCE_INLINE_ bool operator()(const Ref<GOAPGoal> &l, const Ref<GOAPGoal> &r) const;
	};

    void update_goal_priorities();

    void update();

    void register_action(const Ref<GOAPAction> &p_goap_action);

    void register_goal(const Ref<GOAPGoal> &p_goap_goal);
    GOAPActionPlanner();
};

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
    GOAPActionNPC(NPCBrains *p_brains) : brains(SafeGodotObjIDRef<NPCBrains>::from_object(p_brains)) {
    }
};

class GOAPGoalNPC : public GOAPGoal {
    SafeGodotObjIDRef<NPCBrains> brains;
protected:
    NPCBrains *get_brains() const {
        return *brains;
    }
public:
    GOAPGoalNPC(NPCBrains *p_brains) : brains(SafeGodotObjIDRef<NPCBrains>::from_object(p_brains)) {
    }
};


#endif // NPC_BRAINS_H
