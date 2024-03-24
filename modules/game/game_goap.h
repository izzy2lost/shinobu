/**************************************************************************/
/*  game_goap.h                                                           */
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

#ifndef GAME_GOAP_H
#define GAME_GOAP_H

#include "core/object/ref_counted.h"
#include "modules/game/console_system.h"
#include "modules/game/game_world.h"
#include "thirdparty/goap/goap.h"

static constexpr const char *PATROL_DONE_ATOM_NAME = "patrol_done";

class GOAPDebugger : public Node {
	GDCLASS(GOAPDebugger, Node);
	static GOAPDebugger *singleton;

public:
	typedef void (*GOAPDebugCallback)();

private:
	LocalVector<GOAPDebugCallback> callbacks;

protected:
	void _notification(int p_what);

public:
	GOAPDebugger *get_singleton();
	void add_debug_callback(const GOAPDebugCallback &p_callback);
	void remove_debug_callback(const GOAPDebugCallback &p_callback);

	GOAPDebugger();
};

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
	virtual void enter(const Ref<GameWorldState> &p_world_state){};
	virtual void exit(const Ref<GameWorldState> &p_world_state){};
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
	static CVar ai_disabled;
	static CVar goap_debugger_enabled;

	Vector<Ref<GOAPAction>> actions;
	Vector<Ref<GOAPGoal>> goals;
	bool is_plan_valid = false;

	Ref<GOAPGoal> current_goal;
	Vector<Ref<GOAPAction>> current_plan;
	int current_plan_action = -1;
	Ref<GameWorldState> world_state;
	HBAgent *agent = nullptr;

	actionplanner_t action_planner;

public:
#ifdef DEBUG_ENABLED
	static constexpr int MAX_DEBUG_GOALS = 16;
	struct DebugUIInfo {
		int selected_goal = 0;
		int debug_goal_count = 0;
		char *goals[MAX_DEBUG_GOALS];
	} debug_info;
#endif
	void get_goap_state_from_game_state(worldstate_t *p_state);

	void replan(const Ref<GOAPGoal> &p_goal);

	struct GoalCompare {
		_FORCE_INLINE_ bool operator()(const Ref<GOAPGoal> &l, const Ref<GOAPGoal> &r) const;
	};

	void update_goal_priorities();

	void update();

	void register_action(const Ref<GOAPAction> &p_goap_action);

	void register_goal(const Ref<GOAPGoal> &p_goap_goal);
	GOAPActionPlanner(const Ref<GameWorldState> &p_world_state, HBAgent *p_agent);
	~GOAPActionPlanner();
};

#endif // GOAP_H
