#include "npc_brains.h"
#include "modules/game/npc_brains/npc_patrol_action.h"
#include "modules/game/npc_brains/npc_patrol_goal.h"

void GOAPActionPlanner::replan(const Ref<GOAPGoal> &p_goal) {
    worldstate_t states[16];
    const char *plan[16];

    worldstate_t goap_current_world_state;
    worldstate_t goap_target_world_state;
    get_goap_state_from_game_state(&goap_current_world_state);
    p_goal->get_desired_world_state(&action_planner, &goap_target_world_state);

    int plan_size = 16;

    char c[2048];
    goap_worldstate_description(&action_planner, &goap_current_world_state, c, sizeof(c));
    goap_worldstate_description(&action_planner, &goap_current_world_state, c, sizeof(c));

    goap_worldstate_description(&action_planner, &goap_target_world_state, c, sizeof(c));

    const int plan_cost = astar_plan(&action_planner, goap_current_world_state, goap_target_world_state, plan, states, &plan_size);
        // Something went wrong
    if (plan_size == 0) {
        is_plan_valid = false;
        return;
    }
    
    Vector<Ref<GOAPAction>> plan_actions;

    for (int i = 0; i < plan_size; i++) {
        const char *action_name = plan[i];
        for(int j = 0; j < actions.size(); i++) {
            if (String(action_name) == actions[j]->get_action_name()) {
                plan_actions.push_back(actions[j]);
                break;
            }
        }
    }

    is_plan_valid = true;
    current_plan_action = 0;
    current_plan = plan_actions;
    current_plan[0]->enter();
    current_goal = p_goal;

    DEV_ASSERT(plan_actions.size() == plan_size);
}

void GOAPActionPlanner::update_goal_priorities() {
    // Update the priorities of each goal
    for (Ref<GOAPGoal> goal : goals) {
        goal->update_priority(world_state);
    }
    goals.sort_custom<GoalCompare>();
}

void GOAPActionPlanner::update() {
    // Update and sort goals by priority
    update_goal_priorities();

    // Validate if the plan is valid
    if (is_plan_valid) {
        // We don't need to validate already done actions, what is in the past is in the past
        for (int i = current_plan_action; i < current_plan.size(); i++) {
            if (!current_plan[i]->is_valid(world_state)) {
                // Plan wasn't valid, bail out and hope we can find another goal
                is_plan_valid = false;
                break;
            }
        }
    }

    // Validate if the goal is still possible
    if (!is_plan_valid || !current_goal->is_valid(world_state)) {
        is_plan_valid = false;
        for(Ref<GOAPGoal> goal : goals) {
            if (goal->is_valid(world_state)) {
                replan(goal);
                break;
            }
        }
    }

    // try to run a plan, if we have one
    if (is_plan_valid) {
        if (current_plan[current_plan_action]->execute(world_state)) {
            current_plan_action++;
            if (current_plan_action == current_plan.size()) {
                // Plan done!
                current_plan[current_plan_action-1]->exit();
                is_plan_valid = false;
            } else {
                current_plan[current_plan_action]->enter();
            }
        }
    }
}

void GOAPActionPlanner::register_action(const Ref<GOAPAction> &p_goap_action) {
    actions.push_back(p_goap_action);
    p_goap_action->set_effects(&action_planner);
    p_goap_action->set_prerequisites(&action_planner);
}

void GOAPActionPlanner::register_goal(const Ref<GOAPGoal> &p_goap_goal) {
    DEV_ASSERT(p_goap_goal.is_valid());
    goals.push_back(p_goap_goal);
}

GOAPActionPlanner::GOAPActionPlanner() {
    goap_actionplanner_clear(&action_planner);
}

_FORCE_INLINE_ bool GOAPActionPlanner::GoalCompare::operator()(const Ref<GOAPGoal> &l, const Ref<GOAPGoal> &r) const {
            return l->get_priority() < r->get_priority(); 
}

void NPCBrains::_bind_methods() {
}

void NPCBrains::_notification(int p_what) {
    switch(p_what) {
        case NOTIFICATION_PROCESS: {
            action_planner->update();
        } break;
        case NOTIFICATION_PARENTED: {
            agent = Object::cast_to<HBNPCAgent>(get_parent());
        }
    }
}

void NPCBrains::register_actions() {
    action_planner->register_action(memnew(GOAPPatrolAction(this)));
    action_planner->register_goal(memnew(GOAPPatrolGoalNPC(this)));
}

HBNPCAgent *NPCBrains::get_agent() const {
    return agent;
}

NPCBrains::NPCBrains() {
    if (!Engine::get_singleton()->is_editor_hint()) {
        action_planner.instantiate();
        register_actions();
        set_process(true);
    }
}
