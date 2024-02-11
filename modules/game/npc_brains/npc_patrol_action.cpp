#include "npc_patrol_action.h"

void GOAPPatrolAction::set_cost(actionplanner_t *p_action_planner) const {
    goap_set_cost(p_action_planner, get_action_name(), get_cost());
};

int GOAPPatrolAction::get_cost() const {
    return 0;
};

bool GOAPPatrolAction::execute(const Ref<GameWorldState> &p_world_state) {
    HBNPCAgent *agent = get_brains()->get_agent();
    
    if (agent->move_to_target(agent->get_process_delta_time())) {
        // Go to next point if we reached it
        Ref<Curve3D> curve = agent->get_patrol_route_node()->get_route();
        target_point = (target_point + 1) % curve->get_point_count();
        print_line(target_point);
        agent->set_navigation_target_position(curve->get_point_position(target_point));
    }

    // patrolling never finishes and should be interrupted by GOAP
    return false;
};

void GOAPPatrolAction::enter() {
    HBNPCAgent *agent = get_brains()->get_agent();
    Ref<Curve3D> path = agent->get_patrol_route_node()->get_route();
    float closest_point_dist = 1000000.0;
    int closest_point = -1;

    const Vector3 agent_pos = agent->get_global_position();

    for(int i = 0; i < path->get_point_count(); i++) {
        float dist = path->get_point_position(i).distance_to(agent_pos);
        if (dist < closest_point_dist) {
            closest_point_dist = dist;
        }
        closest_point = i;
    }

    DEV_ASSERT(closest_point != -1);

    target_point = (closest_point+1) % path->get_point_count();
    get_brains()->get_agent()->set_navigation_target_position(path->get_point_position(target_point));
}

bool GOAPPatrolAction::is_valid(const Ref<GameWorldState> &p_world_state) const {
    // Patrolling should always be possible, if we have a patrol route
    HBRoute *patrol_route_node = get_brains()->get_agent()->get_patrol_route_node();
    return patrol_route_node != nullptr;
}

const char *GOAPPatrolAction::get_action_name() const {
    return PATROL_ACTION_NAME;
};

void GOAPPatrolAction::set_effects(actionplanner_t *p_action_planner) const {
    goap_set_pst(p_action_planner, PATROL_ACTION_NAME, PATROL_DONE_ATOM_NAME, true);
};

void GOAPPatrolAction::set_prerequisites(actionplanner_t *p_action_planner) const {
    // No prerequisites for patrolling around
};
