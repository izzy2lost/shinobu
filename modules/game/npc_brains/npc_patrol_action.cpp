/**************************************************************************/
/*  npc_patrol_action.cpp                                                 */
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

#include "npc_patrol_action.h"

int GOAPPatrolAction::get_cost() const {
	return 0;
};

bool GOAPPatrolAction::execute(const Ref<GameWorldState> &p_world_state) {
	HBNPCAgent *agent = get_brains()->get_agent();

	if (agent->move_to_target(agent->get_process_delta_time())) {
		// Go to next point if we reached it
		Ref<Curve3D> curve = agent->get_patrol_route_node()->get_route();
		target_point = (target_point + 1) % curve->get_point_count();
		agent->set_navigation_target_position(curve->get_point_position(target_point));
	}

	// patrolling never finishes and should be interrupted by GOAP when a higher priority goal takes place
	return false;
};

void GOAPPatrolAction::enter(const Ref<GameWorldState> &p_world_state) {
	HBNPCAgent *agent = get_brains()->get_agent();
	Ref<Curve3D> path = agent->get_patrol_route_node()->get_route();
	float closest_point_dist = 1000000.0;
	int closest_point = -1;

	const Vector3 agent_pos = agent->get_global_position();

	for (int i = 0; i < path->get_point_count(); i++) {
		float dist = path->get_point_position(i).distance_to(agent_pos);
		if (dist < closest_point_dist) {
			closest_point_dist = dist;
		}
		closest_point = i;
	}

	DEV_ASSERT(closest_point != -1);

	target_point = (closest_point + 1) % path->get_point_count();
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
