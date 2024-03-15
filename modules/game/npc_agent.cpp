/**************************************************************************/
/*  npc_agent.cpp                                                         */
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

#include "npc_agent.h"
#include "core/object/callable_method_pointer.h"

CVar HBNPCAgent::show_pathfinding = CVar("npc_show_pathfinding", Variant::BOOL, false);

void HBNPCAgent::_bind_methods() {
	NODE_CACHE_BIND(patrol_route, HBRoute, HBNPCAgent);
}

void HBNPCAgent::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PROCESS: {
			if (Input::get_singleton()->is_key_pressed(Key::A)) {
				show_pathfinding.data->get_signaler()->emit_signal("changed");
			}
		} break;
	}
}

void HBNPCAgent::_on_show_pathfinding_updated() {
	if (navigation_agent) {
		navigation_agent->set_debug_enabled(show_pathfinding.get());
	}
}

NavigationAgent3D *HBNPCAgent::get_navigation_agent() const {
	if (!navigation_agent) {
		const_cast<HBNPCAgent *>(this)->navigation_agent = memnew(NavigationAgent3D);
		const_cast<HBNPCAgent *>(this)->add_child(navigation_agent);
	}
	return navigation_agent;
}

void HBNPCAgent::set_navigation_target_position(const Vector3 &p_target_position) {
	get_navigation_agent()->set_target_position(p_target_position);
}

bool HBNPCAgent::move_to_target(float p_delta, bool p_run) {
	set_movement_input(Vector3());

	Vector3 next_pos = navigation_agent->get_next_path_position();
	Vector3 dir = get_global_position().direction_to(next_pos);
	dir.y = 0.0;
	dir = dir.normalized();

	if (!dir.is_normalized()) {
		return true;
	}

	set_movement_input(dir);

	set_input_action_state(AgentInputAction::INPUT_ACTION_RUN, p_run);
	if (navigation_agent->is_navigation_finished()) {
		set_movement_input(Vector3());
	}
	return navigation_agent->is_navigation_finished();
}

HBNPCAgent::HBNPCAgent() {
	show_pathfinding.data->get_signaler()->connect("changed", callable_mp(this, &HBNPCAgent::_on_show_pathfinding_updated));
	set_process(true);
}

void HBNPCAgentPatrol::physics_process(float p_delta) {
}

void HBRoute::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_route"), &HBRoute::get_route);
	ClassDB::bind_method(D_METHOD("set_route", "route"), &HBRoute::set_route);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "route", PROPERTY_HINT_RESOURCE_TYPE, "Curve3D"), "set_route", "get_route");
}

void HBRoute::_editor_build(const EntityCompileInfo &p_info, const HashMap<StringName, EntityCompileInfo> &p_entities) {
	if (p_info.parents.size() > 0) {
		queue_free();
		return;
	}
	Vector<StringName> children = p_info.children;

	if (!route.is_valid()) {
		route.instantiate();
	}

	route->add_point(get_global_position());
	while (children.size() > 0) {
		StringName child = children[0];
		const EntityCompileInfo *child_info = p_entities.getptr(child);
		if (!child_info) {
			break;
		}

		HBRoute *next_route_point = Object::cast_to<HBRoute>(child_info->node);

		DEV_ASSERT(next_route_point != nullptr);

		if (next_route_point) {
			route->add_point(next_route_point->get_position());
		}
		children = child_info->children;
	}
}
