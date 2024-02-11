#include "npc_agent.h"

void HBNPCAgent::_bind_methods() {
    NODE_CACHE_BIND(patrol_route, HBRoute, HBNPCAgent);
}

NavigationAgent3D *HBNPCAgent::get_navigation_agent() const {
    if (!navigation_agent) {
        const_cast<HBNPCAgent*>(this)->navigation_agent = memnew(NavigationAgent3D);
        const_cast<HBNPCAgent*>(this)->add_child(navigation_agent);
    }
    navigation_agent->set_debug_enabled(true);
    return navigation_agent;
}

void HBNPCAgent::set_navigation_target_position(const Vector3 &p_target_position) {
    get_navigation_agent()->set_target_position(p_target_position);
}

bool HBNPCAgent::move_to_target(float p_delta) {
    set_movement_input(Vector3());
    
    Vector3 next_pos = navigation_agent->get_next_path_position();
    Vector3 dir = get_global_position().direction_to(next_pos);
    dir.y = 0.0;
    dir = dir.normalized();
    
    if (!dir.is_normalized()) {
        return true;
    }

    set_movement_input(dir);

    return navigation_agent->is_navigation_finished();
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
        print_line("ABORT CHILD");
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
