#ifndef AGENT_STRING_NAMES_H
#define AGENT_STRING_NAMES_H

#include "core/object/class_db.h"
#include "core/object/object.h"

class AgentStringNames : public Object {
	GDCLASS(AgentStringNames, Object);
	static AgentStringNames *singleton;

public:
	const StringName move_state;
	const StringName turn_state;
	const StringName ledge_grabbed_state;
	const StringName fall_state;
	const StringName ledge_get_up_state;
	const StringName wall_parkour_state;
	const StringName beam_walk_state;
	const StringName root_motion_state;
	const StringName wall_parkour_transition_state;

	const StringName right_hand_ik_node;
	const StringName left_hand_ik_node;
	const StringName right_foot_ik_node;
	const StringName left_foot_ik_node;
	
	const StringName ledge_getup_animation_node;
	const StringName ledge_to_ground_animation_node;
	const StringName ledge_hop_up_animation_node;
	const StringName standing_drop_to_ledge_animation_node;
	const StringName standing_jump_to_ledge_animation_node;
	const StringName short_hop_animation_node;
	const StringName back_eject_to_ledge_animation_node;
	const StringName back_eject_to_surface_animation_node;
	const StringName wallparkour_up_long_jump_animation_node;
	const StringName wallparkour_down_long_jump_animation_node;
	const StringName wallparkour_left_long_jump_animation_node;
	const StringName wallparkour_right_long_jump_animation_node;

	const StringName wall_parkour_cat_animation_node;
	const StringName wall_parkour_cat_point_wp_name;

	const StringName lookat_head_node_name;
	const StringName lookat_torso_node_name;

	static AgentStringNames *get_singleton() {
		return singleton;
	}

protected:
	static void _bind_methods();
public:

	AgentStringNames();
};

#endif // AGENT_STRING_NAMES_H
