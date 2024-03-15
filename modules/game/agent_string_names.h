/**************************************************************************/
/*  agent_string_names.h                                                  */
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
	const StringName combat_move_state;
	const StringName combat_attack_state;
	const StringName combat_hit_state;
	const StringName dead_state;

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
	const StringName sword_parry_animation_node;
	const StringName roll_animation_node;

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
