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
	const StringName wallrun_state;
	const StringName ledge_grabbed_state;
	const StringName fall_state;
	const StringName ledge_get_up_state;
	const StringName wall_parkour_state;
	const StringName autojump_state;
	const StringName beam_walk_state;
	const StringName root_motion_state;

	const StringName right_foot_ik_node;
	const StringName left_foot_ik_node;
	const StringName ledge_to_ground_node;

	static AgentStringNames *get_singleton() {
		return singleton;
	}

	AgentStringNames();
};

#endif // AGENT_STRING_NAMES_H
