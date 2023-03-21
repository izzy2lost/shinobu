
#ifndef AGENT_H
#define AGENT_H

#include "agent_constants.h"
#include "inertialization.h"
#include "scene/3d/physics_body_3d.h"

class HBAgent : public CharacterBody3D {
	GDCLASS(HBAgent, CharacterBody3D);

public:
	enum MovementMode {
		MOVE_GROUNDED,
		MOVE_MANUAL
	};

	enum AgentInputAction {
		INPUT_ACTION_RUN,
		INPUT_ACTION_MAX
	};

	struct InputState {
		Vector3 movement;
		bool action_states[AgentInputAction::INPUT_ACTION_MAX] = {};
	};

	InputState current_input_state;
	InputState prev_input_state;

private:
	MovementMode movement_mode = MovementMode::MOVE_GROUNDED;

	Vector3 velocity_spring_acceleration;

	Vector3 tilt_spring_velocity;

	NodePath graphics_node;
	NodePath tilt_node;
	ObjectID tilt_node_cache;
	Vector3 graphics_lookat_normal;
	ObjectID graphics_node_cache;
	Ref<HBAgentConstants> agent_constants;

	Quaternion last_rotation;
	Quaternion last_last_rotation;
	Ref<RotationInertializer> look_at_rot_inertializer;
	// look at rot before inertialization being applied
	Quaternion look_at_rot;

	void _update_graphics_node_cache();
	void _update_tilt_node_cache();
	Node3D *_get_graphics_node();
	Node3D *_get_tilt_node();
	void _rotate_towards_velocity(float p_delta);
	void _tilt_towards_acceleration(float p_delta);

	void _physics_process(float p_delta);

	// Input handling
	bool is_action_pressed(AgentInputAction p_action) const;
	bool is_action_just_pressed(AgentInputAction p_action) const;
	bool is_action_just_released(AgentInputAction p_action) const;

protected:
	static void _bind_methods();
	void _notification(int p_what);
	Ref<HBAgentConstants> _get_agent_constants() const;
	Vector3 _get_desired_velocity() const;

public:
	void set_input_action_state(AgentInputAction p_event, bool p_state);
	Vector3 get_desired_movement_input() const;
	void set_movement_input(Vector3 p_movement_input);
	void set_graphics_node(NodePath p_path);
	NodePath get_graphics_node() const;
	void set_tilt_node(NodePath p_path);
	NodePath get_tilt_node() const;
	void set_movement_mode(MovementMode p_movement_mode);
	MovementMode get_movement_mode() const;

	Ref<HBAgentConstants> get_agent_constants() const;
	void set_agent_constants(const Ref<HBAgentConstants> &p_agent_constants);

#ifdef DEBUG_ENABLED
	const int VELOCITY_PLOT_SIZE = 90;
	float plot_t = 0.0f;
	Vector<float> velocity_plot_lines_x;
	Vector<float> velocity_plot_lines_y;
	Vector<float> desired_velocity_plot_lines_y;
	Vector<float> acceleration_plot_lines_x;
	Vector<float> acceleration_plot_lines_y;
#endif

	HBAgent();
	virtual ~HBAgent();
};

VARIANT_ENUM_CAST(HBAgent::AgentInputAction);

#endif // AGENT_H
