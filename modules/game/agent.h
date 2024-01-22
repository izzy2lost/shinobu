
#ifndef AGENT_H
#define AGENT_H

#include "agent_constants.h"
#include "animation_system/epas_controller.h"
#include "animation_system/epas_oneshot_animation_node.h"
#include "inertialization.h"
#include "jolt_character_body.h"
#include "scene/3d/area_3d.h"
#include "scene/3d/navigation_agent_3d.h"
#include "scene/3d/physics_body_3d.h"

class HBAgentParkourLedge;

class HBAgent : public JoltCharacterBody3D {
	GDCLASS(HBAgent, JoltCharacterBody3D);

public:
	enum MovementMode {
		MOVE_GROUNDED,
		MOVE_MANUAL,
		MOVE_FALL
	};

	enum AgentInputAction {
		INPUT_ACTION_RUN,
		INPUT_ACTION_PARKOUR_DOWN,
		INPUT_ACTION_PARKOUR_UP,
		INPUT_ACTION_MAX
	};

private:
	NodePath ledge_detector_path;
	struct InputState {
		Vector3 movement;
		Quaternion movement_input_rotation;
		bool action_states[AgentInputAction::INPUT_ACTION_MAX] = {};
	};

	InputState current_input_state;
	InputState prev_input_state;

	Vector3 desired_velocity;
	Vector3 inertialization_prev_positions[2];
	bool inertialization_queued = false;
	float inertialization_duration;

	Vector3 smoothed_accel = Vector3(0.0, 0.0, 0.0);
	Vector3 prev_position;

	NavigationAgent3D *navigation_agent;
	MovementMode movement_mode = MovementMode::MOVE_GROUNDED;

	Quaternion current_rotation;
	Quaternion rotation_spring_target;
	Vector3 rotation_spring_velocity;
	Vector3 tilt_spring_velocity;

	NodePath graphics_node;
	NodePath tilt_node;
	ObjectID tilt_node_cache;
	Vector3 graphics_lookat_normal;
	ObjectID graphics_node_cache;
	Ref<HBAgentConstants> agent_constants;

	NodePath epas_controller_node;
	ObjectID epas_controller_cache;

	Quaternion last_rotation;
	Quaternion last_last_rotation;

	void _update_graphics_node_cache();
	void _update_tilt_node_cache();
	void _update_epas_controller_cache();
	Node3D *_get_graphics_node();
	Node3D *_get_tilt_node();
	EPASController *_get_epas_controller();
	void _rotate_towards_velocity(float p_delta);
	void _tilt_towards_acceleration(float p_delta);

	void _physics_process(float p_delta);

	Ref<PositionInertializer> graphics_position_intertializer;
	Ref<RotationInertializer> graphics_rotation_intertializer;
	Quaternion graphics_rotation;
	Quaternion prev_graphics_rotation;
	Vector3 prev_graphics_position;

protected:
	static void _bind_methods();
	void _notification(int p_what);
	Ref<HBAgentConstants> _get_agent_constants() const;
	Vector3 _get_desired_velocity() const;

	void _start_inertialize_graphics_position(const Vector3 &p_prev_prev, const Vector3 &p_prev, const Vector3 &p_target, float p_delta, float p_duration = 0.25f);

public:
	// Input handling
	bool is_action_pressed(AgentInputAction p_action) const;
	bool is_action_just_pressed(AgentInputAction p_action) const;
	bool is_action_just_released(AgentInputAction p_action) const;

	void flush_inputs();
	void set_input_action_state(AgentInputAction p_event, bool p_state);
	Vector3 get_desired_movement_input() const;
	Vector3 get_desired_movement_input_transformed() const;
	void set_movement_input(Vector3 p_movement_input);
	Vector3 get_movement_input() const;
	void set_movement_input_rotation(Quaternion p_movement_input_rotation);
	Quaternion get_movement_input_rotation() const;
	void set_graphics_node(NodePath p_path);
	NodePath get_graphics_node() const;
	void set_tilt_node(NodePath p_path);
	NodePath get_tilt_node() const;
	void set_movement_mode(MovementMode p_movement_mode);
	MovementMode get_movement_mode() const;
	NodePath get_epas_controller_node() const;
	void set_epas_controller_node(const NodePath &p_epas_controller_node);
	Vector3 get_previous_position() const;

	Ref<HBAgentConstants> get_agent_constants() const;
	void set_agent_constants(const Ref<HBAgentConstants> &p_agent_constants);
	void apply_root_motion(const Ref<EPASOneshotAnimationNode> &p_animation_node, float p_delta);
	float get_height() const;
	float get_radius() const;
	Vector3 process_graphics_position_inertialization(const float &p_delta);
	Quaternion process_graphics_rotation_inertialization(const float &p_delta);
	void root_motion_begin(Ref<EPASOneshotAnimationNode> p_animation_node, float p_delta);
	Ref<Shape3D> get_collision_shape();
	Vector3 get_movement_spring_velocity() const;
	void inertialize_graphics_transform(const Transform3D &p_target, float p_duration);
	void inertialize_graphics_rotation(Quaternion p_target_rot, bool p_now = false);
	bool is_at_edge(Vector3 p_direction);
	void reset_desired_input_velocity_to(const Vector3 &p_new_vel);

	Area3D *get_ledge_detector() const;
	LocalVector<HBAgentParkourLedge *> get_overlapping_ledges() const;

	NodePath get_ledge_detector_node() const;
	void set_ledge_detector_node(const NodePath &p_ledge_detector_node);

#ifdef DEBUG_ENABLED
	const int VELOCITY_PLOT_SIZE = 90;
	float plot_t = 0.0f;
	Vector<float> velocity_plot_lines_x;
	Vector<float> velocity_plot_lines_y;
	Vector<float> desired_velocity_plot_lines_y;
#endif

	HBAgent();
	virtual ~HBAgent();

	Quaternion get_graphics_rotation() const;
	void set_graphics_rotation(const Quaternion &p_graphics_rotation);

	friend class HBAgentState;
};

VARIANT_ENUM_CAST(HBAgent::AgentInputAction);

#endif // AGENT_H
