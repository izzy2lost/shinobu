
#ifndef AGENT_H
#define AGENT_H

#include "agent_constants.h"
#include "animation_system/epas_controller.h"
#include "animation_system/epas_oneshot_animation_node.h"
#include "inertialization.h"
#include "jolt_character_body.h"
#include "scene/3d/navigation_agent_3d.h"
#include "scene/3d/physics/area_3d.h"

class HBAgentParkourLedge;

class HBAttackData : public RefCounted {
	GDCLASS(HBAttackData, RefCounted);

public:
	enum AttackDirection {
		RIGHT
	};

private:
	StringName name;
	Ref<EPASAnimation> animation;
	Ref<Mesh> mesh;
	int transition_index;
	float hit_time = -1.0f;
	float hitstop_duration = 0.1f;
	float cancel_time = -1.0f;
	StringName next_attack;
	AttackDirection attack_direction = AttackDirection::RIGHT;
	int damage = 2;

protected:
	static void _bind_methods();

public:
	StringName get_name() const { return name; }
	void set_name(const StringName &p_name) { name = p_name; }
	Ref<EPASAnimation> get_animation() const { return animation; }
	void set_animation(const Ref<EPASAnimation> &p_animation) { animation = p_animation; }
	Ref<Mesh> get_mesh() const { return mesh; }
	void set_mesh(const Ref<Mesh> &p_mesh) { mesh = p_mesh; }

	int get_transition_index() const { return transition_index; }
	void set_transition_index(int p_transition_index) { transition_index = p_transition_index; }

	float get_hit_time() const { return hit_time; }
	void set_hit_time(float p_hit_time) { hit_time = p_hit_time; }

	float get_hitstop_duration() const { return hitstop_duration; }
	void set_hitstop_duration(float p_hitstop_duration) { hitstop_duration = p_hitstop_duration; }

	AttackDirection get_attack_direction() const { return attack_direction; }
	void set_attack_direction(AttackDirection p_attack_direction) { attack_direction = p_attack_direction; }

	float get_cancel_time() const;
	void set_cancel_time(float p_cancel_time);

	StringName get_next_attack() const { return next_attack; }
	void set_next_attack(const StringName &p_next_attack) { next_attack = p_next_attack; }

	int get_damage() const { return damage; }
	void set_damage(int p_damage) { damage = p_damage; }
};

VARIANT_ENUM_CAST(HBAttackData::AttackDirection);

#define NODE_CACHE_IMPL(node_name, node_type)                                                                                 \
private:                                                                                                                      \
	ObjectID node_name##_cache;                                                                                               \
	NodePath node_name;                                                                                                       \
	void _update_##node_name##_cache() {                                                                                      \
		node_name##_cache = ObjectID();                                                                                       \
		if (has_node(node_name)) {                                                                                            \
			Node *node = get_node(node_name);                                                                                 \
			ERR_FAIL_COND_MSG(!node, vformat("Cannot update %s cache: Node cannot be found!", _STR(node_name)));              \
			node_type *nd = Object::cast_to<node_type>(node);                                                                 \
			ERR_FAIL_COND_MSG(!nd, "Cannot update actor graphics cache: NodePath does not point to a correctly typed node!"); \
			node_name##_cache = nd->get_instance_id();                                                                        \
		}                                                                                                                     \
	}                                                                                                                         \
                                                                                                                              \
public:                                                                                                                       \
	NodePath get_##node_name() const {                                                                                        \
		return node_name;                                                                                                     \
	}                                                                                                                         \
	node_type *get_##node_name##_node() {                                                                                     \
		if (node_name##_cache.is_valid()) {                                                                                   \
			return Object::cast_to<node_type>(ObjectDB::get_instance(node_name##_cache));                                     \
		} else {                                                                                                              \
			_update_##node_name##_cache();                                                                                    \
			if (node_name##_cache.is_valid()) {                                                                               \
				return Object::cast_to<node_type>(ObjectDB::get_instance(node_name##_cache));                                 \
			}                                                                                                                 \
		}                                                                                                                     \
		return nullptr;                                                                                                       \
	}                                                                                                                         \
	void set_##node_name(NodePath p_path) {                                                                                   \
		node_name = p_path;                                                                                                   \
		_update_##node_name##_cache();                                                                                        \
	}

#define NODE_CACHE_BIND(node_name, node_type, class_name)                                                          \
	ClassDB::bind_method(D_METHOD("set_" _STR(node_name), _STR(node_name) "_path"), &class_name::set_##node_name); \
	ClassDB::bind_method(D_METHOD("get_" _STR(node_name)), &class_name::get_##node_name);                          \
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, _STR(node_name), PROPERTY_HINT_NODE_PATH_VALID_TYPES, _STR(node_type)), "set_" _STR(node_name), "get_" _STR(node_name));

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
		INPUT_ACTION_TARGET,
		INPUT_ACTION_ATTACK,
		INPUT_ACTION_PARRY,
		INPUT_ACTION_MAX
	};

	enum AgentOutlineMode {
		DISABLED,
		SOFT_TARGET,
		TARGET_ASSASSINATE,
		TARGET_COMBAT
	};

private:
	Ref<ShaderMaterial> outline_material;
	NodePath ledge_detector_path;
	struct InputState {
		Vector3 movement;
		Quaternion movement_input_rotation;
		bool action_states[AgentInputAction::INPUT_ACTION_MAX] = {};
		uint64_t last_action_press_times[AgentInputAction::INPUT_ACTION_MAX] = {};
		uint64_t last_action_release_times[AgentInputAction::INPUT_ACTION_MAX] = {};
	};

	InputState current_input_state;
	InputState prev_input_state;

	Vector3 desired_velocity;
	Vector3 inertialization_prev_positions[2];
	bool inertialization_queued = false;
	float inertialization_duration;

	float starting_heading = 0.0f;

	NavigationAgent3D *navigation_agent;
	MovementMode movement_mode = MovementMode::MOVE_GROUNDED;

	Quaternion current_rotation;
	Quaternion rotation_spring_target;
	Vector3 rotation_spring_velocity;
	Vector3 tilt_spring_velocity;

	NodePath graphics_node;
	Vector3 graphics_lookat_normal;
	ObjectID graphics_node_cache;
	Ref<HBAgentConstants> agent_constants;

	Quaternion last_rotation;
	Quaternion last_last_rotation;

	void _update_graphics_node_cache();
	Node3D *_get_graphics_node();
	void _rotate_towards_velocity(float p_delta);
	void _tilt_towards_acceleration(float p_delta);

	void _physics_process(float p_delta);

	Ref<PositionInertializer> graphics_position_intertializer;
	Ref<RotationInertializer> graphics_rotation_intertializer;
	Quaternion graphics_rotation;
	Quaternion prev_graphics_rotation;
	Vector3 prev_graphics_position;
	ObjectID target_agent;
	bool is_player_controlled = false;
	bool is_parrying = false;

	HashMap<StringName, Ref<HBAttackData>> attack_datas;

	int health = 10;
	int max_health = 10;

	bool is_invulnerable = false;

protected:
	static void _bind_methods();
	void _notification(int p_what);
	Ref<HBAgentConstants> _get_agent_constants() const;
	Vector3 _get_desired_velocity() const;

	void _start_inertialize_graphics_position(const Vector3 &p_prev_prev, const Vector3 &p_prev, const Vector3 &p_target, float p_delta, float p_duration = 0.25f);

	NODE_CACHE_IMPL(attack_trail, MeshInstance3D);
	NODE_CACHE_IMPL(tilt_node, Node3D);
	NODE_CACHE_IMPL(epas_controller, EPASController);
	NODE_CACHE_IMPL(skeleton, Skeleton3D);

public:
	// Input handling
	bool is_action_pressed(AgentInputAction p_action) const;
	bool is_action_just_pressed(AgentInputAction p_action) const;
	bool is_action_just_released(AgentInputAction p_action) const;
	uint64_t get_last_action_press_time(AgentInputAction p_action) const;
	uint64_t get_last_action_release_time(AgentInputAction p_action) const;

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
	void set_movement_mode(MovementMode p_movement_mode);
	MovementMode get_movement_mode() const;
	// Damage
	void receive_damage(const int p_damage);

	Ref<HBAgentConstants> get_agent_constants() const;
	void set_agent_constants(const Ref<HBAgentConstants> &p_agent_constants);
	void apply_root_motion(const Ref<EPASOneshotAnimationNode> &p_animation_node, bool p_collide, float p_delta);
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
	void add_attack(const Ref<HBAttackData> &p_attack_data);

#ifdef DEBUG_ENABLED
	const int VELOCITY_PLOT_SIZE = 90;
	float plot_t = 0.0f;
	Vector<float> velocity_plot_lines_x;
	Vector<float> velocity_plot_lines_y;
	Vector<float> desired_velocity_plot_lines_y;
#endif

	Quaternion get_graphics_rotation() const;
	void set_graphics_rotation(const Quaternion &p_graphics_rotation);

	float get_starting_heading() const { return starting_heading; }
	void set_starting_heading(float p_starting_heading) { starting_heading = p_starting_heading; }
	Vector<HBAgent *> find_nearby_agents(float p_radius) const;

	HBAgent();
	virtual ~HBAgent();

	bool get_is_player_controlled() const;
	void set_is_player_controlled(bool p_is_player_controlled);

	HBAgent *get_target() const;
	void set_target(HBAgent *p_target);

	Ref<HBAttackData> get_attack_data(const StringName &p_name);
	void receive_attack(HBAgent *p_attacker, Ref<HBAttackData> p_attack_data);

	void set_outline_mode(AgentOutlineMode p_outline_mode);

	bool get_is_parrying() const { return is_parrying; }
	void set_is_parrying(bool is_parrying_) { is_parrying = is_parrying_; }

	int get_health() const { return health; }
	void set_health(int p_health) { health = p_health; }

	int get_max_health() const { return max_health; }
	void set_max_health(int p_max_health) { max_health = p_max_health; }

	bool get_is_invulnerable() const;
	void set_is_invulnerable(bool p_is_invulnerable);

	bool is_dead() const {
		return health == 0;
	}

	friend class HBAgentState;
};

VARIANT_ENUM_CAST(HBAgent::AgentInputAction);

#endif // AGENT_H
