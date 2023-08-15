#ifndef AGENT_STATE_H
#define AGENT_STATE_H

#include "agent.h"
#include "animation_system/epas_blend_node.h"
#include "animation_system/epas_ik_node.h"
#include "animation_system/epas_inertialization_node.h"
#include "animation_system/epas_oneshot_animation_node.h"
#include "animation_system/epas_softness_node.h"
#include "animation_system/epas_transition_node.h"
#include "animation_system/epas_wheel_locomotion.h"
#include "debug_geometry.h"
#include "modules/game/agent_parkour.h"
#include "scene/animation/tween.h"
#include "scene/resources/cylinder_shape_3d.h"
#include "state_machine.h"

enum AgentIKLimbType {
	HAND_LEFT,
	HAND_RIGHT,
	FOOT_LEFT,
	FOOT_RIGHT,
	LIMB_TYPE_MAX
};

struct AgentIKPose {
	Transform3D target_transforms[AgentIKLimbType::LIMB_TYPE_MAX];
	// Magnet positions, in skeleton space
	Vector3 magnet_positions[AgentIKLimbType::LIMB_TYPE_MAX];
	Transform3D actor_transform;
	bool dangling[AgentIKLimbType::LIMB_TYPE_MAX];
	bool valid = false;
};

class HBAgentState : public HBStateMachineState {
	GDCLASS(HBAgentState, HBStateMachineState);
	HBDebugGeometry *debug_geo = nullptr;
	HBDebugGeometry *_get_debug_geo();
	bool draw_debug_geometry = true;
	bool ui_settings_init = false; // Lazily initialize loading of settings
	void _init_ui_settings_if_needed();

protected:
	void debug_draw_shape(Ref<Shape3D> p_shape, const Vector3 &p_position, const Color &p_color = Color());
	void debug_draw_clear();
	void debug_draw_raycast(const PhysicsDirectSpaceState3D::RayParameters &p_params, const Color &p_color = Color());
	void debug_draw_sphere(const Vector3 &p_position, float p_radius = 0.05f, const Color &p_color = Color());
	void debug_draw_line(const Vector3 &p_from, const Vector3 &p_to, const Color &p_color = Color());
	void debug_draw_cast_motion(const Ref<Shape3D> &p_shape, const PhysicsDirectSpaceState3D::ShapeParameters &p_shape_cast_3d, const Color &p_color = Color());
	bool find_facing_wall(PhysicsDirectSpaceState3D::RayResult &p_result);
	bool find_ledge(const Vector3 &p_wall_base_point, const Vector3 &p_wall_normal, Transform3D &p_ledge_transform);
	HBDebugGeometry *get_debug_geometry();

public:
	HBAgent *get_agent() const;
	EPASController *get_epas_controller() const;
	Ref<EPASTransitionNode> get_movement_transition_node() const;
	Ref<EPASInertializationNode> get_inertialization_node() const;
	Skeleton3D *get_skeleton() const;
	Node3D *get_graphics_node() const;
	Ref<EPASSoftnessNode> get_softness_node() const;
	Ref<EPASWheelLocomotion> get_wheel_locomotion_node() const;
	virtual void debug_ui_draw() override;
	friend class HBStateMachine;
};

class HBAgentMoveState : public HBAgentState {
	GDCLASS(HBAgentMoveState, HBAgentState);

protected:
	virtual void enter(const Dictionary &p_args) override;
	virtual void physics_process(float p_delta) override;
	bool _handle_parkour_down();
	bool _handle_parkour_up();
	bool _handle_parkour_mid();
	bool _check_wall(PhysicsDirectSpaceState3D::RayResult &p_result);
};

class HBAgentVaultState : public HBAgentState {
	GDCLASS(HBAgentVaultState, HBAgentState);

	Ref<EPASOneshotAnimationNode> animation_node;
	Vector3 prev_position;

private:
	void _on_animation_finished();

protected:
	virtual void enter(const Dictionary &p_args) override;
	virtual void process(float p_delta) override;
};

class HBAgentTurnState : public HBAgentState {
	GDCLASS(HBAgentTurnState, HBAgentState);
	Ref<EPASOneshotAnimationNode> animation_node;
	Quaternion starting_rot;
	Quaternion target_rot;
	float anim_length = 0.0f;
	float time = 0.0f;
	void _on_animation_finished();

protected:
	virtual void enter(const Dictionary &p_args) override;
	virtual void process(float p_delta) override;
};

class HBAgentWallrunState : public HBAgentState {
	GDCLASS(HBAgentWallrunState, HBAgentState);
	Ref<EPASOneshotAnimationNode> animation_node;
	EPASOneshotAnimationNodeDebug *dbg = nullptr;
	Vector3 ledge_position;
	Vector3 ledge_normal;
	StaticBody3D *parkour_point_target = nullptr;
	Transform3D ledge_transform;

public:
	enum WallrunParams {
		PARAM_BASE,
		PARAM_EDGE,
		PARAM_WALLRUN_TYPE,
		PARAM_TARGET_PARKOUR_NODE
	};
	enum WallrunType {
		TO_LEDGE,
		TO_PARKOUR_POINT,
		EMPTY_CLIMB
	};

private:
	int wallrun_type = WallrunType::EMPTY_CLIMB;

protected:
	virtual void enter(const Dictionary &p_args) override;
	virtual void process(float p_delta) override;
};

class HBAgentLedgeGrabbedState : public HBAgentState {
	GDCLASS(HBAgentLedgeGrabbedState, HBAgentState);

public:
	enum WallGrabbedParams {
		PARAM_LEDGE_TRF,
	};

private:
	enum LedgeIKPointRaycastType {
		RAYCAST_HAND,
		RAYCAST_FOOT,
		RAYCAST_MAX
	};
	struct LedgeIKLimb {
		Ref<EPASIKNode> ik_node;
		Vector3 position;
		Vector3 target_position;
		int bone_idx;
		// Magnet pos in local space
		Vector3 magnet_position;
		// Hand raycasts are different, they try to find the ledge, while foot raycasts just
		// try to find a point to place the foot upon.
		LedgeIKPointRaycastType raycast_type;
		// Raycast origin used to find the edge, in local space
		Vector3 raycast_origin;
		// Raycast target used to find the edge, in local space
		Vector3 raycast_target;
		// Offset applied to the edge
		Vector3 target_offset;
		Ref<Tween> tween;
		// Normal of the wall the limb is resting against
		Vector3 wall_normal;
		// For hands we also know the normal of the ledge (AKA pointing up)
		Vector3 ledge_normal;
		// Bone transform in skeleton space, used to aim stuff
		Transform3D bone_base_trf;
		float start_time = 0.0f;
		float end_time = 0.0f;
		Color debug_color;
		bool is_dangling = false;

		float get_weight(float p_time) const {
			if (p_time <= 0.0f || p_time >= 1.0f) {
				return 1.0f;
			}
			float w = 1.0f - sin(p_time * Math_PI);
			return w * w;
		}
	};

	float animation_time = 0.0f;
	int animation_direction = 1; // Direction in which the animation should be played
								 // it is okay if the animationa and movement direction
								 // it doesn't look too bad, it's just eyecandy :P

	// Velocity spring values
	float ledge_movement_velocity = 0.0f;
	float ledge_movement_acceleration = 0.0f;

	Vector<LedgeIKLimb> ledge_ik_points;

	Vector3 animation_root_offset;
	Vector3 target_agent_position;
	float target_spring_halflife = 0.175f;
	Vector3 target_agent_spring_velocity;
	Vector3 limb_rotation_spring_velocity;
	float limb_rotation_spring_halflife = 0.175;

	bool inertialization_finished = false;

	struct IKDebugInfo { // Debug graph info
		bool ui_config_init = false;
		static const int IK_OFFSET_PLOT_SIZE = 90;
		float plot_time = 0.0f;
		float graph_x_time[IK_OFFSET_PLOT_SIZE] = { 0.0f };
		float ik_tip_influence_y_graph[AgentIKLimbType::LIMB_TYPE_MAX][IK_OFFSET_PLOT_SIZE] = { { 0.0f } };
		float ik_hip_offset_graph[IK_OFFSET_PLOT_SIZE] = { 0.0f };
		// We do this because the physics threads updates at a different rate than the process thread the UI uses
		bool last_data_is_valid = true;
		// This is here because we migh do more than one physics step between
		float physics_time_delta = 0.0f;
		float last_tip_weights[AgentIKLimbType::LIMB_TYPE_MAX] = { 0.0f };
		float last_hip_offset_x = 0.0f;
		Vector2 transformed_movement_input;
		bool show_limb_raycasts = false;
		bool show_center_raycast = false;
	} ik_debug_info;

	Transform3D _get_ledge_point_target_trf(const Transform3D &p_graphics_trf, const Vector3 &p_limb_position_world, const Vector3 &p_wall_normal) const;
	bool _find_ledge(const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_wall_normal, Vector3 &p_out_ledge_normal, const Color &p_debug_color) const;
	bool _find_ledge_sweep(const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_wall_normal, Vector3 &p_out_ledge_normal, const Color &p_debug_color, Vector3 p_sweep_offset, int p_sweep_iterations = 5) const;
	bool _find_wall_point(const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_normal, const Color &p_debug_color) const;
	void _init_ik_points();
	bool _handle_getup();
	void _debug_init_settings();
	bool _handle_transition_inputs();

protected:
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void physics_process(float p_delta) override;
	virtual void debug_ui_draw() override;

public:
	struct WallGrabbedStateInitialPoseParams {
		Transform3D ledge_transform;
	};
	struct LedgeAgentIKPose {
		AgentIKPose pose;
		Vector3 wall_normals[AgentIKLimbType::LIMB_TYPE_MAX];
		Vector3 ledge_normals[AgentIKLimbType::LIMB_TYPE_MAX];
		Transform3D ledge_transforms[AgentIKLimbType::LIMB_TYPE_MAX];
	};
	bool find_initial_pose(LedgeAgentIKPose &p_pose, const WallGrabbedStateInitialPoseParams &p_params) const;
	Vector3 calculate_animation_root_offset();
	Transform3D _get_limb_ik_target_trf(const Transform3D &p_graphics_trf, const Transform3D &p_ledge_trf, const Transform3D &p_bone_base_trf, const Vector3 &p_bone_offset) const;
};

class HBAgentFallState : public HBAgentState {
	GDCLASS(HBAgentFallState, HBAgentState);

protected:
	virtual void enter(const Dictionary &p_args) override;
	virtual void physics_process(float p_delta) override;
};

class HBAgentLedgeGetUpState : public HBAgentState {
	GDCLASS(HBAgentLedgeGetUpState, HBAgentState);
	Ref<EPASOneshotAnimationNode> animation_node;
	Vector3 prev_position;

	void _on_animation_finished();
	StringName target_state;
	Dictionary target_state_args;

public:
	enum LedgeGetUpParams {
		TARGET_STATE,
		TARGET_STATE_ARGS
	};

protected:
	void enter(const Dictionary &p_args) override;
	void process(float p_delta) override;
};

class HBAgentWallParkourState : public HBAgentState {
	GDCLASS(HBAgentWallParkourState, HBAgentState);
	const float ANIMATION_DURATION = 0.4f;

	StaticBody3D *parkour_node;

	// Final agent target when doing a ledge transition
	Transform3D ledge_transition_agent_target;

	enum WallParkourTargetType {
		TARGET_LOCATION, // Used for feet when a place to put them can't be found
		TARGET_PARKOUR_NODE // Used for legs and hands when they are grabbed into a parkour point
	};

	float animation_time = -1.0f;
	// automatically calculated offset from the center of all limbs, with forward being towards the wall
	Vector3 agent_offset_base;
	// Manual adjustment for agent_offset, in skeleton space
	const Vector3 agent_offset_adjustment = Vector3(0.0f, 0.1f, 0.15f);
	// Current offset of the agent from the center of all limbs, in world space
	Vector3 agent_offset_target;

	// The target position of the agent is on a spring
	Vector3 agent_position_target;
	Vector3 agent_position_spring_velocity;

	// Rotation spring
	Quaternion graphics_rotation_spring_target;
	Vector3 graphics_rotation_spring_velocity;
	float graphics_rotation_spring_halflife = 0.1f;

	Ref<EPASBlendNode> back_straightness_blend_node;
	const float target_leg_height_from_node = 0.65f;

	struct WallParkourLimb {
		Ref<EPASIKNode> ik_node;
		StringName bone_name;
		// Visual offset, in graphics node space
		Vector3 visual_offset;
		// Base bone transform in skeleton space
		Transform3D bone_base_trf;
		// Magnet position, in skeleton space
		Vector3 local_magnet_pos;
		// Magnet position at the start of the state, in skeleton space
		Vector3 default_magnet_pos;
		// Transform of the current point, with forward pointing towards the agent
		struct {
			WallParkourTargetType type = WallParkourTargetType::TARGET_LOCATION;
			HBAgentParkourPoint *parkour_node;
			Transform3D transform;
		} current;
		// Transform of the target point, with forward pointing towards the agent
		struct {
			WallParkourTargetType type = WallParkourTargetType::TARGET_LOCATION;
			HBAgentParkourPoint *parkour_node;
			Transform3D transform;
		} target;
		// Target IK transform of the limb in world space
		Transform3D target_ik_transform;
		Transform3D current_ik_transform;
		// How much to offset the position of the limb during traveling to a new destination
		// this is in skeleton space
		Vector3 animating_peak_offset;
		float animation_start = 0.0f;
		float animation_end = 0.0f;

		// These are used in case we can't find a target for feet
		// these are in graphics node space
		Vector3 fallback_location_raycast_start;
		Vector3 fallback_location_raycast_end;

		// The position of the limb is on a spring
		Vector3 position_spring_velocity;
		float position_spring_halflife = 0.1f;

		Transform3D get_current_transform() const {
			switch (current.type) {
				case WallParkourTargetType::TARGET_LOCATION: {
					return current.transform;
				} break;
				case (WallParkourTargetType::TARGET_PARKOUR_NODE): {
					return current.parkour_node->get_global_transform();
				}
			}
		};

		Transform3D get_target_transform() const {
			switch (target.type) {
				case WallParkourTargetType::TARGET_LOCATION: {
					return target.transform;
				} break;
				case (WallParkourTargetType::TARGET_PARKOUR_NODE): {
					return target.parkour_node->get_global_transform();
				}
			}
		};
	};

	Transform3D target_ledge_trf;

	enum ParkourStage {
		NORMAL, // Normal character wall movement
		TO_LEDGE_1, // Transitioning to a ledge, moving first limb
		TO_LEDGE_2, // Transitioning to a ledge, moving second limb
	};

	ParkourStage parkour_stage;

	bool limbs_init = false;

	WallParkourLimb parkour_limbs[LIMB_TYPE_MAX];

	// Shape used to check for nodes in a certain direction
	Ref<CylinderShape3D> dir_check_mesh;

protected:
	Transform3D _calculate_limb_target_transform(const Quaternion &p_target_graphics_rot, const Transform3D &p_target_point_trf, const Transform3D &p_bone_base_trf, Vector3 p_visual_offset = Vector3());
	Vector3 _calculate_limb_current_position(const WallParkourLimb &p_limb, bool p_use_visual_offset = true);
	Quaternion _calculate_target_graphics_node_rotation(const Transform3D &left_hand_target_transform, const Transform3D &right_hand_target_transform) const;
	void _handle_horizontal_parkour_movement(const Vector3 &p_movement_input);
	bool _handle_vertical_parkour_movement(const Vector3 &p_movement_input);
	bool _try_reach_ledge(WallParkourLimb *p_hand_to_move, WallParkourLimb *p_foot_to_move);
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void physics_process(float p_delta) override;
	virtual void debug_ui_draw() override;
	void _notification(int p_what);

public:
	enum WallParkourParams {
		PARAM_TARGET_PARKOUR_NODE
	};
	HBAgentWallParkourState();
};

class HBAgentParkourAutoJumpState : public HBAgentState {
	GDCLASS(HBAgentParkourAutoJumpState, HBAgentState);
	ParkourAutojump::AgentParkourAutojumpData autojump_data;
	float time;

public:
	void set_autojump_data(ParkourAutojump::AgentParkourAutojumpData p_autojump_data);
	virtual void enter(const Dictionary &p_args) override;
	virtual void physics_process(float p_delta) override;
};

class HBAgentParkourBeamWalk : public HBAgentState {
	GDCLASS(HBAgentParkourBeamWalk, HBAgentState);
	HBAgentParkourBeam *beam = nullptr;
	Ref<PositionInertializer> pos_inertializer;

public:
	enum ParkourBeamWalkParams {
		PARAM_BEAM_NODE,
		PARAM_PREV_POSITION
	};
	float curve_offset = 0.0f;
	Vector3 agent_global_position;
	virtual void enter(const Dictionary &p_args) override;
	virtual void physics_process(float p_delta) override;

	bool try_ledge_drop();

	Vector3 rotation_spring_vel;
	Quaternion rotation_spring_goal;

	Vector3 velocity_spring_accel;
	Vector3 velocity_spring_vel;
};

class HBAgentLedgeDropState : public HBAgentState {
	GDCLASS(HBAgentLedgeDropState, HBAgentState);
	Transform3D ledge_trf;
	Ref<EPASOneshotAnimationNode> animation_node;

public:
	enum LedgeDropParams {
		PARAM_EDGE
	};
	virtual void enter(const Dictionary &p_args) override;
	virtual void process(float p_delta) override;
};

#endif // AGENT_STATE_H
