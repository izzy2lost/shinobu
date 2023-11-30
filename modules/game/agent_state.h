#ifndef AGENT_STATE_H
#define AGENT_STATE_H

#include "agent.h"
#include "agent_procedural_anim.h"
#include "animation_system/epas_blend_node.h"
#include "animation_system/epas_ik_node.h"
#include "animation_system/epas_inertialization_node.h"
#include "animation_system/epas_oneshot_animation_node.h"
#include "animation_system/epas_softness_node.h"
#include "animation_system/epas_transition_node.h"
#include "animation_system/epas_wheel_locomotion.h"
#include "debug_geometry.h"
#include "ledge_traversal_controller.h"
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
	bool _try_slide_edge(Vector3 p_dir);
	bool _try_find_parkour_point(Vector3 p_dir, float p_length, float p_radius, HBAgentParkourPoint **p_out_parkour_point);

protected:
	void debug_draw_shape(Ref<Shape3D> p_shape, const Vector3 &p_position, const Color &p_color = Color());
	void debug_draw_clear();
	void debug_draw_raycast(const PhysicsDirectSpaceState3D::RayParameters &p_params, const Color &p_color = Color());
	void debug_draw_sphere(const Vector3 &p_position, float p_radius = 0.05f, const Color &p_color = Color());
	void debug_draw_line(const Vector3 &p_from, const Vector3 &p_to, const Color &p_color = Color());
	void debug_draw_cast_motion(const Ref<Shape3D> &p_shape, const PhysicsDirectSpaceState3D::ShapeParameters &p_shape_cast_3d, const Color &p_color = Color());
	bool find_facing_wall(PhysicsDirectSpaceState3D::RayResult &p_result);
	bool find_ledge(const Vector3 &p_wall_base_point, const Vector3 &p_wall_normal, Transform3D &p_ledge_transform);
	bool find_lege_wall_sweep(const Vector3 &p_from, const Vector3 &p_to, const Vector3 &p_offset, Transform3D &p_out_edge_trf, HBAgentParkourLedge **p_out_ledge, int p_iterations = 5);
	HBDebugGeometry *get_debug_geometry();
	bool handle_autojump_mid(Vector3 p_dir, bool p_needs_gap = false);
	bool handle_autojump_down(Vector3 p_dir, bool p_needs_gap = false);

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

public:
	enum MoveStateParams {
		PARAM_TRANSITION_DURATION
	};

protected:
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void physics_process(float p_delta) override;
	void _update_lookat();
	void _on_agent_edge_hit();
	bool _try_vault_over_obstacle();
	void _transition_to_short_hop(const Vector3 &p_target_point, const StringName p_next_state, const Dictionary &p_next_state_args = {});
	bool _handle_parkour_up();
	bool _handle_parkour_mid();
	bool _handle_jump_to_ledge(Ref<Shape3D> p_shape, const Transform3D &p_shape_trf);
	bool _handle_parkour();
	bool _handle_parkour_down();
	bool _check_wall(PhysicsDirectSpaceState3D::RayResult &p_result);
};

VARIANT_ENUM_CAST(HBAgentMoveState::MoveStateParams);

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
	HBAgentParkourLedge *parkour_ledge_target = nullptr;
	Transform3D ledge_transform;
	bool autojump_queued = false;
	Vector3 autojump_dir;

public:
	enum WallrunParams {
		PARAM_BASE,
		PARAM_EDGE,
		PARAM_WALLRUN_TYPE,
		PARAM_TARGET_PARKOUR_NODE,
		PARAM_TARGET_PARKOUR_LEDGE
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
	virtual void physics_process(float p_delta) override;
};
class HBAgentLedgeGrabbedStateNew : public HBAgentState {
	GDCLASS(HBAgentLedgeGrabbedStateNew, HBAgentState);
	HBLedgeTraversalController *controller = nullptr;
	AgentProceduralAnimator animator;
	AgentProceduralAnimator::AgentProceduralAnimOptions animator_options;
	struct limbs {
		Ref<EPASIKNode> ik_node;
		StringName bone_name;
		StringName magnet_name;
		bool dangle_status = false;
	} limbs[AgentProceduralAnimator::LIMB_MAX];

	Vector3 hand_offset_euler;
	HBAgentParkourLedge *ledge = nullptr;

public:
	enum WallGrabbedParams {
		PARAM_LEDGE,
	};
	void _update_ik_transforms(AgentProceduralAnimator::AgentProceduralPose &p_pose);
	void _apply_ik_transforms(AgentProceduralAnimator::AgentProceduralPose &p_pose, bool p_inertialize_graphics_trf = false);
	void _update_animator();
	bool _handle_getup();

public:
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void physics_process(float p_delta) override;
	virtual void debug_ui_draw() override;
	void get_initial_pose_for_edge(HBAgentParkourLedge *p_ledge, AgentProceduralAnimator::AgentProceduralPose &p_pose, float p_offset);

	HBAgentLedgeGrabbedStateNew();
	~HBAgentLedgeGrabbedStateNew();
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

		bool dangling = false;

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
	HBAgentParkourLedge *ledge_node = nullptr;
	float ledge_offset = 0.0f;
	Dictionary target_state_args;
	StringName target_state_name;

public:
	enum AutoJumpParams {
		PARAM_TYPE,
		PARAM_LEDGE,
		PARAM_LEDGE_OFFSET,
		PARAM_TARGET_STATE_NAME,
		PARAM_TARGET_STATE_ARGS
	};
	enum AutoJumpType {
		AUTOJUMP_WORLD,
		AUTOJUMP_LEDGE,
		AUTOJUMP_TO_STATE,
	};

	AutoJumpType type;

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

class HBAgentRootMotionState : public HBAgentState {
	GDCLASS(HBAgentRootMotionState, HBAgentState);

public:
	enum VelocityMode {
		ANIMATION_DRIVEN,
		CONSERVE
	};

private:
	VelocityMode velocity_mode;
	Ref<EPASOneshotAnimationNode> animation_node;
	Dictionary next_state_args;
	StringName next_state;
	Vector3 prev_pos;
	Vector3 prev_prev_pos;
	Vector3 starting_velocity;

public:
	enum RootMotionParams {
		PARAM_WARP_POINTS,
		PARAM_PREV_POSITION,
		PARAM_ANIMATION_NODE_NAME,
		PARAM_TRANSITION_NODE_INDEX,
		PARAM_NEXT_STATE,
		PARAM_NEXT_STATE_ARGS,
		PARAM_VELOCITY_MODE
	};
	virtual void enter(const Dictionary &p_args) override;
	virtual void physics_process(float p_delta) override;
};

#endif // AGENT_STATE_H
