/**************************************************************************/
/*  agent_state.h                                                         */
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
#include "modules/game/hit_stop.h"
#include "scene/resources/3d/box_shape_3d.h"
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
#ifdef DEBUG_ENABLED
	HBDebugGeometry *debug_geo = nullptr;
	HBDebugGeometry *_get_debug_geo();
	bool draw_debug_geometry = true;
	bool ui_settings_init = false; // Lazily initialize loading of settings
	void _init_ui_settings_if_needed();
#endif
protected:
#ifdef DEBUG_ENABLED
	void debug_draw_shape(Ref<Shape3D> p_shape, const Vector3 &p_position, const Color &p_color = Color());
	void debug_draw_clear();
	void debug_draw_raycast(const PhysicsDirectSpaceState3D::RayParameters &p_params, const Color &p_color = Color());
	void debug_draw_sphere(const Vector3 &p_position, float p_radius = 0.05f, const Color &p_color = Color());
	void debug_draw_line(const Vector3 &p_from, const Vector3 &p_to, const Color &p_color = Color());
	void debug_draw_cast_motion(const Ref<Shape3D> &p_shape, const PhysicsDirectSpaceState3D::ShapeParameters &p_shape_cast_3d, const Color &p_color = Color());
	HBDebugGeometry *get_debug_geometry();
#else
	void debug_draw_shape(Ref<Shape3D> p_shape, const Vector3 &p_position, const Color &p_color = Color()){};
	void debug_draw_clear(){};
	void debug_draw_raycast(const PhysicsDirectSpaceState3D::RayParameters &p_params, const Color &p_color = Color()){};
	void debug_draw_sphere(const Vector3 &p_position, float p_radius = 0.05f, const Color &p_color = Color()){};
	void debug_draw_line(const Vector3 &p_from, const Vector3 &p_to, const Color &p_color = Color()){};
	void debug_draw_cast_motion(const Ref<Shape3D> &p_shape, const PhysicsDirectSpaceState3D::ShapeParameters &p_shape_cast_3d, const Color &p_color = Color()){};
	HBDebugGeometry *get_debug_geometry() { return nullptr; };
#endif
	bool find_facing_wall(PhysicsDirectSpaceState3D::RayResult &p_result) const;
	void _transition_to_short_hop(const Vector3 &p_target_point, const StringName p_next_state, const Dictionary &p_next_state_args = {});
	bool whisker_reach_check(const Vector3 &p_from, const Vector3 &p_target, const float p_height_start, const float p_height_end);
	static void _bind_methods();

	void handle_player_specific_inputs();

	virtual void _on_attack_received(HBAgent *p_attacker, Ref<HBAttackData> p_attack_data);
	void setup_attack_reception();
	void remove_attack_reception();
	bool handle_parrying();

public:
	HBAgent *get_agent() const;
	EPASController *get_epas_controller() const;
	Ref<EPASTransitionNode> get_movement_transition_node() const;
	Ref<EPASInertializationNode> get_inertialization_node() const;
	Skeleton3D *get_skeleton() const;
	Node3D *get_graphics_node() const;
	Ref<EPASSoftnessNode> get_softness_node() const;
	Ref<EPASWheelLocomotion> get_wheel_locomotion_node() const;
	HBAgent *get_highlighted_agent() const;
	void exit_combat();
#ifdef DEBUG_ENABLED
	virtual void debug_ui_draw() override;
#endif
	friend class HBStateMachine;
};

class HBAgentGroundStateBase : public HBAgentState {
	GDCLASS(HBAgentGroundStateBase, HBAgentState);

protected:
	bool try_vault_over_obstacle();
	bool handle_parkour_up();
	bool handle_parkour_mid();
	bool handle_parkour_down();
	bool handle_parkour();
	bool handle_jump_to_ledge(Ref<BoxShape3D> p_test_shape, const Transform3D &p_shape_trf);
	bool handle_jump_to_parkour_point(Ref<BoxShape3D> p_shape, const Transform3D &p_shape_trf);
	// Wall trf = wall base transform facing away from the wall
	bool handle_wallrun_to_parkour_point(const Ref<Shape3D> &p_shape, const Transform3D &p_shape_trf, const Transform3D &p_wall_base_trf);
};

class HBAgentMoveState : public HBAgentGroundStateBase {
	GDCLASS(HBAgentMoveState, HBAgentGroundStateBase);

public:
	enum MoveStateParams {
		PARAM_TRANSITION_DURATION,
		PARAM_WAIT_FOR_TRANSITION
	};
	Vector3 velocity_spring_acceleration;

	bool wait_for_transition = false;

protected:
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void physics_process(float p_delta) override;
	void _update_lookat();
	void _on_agent_edge_hit();
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
	float target_angle;

	void _on_animation_finished();

public:
	enum TurnStateParams {
		PARAM_ANGLE
	};

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
	bool are_limbs_init = false;
	void _init_limbs();

public:
	enum WallGrabbedParams {
		PARAM_LEDGE,
	};
	void _update_ik_transforms(AgentProceduralAnimator::AgentProceduralPose &p_pose);
	void _apply_ik_transforms(AgentProceduralAnimator::AgentProceduralPose &p_pose, bool p_inertialize_graphics_trf = false);
	void _update_animator();
	bool _handle_getup();
	bool _handle_parkour_mid();
	bool _handle_drop_to_parkour_point();

public:
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void physics_process(float p_delta) override;
#ifdef DEBUG_ENABLED
	virtual void debug_ui_draw() override;
#endif
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

class HBAgentWallParkourStateNew : public HBAgentState {
	GDCLASS(HBAgentWallParkourStateNew, HBAgentState);
	AgentProceduralAnimator animator;

public:
	struct WallParkourInitialPose {
		AgentProceduralAnimator::AgentProceduralPose pose;
		HBAgentParkourPoint *parkour_points[AgentProceduralAnimator::LIMB_MAX] = { nullptr };
	};
	enum WallParkourParams {
		PARAM_PARKOUR_NODE
	};
	static constexpr float SHORT_GRAB_REACH = 0.75f;
	static constexpr float LONG_GRAB_REACH = 1.5f;

private:
	WallParkourInitialPose target_pose;
	AgentProceduralAnimator::AgentProceduralAnimOptions animator_options;
	// Parkour point currently being moved towards
	HBAgentParkourPoint *current_parkour_point_target = nullptr;
	// Limb currently being moved
	AgentProceduralAnimator::AgentLimb current_limb;

	// Back straightness
	float back_straightness_target = 0.0f;
	float back_straightness_velocity = 0.0f;
	Ref<EPASBlendNode> back_straightness_blend_node;

	float straight_side_target = 0.0f;
	float straight_side_velocity = 0.0f;
	Ref<EPASBlendNode> straight_side_blend_node;

	void apply_offsets(AgentProceduralAnimator::AgentProceduralPose &p_pose) const;
	void apply_pose(AgentProceduralAnimator::AgentProceduralPose &p_pose);
	void update_animator(const AgentProceduralAnimator::AgentLimb p_limb_to_move);
	void update_animator_initial();
	void bring_hands_together();
	void calculate_magnet_for_limbs(AgentProceduralAnimator::AgentProceduralPose &p_pose);
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void physics_process(float p_delta) override;
	bool _handle_parkour_mid();
	HBAgentParkourPoint *find_reachable_parkour_point(const AgentProceduralAnimator::AgentLimb p_sampling_limb, const Vector3 &p_direction, const float &p_reach) const;
	bool find_reachable_parkour_ledge(const AgentProceduralAnimator::AgentLimb p_sampling_limb, const Vector3 &p_direction, const float &p_reach, HBAgentParkourLedge **r_ledge, float &r_offset);

public:
	void find_initial_pose(WallParkourInitialPose &p_pose, const HBAgentParkourPoint *p_point) const;
	HBAgentWallParkourStateNew();
};

class HBAgentParkourBeamWalk : public HBAgentGroundStateBase {
	GDCLASS(HBAgentParkourBeamWalk, HBAgentGroundStateBase);
	HBAgentParkourBeam *beam = nullptr;
	Ref<PositionInertializer> pos_inertializer;

public:
	enum ParkourBeamWalkParams {
		PARAM_BEAM_NODE
	};
	float curve_offset = 0.0f;
	Vector3 agent_global_position;
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override{};
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
	bool inertialization_init = false;
	bool collisions_enabled = false;

public:
	enum RootMotionParams {
		PARAM_WARP_POINTS,
		PARAM_PREV_POSITION,
		PARAM_ANIMATION_NODE_NAME,
		PARAM_TRANSITION_NODE_INDEX,
		PARAM_NEXT_STATE,
		PARAM_NEXT_STATE_ARGS,
		PARAM_VELOCITY_MODE,
		PARAM_COLLIDE,
		PARAM_INVULNERABLE,
		PARAM_MAX
	};
	bool hack = false;
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void physics_process(float p_delta) override;
	virtual void animation_finished(float p_delta);
#ifdef DEBUG_ENABLED
	virtual void debug_ui_draw() override;
#endif
};

class HBAgentWallTransitionState : public HBAgentState {
	GDCLASS(HBAgentWallTransitionState, HBAgentState);

	const float TRANSITION_DURATION = 0.5f;

public:
	enum TransitionMode {
		TO_WALL,
		TO_LEDGE,
	};

	enum WallParkourTransitionParams {
		PARAM_TRANSITION_MODE,
		PARAM_LEDGE_OFFSET,
		PARAM_LEDGE,
		PARAM_FIRST_HAND,
		PARAM_PARKOUR_POINT
	};

	float ledge_offset = 0.0f;
	HBAgentParkourLedge *ledge = nullptr;
	HBAgentParkourPoint *parkour_point = nullptr;

	AgentProceduralAnimator animator;
	AgentProceduralAnimator::AnimatorState initial_animator_state;

	AgentProceduralAnimator::AgentProceduralPose starting_pose;
	AgentProceduralAnimator::AgentProceduralAnimOptions animator_options;
	AgentProceduralAnimator::AgentLimb first_hand;

	Vector3 relative_magnet_pos[AgentProceduralAnimator::LIMB_MAX];

private:
	TransitionMode transition_mode;
	void apply_animator_pose(bool p_inertialize_transform = false);
	void calculate_magnet_for_limbs(AgentProceduralAnimator::AgentProceduralPose &p_pose);

public:
	void set_starting_pose(const AgentProceduralAnimator::AgentProceduralPose &p_starting_pose);
	virtual void enter(const Dictionary &p_args) override;
	virtual void physics_process(float p_delta) override;
};

class HBAgentCombatMoveState : public HBAgentState {
	GDCLASS(HBAgentCombatMoveState, HBAgentState);
	Quaternion agent_rotation_target;
	Vector3 agent_rotation_velocity;

	float agent_rotation_halflife = 0.1f;

	HBAgent *target = nullptr;
	Vector3 desired_velocity_spring_acceleration;
	Vector3 desired_velocity_ws;
	void calculate_desired_movement_velocity(float p_delta);
	void rotate_towards_target(float p_delta);
	void update_orientation_warp();
	bool handle_attack();
	void reset();

public:
	enum CombatMoveParams {
		PARAM_TARGET
	};
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void physics_process(float p_delta) override;
};

class HBAgentCombatAttackState : public HBAgentRootMotionState {
	GDCLASS(HBAgentCombatAttackState, HBAgentRootMotionState);

	bool attack_connected = false;
	HBAgent *target = nullptr;
	Ref<HBAttackData> attack;
	MeshInstance3D *attack_mesh_instance = nullptr;
	Ref<EPASOneshotAnimationNode> animation_node;
	float prev_playback_position = 0.0f;
	Ref<HitStopSolver> hit_stop_solver;
	// When attacking cancelling requires you to repress the attack button while the attack is happening
	bool was_attack_repressed = false;

private:
	bool handle_attack();
	void handle_hitstop(float p_delta);
	void handle_sending_attack_to_target();
	void _on_attack_parried();
	void _on_target_died();

public:
	enum CombatAttackParams {
		PARAM_ATTACK_NAME = HBAgentRootMotionState::PARAM_MAX + 1,
		PARAM_TARGET
	};
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void physics_process(float p_delta) override;
	virtual void animation_finished(float p_delta) override;
};

class HBAgentCombatHitState : public HBAgentRootMotionState {
	GDCLASS(HBAgentCombatHitState, HBAgentRootMotionState);
	Ref<HBAttackData> attack;
	HBAgent *attacker;
	Ref<HitStopSolver> hit_stop_solver;

public:
	enum CombatHitStateParams {
		PARAM_ATTACK,
		PARAM_ATTACKER,
	};
	void look_towards_attacker();
	void setup_animation();

public:
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void physics_process(float p_delta) override;
};

class HBAgentDeadState : public HBAgentState {
	GDCLASS(HBAgentDeadState, HBAgentState);

public:
	enum DeadStateParams {
		PARAM_DEATH_FORCE
	};
	virtual void enter(const Dictionary &p_args) override;
};

#endif // AGENT_STATE_H
