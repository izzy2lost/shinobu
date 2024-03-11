/**************************************************************************/
/*  agent_state.cpp                                                       */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
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

#include "agent_state.h"
#include "agent_parkour.h"
#include "animation_system/epas_animation_node.h"
#include "modules/game/animation_system/epas_lookat_node.h"
#include "modules/game/animation_system/epas_orientation_warp_node.h"

#ifdef DEBUG_ENABLED
#include "implot.h"
#include "modules/imgui/godot_imgui.h"
#endif
#include "modules/tracy/tracy.gen.h"
#include "physics_layers.h"
#include "scene/3d/physics/physical_bone_3d.h"
#include "scene/resources/3d/cylinder_shape_3d.h"
#include "springs.h"

#include "agent_string_names.h"

#include "core/os/time.h"

bool is_wall(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) > p_floor_max_angle;
}

bool is_floor(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) <= p_floor_max_angle;
}

const auto ASN = AgentStringNames::get_singleton;

#ifdef DEBUG_ENABLED
HBDebugGeometry *HBAgentState::_get_debug_geo() {
	if (debug_geo == nullptr) {
		debug_geo = memnew(HBDebugGeometry);
		debug_geo->set_as_top_level(true);
		add_child(debug_geo, false, INTERNAL_MODE_FRONT);
		debug_geo->set_position(Vector3());
		_init_ui_settings_if_needed();
		debug_geo->set_visible(draw_debug_geometry);
		Array groups = GodotImGui::get_singleton()->get_config_value(state_machine, String(get_name()) + "/debug_geo_groups", Array());
		for (int i = 0; i < groups.size(); i++) {
			String group_name = groups[i];
			if (!group_name.is_empty()) {
				bool visible = GodotImGui::get_singleton()->get_config_value(state_machine, String(get_name()) + "/" + group_name + "_visible", true);
				debug_geo->add_group(group_name);
				debug_geo->set_group_visible(group_name, visible);
			}
		}
	}
	return debug_geo;
}

void HBAgentState::_init_ui_settings_if_needed() {
	if (!ui_settings_init) {
		draw_debug_geometry = GodotImGui::get_singleton()->get_config_value(state_machine, String(get_name()) + "/draw_debug_geometry", false);
		ui_settings_init = true;
	}
}

void HBAgentState::debug_draw_shape(Ref<Shape3D> p_shape, const Vector3 &p_position, const Color &p_color) {
	HBDebugGeometry *dbg_geo = _get_debug_geo();
	if (dbg_geo) {
		dbg_geo->debug_shape(p_shape, Transform3D(Basis(), p_position), p_color);
	}
}

void HBAgentState::debug_draw_clear() {
	HBDebugGeometry *dbg_geo = _get_debug_geo();
	if (dbg_geo) {
		dbg_geo->clear();
	}
}

void HBAgentState::debug_draw_raycast(const PhysicsDirectSpaceState3D::RayParameters &p_params, const Color &p_color) {
	HBDebugGeometry *dbg_geo = _get_debug_geo();
	if (dbg_geo) {
		dbg_geo->debug_raycast(p_params, p_color);
	}
}

void HBAgentState::debug_draw_sphere(const Vector3 &p_position, float p_radius, const Color &p_color) {
	HBDebugGeometry *dbg_geo = _get_debug_geo();
	if (dbg_geo) {
		dbg_geo->debug_sphere(p_position, p_radius, p_color);
	}
}

void HBAgentState::debug_draw_line(const Vector3 &p_from, const Vector3 &p_to, const Color &p_color) {
	HBDebugGeometry *dbg_geo = _get_debug_geo();
	if (dbg_geo) {
		dbg_geo->debug_line(p_from, p_to, p_color);
	}
}

void HBAgentState::debug_draw_cast_motion(const Ref<Shape3D> &p_shape, const PhysicsDirectSpaceState3D::ShapeParameters &p_shape_cast_3d, const Color &p_color) {
	HBDebugGeometry *dbg_geo = _get_debug_geo();
	if (dbg_geo) {
		dbg_geo->debug_cast_motion(p_shape, p_shape_cast_3d, p_color);
	}
}

HBDebugGeometry *HBAgentState::get_debug_geometry() {
	return _get_debug_geo();
}
#endif

bool HBAgentState::find_facing_wall(PhysicsDirectSpaceState3D::RayResult &p_result) const {
	// Finds a wall that is in front of the agent
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(agent == nullptr, false);

	if (agent->get_desired_movement_input_transformed().length() == 0.0f) {
		return false;
	}

	const float parkour_wall_check_distance = agent->get_agent_constants()->get_parkour_wall_check_distance();
	const float parkour_max_wall_facing_angle_degrees = Math::deg_to_rad(agent->get_agent_constants()->get_parkour_max_wall_facing_angle_degrees());
	const float floor_max_angle = agent->get_floor_max_angle();

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

	float mid_height = agent->get_height() * 0.25f;

	ray_params.from = agent->get_global_position() + Vector3(0.0f, mid_height, 0.0f);
	Vector3 movement_input_dir = agent->get_desired_movement_input_transformed();
	ray_params.to = ray_params.from + movement_input_dir * parkour_wall_check_distance;

	if (!dss->intersect_ray(ray_params, p_result) || !is_wall(p_result.normal, floor_max_angle)) {
		// Not a wall, abort
		return false;
	}

	if (p_result.normal.angle_to(-movement_input_dir) >= parkour_max_wall_facing_angle_degrees) {
		// We are heading towards a wall at a too wide of an angle, abort
		return false;
	}

	return true;
}

void HBAgentState::_transition_to_short_hop(const Vector3 &p_target_point, const StringName p_next_state, const Dictionary &p_next_state_args) {
	Dictionary root_motion_args;

	Dictionary warp_points;
	Transform3D wp_trf;
	wp_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), get_agent()->get_desired_movement_input_transformed().normalized());
	wp_trf.origin = p_target_point;

	warp_points[StringName("Ledge")] = wp_trf;
	root_motion_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MovementTransitionInputs::MOVEMENT_SHORT_HOP;
	root_motion_args[HBAgentRootMotionState::PARAM_VELOCITY_MODE] = HBAgentRootMotionState::VelocityMode::CONSERVE;
	root_motion_args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;
	root_motion_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->short_hop_animation_node;
	root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = p_next_state;
	root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = p_next_state_args;
	state_machine->transition_to(ASN()->root_motion_state, root_motion_args);
}

bool HBAgentState::whisker_reach_check(const Vector3 &p_from, const Vector3 &p_target, const float p_height_start, const float p_height_end) {
	HBAgent *agent = get_agent();
	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
	// Check if it's actually reachable by ray casting from the character at various heights
	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

	static int const constexpr WHISHKER_ITERS = 10;

	for (int j = 0; j < WHISHKER_ITERS; j++) {
		float percentage = j / (float)(WHISHKER_ITERS - 1);
		ray_params.from = p_from;
		ray_params.from.y += Math::lerp(p_height_start, p_height_end, percentage);
		ray_params.to = p_target;
		PhysicsDirectSpaceState3D::RayResult result;
		debug_draw_raycast(ray_params, Color("RED"));
		if (dss->intersect_ray(ray_params, result)) {
			return false;
		}
	}
	return true;
}

void HBAgentState::_bind_methods() {
	ClassDB::bind_method(D_METHOD("whisker_reach_check", "from", "to", "height_start", "height_end"), &HBAgentState::whisker_reach_check);
}

void HBAgentState::handle_player_specific_inputs() {
	HBAgent *agent = get_agent();
	if (!agent->get_is_player_controlled()) {
		return;
	}
	if (agent->is_action_just_pressed(HBAgent::INPUT_ACTION_TARGET)) {
		HBAgent *highlighted_agent = get_highlighted_agent();
		if (!highlighted_agent) {
			return;
		}
		agent->set_target(highlighted_agent);
		highlighted_agent->set_outline_mode(HBAgent::AgentOutlineMode::TARGET_COMBAT);
	}
}

void HBAgentState::_on_attack_received(HBAgent *p_attacker, Ref<HBAttackData> p_attack_data) {
	HBAgent *agent = get_agent();

	const uint64_t time = OS::get_singleton()->get_ticks_usec();
	const uint64_t last_parry_time = agent->get_last_action_press_time(HBAgent::INPUT_ACTION_PARRY);
	const bool is_parrying = agent->get_is_parrying();

	agent->set_is_parrying(false);
	if (is_parrying && time - last_parry_time < HBAgentConstants::PARRY_WINDOW) {
		// It's parrying time
		Dictionary args;
		args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->sword_parry_animation_node;
		args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MOVEMENT_COMBAT_PARRY;
		args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->combat_move_state;
		Dictionary next_state_args;

		// When parrying an attack we automatically target the attacker
		agent->set_target(p_attacker);

		args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = next_state_args;
		state_machine->transition_to(ASN()->root_motion_state, args);
		agent->emit_signal("parried");
		return;
	}

	agent->receive_damage(p_attack_data->get_damage());

	// We kicked the bucket, go to dead state
	if (agent->is_dead()) {
		Dictionary state_args;
		state_args[HBAgentDeadState::PARAM_DEATH_FORCE] = Vector3(0.0, 0.0, 0.0f);
		state_machine->transition_to(ASN()->dead_state, state_args);
		return;
	}

	// We didn't parry in time, and thus got hit
	Dictionary state_args;
	state_args[HBAgentCombatHitState::PARAM_ATTACKER] = p_attacker;
	state_args[HBAgentCombatHitState::PARAM_ATTACK] = p_attack_data;
	state_machine->transition_to(ASN()->combat_hit_state, state_args);
}

void HBAgentState::setup_attack_reception() {
	get_agent()->connect("attack_received", callable_mp(this, &HBAgentState::_on_attack_received));
}

void HBAgentState::remove_attack_reception() {
	get_agent()->disconnect("attack_received", callable_mp(this, &HBAgentState::_on_attack_received));
}

bool HBAgentState::handle_parrying() {
	HBAgent *agent = get_agent();
	// Parry is on release, blocking is on press
	if (agent->is_action_just_pressed(HBAgent::INPUT_ACTION_PARRY)) {
		agent->set_is_parrying(true);
		return true;
	}

	return false;
}

HBAgent *HBAgentState::get_agent() const {
	Node *actor = get_actor();
	ERR_FAIL_COND_V_MSG(!actor, nullptr, "Error getting the agent node: Couldn't be found");
	HBAgent *agent = Object::cast_to<HBAgent>(actor);
	ERR_FAIL_COND_V_MSG(!actor, nullptr, "Error getting the agent node: was not of HBAgent type");
	return agent;
}

EPASController *HBAgentState::get_epas_controller() const {
	ERR_FAIL_COND_V(get_agent() == nullptr, nullptr);
	return get_agent()->get_epas_controller_node();
}

Ref<EPASTransitionNode> HBAgentState::get_movement_transition_node() const {
	ERR_FAIL_COND_V(get_epas_controller() == nullptr, Ref<EPASTransitionNode>());
	return get_epas_controller()->get_epas_node(SNAME("MovementTransition"));
}

Ref<EPASInertializationNode> HBAgentState::get_inertialization_node() const {
	ERR_FAIL_COND_V(get_epas_controller() == nullptr, Ref<EPASInertializationNode>());
	return get_epas_controller()->get_epas_node(SNAME("Inertialization"));
}

Skeleton3D *HBAgentState::get_skeleton() const {
	ERR_FAIL_COND_V(get_epas_controller() == nullptr, nullptr);
	Skeleton3D *skel = get_epas_controller()->get_skeleton();
	return skel;
}

Node3D *HBAgentState::get_graphics_node() const {
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(!agent, nullptr);
	return agent->_get_graphics_node();
}

Ref<EPASSoftnessNode> HBAgentState::get_softness_node() const {
	ERR_FAIL_COND_V(get_epas_controller() == nullptr, Ref<EPASInertializationNode>());
	return get_epas_controller()->get_epas_node(SNAME("Softness"));
}

Ref<EPASWheelLocomotion> HBAgentState::get_wheel_locomotion_node() const {
	ERR_FAIL_COND_V(get_epas_controller() == nullptr, Ref<EPASWheelLocomotion>());
	return get_epas_controller()->get_epas_node(SNAME("MovementWheel"));
}

HBAgent *HBAgentState::get_highlighted_agent() const {
	HBAgent *agent = get_agent();
	Vector<HBAgent *> candidate_agents = agent->find_nearby_agents(40.0f);

	struct AgentTarget {
		HBAgent *agent;
		float distance_to_screen_center;
	};

	struct TargetComparator {
		_FORCE_INLINE_ bool operator()(const AgentTarget &a, const AgentTarget &b) const {
			return a.distance_to_screen_center < b.distance_to_screen_center;
		}
	};

	Vector<AgentTarget> targeteable_agents;
	agent->get_world_3d()->get_cameras();
	Camera3D *camera = get_viewport()->get_camera_3d();
	const Vector2 viewport_size = get_viewport()->get_visible_rect().size;
	const Vector2 viewport_center_normalized = Vector2(0.5f, 0.5f);
	// First we discard agents we cannot target
	for (int i = 0; i < candidate_agents.size(); i++) {
		// Target must be alive
		if (candidate_agents[i]->is_dead()) {
			continue;
		}
		// Agent must be in frustrum
		const Vector3 agent_position = candidate_agents[i]->get_global_position();
		if (!camera->is_position_in_frustum(agent_position)) {
			continue;
		}
		AgentTarget target;
		target.agent = candidate_agents[i];
		// While we are here, let's calculate how close we are to the center of the screen
		target.distance_to_screen_center = viewport_center_normalized.distance_to(camera->unproject_position(agent_position) / viewport_size);

		targeteable_agents.push_back(target);
	}

	if (targeteable_agents.size() == 0) {
		return nullptr;
	}

	// Sort them by closest to the center of the screen
	targeteable_agents.sort_custom<TargetComparator>();

	return targeteable_agents[0].agent;
}

void HBAgentState::exit_combat() {
	HBAgent *agent = get_agent();
	agent->set_target(nullptr);
	agent->emit_signal("exited_combat");
	state_machine->transition_to(ASN()->move_state);
}

#ifdef DEBUG_ENABLED
void HBAgentState::debug_ui_draw() {
	_init_ui_settings_if_needed();
	if (ImGui::Checkbox("Draw debug geometry", &const_cast<HBAgentState *>(this)->draw_debug_geometry)) {
		if (debug_geo) {
			debug_geo->set_visible(draw_debug_geometry);
		}
		GodotImGui::get_singleton()->set_config_value(state_machine, String(get_name()) + "/draw_debug_geometry", draw_debug_geometry);
		GodotImGui::get_singleton()->save_config();
	}
	if (debug_geo) {
		for (int i = 0; i < debug_geo->get_group_count(); i++) {
			StringName group_name = debug_geo->get_group_name(i);
			bool is_visible = debug_geo->get_group_visible(group_name);
			if (ImGui::Checkbox(String(group_name).utf8().get_data(), &is_visible)) {
				debug_geo->set_group_visible(group_name, is_visible);
				GodotImGui::get_singleton()->set_config_value(state_machine, String(get_name()) + group_name + "_visible", is_visible);
			}
		}
	}
}
#endif

/**********************
	MOVE STATE
***********************/

void HBAgentMoveState::enter(const Dictionary &p_args) {
	get_agent()->set_movement_mode(HBAgent::MovementMode::MOVE_GROUNDED);
	Ref<EPASTransitionNode> transition_node = get_movement_transition_node();
	ERR_FAIL_COND(!transition_node.is_valid());
	transition_node->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_MOVE);
	float x_blend = get_agent()->get_velocity().length() / get_agent()->get_agent_constants()->get_max_move_velocity();
	x_blend = CLAMP(x_blend, 0.0f, 1.0f);
	get_wheel_locomotion_node()->set_x_blend(0.0f);
	get_wheel_locomotion_node()->set_linear_velocity(Vector3());
	get_wheel_locomotion_node()->reset_foot_ik();
	velocity_spring_acceleration = Vector3();
	get_agent()->set_desired_velocity(Vector3());
	//get_inertialization_node()->inertialize(p_args.get(MoveStateParams::PARAM_TRANSITION_DURATION, 0.2f));
	get_agent()->connect("stopped_at_edge", callable_mp(this, &HBAgentMoveState::_on_agent_edge_hit));
	Ref<EPASLookatNode> torso_lookat_node = get_epas_controller()->get_epas_node("TorsoLookAt");
	Ref<EPASLookatNode> head_lookat_node = get_epas_controller()->get_epas_node("HeadLookAt");
	torso_lookat_node->set_influence(0.0f);
	head_lookat_node->set_influence(0.0f);
	get_softness_node()->set_influence(1.0f);
	get_wheel_locomotion_node()->set_use_foot_ik(false);
	Ref<EPASIKNode> left_foot_ik_node = get_epas_controller()->get_epas_node(ASN()->left_foot_ik_node);
	Ref<EPASIKNode> right_foot_ik_node = get_epas_controller()->get_epas_node(ASN()->right_foot_ik_node);

	wait_for_transition = p_args.get(PARAM_WAIT_FOR_TRANSITION, false);

	left_foot_ik_node->set_ik_influence(0.0f);
	right_foot_ik_node->set_ik_influence(0.0f);

	setup_attack_reception();
}

void HBAgentMoveState::exit() {
	remove_attack_reception();
	Ref<EPASIKNode> left_foot_ik_node = get_epas_controller()->get_epas_node(ASN()->left_foot_ik_node);
	Ref<EPASIKNode> right_foot_ik_node = get_epas_controller()->get_epas_node(ASN()->right_foot_ik_node);
	left_foot_ik_node->set_ik_influence(0.0f);
	right_foot_ik_node->set_ik_influence(0.0f);
	get_agent()->disconnect("stopped_at_edge", callable_mp(this, &HBAgentMoveState::_on_agent_edge_hit));
	Ref<EPASLookatNode> torso_lookat_node = get_epas_controller()->get_epas_node("TorsoLookAt");
	Ref<EPASLookatNode> head_lookat_node = get_epas_controller()->get_epas_node("HeadLookAt");
	torso_lookat_node->set_influence(0.0f);
	head_lookat_node->set_influence(0.0f);
	get_softness_node()->set_influence(0.0f);
}

void HBAgentMoveState::physics_process(float p_delta) {
	ZoneScopedN("HBAgentMoveState physics process");

	handle_player_specific_inputs();

	HBAgent *agent = get_agent();
	if (HBAgent *target = agent->get_target(); target) {
		Dictionary args;
		state_machine->transition_to(ASN()->combat_move_state, args);
		return;
	}

	if (wait_for_transition) {
		if (get_inertialization_node()->is_inertializing()) {
			return;
		}
		wait_for_transition = false;
	}

	Vector3 input_vector = agent->get_desired_movement_input_transformed();
	Vector3 target_desired_velocity = input_vector * agent->get_agent_constants()->get_max_move_velocity();
	Vector3 desired_velocity = agent->get_desired_velocity();
	HBSprings::velocity_spring_vector3(
			desired_velocity,
			velocity_spring_acceleration,
			target_desired_velocity,
			agent->get_agent_constants()->get_velocity_spring_halflife(),
			p_delta);
	agent->set_desired_velocity(desired_velocity);

	Ref<EPASIKNode> left_foot_ik_node = get_epas_controller()->get_epas_node(ASN()->left_foot_ik_node);
	Ref<EPASIKNode> right_foot_ik_node = get_epas_controller()->get_epas_node(ASN()->right_foot_ik_node);

	debug_draw_clear();
	Vector3 parkour_dir = agent->get_desired_movement_input_transformed().normalized();

	if (parkour_dir.is_normalized()) {
		if (handle_parkour()) {
			return;
		}
	}

	String bones[2] = { "hand.R", "hand.L" };

	for (int i = 0; i < 2; i++) {
		Transform3D bone_trf = get_skeleton()->get_global_transform() * get_skeleton()->get_bone_global_pose(get_skeleton()->find_bone(bones[i]));
		Vector3 origin = bone_trf.origin;
		Vector3 x = bone_trf.xform(Vector3(0.1f, 0.0, 0.0));
		Vector3 y = bone_trf.xform(Vector3(0.0f, 0.1, 0.0));
		Vector3 z = bone_trf.xform(Vector3(0.0f, 0.0, 0.1));

		debug_draw_line(origin, x, Color("Red"));
		debug_draw_line(origin, y, Color("Green"));
		debug_draw_line(origin, z, Color("Blue"));
	}

	Node3D *gn = get_graphics_node();
	Vector3 movement_input = agent->get_desired_movement_input_transformed();

	/*Ref<EPASLookatNode> torso_lookat_node = get_epas_controller()->get_epas_node("TorsoLookAt");
	Ref<EPASLookatNode> head_lookat_node = get_epas_controller()->get_epas_node("HeadLookAt");

	if (movement_input.length() > 0.0) {
		torso_lookat_node->set_target_world(get_agent()->get_global_position() + movement_input.normalized());
		head_lookat_node->set_target_world(get_agent()->get_global_position() + movement_input.normalized());
		if (torso_lookat_node->get_influence() == 0.0f) {
			torso_lookat_node->set_influence(1.0f);
			head_lookat_node->set_influence(1.0f);
			head_lookat_node->reset();
			torso_lookat_node->reset();
			get_inertialization_node()->inertialize();
		}
	} else {
		if (torso_lookat_node->get_influence() == 1.0f) {
			torso_lookat_node->set_influence(0.0f);
			head_lookat_node->set_influence(0.0f);
			get_inertialization_node()->inertialize();
		}
	}*/

	{
		Ref<EPASOrientationWarpNode> orientation_warp_node = get_epas_controller()->get_epas_node("Orientation");
		float angle = agent->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, -1.0f)).angle_to(movement_input);
		float rotation_angle = agent->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, -1.0f)).signed_angle_to(movement_input, Vector3(0.0f, 1.0f, 0.0f));
		if (movement_input.length_squared() > 0) {
			// Initiate turn if needed

			Vector3 facing_dir = agent->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, -1.0f));
			if (agent->get_linear_velocity().project(facing_dir).length() < get_agent()->get_agent_constants()->get_max_move_velocity() * 0.5f && angle > Math::deg_to_rad(90.0f)) {
				print_line("Angle was", Math::rad_to_deg(angle), movement_input, agent->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, -1.0f)));
				Dictionary args;
				args[HBAgentTurnState::PARAM_ANGLE] = Math::rad_to_deg(rotation_angle);
				orientation_warp_node->set_orientation_angle(0.0f);
				state_machine->transition_to(ASN()->turn_state, args);
				return;
			}
		}

		// Disable forward strafing when walking

		if (!agent->is_action_pressed(HBAgent::AgentInputAction::INPUT_ACTION_RUN)) {
			orientation_warp_node->set_orientation_angle(0.0f);
			Vector3 dir = movement_input.normalized();
			if (agent->is_action_just_released(HBAgent::INPUT_ACTION_RUN)) {
				Transform3D graphics_trf = agent->get_global_transform();
				graphics_trf.basis = agent->get_graphics_rotation();
				agent->inertialize_graphics_transform(graphics_trf, 0.15f);
			} else if (dir.is_normalized()) {
				Basis basis = Basis::looking_at(dir);
				agent->set_graphics_rotation(basis.get_rotation_quaternion());
			}
			if (movement_input.length_squared() > 0 && angle > Math::deg_to_rad(45.0f)) {
				if (dir.is_normalized()) {
					Transform3D graphics_trf = Transform3D(Basis::looking_at(dir).get_rotation_quaternion(), get_agent()->get_global_position());
					agent->inertialize_graphics_transform(graphics_trf, 0.25f);
				}
			}

		} else if (movement_input.length_squared() > 0) {
			// When running we can inertialize and reset the orientation warp every 45 degrees
			orientation_warp_node->set_orientation_angle(Math::rad_to_deg(rotation_angle));
			if (angle > Math::deg_to_rad(45.0f)) {
				Vector3 dir = movement_input.normalized();

				Transform3D trf = agent->get_global_transform();
				trf.basis = Basis::looking_at(dir);
				agent->inertialize_graphics_transform(trf, 0.25f);
			}
		}
	}

	if (gn && movement_input.length() > 0.0f) {
		Vector3 forward = gn->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
		forward.y = 0.0f;
		forward.normalize();

		float turn_threshold = Math::deg_to_rad(agent->get_agent_constants()->get_turn_animation_threshold_degrees());
		if (movement_input.angle_to(forward) > turn_threshold && agent->get_linear_velocity().length() < 0.1f) {
			//state_machine->transition_to("Turn");
		}

		PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
		PhysicsDirectSpaceState3D::ShapeParameters shape_params;
		shape_params.collide_with_areas = true;
		shape_params.collide_with_bodies = false;
		shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
		Ref<Shape3D> col_shape = agent->get_collision_shape();
		shape_params.shape_rid = col_shape->get_rid();
		shape_params.transform.origin = agent->get_global_position();

		Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
		results.resize(32);

		shape_params.transform.basis.scale(Vector3(1.5f, 1.5f, 1.5f));

		debug_draw_cast_motion(col_shape, shape_params);
		int result_count = dss->intersect_shape(shape_params, results.ptrw(), 32);
		for (int i = 0; i < result_count; i++) {
			HBAgentParkourBeam *beam = Object::cast_to<HBAgentParkourBeam>(results[i].collider);
			if (!beam) {
				continue;
			}

			Ref<Curve3D> curve = beam->get_curve();
			if (!curve.is_valid()) {
				continue;
			}

			float offset = curve->get_closest_offset(beam->to_local(agent->get_global_position()));

			Transform3D curve_sample = curve->sample_baked_with_rotation(offset);
			curve_sample = beam->get_global_transform() * curve_sample;

			Vector3 curve_forward = curve_sample.basis.xform(Vector3(0.0f, 0.0f, -1.0f));
			curve_forward.y = 0.0f;
			curve_forward.normalize();
			if (!curve_forward.is_normalized()) {
				continue;
			}
			debug_draw_sphere(curve_sample.origin);
			if (movement_input.angle_to(get_agent()->get_global_position().direction_to(curve_sample.origin)) > Math::deg_to_rad(90.0f)) {
				continue;
			}

			if (curve_sample.origin.is_equal_approx(get_agent()->get_global_position())) {
				continue;
			}

			Dictionary dict;
			dict[HBAgentParkourBeamWalk::ParkourBeamWalkParams::PARAM_BEAM_NODE] = beam;
			state_machine->transition_to(ASN()->beam_walk_state, dict);
		}
	}
	agent->handle_input(desired_velocity, p_delta);
	agent->update(p_delta);
	Vector3 linear_vel = agent->get_linear_velocity();
	Vector3 dir = linear_vel;
	dir.y = 0.0f;
	dir.normalize();
	if (dir.is_normalized()) {
		//agent->set_graphics_rotation(Basis::looking_at(dir).get_rotation_quaternion());
	}
	Vector3 linear_vel_horizontal = agent->get_linear_velocity().project(dir);
	if (!linear_vel_horizontal.is_finite()) {
		linear_vel_horizontal = Vector3();
	}
	get_wheel_locomotion_node()->set_linear_velocity(linear_vel_horizontal);
	get_wheel_locomotion_node()->set_x_blend(CLAMP(linear_vel_horizontal.length() / agent->get_agent_constants()->get_max_move_velocity(), 0.0, 1.0));
}

void HBAgentMoveState::_update_lookat() {
}

void HBAgentMoveState::_on_agent_edge_hit() {
	get_wheel_locomotion_node()->set_x_blend(0.0f);
	get_inertialization_node()->inertialize(0.25f);
}

/*
 * Vault points:
 *
 * Edge near --> |-----| <-- Edge far
 *               |     |
 *               |     |
 * Base near --> |-----| <-- Base far
 */
bool HBAgentGroundStateBase::try_vault_over_obstacle() {
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(agent == nullptr, false);

	Vector3 movement_input_dir = agent->get_desired_movement_input_transformed().normalized();
	if (movement_input_dir.length_squared() == 0.0f) {
		return false;
	}

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

	Ref<Shape3D> body_shape = agent->get_collision_shape();

	float mid_height = get_agent()->get_height() * 0.5f;
	float floor_max_angle = agent->get_floor_max_angle();

	// Vault constants
	const float vault_max_obstacle_width = agent->get_agent_constants()->get_vault_max_obstacle_width();

	PhysicsDirectSpaceState3D::RayResult ray_result;

	// Check if there's a wall in front of us
	if (!find_facing_wall(ray_result)) {
		return false;
	}

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

	Vector3 vault_base_near = ray_result.position;
	Vector3 wall_normal = ray_result.normal;
	// Find vault base near height
	ray_params.from = vault_base_near;
	ray_params.to = vault_base_near;
	ray_params.to.y -= agent->get_height();

	if (!dss->intersect_ray(ray_params, ray_result) || !is_floor(ray_result.normal, floor_max_angle)) {
		return false;
	}

	vault_base_near.y = ray_result.position.y;

	// Find vaultable near edge
	ray_params.from = vault_base_near;
	ray_params.from -= wall_normal * 0.05f;
	ray_params.from.y += agent->get_height() * 1.5f;
	ray_params.to = ray_result.position - ray_result.normal * 0.05f;

	if (!dss->intersect_ray(ray_params, ray_result) || !is_floor(ray_result.normal, floor_max_angle)) {
		// Not a floor or not hit
		return false;
	}

	Vector3 vault_edge_near = vault_base_near;
	vault_edge_near.y = ray_result.position.y;

	// Do a backwards ray towards us to check if there's a wall on the other side
	ray_params.to = vault_base_near;
	ray_params.to.y += agent->get_height() * 0.5f;
	ray_params.from = ray_params.to + movement_input_dir * vault_max_obstacle_width;

	if (!dss->intersect_ray(ray_params, ray_result) || !is_wall(ray_result.normal, floor_max_angle)) {
		// No wall on the other side!
		return false;
	}

	// Find the far vault edge, which should obviously also be a floor
	Vector3 vault_base_far = ray_result.position;
	vault_base_far.y -= mid_height;
	ray_params.from = ray_result.position;
	ray_params.from.y += mid_height;
	ray_params.from -= movement_input_dir * 0.01f;
	ray_params.to = vault_base_far - movement_input_dir * 0.01f;

	if (!dss->intersect_ray(ray_params, ray_result) || !is_floor(ray_result.normal, floor_max_angle)) {
		return false;
	}

	Vector3 vault_edge_far = vault_base_far;
	vault_edge_far.y = ray_result.position.y;

	// Finally check if the player fits on the other side
	PhysicsDirectSpaceState3D::ShapeParameters shape_query_params;
	shape_query_params.shape_rid = body_shape->get_rid();
	shape_query_params.transform.origin = vault_base_far + movement_input_dir * (agent->get_radius() + 0.5f);
	shape_query_params.transform.origin.y += mid_height * 2.01f;
	shape_query_params.collision_mask = ray_params.collision_mask;
	PhysicsDirectSpaceState3D::ShapeResult shape_query_result;

	debug_draw_cast_motion(body_shape, shape_query_params);

	if (dss->intersect_shape(shape_query_params, &shape_query_result, 1) > 0) {
		return false;
	}
	// We have space, time to vault
	// Prepare the required arguments for the vault state
	Dictionary root_motion_args;
	Dictionary warp_points;
	Transform3D temp_trf;

	root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->move_state;
	root_motion_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = StringName("Vault");
	root_motion_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MOVEMENT_VAULT;
	root_motion_args[HBAgentRootMotionState::PARAM_VELOCITY_MODE] = HBAgentRootMotionState::VelocityMode::CONSERVE;

	temp_trf.origin = vault_base_near;
	temp_trf.basis = Basis().looking_at(-movement_input_dir);
	warp_points[SNAME("VaultBaseNear")] = temp_trf;
	temp_trf.origin = vault_edge_near;
	warp_points[SNAME("VaultEdgeNear")] = temp_trf;

	temp_trf.basis = Basis().looking_at(-movement_input_dir);
	temp_trf.origin = vault_edge_far;
	warp_points[SNAME("VaultEdgeFar")] = temp_trf;
	temp_trf.origin = vault_base_far;
	warp_points[SNAME("VaultBaseFar")] = temp_trf;

	root_motion_args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;

	state_machine->transition_to(ASN()->root_motion_state, root_motion_args);

	return true;
}

bool HBAgentGroundStateBase::handle_parkour_up() {
	HBAgent *agent = get_agent();

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	ray_params.from = agent->get_global_position() + Vector3(0.0f, agent->get_height() * 0.5f, 0.0f);
	ray_params.to = ray_params.from + agent->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, -1.0f)) * 0.4f;

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::RayResult ray_result;
	bool hit = dss->intersect_ray(ray_params, ray_result);

	if (!hit || !is_wall(ray_result.normal, agent->get_floor_max_angle())) {
		return false;
	}

	const float agent_height = agent->get_height();
	Ref<CylinderShape3D> cyl_check_shape;
	cyl_check_shape.instantiate();
	cyl_check_shape->set_radius(agent->get_radius());
	const float reachable_height = agent_height * 3.0f;
	cyl_check_shape->set_height(reachable_height);

	Transform3D wall_trf;
	wall_trf.origin = ray_result.position;
	wall_trf.basis = Basis::looking_at(ray_result.normal);

	Transform3D pp_check_shape_trf;
	pp_check_shape_trf.origin = wall_trf.xform(Vector3(0.0f, reachable_height * 0.5f, 0.0f));

	if (handle_wallrun_to_parkour_point(cyl_check_shape, pp_check_shape_trf, wall_trf)) {
		return true;
	}

	Ref<Shape3D> agent_shape = agent->get_collision_shape();
	// Find a wall in front
	PhysicsDirectSpaceState3D::ShapeParameters shape_params;
	shape_params.shape_rid = agent_shape->get_rid();
	shape_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	shape_params.transform.origin = get_agent()->get_global_position();
	shape_params.transform.origin.y += agent_height * 1.5f;

	const Vector3 input_forward = get_agent()->get_desired_movement_input_transformed().normalized();
	shape_params.motion = input_forward;

	real_t closest_safe, closest_unsafe;
	dss->cast_motion(shape_params, closest_safe, closest_unsafe);
	hit = closest_safe != 1.0f && closest_unsafe != 1.0f;

	if (hit) {
		//TODO: Check if there's a wall in front of us
		Vector3 base_wp_origin = shape_params.transform.origin + shape_params.motion * closest_safe + input_forward * agent->get_radius();
		base_wp_origin.y -= agent_height * 1.5f;

		shape_params.transform.origin = shape_params.transform.origin + shape_params.motion * closest_safe;
		shape_params.collide_with_areas = true;
		shape_params.collide_with_bodies = false;
		shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
		shape_params.motion = Vector3();
		shape_params.shape_rid = cyl_check_shape->get_rid();

		static const int constexpr MAX_RESULTS = 5;
		Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
		results.resize(MAX_RESULTS);
		int result_count = dss->intersect_shape(shape_params, results.ptrw(), MAX_RESULTS);
		for (int i = 0; i < result_count; i++) {
			HBAgentParkourLedge *ledge = Object::cast_to<HBAgentParkourLedge>(results[i].collider);
			if (!ledge) {
				continue;
			}
			const float offset = ledge->get_closest_offset(shape_params.transform.origin);
			const Transform3D ledge_trf = ledge->get_ledge_transform_at_offset(offset);

			const Vector3 a_l = agent->get_global_position().direction_to(ledge_trf.origin);

			debug_draw_sphere(ledge_trf.origin, 0.05f, Color("RED"));
			debug_draw_line(ledge_trf.origin, ledge_trf.origin + ledge_trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f)), Color("GREEN"));

			debug_draw_line(agent->get_global_position(), agent->get_global_position() + a_l);

			if (a_l.dot(ledge_trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f))) > 0.0) {
				// Do we fit there?
				if (!ledge->check_agent_fits(agent, offset, get_debug_geometry())) {
					continue;
				}

				Basis ledge_wp_basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), ledge_trf.basis.xform(Vector3(0.0, 0.0, 1.0)));

				Transform3D base_wp;
				base_wp.origin = base_wp_origin;
				base_wp.basis = ledge_wp_basis;

				Transform3D ledge_wp;
				ledge_wp.basis = ledge_wp_basis;
				ledge_wp.origin = ledge_trf.origin;

				Dictionary warp_points;
				warp_points[StringName("WallrunBase")] = base_wp;
				warp_points[StringName("WallrunEdge")] = ledge_wp;

				Dictionary ledge_state_args;
				ledge_state_args[HBAgentLedgeGrabbedStateNew::PARAM_LEDGE] = ledge;

				Dictionary root_motion_args;

				root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->ledge_grabbed_state;
				root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = ledge_state_args;
				root_motion_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MOVEMENT_WALLRUN;
				root_motion_args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;
				root_motion_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = StringName("Wallrun");

				state_machine->transition_to(ASN()->root_motion_state, root_motion_args);

				return true;
			}
		}
	}

	return false;
}

bool HBAgentGroundStateBase::handle_parkour_mid() {
	const Vector3 input_forward = get_agent()->get_desired_movement_input_transformed().normalized();
	const Vector3 agent_pos = get_agent()->get_global_position();

	HBAgent *agent = get_agent();
	const bool is_at_edge = input_forward.is_normalized() && agent->is_at_edge(input_forward);
	Ref<Shape3D> agent_shape = get_agent()->get_collision_shape();
	PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();

	// Find a gap
	// Short jump to another position
	PhysicsDirectSpaceState3D::ShapeParameters hull_cast_params;
	hull_cast_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	hull_cast_params.shape_rid = agent_shape->get_rid();
	bool gap_found = false;
	static constexpr int GAPPED_JUMP_STEPS = 16;
	static constexpr float GAPPED_JUMP_DIST = 1.6f;
	if (is_at_edge) {
		for (int i = 0; i < GAPPED_JUMP_STEPS; i++) {
			const float current_dist = ((i + 1) / (float)GAPPED_JUMP_STEPS) * GAPPED_JUMP_DIST;
			hull_cast_params.transform.origin = agent_pos + input_forward * current_dist;
			hull_cast_params.transform.origin.y += get_agent()->get_height();
			// Hull cast forward and above to see if we are clear more or less above the next position
			hull_cast_params.motion = input_forward * current_dist;
			real_t closest_safe, closest_unsafe;
			debug_draw_cast_motion(agent_shape, hull_cast_params);
			dss->cast_motion(hull_cast_params, closest_safe, closest_unsafe);

			bool hit = closest_safe != 1.0f && closest_unsafe != 1.0f;
			if (hit) {
				// We hit something early, end the process here
				break;
			}

			// Use a ray to find a gap, this is because the shape cast might be too big for smaller gaps
			PhysicsDirectSpaceState3D::RayParameters ray_params;
			PhysicsDirectSpaceState3D::RayResult ray_result;
			ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
			ray_params.from = hull_cast_params.transform.origin + hull_cast_params.motion;
			ray_params.from.y += get_agent()->get_height() * 0.5f;
			ray_params.to = hull_cast_params.transform.origin + hull_cast_params.motion;
			ray_params.to.y -= get_agent()->get_height() * 2.0f;
			debug_draw_raycast(ray_params, Color("RED"));
			hit = dss->intersect_ray(ray_params, ray_result);
			const float floor_max_angle = get_agent()->get_floor_max_angle();
			if (!hit) {
				// Got a gap
				gap_found = true;
				continue;
			}

			if (!hit || !gap_found || !is_floor(ray_result.normal, floor_max_angle)) {
				continue;
			}

			// We have a ground and a gap
			// Try shape cast down to see if we can fit in there
			hull_cast_params.transform.origin += hull_cast_params.motion;

			hull_cast_params.motion = Vector3(0.0f, -get_agent()->get_height() * 1.5f, 0.0f);

			hit = dss->cast_motion(hull_cast_params, closest_safe, closest_unsafe);
			hit = closest_safe != 1.0f && closest_unsafe != 1.0f;
			// Strangely we hit nothing at all, this should never happen
			if (!hit) {
				continue;
			}
			// Found a place, this is the final landing spot
			if (hit) {
				Vector3 landing_point = hull_cast_params.transform.origin + hull_cast_params.motion * closest_unsafe;
				landing_point.y -= get_agent()->get_height() * 0.5f;
				debug_draw_sphere(landing_point, 0.05f, Color("PINK"));
				debug_draw_cast_motion(agent_shape, hull_cast_params, Color("GREEN"));
				debug_draw_shape(agent_shape, hull_cast_params.transform.origin + hull_cast_params.motion * closest_safe, Color("BLUE"));
				hull_cast_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
				hull_cast_params.collide_with_bodies = false;
				hull_cast_params.collide_with_areas = true;
				hull_cast_params.transform.origin += hull_cast_params.motion * closest_safe;
				Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
				const constexpr static int MAX_RESULTS = 10;
				results.resize(MAX_RESULTS);
				int result_count = dss->intersect_shape(hull_cast_params, results.ptrw(), MAX_RESULTS);
				HBAgentParkourBeam *beam = nullptr;
				if (hit) {
					for (int j = 0; j < result_count; j++) {
						beam = Object::cast_to<HBAgentParkourBeam>(results[j].collider);
						if (beam) {
							break;
						}
					}
				}

				Dictionary next_state_args;
				StringName next_state;

				if (beam) {
					Ref<Curve3D> curve = beam->get_curve();
					float offset = curve->get_closest_offset(beam->to_local(landing_point));
					landing_point = beam->to_global(curve->sample_baked(offset));

					next_state = ASN()->beam_walk_state;
					next_state_args[HBAgentParkourBeamWalk::PARAM_BEAM_NODE] = beam;
				} else {
					next_state = ASN()->move_state;
					next_state_args[HBAgentMoveState::PARAM_WAIT_FOR_TRANSITION] = true;
				}

				_transition_to_short_hop(landing_point, next_state, next_state_args);

				return true;
			}
		}
	}

	// Second alternative, a short hop up
	// We first cast our shape forward a bit, colliding with areas
	// If we hit a ledge it means there's a point we can jump up to without having to grab the ledge first
	static const int constexpr STEPS = 5;

	for (int i = 0; i < STEPS; i++) {
		PhysicsDirectSpaceState3D::ShapeParameters shape_params;
		shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
		shape_params.shape_rid = agent_shape->get_rid();
		shape_params.collide_with_areas = true;
		shape_params.collide_with_bodies = false;
		shape_params.transform.origin = agent->get_global_position();
		shape_params.transform.origin.y += agent->get_height() * 0.5f;
		shape_params.transform.origin.y += agent->get_walk_stairs_step_up().y;
		shape_params.transform.origin += input_forward * agent->get_radius() * 2.0f * (i + 1);

		const constexpr static int MAX_RESULTS = 10;
		Vector<PhysicsDirectSpaceState3D::ShapeResult> shape_results;
		shape_results.resize(MAX_RESULTS);

		debug_draw_cast_motion(agent_shape, shape_params, Color("RED"));
		int result_count = dss->intersect_shape(shape_params, shape_results.ptrw(), MAX_RESULTS);

		// We must make sure the ledge is roughly pointing towards us
		for (int result_i = 0; result_i < result_count; result_i++) {
			HBAgentParkourLedge *ledge_candidate = Object::cast_to<HBAgentParkourLedge>(shape_results[result_i].collider);
			if (!ledge_candidate) {
				continue;
			}
			const float offset = ledge_candidate->get_closest_offset(shape_params.transform.origin);
			Transform3D ledge_trf = ledge_candidate->get_ledge_transform_at_offset(offset);
			Vector3 ledge_forward = ledge_trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f));
			if (input_forward.dot(ledge_forward) > 0.0f) {
				// finally, we find the character's landing position
				PhysicsDirectSpaceState3D::ShapeParameters landing_params;
				landing_params.shape_rid = agent_shape->get_rid();
				landing_params.transform.origin = ledge_trf.origin + (ledge_forward * agent->get_radius());
				landing_params.transform.origin.y += agent->get_height();
				landing_params.motion.y -= agent->get_height() * 1.5f;

				real_t closest_safe, closest_unsafe;

				debug_draw_cast_motion(agent_shape, landing_params, Color("HOTPINK"));

				dss->cast_motion(landing_params, closest_safe, closest_unsafe);

				if (closest_safe != 1.0f && closest_unsafe != 1.0f) {
					Vector3 landing_point = landing_params.transform.origin + landing_params.motion * closest_unsafe;
					landing_point.y -= agent->get_height() * 0.5f;
					// Check if landing point is too high
					if (landing_point.y - agent->get_global_position().y > agent->get_height() * 0.75f) {
						continue;
					}
					_transition_to_short_hop(landing_point, ASN()->move_state);
					return true;
				}
			}
		}
	}
	// Third alternative, jump straight to ledge/parkour point
	// there are two possibilities here, a longer but lower jump or a shorter
	// but higher reaching jump
	static constexpr const float STRAIGHT_TO_LEDGE_SHORT_REACH = 1.75f;
	static constexpr const float STRAIGHT_TO_LEDGE_LONG_REACH = 3.0f;
	Ref<BoxShape3D> box_shape;
	box_shape.instantiate();
	box_shape->set_size(Vector3(agent->get_radius() * 2.0f, agent->get_height(), STRAIGHT_TO_LEDGE_SHORT_REACH));
	Transform3D shape_trf;
	shape_trf.origin = agent->get_global_position() + input_forward * STRAIGHT_TO_LEDGE_SHORT_REACH * 0.5f;
	// add a margin so we don't hit ledges at our height
	shape_trf.origin.y += agent->get_height() * 0.5f + agent->get_walk_stairs_step_up().y;
	shape_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), input_forward);
	if (handle_jump_to_ledge(box_shape, shape_trf)) {
		return true;
	}
	box_shape.instantiate();
	box_shape->set_size(Vector3(agent->get_radius() * 2.0f, agent->get_height(), STRAIGHT_TO_LEDGE_LONG_REACH));

	// This jump should only happen at edges
	shape_trf.origin = agent->get_global_position() + input_forward * (STRAIGHT_TO_LEDGE_LONG_REACH) * 0.5f;
	// add a margin so we don't hit ledges at our height
	shape_trf.origin.y -= (box_shape->get_size().y * 0.5f) - agent->get_walk_stairs_step_up().y;

	if (is_at_edge && handle_jump_to_ledge(box_shape, shape_trf)) {
		return true;
	}

	shape_trf.origin.y += box_shape->get_size().y;

	if (handle_jump_to_parkour_point(box_shape, shape_trf)) {
		return true;
	}

	return false;
}

bool HBAgentGroundStateBase::handle_jump_to_ledge(Ref<BoxShape3D> p_shape, const Transform3D &p_shape_trf) {
	ERR_FAIL_COND_V(!p_shape.is_valid(), false);
	HBAgent *agent = get_agent();
	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

	PhysicsDirectSpaceState3D::ShapeParameters straight_to_ledge_params;
	straight_to_ledge_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;

	straight_to_ledge_params.shape_rid = p_shape->get_rid();

	straight_to_ledge_params.collide_with_bodies = true;
	straight_to_ledge_params.collide_with_areas = true;
	straight_to_ledge_params.transform = p_shape_trf;

	Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
	static constexpr const float MAX_RESULTS = 10;
	results.resize(MAX_RESULTS);

	int result_count = dss->intersect_shape(straight_to_ledge_params, results.ptrw(), MAX_RESULTS);

	debug_draw_cast_motion(p_shape, straight_to_ledge_params, Color("GREEN"));

	const Vector3 input_forward = agent->get_desired_movement_input_transformed().normalized();

	for (int i = 0; i < result_count; i++) {
		HBAgentParkourLedge *ledge = Object::cast_to<HBAgentParkourLedge>(results[i].collider);
		// Ledge check
		if (ledge) {
			const Vector3 closest_test = p_shape_trf.xform(Vector3(0.0f, 0.0f, -p_shape->get_size().z * 0.5f));
			const float offset = ledge->get_closest_offset(closest_test);
			Transform3D ledge_trf = ledge->get_ledge_transform_at_offset(offset);
			// Check if we are facing roughly in the direction of the ledge
			if (input_forward.angle_to(ledge_trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f))) < Math::deg_to_rad(50.0f)) {
				// Do we fit there?
				if (!ledge->check_agent_fits(agent, offset, get_debug_geometry())) {
					continue;
				}

				const Vector3 whishker_target = ledge_trf.origin + ledge_trf.basis.xform(Vector3(0.0f, 0.0f, 1.0f)) * agent->get_radius();

				// Check if it's actually reachable by ray casting from the character at various heights
				if (!whisker_reach_check(agent->get_global_position(), whishker_target, agent->get_walk_stairs_step_up().y, agent->get_height())) {
					continue;
				}

				Dictionary root_motion_args;
				root_motion_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->standing_jump_to_ledge_animation_node;
				root_motion_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MOVEMENT_STANDING_JUMP_TO_LEDGE;
				Dictionary next_state_args;
				next_state_args[HBAgentLedgeGrabbedStateNew::PARAM_LEDGE] = ledge;
				root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = next_state_args;
				root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->ledge_grabbed_state;

				Dictionary warp_points;
				Transform3D ledge_anim_trf;
				ledge_anim_trf.origin = ledge_trf.origin;
				ledge_anim_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), ledge_trf.basis.xform(Vector3(0.0, 0.0, 1.0)));
				warp_points[StringName("Ledge")] = ledge_anim_trf;
				root_motion_args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;

				state_machine->transition_to(ASN()->root_motion_state, root_motion_args);
				return true;
			}
		}
	}
	return false;
}

bool HBAgentGroundStateBase::handle_jump_to_parkour_point(Ref<BoxShape3D> p_shape, const Transform3D &p_shape_trf) {
	ERR_FAIL_COND_V(!p_shape.is_valid(), false);
	HBAgent *agent = get_agent();
	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

	PhysicsDirectSpaceState3D::ShapeParameters straight_to_ledge_params;
	straight_to_ledge_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;

	straight_to_ledge_params.shape_rid = p_shape->get_rid();

	straight_to_ledge_params.collide_with_bodies = true;
	straight_to_ledge_params.collide_with_areas = false;
	straight_to_ledge_params.transform = p_shape_trf;

	Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
	static constexpr const float MAX_RESULTS = 10;
	results.resize(MAX_RESULTS);

	int result_count = dss->intersect_shape(straight_to_ledge_params, results.ptrw(), MAX_RESULTS);

	debug_draw_cast_motion(p_shape, straight_to_ledge_params, Color("GREEN"));

	const Vector3 input_forward = agent->get_desired_movement_input_transformed().normalized();

	for (int i = 0; i < result_count; i++) {
		// Parkour point check
		HBAgentParkourPoint *point = Object::cast_to<HBAgentParkourPoint>(results[i].collider);
		if (point) {
			const Transform3D point_trf = point->get_global_transform();
			const Vector3 point_forward = point_trf.basis.xform(Vector3(0.0, 0.0, -1.0f));
			if (point_forward.angle_to(-input_forward) > Math::deg_to_rad(45.0f)) {
				continue;
			}

			if (!whisker_reach_check(agent->get_global_position(), point->get_global_position() + point_forward * agent->get_radius(), agent->get_walk_stairs_step_up().y, agent->get_height())) {
				continue;
			}

			Dictionary root_motion_args;
			root_motion_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->standing_jump_to_ledge_animation_node;
			root_motion_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MOVEMENT_STANDING_JUMP_TO_LEDGE;
			Dictionary next_state_args;
			next_state_args[HBAgentWallParkourStateNew::PARAM_PARKOUR_NODE] = point;
			root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = next_state_args;
			root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->wall_parkour_state;

			Dictionary warp_points;
			Transform3D ledge_anim_trf;
			ledge_anim_trf.origin = point_trf.origin;
			//ledge_anim_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), ledge_anim_trf.basis.xform(Vector3(0.0, 0.0, 1.0)));
			ledge_anim_trf.basis = point_trf.basis;
			warp_points[StringName("Ledge")] = ledge_anim_trf;
			root_motion_args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;

			state_machine->transition_to(ASN()->root_motion_state, root_motion_args);
		}
	}
	return false;
}

bool HBAgentGroundStateBase::handle_wallrun_to_parkour_point(const Ref<Shape3D> &p_shape, const Transform3D &p_shape_trf, const Transform3D &p_wall_base_trf) {
	PhysicsDirectSpaceState3D::ShapeParameters params;
	params.shape_rid = p_shape->get_rid();
	params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
	params.transform = p_shape_trf;

	PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();
	const int MAX_RESULTS = 10;
	PhysicsDirectSpaceState3D::ShapeResult results[MAX_RESULTS];
	debug_draw_cast_motion(p_shape, params, Color("GREEN"));
	int result_count = dss->intersect_shape(params, results, MAX_RESULTS);

	HBAgentParkourPoint *parkour_point = nullptr;

	const Vector3 pp_forward = -p_wall_base_trf.basis.get_column(2);
	float pp_dist_to_angent;

	for (int i = 0; i < result_count; i++) {
		// Hosti tu, l'Albiol
		HBAgentParkourPoint *pp_candidate = Object::cast_to<HBAgentParkourPoint>(results[i].collider);
		if (!pp_candidate) {
			continue;
		}
		Vector3 point_forward = pp_candidate->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));

		if (point_forward.dot(pp_forward) > 0.0f) {
			float distance_to_agent = pp_candidate->get_global_position().distance_to(get_agent()->get_global_position());
			if (parkour_point) {
				if (distance_to_agent > pp_dist_to_angent) {
					continue;
				}
			}
			parkour_point = pp_candidate;
			pp_dist_to_angent = distance_to_agent;
		}
	}

	if (!parkour_point) {
		return false;
	}

	Dictionary warp_points;
	warp_points[StringName("WallrunBase")] = p_wall_base_trf;
	warp_points[StringName("WallrunEdge")] = parkour_point->get_global_transform();

	Dictionary wall_parkour_state_args;
	wall_parkour_state_args[HBAgentWallParkourStateNew::PARAM_PARKOUR_NODE] = parkour_point;

	Dictionary root_motion_args;
	root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->wall_parkour_state;
	root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = wall_parkour_state_args;
	root_motion_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MOVEMENT_WALLRUN;
	root_motion_args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;
	root_motion_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = StringName("Wallrun");

	state_machine->transition_to(ASN()->root_motion_state, root_motion_args);

	return true;
}

bool HBAgentGroundStateBase::handle_parkour() {
	bool is_parkour_down_held = get_agent()->is_action_pressed(HBAgent::INPUT_ACTION_PARKOUR_DOWN);
	bool is_parkour_up_held = get_agent()->is_action_pressed(HBAgent::INPUT_ACTION_PARKOUR_UP);
	bool is_run_held = get_agent()->is_action_pressed(HBAgent::INPUT_ACTION_RUN);
	if (is_parkour_down_held && handle_parkour_down()) {
		return true;
	}
	if (is_parkour_down_held) {
		return false;
	}
	if (is_parkour_up_held && handle_parkour_up()) {
		return true;
	}
	if (is_run_held && handle_parkour_mid()) {
		return true;
	}
	return false;
}

bool HBAgentGroundStateBase::handle_parkour_down() {
	HBAgent *agent = get_agent();

	if (try_vault_over_obstacle()) {
		return true;
	}

	// Ledge drop check
	Vector<HBAgentParkourLedge *> overlapping_ledges = agent->get_overlapping_ledges();
	for (HBAgentParkourLedge *ledge : overlapping_ledges) {
		float offset = ledge->get_closest_offset(get_agent()->get_global_position());
		Transform3D ledge_trf = ledge->get_ledge_transform_at_offset(offset);
		// Ledge drop/slide is only allowed when our character's orientation is pointing roughly towards the
		// ledge
		Vector3 ledge_point = ledge_trf.origin;
		if (ledge_trf.origin.y > agent->get_global_position().y + 0.2f) {
			continue;
		}

		Vector3 a_l = agent->get_global_position().direction_to(ledge_point);
		if (a_l.dot(agent->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, -1.0f))) > 0.0f) {
			// We know we are pointing towards the ledge, now we figure out if we need to do a ledge drop or a ledge slide down
			PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
			PhysicsDirectSpaceState3D::ShapeParameters shape_params;
			Ref<Shape3D> agent_shape = get_agent()->get_collision_shape();
			shape_params.shape_rid = agent_shape->get_rid();
			shape_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
			const Vector3 agent_movement_dir = agent->get_desired_movement_input_transformed().normalized();
			shape_params.transform.origin = ledge_trf.origin + agent_movement_dir * agent->get_radius() * 2.5;
			shape_params.transform.origin.y += agent->get_height();
			shape_params.motion.y -= agent->get_height() * 1.5f;

			real_t closest_safe, closest_unsafe;
			dss->cast_motion(shape_params, closest_safe, closest_unsafe);
			bool hit = closest_safe != 1.0f && closest_unsafe != 1.0f;
			bool ledge_slide_down = false;
			Vector3 ledge_slide_down_ground_position;
			if (hit) {
				ledge_slide_down_ground_position = shape_params.transform.origin + shape_params.motion * closest_unsafe;
				ledge_slide_down_ground_position.y -= agent->get_height() * 0.5f;
				if (ledge_slide_down_ground_position.y - ledge_trf.origin.y < -agent->get_walk_stairs_step_up().y) {
					ledge_slide_down = true;
				}
			}

			Dictionary transition_dict;
			Dictionary warp_points;

			if (ledge_slide_down) {
				// Setup ledge slide down anim
				transition_dict[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MovementTransitionInputs::MOVEMENT_LEDGE_TO_GROUND;
				transition_dict[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->ledge_to_ground_animation_node;

				Vector3 dir = a_l;
				dir.y = 0.0f;
				dir.normalize();
				if (!dir.is_normalized()) {
					continue;
				}

				Basis animation_basis = Basis::looking_at(-dir);

				Transform3D surface_anim_trf;
				surface_anim_trf.origin = ledge_slide_down_ground_position;
				surface_anim_trf.basis = animation_basis;

				Transform3D ledge_anim_trf;
				ledge_anim_trf.origin = ledge_trf.origin;
				ledge_anim_trf.basis = animation_basis;

				warp_points[StringName("ledge")] = ledge_anim_trf;
				warp_points[StringName("surface")] = surface_anim_trf;

				debug_draw_line(surface_anim_trf.origin, surface_anim_trf.xform(Vector3(0.0f, 0.0f, -1.0f)));
				debug_draw_line(ledge_anim_trf.origin, ledge_anim_trf.xform(Vector3(0.0f, 0.0f, -1.0f)));
				Dictionary next_state_args;
				transition_dict[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->move_state;
			} else {
				// Ledge drop should only be done if we can fit on the ledge
				debug_draw_cast_motion(agent_shape, shape_params);
				if (!ledge->check_agent_fits(agent, offset)) {
					continue;
				}
				// Setup ledge drop anim
				warp_points[StringName("ledge")] = ledge_trf;
				transition_dict[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MovementTransitionInputs::MOVEMENT_STANDING_DROP_TO_LEDGE;
				transition_dict[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->standing_drop_to_ledge_animation_node;
				transition_dict[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->ledge_grabbed_state;
				Dictionary next_state_args;
				next_state_args[HBAgentLedgeGrabbedStateNew::PARAM_LEDGE] = ledge;
				transition_dict[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = next_state_args;
			}

			transition_dict[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;

			state_machine->transition_to(ASN()->root_motion_state, transition_dict);
			return true;
		}
	}
	return false;
}

/**********************
	Turn state
***********************/

void HBAgentTurnState::_on_animation_finished() {
	Vector3 euler = animation_node->get_root_motion_transform().get_basis().get_euler();
	euler.x = Math::rad_to_deg(euler.x);
	euler.y = Math::rad_to_deg(euler.y);
	euler.z = Math::rad_to_deg(euler.z);
	//get_agent()->apply_root_motion(animation_node);
}

void HBAgentTurnState::enter(const Dictionary &p_args) {
	ERR_FAIL_COND(!get_epas_controller());
	ERR_FAIL_COND(!get_skeleton());
	ERR_FAIL_COND(!get_graphics_node());
	ERR_FAIL_COND(!get_agent());
	ERR_FAIL_COND(!get_movement_transition_node().is_valid());

	target_angle = p_args.get(PARAM_ANGLE, 0.0f);
	DEV_ASSERT(target_angle != 0.0f);
	if (SIGN(target_angle) < 0) {
		animation_node = get_epas_controller()->get_epas_node("TurnRight");
		get_movement_transition_node()->transition_to(HBAgentConstants::MOVEMENT_TURN180_R);
	} else {
		animation_node = get_epas_controller()->get_epas_node("TurnLeft");
		get_movement_transition_node()->transition_to(HBAgentConstants::MOVEMENT_TURN180_L);
	}
	animation_node->set_root_motion_starting_transform(get_skeleton()->get_global_transform());
	animation_node->set_root_motion_forward(Vector3(0.0f, 0.0f, 1.0f));
	animation_node->set_speed_scale(2.0f);
	float target_time = Math::abs(target_angle) / 180.0f;
	target_time = target_time * animation_node->get_animation()->get_length();
	animation_node->set_end_time(target_time);
	animation_node->play();
	get_agent()->root_motion_begin(animation_node, get_physics_process_delta_time());
}

void HBAgentTurnState::physics_process(float p_delta) {
	time += p_delta;
	if (animation_node.is_valid()) {
		get_agent()->apply_root_motion(animation_node, false, p_delta);
		if (!animation_node->is_playing()) {
			animation_node.unref();
			state_machine->transition_to(ASN()->move_state);
		}
	}
}

/**********************
	Ledge grabbed state
***********************/

void HBAgentLedgeGrabbedStateNew::enter(const Dictionary &p_args) {
	if (!controller->is_inside_tree()) {
		add_child(controller);
	}
	DEV_ASSERT(p_args.has(PARAM_LEDGE));
	ledge = Object::cast_to<HBAgentParkourLedge>(p_args[PARAM_LEDGE]);
	float offset = ledge->get_closest_offset(get_agent()->get_global_position());
	offset = p_args.get("OFFSET_TEST", offset);
	controller->move_to_ledge(ledge, offset);
	animator.restart();

	for (int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		animator_options.starting_limb_transforms[i] = controller->get_limb_transform(static_cast<AgentProceduralAnimator::AgentLimb>(i));
		animator_options.target_limb_transforms[i] = animator_options.starting_limb_transforms[i];
		animator_options.limb_position_spring_halflifes[i] = 0.05f;
		animator_options.limb_rotation_spring_halflifes[i] = 0.15f;
	}

	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_LEFT_HAND][0] = 0.0f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_LEFT_HAND][1] = 0.4f;
	animator_options.limb_peak_position[AgentProceduralAnimator::LIMB_LEFT_HAND] = Vector3(0.0, 0.05f, 0.0f);

	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_RIGHT_HAND][0] = 0.5f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_RIGHT_HAND][1] = 0.9f;
	animator_options.limb_peak_position[AgentProceduralAnimator::LIMB_RIGHT_HAND] = Vector3(0.0, 0.05f, 0.0f);

	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_LEFT_FOOT][0] = 0.1f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_LEFT_FOOT][1] = 0.5f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_RIGHT_FOOT][0] = 0.6f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_RIGHT_FOOT][1] = 1.0f;
	animator_options.use_average_for_skeleton_trf = true;
	animator_options.limb_peak_position[AgentProceduralAnimator::LIMB_LEFT_FOOT] = Vector3(0.0, 0.05f, 0.05f);
	animator_options.limb_peak_position[AgentProceduralAnimator::LIMB_RIGHT_FOOT] = Vector3(0.0, 0.05f, 0.05f);

	if (!are_limbs_init) {
		_init_limbs();
	}

	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		limbs[i].ik_node->set_ik_influence(1.0f);
		limbs[i].dangle_status = false;
	}

	get_movement_transition_node()->transition_to(HBAgentConstants::MOVEMENT_WALLGRABBED);
	get_inertialization_node()->inertialize(0.2f);
	animator.seek(1.0f);
	animator_options.anim_blend = 0.0f;
	controller->update(0.0f);
	_update_animator();
	animator.process(animator_options, 0.0f);

	Vector3 agent_pos;
	Basis graphics_basis;

	AgentProceduralAnimator::AgentProceduralPose pose;
	animator.get_output_pose(pose);

	_update_ik_transforms(pose);
	_apply_ik_transforms(pose, true);
}
void HBAgentLedgeGrabbedStateNew::exit() {
	remove_child(controller);

	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		limbs[i].ik_node->set_ik_influence(0.0f);
	}
}

void HBAgentLedgeGrabbedStateNew::get_initial_pose_for_edge(HBAgentParkourLedge *p_ledge, AgentProceduralAnimator::AgentProceduralPose &p_pose, float p_offset) {
	if (!are_limbs_init) {
		_init_limbs();
	}

	bool was_in_tree = controller->is_inside_tree();
	if (!was_in_tree) {
		add_child(controller);
	}
	controller->move_to_ledge(p_ledge, p_offset);
	controller->update(0.0f);

	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		p_pose.ik_targets[i] = controller->get_limb_transform_predicted((AgentProceduralAnimator::AgentLimb)i);
	}
	p_pose.skeleton_trf.origin = controller->get_global_position();
	p_pose.skeleton_trf.basis = controller->get_graphics_rotation();
	_update_ik_transforms(p_pose);

	if (!was_in_tree) {
		remove_child(controller);
	}
}

#ifdef DEBUG_ENABLED
void HBAgentLedgeGrabbedStateNew::debug_ui_draw() {
	HBAgentState::debug_ui_draw();
	ImGui::InputFloat3("Hand rot", hand_offset_euler.coord);
}
#endif
void HBAgentLedgeGrabbedStateNew::physics_process(float p_delta) {
	debug_draw_clear();
	Vector3 input = Vector3(get_agent()->get_movement_input().x, 0.0f, 0.0f);
	input.normalize();
	animator_options.playback_direction = controller->get_movement_velocity() != 0.0f ? -SIGN(controller->get_movement_velocity()) : animator_options.playback_direction;
	Vector3 controller_input = input;
	controller->set_movement_input(controller_input);
	controller->update(p_delta);
	float blend = Math::abs(controller->get_movement_velocity()) / controller->get_max_movement_velocity();
	animator_options.anim_blend = blend;
	_update_animator();

	Transform3D prev_limb_ik_transforms[AgentProceduralAnimator::LIMB_MAX];
	{
		AgentProceduralAnimator::AgentProceduralPose base_pose;
		animator.get_output_pose(base_pose);
		for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
			prev_limb_ik_transforms[i] = base_pose.ik_targets[i];
		}
	}

	animator.process(animator_options, p_delta);

	AgentProceduralAnimator::AgentProceduralPose pose;
	animator.get_output_pose(pose);
	_update_ik_transforms(pose);

	_apply_ik_transforms(pose);

	bool limbs_are_out_of_place = false;

	for (int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		limbs_are_out_of_place = pose.ik_targets[i].origin.distance_to(animator_options.target_limb_transforms[i].origin) > 0.05;
		if (limbs_are_out_of_place) {
			break;
		}
	}

	if (animator.get_playback_position() == 1.0f && (input.x != 0 || limbs_are_out_of_place)) {
		animator.reset();
		for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
			AgentProceduralAnimator::AgentLimb limb = static_cast<AgentProceduralAnimator::AgentLimb>(i);
			animator_options.starting_limb_transforms[limb] = animator_options.target_limb_transforms[limb];
		}
		animator_options.playback_direction = -SIGN(input.x);
	}
	// Input handling

	if (get_agent()->is_action_pressed(HBAgent::AgentInputAction::INPUT_ACTION_PARKOUR_DOWN)) {
		Vector3 global_pos = get_agent()->get_global_position() + get_agent()->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, 0.1f));
		Transform3D graphics_transform = Transform3D(get_agent()->get_graphics_rotation(), global_pos);
		get_agent()->inertialize_graphics_transform(graphics_transform, 0.25f);

		state_machine->transition_to(ASN()->fall_state);
		// Force update
		get_agent()->update(0.0f);
		return;
	}

	if (get_agent()->get_movement_input().dot(Vector3(0.0f, 0.0f, -1.0f)) > 0.95f) {
		if (_handle_getup()) {
			return;
		}
	} else if (get_agent()->get_movement_input().dot(Vector3(0.0f, 0.0f, -1.0f)) < -0.95f) {
		if (_handle_drop_to_parkour_point()) {
			return;
		}
	}

	bool is_run_held = get_agent()->is_action_pressed(HBAgent::INPUT_ACTION_RUN);
	// Side eject
	if (is_run_held) {
		if (_handle_parkour_mid()) {
			return;
		}
	}

	bool is_parkour_up_pressed = get_agent()->is_action_just_pressed(HBAgent::INPUT_ACTION_PARKOUR_UP);
	if (is_parkour_up_pressed) {
		const static constexpr float BACK_EJECT_RANGE = 3.0f;
		Ref<BoxShape3D> box_shape;
		box_shape.instantiate();

		const Vector3 box_size = Vector3(get_agent()->get_radius() * 2.0f, get_agent()->get_height(), BACK_EJECT_RANGE);
		box_shape->set_size(box_size);

		PhysicsDirectSpaceState3D::ShapeParameters shape_params;
		shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
		shape_params.collide_with_areas = true;
		shape_params.collide_with_bodies = false;
		shape_params.shape_rid = box_shape->get_rid();
		const Vector3 back_eject_forward = get_agent()->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, 1.0f));
		shape_params.transform.origin = get_agent()->get_global_position() + back_eject_forward * BACK_EJECT_RANGE * 0.5f;
		shape_params.transform.origin.y += get_agent()->get_height();
		shape_params.transform.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), back_eject_forward);

		PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();
		const static int constexpr MAX_RESULTS = 10;
		Vector<PhysicsDirectSpaceState3D::ShapeResult> shape_results;
		shape_results.resize(MAX_RESULTS);
		int result_count = dss->intersect_shape(shape_params, shape_results.ptrw(), MAX_RESULTS);
		debug_draw_cast_motion(box_shape, shape_params);
		for (int i = 0; i < result_count; i++) {
			HBAgentParkourLedge *ledge_candidate = Object::cast_to<HBAgentParkourLedge>(shape_results[i].collider);
			if (!ledge_candidate) {
				continue;
			}
			float offset = ledge_candidate->get_closest_offset(get_agent()->get_global_position());
			Transform3D ledge_trf = ledge_candidate->get_ledge_transform_at_offset(offset);

			Vector3 a_l = get_agent()->get_global_position().direction_to(ledge_trf.origin);
			if (a_l.dot(ledge_trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f))) > 0.0f) {
				if (!ledge_candidate->check_agent_fits(get_agent(), offset)) {
					continue;
				}

				Dictionary next_state_args;
				next_state_args[HBAgentLedgeGrabbedStateNew::PARAM_LEDGE] = ledge_candidate;

				Dictionary warp_points;
				warp_points[StringName("EndLedge")] = ledge_trf;

				Dictionary root_motion_args;
				root_motion_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MOVEMENT_BACK_EJECT_TO_LEDGE;
				root_motion_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->back_eject_to_ledge_animation_node;
				root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->ledge_grabbed_state;
				root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = next_state_args;
				root_motion_args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;

				state_machine->transition_to(ASN()->root_motion_state, root_motion_args);
				return;
			}
		}
	}

	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		debug_draw_sphere(limbs[i].ik_node->get_magnet_position());
		debug_draw_sphere(limbs[i].ik_node->get_target_transform().origin, 0.05f, Color("GREEN"));
	}

	Vector3 cpos = controller->get_global_position();

	debug_draw_line(cpos, pose.skeleton_trf.xform(Vector3(0.0f, 0.0f, -1.0f)));
}

bool HBAgentLedgeGrabbedStateNew::_handle_getup() {
	Node3D *gn = get_graphics_node();
	ERR_FAIL_COND_V(!gn, false);
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(!agent, false);

	// We need to find the center of both hands, this is because we don't have any information on the
	// height of the ledge at the middle, only at the sides

	// Transform offset from outer ring to internal one first
	const float outer_ring_offset = controller->get_ledge_offset();
	Transform3D ledge_trf = ledge->get_ledge_transform_at_offset(ledge->get_closest_offset(ledge->get_agent_ledge_transform_at_offset(outer_ring_offset).origin));

	Vector3 ledge_position = ledge_trf.origin;

	// Now we find the position where the character will end up after getup is finished
	PhysicsDirectSpaceState3D::RayParameters ray_params;
	PhysicsDirectSpaceState3D::RayResult ray_result;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	ray_params.from = ledge_position + gn->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -agent->get_radius()));
	ray_params.from.y += 0.5f;
	ray_params.to = ray_params.from;
	ray_params.to.y -= 1.0f;

	debug_draw_raycast(ray_params);

	if (!agent->get_world_3d()->get_direct_space_state()->intersect_ray(ray_params, ray_result)) {
		// We failed to find the target getup position
		// This probably means there's some obstacle
		return false;
	}

	if (!is_floor(ray_result.normal, get_agent()->get_floor_max_angle())) {
		// We might be able to clear the edge, but it's possible the new position has a too high of an inclination
		// to stand on
		return false;
	}

	// We cast the agent shape down at the hit position to get the correct position, as we might not fit otherwise
	PhysicsDirectSpaceState3D::ShapeParameters shape_params;
	PhysicsDirectSpaceState3D::ShapeResult shape_result;

	shape_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	Ref<Shape3D> body_shape = agent->get_collision_shape();
	shape_params.shape_rid = body_shape->get_rid();
	shape_params.transform.origin = ray_result.position + Vector3(0.0f, agent->get_height() * 0.5f, 0.0f);
	shape_params.transform.origin.y += 1.5f;
	shape_params.motion.y = -3.0f;

	float closest_safe;
	float closest_unsafe;
	debug_draw_cast_motion(body_shape, shape_params);
	if (!agent->get_world_3d()->get_direct_space_state()->cast_motion(shape_params, closest_safe, closest_unsafe)) {
		// ??? This should never happen
		ERR_FAIL_V_MSG(false, "Something clearly went wrong with the getup logic, call a programmer");
	}

	Ref<CylinderShape3D> cyl = agent->get_collision_shape();

	Vector3 safe_pos = shape_params.transform.origin + shape_params.motion * closest_safe - Vector3(0.0f, agent->get_height() * 0.5f, 0.0f);

	Dictionary warp_points;
	Transform3D temp_trf;
	// root motion basis forward is reversed
	temp_trf.basis = Quaternion(Vector3(0.0f, 0.0f, 1.0f), ledge_trf.basis.xform(Vector3(0.0, 0.0, -1.0f)));
	temp_trf.origin = ledge_position;

	warp_points[StringName("Ledge")] = temp_trf;
	temp_trf.origin = safe_pos;

	warp_points[StringName("GetUpTarget")] = temp_trf;

	Dictionary args;
	args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;
	args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->ledge_getup_animation_node;
	args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MovementTransitionInputs::MOVEMENT_LEDGE_GETUP;
	args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->move_state;
	args[HBAgentRootMotionState::PARAM_VELOCITY_MODE] = HBAgentRootMotionState::VelocityMode::CONSERVE;

	ray_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
	ray_params.collide_with_areas = true;
	ray_params.collide_with_bodies = false;

	shape_params.transform.origin = ledge_position;
	shape_params.transform.origin.y += agent->get_height() * 0.5f;

	const int MAX_RESULTS = 8;
	Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
	results.resize(MAX_RESULTS);

	shape_params.collide_with_areas = true;
	shape_params.collide_with_bodies = false;
	shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;

	for (int i = 0; i < agent->get_world_3d()->get_direct_space_state()->intersect_shape(shape_params, results.ptrw(), MAX_RESULTS); i++) {
		HBAgentParkourBeam *beam = Object::cast_to<HBAgentParkourBeam>(results[i].collider);
		if (beam) {
			Dictionary beam_arg_dict;
			beam_arg_dict[HBAgentParkourBeamWalk::PARAM_BEAM_NODE] = beam;
			args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->beam_walk_state;
			args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = beam_arg_dict;
			args[HBAgentRootMotionState::PARAM_VELOCITY_MODE] = HBAgentRootMotionState::ANIMATION_DRIVEN;
		}
	}

	state_machine->transition_to(ASN()->root_motion_state, args);
	return true;
}
void HBAgentLedgeGrabbedStateNew::_update_animator() {
	bool needs_updating_dangle_status = false;
	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		AgentProceduralAnimator::AgentLimb limb = static_cast<AgentProceduralAnimator::AgentLimb>(i);
		float t = animator.get_limb_time(animator_options, limb);
		// Targets only start changing when the animation for the limb isn't done
		// we also change them when the animation is completely done
		if ((animator.get_playback_position() < 1.0f && t <= 1.0f) || animator.get_playback_position() == 1.0f) {
			bool use_predicted = (limb == AgentProceduralAnimator::LIMB_RIGHT_HAND || limb == AgentProceduralAnimator::LIMB_RIGHT_FOOT) && animator_options.playback_direction == 1.0f;
			use_predicted = use_predicted || ((limb == AgentProceduralAnimator::LIMB_LEFT_HAND || limb == AgentProceduralAnimator::LIMB_LEFT_FOOT) && animator_options.playback_direction == -1.0f);
			if (use_predicted) {
				animator_options.target_limb_transforms[limb] = controller->get_limb_transform_predicted(limb);
			} else {
				animator_options.target_limb_transforms[limb] = controller->get_limb_transform(limb);
			}
			animator_options.target_limb_transforms[limb] = controller->get_limb_transform(limb);
		}

		if (i <= AgentProceduralAnimator::LIMB_RIGHT_HAND) {
			continue;
		}
		if (limbs[i].dangle_status != controller->get_limb_is_dangling(limb)) {
			needs_updating_dangle_status = true;
			// Dangle status changed, so we should update the starting limb transform as well...
			animator_options.target_limb_transforms[limb] = controller->get_limb_transform_predicted(limb);
			animator_options.starting_limb_transforms[limb] = animator_options.target_limb_transforms[limb];
		}
		limbs[i].dangle_status = controller->get_limb_is_dangling(limb);
		animator_options.limb_dangle_status[i] = limbs[i].dangle_status;
	}

	if (needs_updating_dangle_status) {
		TypedArray<StringName> dangling_bones;
		get_skeleton()->physical_bones_stop_simulation();
		if (limbs[AgentProceduralAnimator::LIMB_LEFT_FOOT].dangle_status) {
			dangling_bones.push_back("thigh.L");
			dangling_bones.push_back("shin.L");
			dangling_bones.push_back("foot.L");
		}
		if (limbs[AgentProceduralAnimator::LIMB_RIGHT_FOOT].dangle_status) {
			dangling_bones.push_back("thigh.R");
			dangling_bones.push_back("shin.R");
			dangling_bones.push_back("foot.R");
		}

		//get_inertialization_node()->inertialize();

		get_epas_controller()->clear_ignored_bones();

		if (dangling_bones.size() > 0) {
			get_skeleton()->physical_bones_start_simulation_on(dangling_bones);
			// This makes sure inertialization works when going back to non physics animation
			// TODO: We need to somehow add a mechanism to inertialize physics bones properly, as they are not
			// in the pose outputs that the inertialization node receives
			get_epas_controller()->ignore_bones(dangling_bones);
		} else {
			get_skeleton()->physical_bones_stop_simulation();
		}
	}

	animator_options.starting_skeleton_transform.origin = controller->get_global_position();
	animator_options.target_skeleton_transform.origin = controller->get_global_position();
	animator_options.target_skeleton_transform.basis = controller->get_graphics_rotation();
	animator_options.starting_skeleton_transform.basis = controller->get_graphics_rotation();
}

void HBAgentLedgeGrabbedStateNew::_init_limbs() {
	limbs[AgentProceduralAnimator::LIMB_LEFT_HAND].ik_node = get_epas_controller()->get_epas_node("LeftHandIK");
	limbs[AgentProceduralAnimator::LIMB_RIGHT_HAND].ik_node = get_epas_controller()->get_epas_node("RightHandIK");
	limbs[AgentProceduralAnimator::LIMB_LEFT_FOOT].ik_node = get_epas_controller()->get_epas_node("LeftFootIK");
	limbs[AgentProceduralAnimator::LIMB_RIGHT_FOOT].ik_node = get_epas_controller()->get_epas_node("RightFootIK");

	limbs[AgentProceduralAnimator::LIMB_LEFT_HAND].bone_name = "hand.L";
	limbs[AgentProceduralAnimator::LIMB_RIGHT_HAND].bone_name = "hand.R";
	limbs[AgentProceduralAnimator::LIMB_LEFT_FOOT].bone_name = "foot.L";
	limbs[AgentProceduralAnimator::LIMB_RIGHT_FOOT].bone_name = "foot.R";

	limbs[AgentProceduralAnimator::LIMB_LEFT_HAND].magnet_name = "IK.Magnet.hand.L";
	limbs[AgentProceduralAnimator::LIMB_RIGHT_HAND].magnet_name = "IK.Magnet.hand.R";
	limbs[AgentProceduralAnimator::LIMB_LEFT_FOOT].magnet_name = "IK.Magnet.foot.L";
	limbs[AgentProceduralAnimator::LIMB_RIGHT_FOOT].magnet_name = "IK.Magnet.foot.R";
	are_limbs_init = true;
}

// Modifies the output pose with updated values to be used with IK
void HBAgentLedgeGrabbedStateNew::_update_ik_transforms(AgentProceduralAnimator::AgentProceduralPose &p_pose) {
	bool feet_dangling = limbs[AgentProceduralAnimator::LIMB_LEFT_FOOT].dangle_status || limbs[AgentProceduralAnimator::LIMB_RIGHT_FOOT].dangle_status;

	Vector3 skel_pos = p_pose.skeleton_trf.origin;

	if (feet_dangling) {
		Vector3 hand_avg = limbs[AgentProceduralAnimator::LIMB_LEFT_HAND].ik_node->get_target_transform().origin;
		hand_avg += limbs[AgentProceduralAnimator::LIMB_RIGHT_HAND].ik_node->get_target_transform().origin;
		hand_avg *= 0.5f;
		skel_pos = hand_avg;
		const float HEIGHT = 1.35f;
		skel_pos.y -= HEIGHT * 0.30f;
	}

	Ref<EPASAnimationNode> anim_node = get_epas_controller()->get_epas_node(ASN()->wall_parkour_cat_animation_node);
	Ref<EPASPose> ref_pose = anim_node->get_animation()->get_keyframe(0)->get_pose();

	Transform3D wp_trf = anim_node->get_animation()->get_warp_point(anim_node->get_animation()->find_warp_point("ParkourPoint"))->get_transform();
	Vector3 local_offset = wp_trf.xform_inv(ref_pose->get_bone_position("root", get_epas_controller()->get_base_pose()));

	Transform3D new_skel_trf = p_pose.skeleton_trf;
	Transform3D ledge_trf = new_skel_trf;
	ledge_trf.basis = Basis::looking_at(new_skel_trf.basis.xform(Vector3(0.0f, 0.0f, 1.0f)));

	Vector3 ledge_skel_pos = ledge_trf.origin;
	ledge_skel_pos += ledge_trf.basis.xform(local_offset);

	p_pose.skeleton_trf = new_skel_trf;
	p_pose.skeleton_trf.origin = ledge_skel_pos;

	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		Transform3D bone_skel_trf = ref_pose->calculate_bone_global_transform(limbs[i].bone_name, get_skeleton(), get_epas_controller()->get_base_pose());
		Quaternion bone_skel_rot = bone_skel_trf.basis.get_rotation_quaternion();
		Vector3 controller_forward = p_pose.ik_targets[i].basis.get_rotation_quaternion().xform(Vector3(0.0f, 0.0f, -1.0f));
		Vector3 our_forward = Vector3(0.0f, 0.0f, 1.0f);
		Transform3D trf = p_pose.ik_targets[i];
		trf.basis = Quaternion(our_forward, controller_forward) * bone_skel_rot;
		// NOTE: Our reference wall is 0.35 meters forward of the character in the animation data, this allows us to figure out an offset for it
		Vector3 foot_offset = controller_forward * (bone_skel_trf.origin.z - 0.35f);
		trf.origin += foot_offset;

		Vector3 magnet_pos = ref_pose->get_bone_position(limbs[i].magnet_name);
		//p_pose.ik_magnet_positions[i] = get_agent()->get_global_position() + p_pose.skeleton_trf.basis.get_rotation_quaternion().xform(magnet_pos);
		p_pose.ik_magnet_positions[i] = get_agent()->get_global_position() + Quaternion(our_forward, p_pose.skeleton_trf.basis.get_rotation_quaternion().xform(Vector3(0.0f, 0.0f, -1.0f))).normalized().xform(magnet_pos);

		p_pose.ik_targets[i] = trf;

		debug_draw_sphere(p_pose.ik_magnet_positions[i], 0.05f, Color("GREEN"));
	}
}

void HBAgentLedgeGrabbedStateNew::_apply_ik_transforms(AgentProceduralAnimator::AgentProceduralPose &p_pose, bool p_inertialize_graphics_trf) {
	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		limbs[i].ik_node->set_target_transform(p_pose.ik_targets[i]);
		limbs[i].ik_node->set_magnet_position(p_pose.ik_magnet_positions[i]);
	}
	if (p_inertialize_graphics_trf) {
		Transform3D graphics_trf = p_pose.skeleton_trf;
		get_agent()->inertialize_graphics_transform(graphics_trf, 0.1f);
	} else {
		get_agent()->set_global_position(p_pose.skeleton_trf.origin + p_pose.skeleton_position_offset);
		get_agent()->set_graphics_rotation(p_pose.skeleton_trf.basis.get_rotation_quaternion());

		debug_draw_sphere(p_pose.skeleton_trf.origin);
		debug_draw_sphere(p_pose.skeleton_trf.origin + p_pose.skeleton_position_offset, 0.05f, Color("BLUE"));
	}
}

HBAgentLedgeGrabbedStateNew::HBAgentLedgeGrabbedStateNew() {
	controller = memnew(HBLedgeTraversalController);
	CollisionShape3D *cs = memnew(CollisionShape3D);
	controller->add_child(cs);
	Ref<SphereShape3D> sphere;
	sphere.instantiate();
	sphere->set_radius(0.25f);
	cs->set_shape(sphere);
	animator.seek(1.0f);
	add_child(controller);
	controller->set_as_top_level(true);
}
HBAgentLedgeGrabbedStateNew::~HBAgentLedgeGrabbedStateNew() {
	if (!is_inside_tree()) {
		controller->queue_free();
	}
}
/**********************
	Fall State
***********************/
static bool first = false;
void HBAgentFallState::enter(const Dictionary &p_args) {
	Ref<EPASTransitionNode> movement_transition = get_movement_transition_node();
	ERR_FAIL_COND(!movement_transition.is_valid());

	movement_transition->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_FALL);

	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);
	agent->set_movement_mode(HBAgent::MovementMode::MOVE_FALL);
	agent->set_velocity(Vector3());
	first = true;
}

void HBAgentFallState::physics_process(float p_delta) {
	if (first) {
		first = false;
		return;
	}
	HBAgent *agent = get_agent();
	if (agent->is_on_floor()) {
		state_machine->transition_to(ASN()->move_state);
	}
}

void HBAgentParkourBeamWalk::enter(const Dictionary &p_args) {
	beam = Object::cast_to<HBAgentParkourBeam>(p_args.get(PARAM_BEAM_NODE, Variant()));

	DEV_ASSERT(beam != nullptr);
	DEV_ASSERT(beam->get_curve().is_valid());

	curve_offset = beam->get_curve()->get_closest_offset(beam->to_local(get_agent()->get_global_position()));

	Transform3D global_curve_trf = beam->get_global_transform() * beam->get_curve()->sample_baked_with_rotation(curve_offset);
	Vector3 global_curve_point = global_curve_trf.origin;

	Transform3D target_trf;
	target_trf.origin = global_curve_point;
	target_trf.basis = get_graphics_node()->get_global_transform().basis;

	get_agent()->inertialize_graphics_transform(target_trf, 0.25f);
	agent_global_position = global_curve_point;
	debug_draw_sphere(global_curve_point, 0.05f, Color("PURPLE"));
	velocity_spring_accel = Vector3();
	velocity_spring_vel = get_agent()->get_velocity();
	get_movement_transition_node()->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_MOVE_BEAM);
	get_agent()->set_movement_mode(HBAgent::MOVE_MANUAL);
	rotation_spring_goal = get_graphics_node()->get_global_transform().basis;
	Ref<EPASWheelLocomotion> loc = get_epas_controller()->get_epas_node("MovementWheelBeam");
	loc->set_x_blend(get_agent()->get_velocity().length() / 2.8);
	loc->set_linear_velocity(get_agent()->get_velocity());
}

void HBAgentParkourBeamWalk::physics_process(float p_delta) {
	debug_draw_clear();
	Vector3 movement_input = get_agent()->get_desired_movement_input_transformed();

	Transform3D curr_position_trf = beam->get_curve()->sample_baked_with_rotation(curve_offset);
	curr_position_trf = beam->get_global_transform() * curr_position_trf;
	Vector3 forward = curr_position_trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f));
	forward.y = 0.0f;
	forward.normalize();
	Vector3 desired_movement_input;
	Vector3 movement_dir = movement_input.normalized();
	bool movement_goes_through_beam = movement_dir.angle_to(forward) < Math::deg_to_rad(45.0f) || movement_dir.angle_to(-forward) < Math::deg_to_rad(45.0f);

	if (handle_parkour()) {
		return;
	}

	if (movement_input.length_squared() > 0) {
		if (movement_goes_through_beam) {
			desired_movement_input = movement_input.project(forward);
			desired_movement_input = desired_movement_input.normalized() * movement_input.length();
		}
	}

	Ref<HBAgentConstants> agent_constants = get_agent()->get_agent_constants();

	Vector3 desired_velocity = desired_movement_input * agent_constants->get_max_move_velocity();
	// Kill velocity near edges
	bool are_end_edges[2] = {
		beam->is_point_edge(0),
		beam->is_point_edge(beam->get_curve()->get_point_count() - 1)
	};

	const int desired_velocity_dir = SIGN(desired_velocity.dot(forward));

	if ((desired_velocity_dir == 1 && are_end_edges[1] > 0 && Math::is_equal_approx(curve_offset, beam->get_curve()->get_baked_length())) ||
			(desired_velocity_dir == -1 && are_end_edges[0] && Math::is_zero_approx(curve_offset))) {
		if (!velocity_spring_vel.is_zero_approx()) {
			get_inertialization_node()->inertialize();
		}
		desired_velocity = Vector3();
		velocity_spring_vel = Vector3();
		velocity_spring_accel = Vector3();
	}
	HBSprings::velocity_spring_vector3(
			velocity_spring_vel,
			velocity_spring_accel,
			desired_velocity,
			agent_constants->get_velocity_spring_halflife(),
			p_delta);

	// We project the velocity vector along the curve to get the change in offset
	Vector3 vel = velocity_spring_vel.project(forward);
	curve_offset += vel.length() * SIGN(vel.dot(forward)) * p_delta;

	float clamped_offset = CLAMP(curve_offset, 0.0f, beam->get_curve()->get_baked_length());
	// We have to check if we are allowed to walk off this end of the beam
	bool is_target_end_edge = clamped_offset > 0 ? are_end_edges[1] : are_end_edges[0];
	if (curve_offset != clamped_offset && !is_target_end_edge) {
		// When the offset goes beyond our clamped offset this means we've gone out of the beam
		curve_offset = clamped_offset;
		agent_global_position = beam->get_global_transform().xform(beam->get_curve()->sample_baked(curve_offset));
		Dictionary args;
		args[HBAgentMoveState::PARAM_TRANSITION_DURATION] = 0.5f;
		state_machine->transition_to(ASN()->move_state, args);
		get_agent()->reset_desired_input_velocity_to(get_agent()->get_linear_velocity());
		return;
	}
	curve_offset = clamped_offset;

	agent_global_position = beam->get_global_transform().xform(beam->get_curve()->sample_baked(curve_offset));

	Vector3 curr_agent_pos = agent_global_position;

	Quaternion global_gn_rot = get_agent()->get_graphics_rotation();
	if (movement_input.length_squared() > 0) {
		Vector3 desired_look_dir = forward;
		desired_look_dir.rotate(Vector3(0.0f, 1.0f, 0.0f), Math::snapped(forward.normalized().signed_angle_to(movement_input.normalized(), Vector3(0.0f, 1.0f, 0.0f)), Math::deg_to_rad(90.0f)));
		if (desired_look_dir.length_squared() > 0) {
			rotation_spring_goal = Quaternion(Vector3(0.0f, 0.0f, -1.0f), desired_look_dir);
		}
	}

	HBSprings::simple_spring_damper_exact_quat(global_gn_rot, rotation_spring_vel, rotation_spring_goal, 0.05f, p_delta);

	get_agent()->set_graphics_rotation(global_gn_rot);

	// Set the velocity of the agent for animation
	get_agent()->set_velocity(vel);
	get_agent()->set_global_position(curr_agent_pos);
	Ref<EPASWheelLocomotion> loc = get_epas_controller()->get_epas_node("MovementWheelBeam");
	loc->set_x_blend(get_agent()->get_velocity().length() / 2.8);
	loc->set_linear_velocity(get_agent()->get_velocity());
}

bool HBAgentParkourBeamWalk::try_ledge_drop() {
	Node3D *gn = get_graphics_node();
	Vector3 right = gn->get_global_transform().basis.xform(Vector3(1.0f, 0.0f, 0.0f));
	Camera3D *cam = get_viewport()->get_camera_3d();
	DEV_ASSERT(cam);

	Vector3 cam_forward = cam->get_basis().xform(Vector3(0.0f, 0.0f, -1.0f));
	float drop_dir_mul = SIGN(cam_forward.dot(right));
	right = drop_dir_mul != 0 ? right * drop_dir_mul : right;

	PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	PhysicsDirectSpaceState3D::RayResult ray_result;

	const int RAY_ITERS = 5;
	const float RAY_MAX = 0.25f;

	Vector3 agent_pos = get_agent()->get_global_position();

	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	bool got_result = false;
	float result_dist = -1;

	for (int i = 0; i < RAY_ITERS; i++) {
		float dist = i / ((float)RAY_ITERS - 1);
		dist *= RAY_MAX;
		ray_params.from = agent_pos + right * dist;
		ray_params.to = ray_params.from;
		ray_params.from.y += 0.2;
		ray_params.to.y -= get_agent()->get_height();

		debug_draw_raycast(ray_params);
		if (!dss->intersect_ray(ray_params, ray_result)) {
			got_result = true;
			result_dist = dist;
			break;
		}
	}

	if (!got_result) {
		return false;
	}

	ray_params.from = get_agent()->get_global_position() + right * result_dist;
	ray_params.to = ray_params.from + ray_params.from.direction_to(get_agent()->get_global_position()) * result_dist;
	ray_params.from.y -= 0.1f;
	ray_params.to.y -= 0.1f;

	debug_draw_raycast(ray_params);
	if (!dss->intersect_ray(ray_params, ray_result)) {
		return false;
	}

	const float floor_max_angle = get_agent()->get_floor_max_angle();

	if (!is_wall(ray_result.normal, floor_max_angle)) {
		return false;
	}

	Vector3 ledge_normal = ray_result.normal;
	Vector3 ledge_position = ray_result.position;

	ray_params.from = ray_result.position;
	ray_params.to = ray_result.position;
	ray_params.from.y += get_agent()->get_height() * 0.5f;
	ray_params.to.y -= get_agent()->get_height() * 0.5f;

	debug_draw_raycast(ray_params);
	if (!dss->intersect_ray(ray_params, ray_result)) {
		return false;
	}

	if (!is_floor(ray_result.normal, floor_max_angle)) {
		return false;
	}

	ledge_position.y = ray_result.position.y;

	Transform3D ledge_trf;
	ledge_trf.origin = ledge_position;
	ledge_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), ledge_normal);

	Dictionary ledge_args;
	// TODO: Fix this
	//ledge_args[HBAgentLedgeGrabbedState::PARAM_LEDGE_TRF] = ledge_trf;

	Dictionary warp_points;
	warp_points[StringName("Edge")] = ledge_trf;

	Dictionary args;
	args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;
	args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = StringName("LedgeDrop");
	args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MovementTransitionInputs::MOVEMENT_LEDGE_DROP;
	args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->ledge_grabbed_state;
	args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = ledge_args;

	state_machine->transition_to(ASN()->root_motion_state, args);
	return true;
}

void HBAgentRootMotionState::enter(const Dictionary &p_args) {
	animation_node = get_epas_controller()->get_epas_node(p_args.get(PARAM_ANIMATION_NODE_NAME, Variant()));
	ERR_FAIL_COND(!p_args.has(PARAM_TRANSITION_NODE_INDEX));
	ERR_FAIL_COND_MSG(!animation_node.is_valid(), "PARAM_ANIMATION_NODE_NAME was missing or was the wrong type.");
	ERR_FAIL_COND_MSG(!animation_node->get_animation().is_valid(), vformat("Animation node %s had no animation.", p_args.get(PARAM_ANIMATION_NODE_NAME, "")));
	Dictionary warp_points = p_args.get(PARAM_WARP_POINTS, Dictionary());
	for (int i = 0; i < animation_node->get_animation()->get_warp_point_count(); i++) {
		Ref<EPASWarpPoint> wp = animation_node->get_animation()->get_warp_point(i);
		ERR_FAIL_COND_MSG(!warp_points.has(wp->get_point_name()), vformat("Needed warp point %s was missing.", wp->get_point_name()));
		animation_node->set_warp_point_transform(wp->get_point_name(), warp_points.get(wp->get_point_name(), Transform3D()));
		Transform3D trf = warp_points.get(wp->get_point_name(), Transform3D());
		debug_draw_sphere(trf.origin, 0.05f, Color("GREEN"));
		debug_draw_line(trf.origin, trf.origin + trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f)));
	}

	velocity_mode = (VelocityMode)(int)p_args.get(PARAM_VELOCITY_MODE, VelocityMode::CONSERVE);

	next_state_args = p_args.get(RootMotionParams::PARAM_NEXT_STATE_ARGS, Dictionary());
	next_state = p_args.get(RootMotionParams::PARAM_NEXT_STATE, "");
	collisions_enabled = p_args.get(RootMotionParams::PARAM_COLLIDE, false);

	Ref<EPASTransitionNode> transition_node = get_movement_transition_node();
	transition_node->transition_to(p_args.get(PARAM_TRANSITION_NODE_INDEX, 0));

	animation_node->set_root_motion_starting_transform(get_skeleton()->get_global_transform());
	animation_node->set_root_motion_forward(Vector3(0.0f, 0.0f, 1.0f));
	animation_node->play();

	starting_velocity = get_agent()->get_velocity();

	get_agent()->set_movement_mode(HBAgent::MOVE_MANUAL);
	get_agent()->set_velocity(Vector3());
	get_agent()->root_motion_begin(animation_node, get_physics_process_delta_time());
	prev_pos = get_agent()->get_global_position();
	inertialization_init = false;
	get_agent()->set_is_invulnerable(p_args.get(PARAM_INVULNERABLE, false));
}

void HBAgentRootMotionState::exit() {
	get_agent()->set_is_invulnerable(false);
}

void HBAgentRootMotionState::physics_process(float p_delta) {
	if (hack) {
		return;
	}
	debug_draw_sphere(animation_node->get_root_motion_starting_transform().origin, 0.05f, Color("GREEN"));
	get_agent()->apply_root_motion(animation_node, collisions_enabled, p_delta);

	if (collisions_enabled) {
		get_agent()->update(p_delta);
	}

	if (!inertialization_init) {
		Transform3D trf = get_agent()->get_global_transform();
		trf.basis = get_agent()->get_graphics_rotation();
		get_agent()->inertialize_graphics_transform(trf, 0.25f);
		inertialization_init = true;
	}

	if (!animation_node->is_playing()) {
		animation_finished(p_delta);
		return;
	}
	prev_pos = get_agent()->get_global_position();
}

void HBAgentRootMotionState::animation_finished(float p_delta) {
	switch (velocity_mode) {
		case ANIMATION_DRIVEN: {
			get_agent()->reset_desired_input_velocity_to((get_agent()->get_global_position() - prev_pos) / get_physics_process_delta_time());
		} break;
		case CONSERVE: {
			get_agent()->set_velocity(get_agent()->get_desired_movement_input_transformed() * get_agent()->get_agent_constants()->get_max_move_velocity());
			get_agent()->reset_desired_input_velocity_to(get_agent()->get_desired_movement_input_transformed() * get_agent()->get_agent_constants()->get_max_move_velocity());
		} break;
	}
	state_machine->transition_to(next_state, next_state_args);
};

#ifdef DEBUG_ENABLED
void HBAgentRootMotionState::debug_ui_draw() {
	HBAgentState::debug_ui_draw();
	ImGui::Checkbox("Fuck", &hack);
}
#endif

bool HBAgentLedgeGrabbedStateNew::_handle_parkour_mid() {
	const Vector3 input = controller->get_graphics_rotation().xform(Vector3(get_agent()->get_movement_input().x, 0.0f, 0.0f).normalized());
	if (input.length_squared() == 0.0f) {
		return false;
	}
	const float EJECT_REACH = 2.0f;
	PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::ShapeParameters shape_params;
	static constexpr const int ITERS = 10;

	Ref<Shape3D> shape = get_agent()->get_collision_shape();
	shape_params.shape_rid = shape->get_rid();
	shape_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	shape_params.transform.origin = controller->get_global_position();
	shape_params.transform.origin.y += get_agent()->get_height() * 0.5f;
	shape_params.transform.origin += controller->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, 0.01f));
	for (int i = 0; i < ITERS; i++) {
		shape_params.motion = input * EJECT_REACH * ((i + 1) / ((float)ITERS));
		real_t closest_safe, closest_unsafe;
		dss->cast_motion(shape_params, closest_safe, closest_unsafe);
		bool first_hit = closest_safe != 1.0f && closest_unsafe != 1.0f;

		PhysicsDirectSpaceState3D::ShapeParameters shape_params_2 = shape_params;
		shape_params_2.transform.origin = shape_params.transform.origin + shape_params.motion * closest_safe;
		shape_params_2.motion = Vector3(0.0f, -get_agent()->get_height() * 1.0f, 0.0f);

		debug_draw_cast_motion(shape, shape_params_2, Color("BLUE"));
		dss->cast_motion(shape_params_2, closest_safe, closest_unsafe);

		bool hit = closest_safe != 1.0f && closest_unsafe != 1.0f;

		if (hit) {
			Dictionary root_motion_args;

			Dictionary warp_points;
			Transform3D wp_trf;
			wp_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), input);
			wp_trf.origin = shape_params_2.transform.origin + shape_params_2.motion * closest_unsafe;
			wp_trf.origin.y -= get_agent()->get_height() * 0.5f;

			//_transition_to_short_hop(wp_trf, ASN()->move_state, Dictionary());

			warp_points[StringName("Ledge")] = wp_trf;
			root_motion_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MovementTransitionInputs::MOVEMENT_SHORT_HOP;
			root_motion_args[HBAgentRootMotionState::PARAM_VELOCITY_MODE] = HBAgentRootMotionState::VelocityMode::CONSERVE;
			root_motion_args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;
			root_motion_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->short_hop_animation_node;
			root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->move_state;

			PhysicsDirectSpaceState3D::RayParameters ray_params;
			ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
			ray_params.from = wp_trf.origin - input * get_agent()->get_radius();
			ray_params.from.y += get_agent()->get_radius();
			ray_params.to = ray_params.from;
			ray_params.to.y -= get_agent()->get_radius() * 2.0f;
			debug_draw_raycast(ray_params);

			PhysicsDirectSpaceState3D::RayResult result;
			if (!dss->intersect_ray(ray_params, result)) {
				continue;
			}

			state_machine->transition_to(ASN()->root_motion_state, root_motion_args);
			return true;
		}

		if (first_hit) {
			break;
		}
	}

	return false;
}

bool HBAgentLedgeGrabbedStateNew::_handle_drop_to_parkour_point() {
	PhysicsDirectSpaceState3D::ShapeParameters params;

	HBAgent *agent = get_agent();

	Ref<CylinderShape3D> shape;
	shape.instantiate();
	shape->set_radius(agent->get_radius());
	shape->set_height(HBAgentWallParkourStateNew::SHORT_GRAB_REACH);

	Transform3D ledge_transform = ledge->get_agent_ledge_transform_at_offset(controller->get_ledge_offset());

	params.shape_rid = shape->get_rid();
	params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
	params.transform.origin = ledge_transform.origin;
	params.transform.origin.y -= shape->get_height() * 0.5f;

	PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();
	const int MAX_RESULTS = 10;
	PhysicsDirectSpaceState3D::ShapeResult results[MAX_RESULTS];
	debug_draw_cast_motion(shape, params, Color("GREEN"));
	int result_count = dss->intersect_shape(params, results, MAX_RESULTS);

	bool is_long_jump = false;

	if (result_count == 0) {
		is_long_jump = true;
		shape->set_height(HBAgentWallParkourStateNew::LONG_GRAB_REACH);
		params.transform.origin = ledge_transform.origin;
		params.transform.origin.y -= shape->get_height() * 0.5f;
		result_count = dss->intersect_shape(params, results, MAX_RESULTS);
	}

	HBAgentParkourPoint *parkour_point = nullptr;

	const Vector3 pp_forward = ledge_transform.basis.xform(Vector3(0.0f, 0.0f, 1.0f));

	for (int i = 0; i < result_count; i++) {
		HBAgentParkourPoint *pp_candidate = Object::cast_to<HBAgentParkourPoint>(results[i].collider);
		if (!pp_candidate) {
			continue;
		}
		Vector3 point_forward = pp_candidate->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));

		if (point_forward.dot(pp_forward) > 0.0f) {
			parkour_point = pp_candidate;
			break;
		}
	}

	if (parkour_point && is_long_jump) {
		Dictionary wp_points;
		Transform3D wp_trf = parkour_point->get_global_transform();
		wp_points[ASN()->wall_parkour_cat_point_wp_name] = wp_trf;

		Dictionary args;
		args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MOVEMENT_WALLPARKOUR_DOWN_LONG_JUMP;
		args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->wallparkour_down_long_jump_animation_node;

		Dictionary next_state_args;
		next_state_args[HBAgentWallParkourStateNew::PARAM_PARKOUR_NODE] = parkour_point;

		args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = next_state_args;
		args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->wall_parkour_state;

		args[HBAgentRootMotionState::PARAM_WARP_POINTS] = wp_points;

		state_machine->transition_to(ASN()->root_motion_state, args);
		return true;
	}

	if (parkour_point) {
		Dictionary state_args;
		state_args[HBAgentWallTransitionState::PARAM_TRANSITION_MODE] = HBAgentWallTransitionState::TO_WALL;
		state_args[HBAgentWallTransitionState::PARAM_PARKOUR_POINT] = parkour_point;

		HBAgentWallTransitionState *wall_transition_state = Object::cast_to<HBAgentWallTransitionState>(state_machine->get_state(ASN()->wall_parkour_transition_state));
		AgentProceduralAnimator::AgentProceduralPose starting_pose;
		animator.get_output_pose(starting_pose);
		_update_ik_transforms(starting_pose);
		wall_transition_state->set_starting_pose(starting_pose);

		state_machine->transition_to(ASN()->wall_parkour_transition_state, state_args);

		return true;
	}

	return false;
}

/**********************
	Wall Parkour State
***********************/
void HBAgentWallParkourStateNew::apply_offsets(AgentProceduralAnimator::AgentProceduralPose &p_pose) const {
}

void HBAgentWallParkourStateNew::apply_pose(AgentProceduralAnimator::AgentProceduralPose &p_pose) {
	Ref<EPASIKNode> ik_nodes[AgentProceduralAnimator::LIMB_MAX] = {
		get_epas_controller()->get_epas_node(ASN()->left_hand_ik_node),
		get_epas_controller()->get_epas_node(ASN()->right_hand_ik_node),
		get_epas_controller()->get_epas_node(ASN()->left_foot_ik_node),
		get_epas_controller()->get_epas_node(ASN()->right_foot_ik_node),
	};
	calculate_magnet_for_limbs(p_pose);
	for (int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		debug_draw_sphere(p_pose.ik_targets[i].origin, 0.06f, Color("BLUE"));
		debug_draw_sphere(p_pose.ik_magnet_positions[i], 0.06f, Color("RED"));
		ik_nodes[i]->set_ik_influence(1.0f);
		ik_nodes[i]->set_magnet_position(p_pose.ik_magnet_positions[i]);
		ik_nodes[i]->set_target_transform(p_pose.ik_targets[i]);
	}

	HBAgent *agent = get_agent();

	agent->set_global_position(p_pose.skeleton_trf.origin + p_pose.skeleton_position_offset);
	agent->set_graphics_rotation(p_pose.skeleton_trf.basis);

	debug_draw_sphere(p_pose.skeleton_trf.origin);
	debug_draw_sphere(p_pose.skeleton_trf.origin + p_pose.skeleton_position_offset, 0.05f, Color("BLUE"));
}

// Returns a simple initial pose without offsets
void HBAgentWallParkourStateNew::find_initial_pose(WallParkourInitialPose &p_pose, const HBAgentParkourPoint *p_point) const {
	Ref<EPASAnimationNode> anim_node = get_epas_controller()->get_epas_node(ASN()->wall_parkour_cat_animation_node);
	Ref<EPASPose> reference_pose = anim_node->get_animation()->get_keyframe(0)->get_pose();

	const int point_wp = anim_node->get_animation()->find_warp_point(ASN()->wall_parkour_cat_point_wp_name);
	Ref<EPASWarpPoint> wp = anim_node->get_animation()->get_warp_point(point_wp);
	const Transform3D wp_trf = wp->get_transform();
	Transform3D root_trf_local = wp_trf.affine_inverse() * reference_pose->get_bone_transform("root", get_epas_controller()->get_base_pose());
	p_pose.pose.skeleton_trf = p_point->get_global_transform() * root_trf_local;
	p_pose.pose.skeleton_trf.basis = Quaternion(Vector3(0.0, 0.0, -1.0f), p_pose.pose.skeleton_trf.basis.xform(Vector3(0.0, 0.0, 1.0f)));

	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		StringName bone_name = AgentProceduralAnimator::limb_to_bone_name(static_cast<AgentProceduralAnimator::AgentLimb>(i));
		Transform3D hand_trf_local = reference_pose->calculate_bone_global_transform(bone_name, get_skeleton(), get_epas_controller()->get_base_pose());
		hand_trf_local = wp_trf.affine_inverse() * hand_trf_local;
		p_pose.pose.ik_targets[i] = p_point->get_global_transform() * hand_trf_local;
		if (i <= AgentProceduralAnimator::LIMB_RIGHT_HAND) {
			p_pose.pose.ik_magnet_positions[i] = get_agent()->get_global_position() + get_agent()->get_graphics_rotation().xform(Vector3(0.0f, -1.0f, 0.0f));
		} else {
			p_pose.pose.ik_magnet_positions[i] = get_agent()->get_global_position() + get_agent()->get_graphics_rotation().xform(Vector3(0.0f, -1.0f, 0.0f));
		}
		p_pose.parkour_points[i] = const_cast<HBAgentParkourPoint *>(p_point);
	}

	p_pose.pose.valid = true;
}

void HBAgentWallParkourStateNew::update_animator(const AgentProceduralAnimator::AgentLimb p_limb_to_move) {
	Ref<EPASIKNode> ik_nodes[AgentProceduralAnimator::LIMB_MAX] = {
		get_epas_controller()->get_epas_node(ASN()->left_hand_ik_node),
		get_epas_controller()->get_epas_node(ASN()->right_hand_ik_node),
		get_epas_controller()->get_epas_node(ASN()->left_foot_ik_node),
		get_epas_controller()->get_epas_node(ASN()->right_foot_ik_node),
	};

	for (int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		animator_options.target_limb_transforms[i] = target_pose.pose.ik_targets[i];
		animator_options.starting_limb_transforms[i] = ik_nodes[i]->get_target_transform();
		animator_options.limb_animation_timings[i][0] = 0.0f;
		animator_options.limb_animation_timings[i][1] = 0.0f;
		animator_options.limb_dangle_status[i] = false;
	}
	animator_options.starting_skeleton_transform.origin = get_agent()->get_global_position();
	animator_options.starting_skeleton_transform.basis = get_agent()->get_graphics_rotation();
	animator_options.target_skeleton_transform = target_pose.pose.skeleton_trf;

	animator_options.limb_animation_timings[p_limb_to_move][0] = 0.15f;
	animator_options.limb_animation_timings[p_limb_to_move][1] = 1.0f;
	const AgentProceduralAnimator::AgentLimb foot_to_move = p_limb_to_move == AgentProceduralAnimator::LIMB_LEFT_HAND ? AgentProceduralAnimator::LIMB_LEFT_FOOT : AgentProceduralAnimator::LIMB_RIGHT_FOOT;

	const AgentProceduralAnimator::AgentLimb hand_to_stay = p_limb_to_move == AgentProceduralAnimator::LIMB_LEFT_HAND ? AgentProceduralAnimator::LIMB_RIGHT_HAND : AgentProceduralAnimator::LIMB_LEFT_HAND;
	const AgentProceduralAnimator::AgentLimb foot_to_stay = hand_to_stay == AgentProceduralAnimator::LIMB_LEFT_HAND ? AgentProceduralAnimator::LIMB_LEFT_FOOT : AgentProceduralAnimator::LIMB_RIGHT_FOOT;

	animator_options.limb_dangle_status[hand_to_stay] = true;
	animator_options.limb_dangle_status[foot_to_stay] = true;

	animator_options.limb_animation_timings[foot_to_move][0] = 0.0f;
	animator_options.limb_animation_timings[foot_to_move][1] = 0.75f;

	/*limb_arm->animation_start = 0.15f;
	limb_arm->animation_end = 1.0f;
	limb_foot->animation_start = 0.0f;
	limb_foot->animation_end = 0.75f;*/
}

void HBAgentWallParkourStateNew::update_animator_initial() {
	Ref<EPASIKNode> ik_nodes[AgentProceduralAnimator::LIMB_MAX] = {
		get_epas_controller()->get_epas_node(ASN()->left_hand_ik_node),
		get_epas_controller()->get_epas_node(ASN()->right_hand_ik_node),
		get_epas_controller()->get_epas_node(ASN()->left_foot_ik_node),
		get_epas_controller()->get_epas_node(ASN()->right_foot_ik_node),
	};

	for (int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		animator_options.target_limb_transforms[i] = target_pose.pose.ik_targets[i];
		animator_options.starting_limb_transforms[i] = target_pose.pose.ik_targets[i];
		animator_options.limb_animation_timings[i][0] = 0.0f;
		animator_options.limb_animation_timings[i][1] = 0.0f;
		animator_options.limb_dangle_status[i] = false;
	}
	animator_options.target_skeleton_transform = target_pose.pose.skeleton_trf;
	animator_options.starting_skeleton_transform.origin = animator_options.target_skeleton_transform.origin;
	animator_options.starting_skeleton_transform.basis = animator_options.target_skeleton_transform.basis;
}

void HBAgentWallParkourStateNew::bring_hands_together() {
	WallParkourInitialPose new_pose;
	find_initial_pose(new_pose, current_parkour_point_target);

	const AgentProceduralAnimator::AgentLimb limb_to_move = current_limb == AgentProceduralAnimator::LIMB_LEFT_HAND ? AgentProceduralAnimator::LIMB_RIGHT_HAND : AgentProceduralAnimator::LIMB_LEFT_HAND;
	const AgentProceduralAnimator::AgentLimb foot_to_move = limb_to_move == AgentProceduralAnimator::LIMB_LEFT_HAND ? AgentProceduralAnimator::LIMB_LEFT_FOOT : AgentProceduralAnimator::LIMB_RIGHT_FOOT;

	target_pose.pose.ik_targets[limb_to_move] = new_pose.pose.ik_targets[limb_to_move];
	target_pose.parkour_points[limb_to_move] = new_pose.parkour_points[limb_to_move];
	target_pose.pose.ik_targets[foot_to_move] = new_pose.pose.ik_targets[foot_to_move];
	target_pose.parkour_points[foot_to_move] = new_pose.parkour_points[foot_to_move];

	target_pose.pose.skeleton_trf = new_pose.pose.skeleton_trf;

	update_animator(limb_to_move);

	back_straightness_target = 0.0f;

	animator.reset();
}

void HBAgentWallParkourStateNew::calculate_magnet_for_limbs(AgentProceduralAnimator::AgentProceduralPose &p_pose) {
	for (int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		if (i <= AgentProceduralAnimator::AgentLimb::LIMB_RIGHT_HAND) {
			// Hands
			p_pose.ik_magnet_positions[i] = p_pose.ik_targets[i].origin;
			p_pose.ik_magnet_positions[i].y -= 1.0f;
		} else {
			// Feet
			p_pose.ik_magnet_positions[i] = p_pose.ik_targets[i].origin + Vector3(0.0f, 1.0f, 0.0f);
		}
	}
}

void HBAgentWallParkourStateNew::enter(const Dictionary &p_args) {
	ERR_FAIL_COND(!p_args.has(PARAM_PARKOUR_NODE));
	HBAgentParkourPoint *starting_parkour_node = Object::cast_to<HBAgentParkourPoint>(p_args.get(PARAM_PARKOUR_NODE, Variant()));
	ERR_FAIL_COND(!starting_parkour_node);
	find_initial_pose(target_pose, starting_parkour_node);
	get_movement_transition_node()->transition_to(HBAgentConstants::MOVEMENT_PARKOUR_CAT);
	Transform3D new_graphics_trf = target_pose.pose.skeleton_trf;
	get_agent()->inertialize_graphics_transform(new_graphics_trf, 0.25f);
	Ref<EPASLookatNode> lookat_node = get_epas_controller()->get_epas_node(ASN()->lookat_head_node_name);
	lookat_node->set_influence(0.0f);
	lookat_node = get_epas_controller()->get_epas_node(ASN()->lookat_torso_node_name);
	lookat_node->set_influence(0.0f);

	update_animator_initial();

	for (int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		animator_options.limb_position_spring_halflifes[i] = 0.0f;
	}
	animator_options.skeleton_position_spring_halflife = 0.0f;
	animator.set_animation_duration(0.4f);
	animator.restart();
	back_straightness_target = 0.0f;
	back_straightness_velocity = 0.0f;
	back_straightness_blend_node = get_epas_controller()->get_epas_node("WallParkourCatBlend");
	ERR_FAIL_COND(!back_straightness_blend_node.is_valid());

	animator.process(animator_options, 0.4f);

	AgentProceduralAnimator::AgentProceduralPose op;
	animator.get_output_pose(op);
	apply_pose(op);

	straight_side_target = 0.0f;
	straight_side_blend_node = 0.0f;
	straight_side_blend_node = get_epas_controller()->get_epas_node("WallParkourCatStraight");
}

void HBAgentWallParkourStateNew::exit() {
	Ref<EPASIKNode> ik_nodes[AgentProceduralAnimator::LIMB_MAX] = {
		get_epas_controller()->get_epas_node(ASN()->left_hand_ik_node),
		get_epas_controller()->get_epas_node(ASN()->right_hand_ik_node),
		get_epas_controller()->get_epas_node(ASN()->left_foot_ik_node),
		get_epas_controller()->get_epas_node(ASN()->right_foot_ik_node),
	};

	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		ik_nodes[i]->set_ik_influence(0.0f);
	}
}

void HBAgentWallParkourStateNew::physics_process(float p_delta) {
	Ref<EPASIKNode> ik_nodes[AgentProceduralAnimator::LIMB_MAX] = {
		get_epas_controller()->get_epas_node(ASN()->left_hand_ik_node),
		get_epas_controller()->get_epas_node(ASN()->right_hand_ik_node),
		get_epas_controller()->get_epas_node(ASN()->left_foot_ik_node),
		get_epas_controller()->get_epas_node(ASN()->right_foot_ik_node),
	};

	float back_straightness = back_straightness_blend_node->get_blend_amount();
	HBSprings::critical_spring_damper_exact(&back_straightness, &back_straightness_velocity, back_straightness_target, 0.15f, p_delta);
	back_straightness_blend_node->set_blend_amount(back_straightness);

	float shoulder_side = straight_side_blend_node->get_blend_amount();
	HBSprings::critical_spring_damper_exact(&shoulder_side, &straight_side_velocity, straight_side_target, 0.15f, p_delta);
	straight_side_blend_node->set_blend_amount(shoulder_side);

	debug_draw_clear();

	HBAgent *agent = get_agent();
	Vector3 input = agent->get_movement_input();
	input.y = -input.z;
	input.z = 0.0f;

	bool animator_was_finished = animator.is_done();
	animator.process(animator_options, p_delta);

	AgentProceduralAnimator::AgentProceduralPose pose;
	animator.get_output_pose(pose);
	apply_pose(pose);

	bool animator_just_finished = !animator_was_finished && animator.is_done();
	bool are_hands_together = target_pose.parkour_points[AgentProceduralAnimator::LIMB_LEFT_HAND] == target_pose.parkour_points[AgentProceduralAnimator::LIMB_RIGHT_HAND];

	int vertical_component = 0;
	int horizontal_component = 0;

	Vector3 snapped_untransformed_dir;

	if (!input.is_zero_approx()) {
		const float snapped_angle = Math::snapped(input.signed_angle_to(Vector3(0.0, 1.0, 0.0), Vector3(0.0f, 0.0f, -1.0f)), Math::deg_to_rad(45.0f));
		snapped_untransformed_dir = Vector3(0.0f, 1.0f, 0.0f).rotated(Vector3(0.0f, 0.0f, 1.0f), snapped_angle);
		vertical_component = Math::is_zero_approx(snapped_untransformed_dir.y) ? 0 : SIGN(snapped_untransformed_dir.y);
		horizontal_component = Math::is_zero_approx(snapped_untransformed_dir.x) ? 0 : SIGN(snapped_untransformed_dir.x);
	}

	if (animator_just_finished && current_parkour_point_target && !are_hands_together) {
		if (vertical_component == 0) {
			bring_hands_together();
		}
	}

	if (!animator.is_done()) {
		return;
	}

	// Everything after this is to accept user input

	if (get_agent()->is_action_pressed(HBAgent::AgentInputAction::INPUT_ACTION_PARKOUR_DOWN)) {
		// Letting go code
		Vector3 global_pos = get_agent()->get_global_position() + get_agent()->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, 0.1f));
		Transform3D graphics_transform = Transform3D(get_agent()->get_graphics_rotation(), global_pos);
		get_agent()->inertialize_graphics_transform(graphics_transform, 0.25f);

		state_machine->transition_to(ASN()->fall_state);
		// Force update so fall state doesn't think we are not falling
		get_agent()->update(0.0f);
		return;
	}

	if (_handle_parkour_mid()) {
		return;
	}

	if (input.length_squared() > 0.0f) {
		input.normalize();

		// Our sampling source is from the parkour point if the hands are together
		AgentProceduralAnimator::AgentLimb sampling_source_limb = AgentProceduralAnimator::LIMB_LEFT_HAND;
		// When going diagonally or horizontally we use the hand that's on the side we want
		if ((vertical_component != 0 && horizontal_component != 0) || horizontal_component != 0) {
			sampling_source_limb = horizontal_component == 1 ? AgentProceduralAnimator::LIMB_RIGHT_HAND : AgentProceduralAnimator::LIMB_LEFT_HAND;
		} else if (!are_hands_together) {
			// When moving vertically
			// we use the lowest/highest hand
			if (vertical_component != 0) {
				const float left_hand_height = target_pose.parkour_points[AgentProceduralAnimator::LIMB_LEFT_HAND]->get_global_position().y;
				const float right_hand_height = target_pose.parkour_points[AgentProceduralAnimator::LIMB_RIGHT_HAND]->get_global_position().y;
				if (vertical_component > 0) {
					sampling_source_limb = left_hand_height > right_hand_height ? AgentProceduralAnimator::LIMB_LEFT_HAND : AgentProceduralAnimator::LIMB_RIGHT_HAND;
				} else {
					sampling_source_limb = left_hand_height < right_hand_height ? AgentProceduralAnimator::LIMB_LEFT_HAND : AgentProceduralAnimator::LIMB_RIGHT_HAND;
				}
			}
		}

		const Transform3D sampling_source_trf = target_pose.parkour_points[sampling_source_limb]->get_global_transform();

		float snapped_angle = Math::snapped(input.signed_angle_to(Vector3(0.0, 1.0, 0.0), Vector3(0.0f, 0.0f, -1.0f)), Math::deg_to_rad(45.0f));
		Vector3 snapped_dir = Vector3(0.0f, 1.0f, 0.0f).rotated(sampling_source_trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f)), snapped_angle);
		snapped_dir.normalize();

		if (vertical_component > 0) {
			// Try to grab a ledge above
			HBAgentParkourLedge *ledge = nullptr;
			float offset;

			if (find_reachable_parkour_ledge(sampling_source_limb, snapped_dir, SHORT_GRAB_REACH, &ledge, offset)) {
				Dictionary args;
				args[HBAgentWallTransitionState::PARAM_LEDGE] = ledge;
				args[HBAgentWallTransitionState::PARAM_LEDGE_OFFSET] = offset;
				args[HBAgentWallTransitionState::PARAM_TRANSITION_MODE] = HBAgentWallTransitionState::TransitionMode::TO_LEDGE;
				args[HBAgentWallTransitionState::PARAM_FIRST_HAND] = AgentProceduralAnimator::AgentLimb::LIMB_RIGHT_HAND;

				if (!are_hands_together) {
					AgentProceduralAnimator::AgentLimb limb_to_move = sampling_source_limb;
					limb_to_move = limb_to_move == AgentProceduralAnimator::LIMB_LEFT_HAND ? AgentProceduralAnimator::LIMB_RIGHT_HAND : AgentProceduralAnimator::LIMB_LEFT_HAND;
					args[HBAgentWallTransitionState::PARAM_FIRST_HAND] = limb_to_move;
				}

				HBAgentWallTransitionState *new_state = Object::cast_to<HBAgentWallTransitionState>(state_machine->get_state(ASN()->wall_parkour_transition_state));

				AgentProceduralAnimator::AgentProceduralPose new_pose;
				new_pose.skeleton_trf = animator_options.target_skeleton_transform;
				for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
					new_pose.ik_targets[i] = animator_options.target_limb_transforms[i];
					new_pose.ik_magnet_positions[i] = pose.ik_magnet_positions[i];
				}
				new_state->set_starting_pose(new_pose);

				state_machine->transition_to(ASN()->wall_parkour_transition_state, args);
				return;
			}

			if (find_reachable_parkour_ledge(sampling_source_limb, snapped_dir, LONG_GRAB_REACH, &ledge, offset)) {
				StringName animation_node_name = ASN()->wallparkour_up_long_jump_animation_node;

				Dictionary args;
				args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MOVEMENT_WALLPARKOUR_UP_LONG_JUMP;
				args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->wallparkour_up_long_jump_animation_node;

				Dictionary state_args;
				state_args[HBAgentLedgeGrabbedStateNew::PARAM_LEDGE] = ledge;
				state_args["OFFSET_TEST"] = offset;

				args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = state_args;
				args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->ledge_grabbed_state;

				Dictionary wp_points;
				Transform3D ledge_trf = ledge->get_ledge_transform_at_offset(offset);
				ledge_trf.basis = Basis::looking_at(ledge_trf.basis.xform(Vector3(0.0f, 0.0f, 1.0f)));
				wp_points[ASN()->wall_parkour_cat_point_wp_name] = ledge_trf;

				args[HBAgentRootMotionState::PARAM_WARP_POINTS] = wp_points;

				state_machine->transition_to(ASN()->root_motion_state, args);
				return;
			}
		}

		HBAgentParkourPoint *point_to_reach = find_reachable_parkour_point(sampling_source_limb, snapped_dir, SHORT_GRAB_REACH);

		if (!point_to_reach) {
			const Vector3 sampling_source_fwd = sampling_source_trf.basis.xform(Vector3(0.0f, 0.0f, 1.0f));
			Vector3 rot_y = sampling_source_fwd.cross(snapped_dir).normalized();
			point_to_reach = find_reachable_parkour_point(sampling_source_limb, snapped_dir.rotated(rot_y, Math::deg_to_rad(-45.0f)), SHORT_GRAB_REACH);
		}

		if (point_to_reach) {
			debug_draw_sphere(point_to_reach->get_global_position(), 0.05f, Color("HOTPINK"));
			AgentProceduralAnimator::AgentLimb limb_to_move = sampling_source_limb;
			// When going vertically we allow chaining of movement without first having to have both hands settle
			// on the same point, but this is not allowed horizontally
			if (horizontal_component != 0) {
				if (!are_hands_together) {
					limb_to_move = limb_to_move == AgentProceduralAnimator::LIMB_LEFT_HAND ? AgentProceduralAnimator::LIMB_RIGHT_HAND : AgentProceduralAnimator::LIMB_LEFT_HAND;
				}
			}
			// When we have just finished the first part of the vertical movement animation we can immediately initiate a continuation of the climb
			// without bringing the hands together first
			if (!are_hands_together && vertical_component != 0 && animator_just_finished) {
				const float left_hand_height = target_pose.parkour_points[AgentProceduralAnimator::LIMB_LEFT_HAND]->get_global_position().y;
				const float right_hand_height = target_pose.parkour_points[AgentProceduralAnimator::LIMB_RIGHT_HAND]->get_global_position().y;
				if (vertical_component > 0) {
					limb_to_move = left_hand_height < right_hand_height ? AgentProceduralAnimator::LIMB_LEFT_HAND : AgentProceduralAnimator::LIMB_RIGHT_HAND;
				} else {
					limb_to_move = left_hand_height > right_hand_height ? AgentProceduralAnimator::LIMB_LEFT_HAND : AgentProceduralAnimator::LIMB_RIGHT_HAND;
				}
			}

			WallParkourInitialPose new_pose;
			find_initial_pose(new_pose, point_to_reach);

			const AgentProceduralAnimator::AgentLimb foot_to_move = limb_to_move == AgentProceduralAnimator::LIMB_LEFT_HAND ? AgentProceduralAnimator::LIMB_LEFT_FOOT : AgentProceduralAnimator::LIMB_RIGHT_FOOT;

			target_pose.pose.ik_targets[limb_to_move] = new_pose.pose.ik_targets[limb_to_move];
			target_pose.parkour_points[limb_to_move] = new_pose.parkour_points[limb_to_move];
			target_pose.pose.ik_targets[foot_to_move] = new_pose.pose.ik_targets[foot_to_move];
			target_pose.parkour_points[foot_to_move] = new_pose.parkour_points[foot_to_move];

			target_pose.pose.skeleton_trf = new_pose.pose.skeleton_trf;

			update_animator(limb_to_move);
			if (vertical_component != 0) {
				back_straightness_target = 1.0f;
			} else {
				back_straightness_target = 0.0f;
			}

			if (limb_to_move == AgentProceduralAnimator::LIMB_LEFT_HAND) {
				straight_side_target = 0.0f;
			} else {
				straight_side_target = 1.0f;
			}
			animator.reset();
			current_parkour_point_target = point_to_reach;
			current_limb = limb_to_move;
		} else {
			// Try if we can find a point to long jump to
			HBAgentParkourPoint *long_grab_point = find_reachable_parkour_point(sampling_source_limb, snapped_dir, LONG_GRAB_REACH);

			StringName animation_node_name = ASN()->wallparkour_up_long_jump_animation_node;
			int transition_node_index = HBAgentConstants::MOVEMENT_WALLPARKOUR_UP_LONG_JUMP;

			if (vertical_component < 0) {
				animation_node_name = ASN()->wallparkour_down_long_jump_animation_node;
				transition_node_index = HBAgentConstants::MOVEMENT_WALLPARKOUR_DOWN_LONG_JUMP;
			}

			if (horizontal_component > 0) {
				animation_node_name = ASN()->wallparkour_right_long_jump_animation_node;
				transition_node_index = HBAgentConstants::MOVEMENT_WALLPARKOUR_RIGHT_LONG_JUMP;
			} else if (horizontal_component < 0) {
				animation_node_name = ASN()->wallparkour_left_long_jump_animation_node;
				transition_node_index = HBAgentConstants::MOVEMENT_WALLPARKOUR_LEFT_LONG_JUMP;
			}

			if (long_grab_point) {
				Dictionary root_motion_args;

				Dictionary warp_points;
				warp_points[ASN()->wall_parkour_cat_point_wp_name] = long_grab_point->get_global_transform();
				root_motion_args[HBAgentRootMotionState::RootMotionParams::PARAM_WARP_POINTS] = warp_points;
				root_motion_args[HBAgentRootMotionState::RootMotionParams::PARAM_NEXT_STATE] = ASN()->wall_parkour_state;
				root_motion_args[HBAgentRootMotionState::RootMotionParams::PARAM_ANIMATION_NODE_NAME] = animation_node_name;
				root_motion_args[HBAgentRootMotionState::RootMotionParams::PARAM_TRANSITION_NODE_INDEX] = transition_node_index;

				Dictionary next_state_args;
				next_state_args[HBAgentWallParkourStateNew::PARAM_PARKOUR_NODE] = long_grab_point;
				root_motion_args[HBAgentRootMotionState::RootMotionParams::PARAM_NEXT_STATE_ARGS] = next_state_args;

				state_machine->transition_to(ASN()->root_motion_state, root_motion_args);
				return;
			}
		}

		// We cannot reach higher, so bring the hands together
		if (!are_hands_together && vertical_component != 0 && animator_just_finished && !point_to_reach) {
			bring_hands_together();
		}
	}
}
bool HBAgentWallParkourStateNew::_handle_parkour_mid() {
	if (!get_agent()->is_action_pressed(HBAgent::AgentInputAction::INPUT_ACTION_RUN)) {
		return false;
	}

	const Vector3 input = get_agent()->get_graphics_rotation().xform(Vector3(get_agent()->get_movement_input().x, 0.0f, 0.0f).normalized());
	if (input.length_squared() == 0.0f) {
		return false;
	}
	const float EJECT_REACH = 2.0f;
	PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::ShapeParameters shape_params;
	static constexpr const int ITERS = 10;

	Ref<Shape3D> shape = get_agent()->get_collision_shape();
	shape_params.shape_rid = shape->get_rid();
	shape_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	shape_params.transform.origin = get_agent()->get_global_position();
	shape_params.transform.origin.y += get_agent()->get_height() * 1.5f;
	shape_params.transform.origin += get_agent()->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, 0.01f));
	for (int i = 0; i < ITERS; i++) {
		shape_params.motion = input * EJECT_REACH * ((i + 1) / ((float)ITERS));
		real_t closest_safe, closest_unsafe;
		debug_draw_cast_motion(shape, shape_params, Color("RED"));
		dss->cast_motion(shape_params, closest_safe, closest_unsafe);
		bool first_hit = closest_safe != 1.0f && closest_unsafe != 1.0f;

		PhysicsDirectSpaceState3D::ShapeParameters shape_params_2 = shape_params;
		shape_params_2.transform.origin = shape_params.transform.origin + shape_params.motion * closest_safe;
		shape_params_2.motion = Vector3(0.0f, -get_agent()->get_height() * 2.0f, 0.0f);

		debug_draw_cast_motion(shape, shape_params_2, Color("BLUE"));
		dss->cast_motion(shape_params_2, closest_safe, closest_unsafe);

		bool hit = closest_safe != 1.0f && closest_unsafe != 1.0f;

		if (hit) {
			Dictionary root_motion_args;

			Dictionary warp_points;
			Transform3D wp_trf;
			wp_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), input);
			wp_trf.origin = shape_params_2.transform.origin + shape_params_2.motion * closest_unsafe;
			wp_trf.origin.y -= get_agent()->get_height() * 0.5f;

			//_transition_to_short_hop(wp_trf, ASN()->move_state, Dictionary());

			warp_points[StringName("Ledge")] = wp_trf;
			root_motion_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MovementTransitionInputs::MOVEMENT_SHORT_HOP;
			root_motion_args[HBAgentRootMotionState::PARAM_VELOCITY_MODE] = HBAgentRootMotionState::VelocityMode::CONSERVE;
			root_motion_args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;
			root_motion_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->short_hop_animation_node;
			root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->move_state;

			PhysicsDirectSpaceState3D::RayParameters ray_params;
			ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
			ray_params.from = wp_trf.origin - input * get_agent()->get_radius();
			ray_params.from.y += get_agent()->get_radius();
			ray_params.to = ray_params.from;
			ray_params.to.y -= get_agent()->get_radius() * 2.0f;
			debug_draw_raycast(ray_params);

			PhysicsDirectSpaceState3D::RayResult result;
			if (!dss->intersect_ray(ray_params, result)) {
				continue;
			}

			state_machine->transition_to(ASN()->root_motion_state, root_motion_args);
			return true;
		}

		if (first_hit) {
			break;
		}
	}

	return false;
}
HBAgentParkourPoint *HBAgentWallParkourStateNew::find_reachable_parkour_point(const AgentProceduralAnimator::AgentLimb p_sampling_limb, const Vector3 &p_direction, const float &p_reach) const {
	HBAgent *agent = get_agent();

	const Transform3D sampling_source_trf = target_pose.parkour_points[p_sampling_limb]->get_global_transform();

	Ref<CylinderShape3D> cyl_shape;
	cyl_shape.instantiate();

	cyl_shape->set_height(p_reach);
	cyl_shape->set_radius(agent->get_radius());

	PhysicsDirectSpaceState3D::ShapeParameters shape_params;
	shape_params.transform.origin = sampling_source_trf.origin + p_direction * p_reach * 0.5f;
	shape_params.transform.basis = Quaternion(Vector3(0.0f, 1.0f, 0.0f), p_direction);
	shape_params.shape_rid = cyl_shape->get_rid();
	shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;

	constexpr int static MAX_RESULTS = 10;
	Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
	results.resize(MAX_RESULTS);

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
	int result_count = dss->intersect_shape(shape_params, results.ptrw(), MAX_RESULTS);
	HBAgentParkourPoint *point_to_reach = nullptr;

	const_cast<HBAgentWallParkourStateNew *>(this)->debug_draw_cast_motion(cyl_shape, shape_params);

	// Now find the closest point
	for (int i = 0; i < result_count; i++) {
		HBAgentParkourPoint *pp = Object::cast_to<HBAgentParkourPoint>(results[i].collider);
		if (!pp) {
			continue;
		}

		// We ignore parkour points that are already being used by any of the hands
		for (int j = 0; j <= AgentProceduralAnimator::AgentLimb::LIMB_RIGHT_HAND; j++) {
			if (target_pose.parkour_points[j] == pp) {
				goto skip_iter;
			}
		}

		if (p_direction.angle_to(sampling_source_trf.origin.direction_to(pp->get_global_position())) > Math::deg_to_rad(45.0f)) {
			continue;
		}

		if (point_to_reach) {
			// Perhaps we should project the hit nodes onto the input direction to use as a distance...
			const float distance_to_current_point = sampling_source_trf.origin.distance_to(point_to_reach->get_global_position());
			const float distance_to_candidate_point = sampling_source_trf.origin.distance_to(pp->get_global_position());
			const float angle_to_current_point = sampling_source_trf.origin.direction_to(point_to_reach->get_global_position()).angle_to(p_direction);
			const float angle_to_candidate_point = sampling_source_trf.origin.direction_to(pp->get_global_position()).angle_to(p_direction);
			if (distance_to_candidate_point > distance_to_current_point || angle_to_candidate_point > angle_to_current_point) {
				continue;
			}
		}

		point_to_reach = pp;
	skip_iter:;
	}
	return point_to_reach;
}
bool HBAgentWallParkourStateNew::find_reachable_parkour_ledge(const AgentProceduralAnimator::AgentLimb p_sampling_limb, const Vector3 &p_direction, const float &p_reach, HBAgentParkourLedge **r_ledge, float &r_offset) {
	HBAgent *agent = get_agent();

	const Transform3D sampling_source_trf = target_pose.parkour_points[p_sampling_limb]->get_global_transform();

	Ref<CylinderShape3D> cyl_shape;
	cyl_shape.instantiate();

	cyl_shape->set_height(p_reach);
	cyl_shape->set_radius(agent->get_radius());

	PhysicsDirectSpaceState3D::ShapeParameters shape_params;
	shape_params.transform.origin = sampling_source_trf.origin + p_direction * p_reach * 0.5f;
	shape_params.transform.basis = Quaternion(Vector3(0.0f, 1.0f, 0.0f), p_direction);
	shape_params.shape_rid = cyl_shape->get_rid();
	shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
	shape_params.collide_with_bodies = false;
	shape_params.collide_with_areas = true;

	constexpr int static MAX_RESULTS = 10;
	Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
	results.resize(MAX_RESULTS);

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
	int result_count = dss->intersect_shape(shape_params, results.ptrw(), MAX_RESULTS);

	const_cast<HBAgentWallParkourStateNew *>(this)->debug_draw_cast_motion(cyl_shape, shape_params);

	// Now find the closest point
	for (int i = 0; i < result_count; i++) {
		HBAgentParkourLedge *pl = Object::cast_to<HBAgentParkourLedge>(results[i].collider);

		if (!pl) {
			continue;
		}

		*r_ledge = pl;
		r_offset = pl->get_closest_offset(get_agent()->get_global_position());

		return true;
	}

	return false;
}
HBAgentWallParkourStateNew::HBAgentWallParkourStateNew() {
}

void HBAgentWallTransitionState::set_starting_pose(const AgentProceduralAnimator::AgentProceduralPose &p_starting_pose) {
	starting_pose = p_starting_pose;
}

void HBAgentWallTransitionState::apply_animator_pose(bool p_inertialize_transform) {
	AgentProceduralAnimator::AgentProceduralPose out_pose;
	animator.get_output_pose(out_pose);

	calculate_magnet_for_limbs(out_pose);

	Ref<EPASIKNode> ik_nodes[AgentProceduralAnimator::LIMB_MAX] = {
		get_epas_controller()->get_epas_node(ASN()->left_hand_ik_node),
		get_epas_controller()->get_epas_node(ASN()->right_hand_ik_node),
		get_epas_controller()->get_epas_node(ASN()->left_foot_ik_node),
		get_epas_controller()->get_epas_node(ASN()->right_foot_ik_node),
	};
	for (int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		ik_nodes[i]->set_ik_influence(1.0f);
		ik_nodes[i]->set_magnet_position(out_pose.ik_magnet_positions[i]);
		ik_nodes[i]->set_target_transform(out_pose.ik_targets[i]);
	}

	if (!p_inertialize_transform) {
		get_agent()->set_global_position(out_pose.skeleton_trf.origin);
		get_agent()->set_graphics_rotation(out_pose.skeleton_trf.basis);
	} else {
		Transform3D trf = out_pose.skeleton_trf;
		get_agent()->inertialize_graphics_transform(trf, 0.1f);
	}
}

void HBAgentWallTransitionState::calculate_magnet_for_limbs(AgentProceduralAnimator::AgentProceduralPose &p_pose) {
	for (int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		if (i <= AgentProceduralAnimator::AgentLimb::LIMB_RIGHT_HAND) {
			// Hands
			p_pose.ik_magnet_positions[i] = p_pose.ik_targets[i].origin;
			p_pose.ik_magnet_positions[i].y -= 3.0f;
		} else {
			// Feet
			p_pose.ik_magnet_positions[i] = p_pose.ik_targets[i].origin + Vector3(0.0f, 3.0f, 0.0f);
			p_pose.ik_magnet_positions[i] += get_agent()->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, -3.0f));
		}
	}
}

void HBAgentWallTransitionState::enter(const Dictionary &p_args) {
	DEV_ASSERT(p_args.has(PARAM_TRANSITION_MODE));
	transition_mode = (TransitionMode)(int)p_args.get(PARAM_TRANSITION_MODE, Variant());

	AgentProceduralAnimator::AgentProceduralPose end_pose;

	if (transition_mode == TO_LEDGE) {
		ledge = Object::cast_to<HBAgentParkourLedge>(p_args.get(PARAM_LEDGE, Variant()));
		DEV_ASSERT(ledge != nullptr);
		DEV_ASSERT(p_args.has(PARAM_LEDGE_OFFSET));
		ledge_offset = p_args.get(PARAM_LEDGE_OFFSET, 0.0f);
		DEV_ASSERT(p_args.has(PARAM_FIRST_HAND));
		first_hand = (AgentProceduralAnimator::AgentLimb)(int)p_args.get((int)PARAM_FIRST_HAND, Variant());
		DEV_ASSERT(first_hand <= AgentProceduralAnimator::LIMB_RIGHT_HAND);
		HBAgentLedgeGrabbedStateNew *ledge_state = Object::cast_to<HBAgentLedgeGrabbedStateNew>(state_machine->get_state(ASN()->ledge_grabbed_state));
		ledge_state->get_initial_pose_for_edge(ledge, end_pose, ledge_offset);
	} else if (transition_mode == TO_WALL) {
		parkour_point = Object::cast_to<HBAgentParkourPoint>(p_args.get(HBAgentWallTransitionState::PARAM_PARKOUR_POINT, Variant()));
		DEV_ASSERT(parkour_point);
		HBAgentWallParkourStateNew *wall_parkour_state = Object::cast_to<HBAgentWallParkourStateNew>(state_machine->get_state(ASN()->wall_parkour_state));
		HBAgentWallParkourStateNew::WallParkourInitialPose initial_pose;
		wall_parkour_state->find_initial_pose(initial_pose, parkour_point);
		end_pose = initial_pose.pose;
		first_hand = AgentProceduralAnimator::LIMB_RIGHT_HAND;
	}

	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		animator_options.starting_limb_transforms[i] = starting_pose.ik_targets[i];
		animator_options.target_limb_transforms[i] = end_pose.ik_targets[i];
		relative_magnet_pos[i] = end_pose.ik_magnet_positions[i] - get_skeleton()->get_global_position();
	}
	animator_options.starting_skeleton_transform = starting_pose.skeleton_trf;
	animator_options.target_skeleton_transform = end_pose.skeleton_trf;

	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_LEFT_HAND][0] = 0.5f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_LEFT_HAND][1] = 0.9f;

	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_RIGHT_HAND][0] = 0.0f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_RIGHT_HAND][1] = 0.4f;

	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_LEFT_FOOT][0] = 0.6f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_LEFT_FOOT][1] = 1.0f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_RIGHT_FOOT][0] = 0.1f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_RIGHT_FOOT][1] = 0.5f;

	if (first_hand == AgentProceduralAnimator::LIMB_LEFT_HAND) {
		for (int i = 0; i < 2; i++) {
			AgentProceduralAnimator::AgentLimb hand = static_cast<AgentProceduralAnimator::AgentLimb>(i);
			AgentProceduralAnimator::AgentLimb foot = static_cast<AgentProceduralAnimator::AgentLimb>(AgentProceduralAnimator::AgentLimb::LIMB_LEFT_FOOT);
			for (int j = 0; j < 2; j++) {
				SWAP(animator_options.limb_animation_timings[hand][j], animator_options.limb_animation_timings[hand + 1][j]);
				SWAP(animator_options.limb_animation_timings[foot][j], animator_options.limb_animation_timings[foot + 1][j]);
			}
		}
	}

	for (int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		animator_options.limb_position_spring_halflifes[i] = 0.05f;
	}
	animator_options.skeleton_position_spring_halflife = 0.0f;

	animator.set_animation_duration(0.8f);
	animator.restart();

	//get_movement_transition_node()->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_WALLGRABBED);
	get_inertialization_node()->inertialize(0.25f);

	animator.process(animator_options, 0.0f);
	apply_animator_pose(true);
}

void HBAgentWallTransitionState::physics_process(float p_delta) {
	animator.process(animator_options, p_delta);
	apply_animator_pose();
	if (animator.is_done() && animator.have_springs_converged()) {
		if (transition_mode == TO_LEDGE) {
			Dictionary args;
			args[HBAgentLedgeGrabbedStateNew::PARAM_LEDGE] = ledge;
			state_machine->transition_to(ASN()->ledge_grabbed_state, args);
		} else {
			Dictionary args;
			args[HBAgentWallParkourStateNew::PARAM_PARKOUR_NODE] = parkour_point;
			state_machine->transition_to(ASN()->wall_parkour_state, args);
		}
	}
}

void HBAgentCombatMoveState::calculate_desired_movement_velocity(float p_delta) {
	HBAgent *agent = get_agent();
	Vector3 input_vector = agent->get_desired_movement_input_transformed();
	Vector3 target_desired_velocity = input_vector * agent->get_agent_constants()->get_max_move_velocity();
	Vector3 current_desired_velocity = agent->get_desired_velocity();
	HBSprings::velocity_spring_vector3(
			current_desired_velocity,
			desired_velocity_spring_acceleration,
			target_desired_velocity,
			agent->get_agent_constants()->get_velocity_spring_halflife(),
			p_delta);
	desired_velocity_ws = current_desired_velocity;
}

void HBAgentCombatMoveState::rotate_towards_target(float p_delta) {
	HBAgent *agent = get_agent();
	Vector3 aim_direction = agent->get_global_position().direction_to(target->get_global_position());
	aim_direction.y = 0.0f;
	aim_direction.normalize();
	if (aim_direction.is_normalized()) {
		agent_rotation_target = Basis::looking_at(aim_direction).get_rotation_quaternion();
	}
	Quaternion current_rot = get_agent()->get_graphics_rotation();
	HBSprings::simple_spring_damper_exact_quat(
			current_rot,
			agent_rotation_velocity,
			agent_rotation_target,
			agent_rotation_halflife,
			p_delta);
	get_agent()->set_graphics_rotation(current_rot);
}

void HBAgentCombatMoveState::update_orientation_warp() {
	HBAgent *agent = get_agent();
	Ref<EPASOrientationWarpNode> orientation_warp = get_epas_controller()->get_epas_node("Orientation");
	DEV_ASSERT(orientation_warp.is_valid());

	float orientation_warp_angle = 0.0f;
	if (desired_velocity_ws.length() > 0.1f) {
		Vector3 forward_direction = agent->get_graphics_rotation().xform(Vector3(0.0f, 0.0f, -1.0f));
		forward_direction.y = 0.0f;
		forward_direction.normalize();
		if (forward_direction.is_normalized()) {
			orientation_warp_angle = Math::rad_to_deg(forward_direction.signed_angle_to(desired_velocity_ws.normalized(), Vector3(0.0f, 1.0f, 0.0f)));
			float mul = MIN(1.0, agent->get_linear_velocity().length() / (agent->get_agent_constants()->get_max_move_velocity() * 0.5f));
			orientation_warp_angle = CLAMP(orientation_warp_angle, -75.0, 75.0) * mul;
		}
	}

	orientation_warp->set_orientation_angle(orientation_warp_angle);
}

bool HBAgentCombatMoveState::handle_attack() {
	HBAgent *agent = get_agent();
	if (agent->is_action_just_pressed(HBAgent::INPUT_ACTION_ATTACK)) {
		Dictionary root_motion_args;

		root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->combat_move_state;
		root_motion_args[HBAgentCombatAttackState::PARAM_ATTACK_NAME] = StringName("Attack1");
		root_motion_args[HBAgentCombatAttackState::PARAM_TARGET] = target;

		Dictionary next_state_args;
		root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = next_state_args;
		state_machine->transition_to(ASN()->combat_attack_state, root_motion_args);
		return true;
	}

	return false;
}

void HBAgentCombatMoveState::reset() {
	desired_velocity_spring_acceleration = Vector3();
	desired_velocity_ws = Vector3();
}

void HBAgentCombatMoveState::enter(const Dictionary &p_args) {
	HBAgent *agent = get_agent();
	agent->emit_signal("entered_combat");
	target = agent->get_target();

	setup_attack_reception();

	get_wheel_locomotion_node()->set_x_blend(0.0f);

	Transform3D graphics_trf = Transform3D(agent->get_graphics_rotation(), agent->get_global_position());

	agent->inertialize_graphics_transform(graphics_trf, 0.2f);

	if (!target) {
		// Target is somehow gone, return to move state
		// TODO: Add functionality to find another target if possible
		state_machine->transition_to(ASN()->move_state);
		return;
	}

	reset();

	agent->set_movement_mode(HBAgent::MOVE_GROUNDED);
	get_movement_transition_node()->transition_to(HBAgentConstants::MOVEMENT_MOVE);
	get_inertialization_node()->inertialize(0.5f);
	emit_signal("entered_combat");
}

void HBAgentCombatMoveState::exit() {
	remove_attack_reception();
}

void HBAgentCombatMoveState::physics_process(float p_delta) {
	if (handle_parrying()) {
		return;
	}

	calculate_desired_movement_velocity(p_delta);
	HBAgent *agent = get_agent();
	Vector3 dodge_dir = desired_velocity_ws;
	dodge_dir.y = 0.0f;
	dodge_dir.normalize();
	if (dodge_dir.length_squared() > 0 && agent->is_action_just_pressed(HBAgent::INPUT_ACTION_PARKOUR_UP)) {
		Dictionary state_args;
		Dictionary next_state_args;
		state_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->combat_move_state;
		state_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = ASN()->roll_animation_node;
		state_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MOVEMENT_COMBAT_ROLL;
		state_args[HBAgentRootMotionState::PARAM_COLLIDE] = true;
		state_args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = next_state_args;
		state_args[HBAgentRootMotionState::PARAM_INVULNERABLE] = true;

		Dictionary warp_points;
		Transform3D wp_trf;
		wp_trf.origin = agent->get_global_position();
		wp_trf.basis = Basis::looking_at(-dodge_dir);
		warp_points[StringName("start")] = wp_trf;
		wp_trf.origin += dodge_dir * 1.5f;
		warp_points[StringName("end")] = wp_trf;

		state_args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;

		state_machine->transition_to(ASN()->root_motion_state, state_args);
		return;
	}
	agent->set_desired_velocity(desired_velocity_ws);
	agent->handle_input(desired_velocity_ws.normalized(), 0.0f);
	agent->update(p_delta);

	rotate_towards_target(p_delta);

	Vector3 linear_vel_horizontal = agent->get_linear_velocity();
	linear_vel_horizontal.y = 0.0f;
	get_wheel_locomotion_node()->set_linear_velocity(linear_vel_horizontal);
	get_wheel_locomotion_node()->set_x_blend(CLAMP(linear_vel_horizontal.length() / agent->get_agent_constants()->get_max_move_velocity(), 0.0, 1.0));

	update_orientation_warp();

	if (handle_attack()) {
		return;
	}
}

bool HBAgentCombatAttackState::handle_attack() {
	if (attack->get_cancel_time() == -1.0f) {
		return false;
	}

	if (String(attack->get_next_attack()).is_empty() || animation_node->get_playback_position() < attack->get_cancel_time() || !was_attack_repressed) {
		return false;
	}

	HBAgent *agent = get_agent();

	if (agent->is_action_pressed(HBAgent::INPUT_ACTION_ATTACK)) {
		Dictionary root_motion_args;

		root_motion_args[HBAgentCombatAttackState::PARAM_ATTACK_NAME] = attack->get_next_attack();
		root_motion_args[HBAgentCombatAttackState::PARAM_TARGET] = target;
		root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->combat_move_state;

		Dictionary next_attack_state_args;

		root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = next_attack_state_args;
		state_machine->transition_to(ASN()->combat_attack_state, root_motion_args);
		return true;
	}
	return false;
}

void HBAgentCombatAttackState::handle_hitstop(float p_delta) {
	if (!attack_connected) {
		return;
	}
	if (prev_playback_position < attack->get_hit_time() && animation_node->get_playback_position() > attack->get_hit_time()) {
		hit_stop_solver.instantiate();
		Vector3 dir = get_graphics_node()->get_global_basis().xform(Vector3(0.0f, 0.0f, -1.0f));
		hit_stop_solver->start(dir, attack->get_hitstop_duration() * 0.5f);
		animation_node->set_speed_scale(0.05f);
	}

	if (hit_stop_solver.is_valid()) {
		if (hit_stop_solver->is_done()) {
			animation_node->set_speed_scale(1.0f);
			hit_stop_solver.unref();
			return;
		}
		hit_stop_solver->advance(1.0f, p_delta);
		Vector3 offset = hit_stop_solver->get_offset();
		Node3D *gn = get_graphics_node();
		gn->set_position(offset);
	}
}

void HBAgentCombatAttackState::handle_sending_attack_to_target() {
	if (target->get_is_invulnerable()) {
		return;
	}
	if (prev_playback_position < attack->get_hit_time() && animation_node->get_playback_position() > attack->get_hit_time()) {
		target->receive_attack(get_agent(), attack);
		attack_connected = true;
		get_agent()->emit_signal("attack_connected");
		if (target->is_dead()) {
			_on_target_died();
		}
	}
}

void HBAgentCombatAttackState::_on_attack_parried() {
	Dictionary state_args;
	state_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->combat_move_state;
	state_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = StringName("HitRight");
	state_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MOVEMENT_COMBAT_HIT_RIGHT;
	state_machine->transition_to(ASN()->root_motion_state, state_args);

	get_agent()->emit_signal("emit_sword_sparks");
}

void HBAgentCombatAttackState::_on_target_died() {
	// TODO: Handle execution animations
}

void HBAgentCombatAttackState::enter(const Dictionary &p_args) {
	HBAgent *agent = get_agent();
	Ref<EPASTransitionNode> transition = get_epas_controller()->get_epas_node("AttackTransition");
	DEV_ASSERT(transition.is_valid());

	target = Object::cast_to<HBAgent>(p_args.get(PARAM_TARGET, Variant()));
	DEV_ASSERT(target);

	DEV_ASSERT(p_args.has(PARAM_ATTACK_NAME));
	StringName attack_name = p_args.get(HBAgentCombatAttackState::PARAM_ATTACK_NAME, "");
	attack = get_agent()->get_attack_data(attack_name);

	DEV_ASSERT(attack.is_valid());

	int transition_idx = attack->get_transition_index();

	transition->transition_to(transition_idx);

	Dictionary root_motion_args = p_args;
	root_motion_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = attack->get_name();
	root_motion_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MOVEMENT_COMBAT_ATTACK;

	if (!root_motion_args.has(HBAgentRootMotionState::PARAM_WARP_POINTS)) {
		Dictionary warp_points;
		Vector3 dir_from_target = target->get_global_position().direction_to(agent->get_global_position());
		dir_from_target.y = 0.0f;
		dir_from_target.normalize();
		warp_points[StringName("agent")] = Transform3D(Basis::looking_at(dir_from_target), target->get_global_position());
		root_motion_args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;
	}

	HBAgentRootMotionState::enter(root_motion_args);
	get_inertialization_node()->inertialize(0.10f);

	attack_mesh_instance = agent->get_attack_trail_node();
	attack_mesh_instance->show();
	attack_mesh_instance->set_mesh(attack->get_mesh());
	animation_node = get_epas_controller()->get_epas_node(attack->get_name());
	attack_mesh_instance->set_instance_shader_parameter("scroll", 0.0f);

	prev_playback_position = 0.0f;
	was_attack_repressed = false;
	attack_connected = true;
	setup_attack_reception();

	target->connect("parried", callable_mp(this, &HBAgentCombatAttackState::_on_attack_parried));
}

void HBAgentCombatAttackState::exit() {
	attack_mesh_instance->hide();
	hit_stop_solver.unref();
	animation_node->set_speed_scale(1.0f);

	remove_attack_reception();

	target->disconnect("parried", callable_mp(this, &HBAgentCombatAttackState::_on_attack_parried));

	Node3D *gn = get_graphics_node();
	gn->set_position(Vector3());

	if (!attack_connected) {
		get_agent()->emit_signal("attack_aborted");
	}
}

void HBAgentCombatAttackState::physics_process(float p_delta) {
	HBAgentRootMotionState::physics_process(p_delta);

	if (get_agent()->is_action_just_released(HBAgent::AgentInputAction::INPUT_ACTION_ATTACK)) {
		was_attack_repressed = true;
	}

	if (!target->is_dead() && handle_attack()) {
		return;
	}

	handle_sending_attack_to_target();
	handle_hitstop(p_delta);

	const float animation_length = animation_node->get_animation()->get_length();
	attack_mesh_instance->set_instance_shader_parameter("scroll", animation_node->get_playback_position() / animation_length);
	prev_playback_position = animation_node->get_playback_position();
}

void HBAgentCombatAttackState::animation_finished(float p_delta) {
	if (target->is_dead()) {
		exit_combat();
	} else {
		HBAgentRootMotionState::animation_finished(p_delta);
	}
}

void HBAgentCombatHitState::look_towards_attacker() {
	HBAgent *agent = get_agent();
	Vector3 attacker_dir = agent->get_global_position().direction_to(attacker->get_global_position());
	attacker_dir.y = 0.0f;
	attacker_dir.normalize();
	if (attacker_dir.is_normalized()) {
		Basis new_rot = Basis::looking_at(attacker_dir);
		agent->set_graphics_rotation(new_rot);
		get_graphics_node()->set_global_basis(new_rot);
	}
}

void HBAgentCombatHitState::setup_animation() {
	HBAgentConstants::MovementTransitionInputs transition = HBAgentConstants::MOVEMENT_COMBAT_HIT_RIGHT;
	StringName animation_node_name;

	Dictionary root_motion_args;

	switch (attack->get_attack_direction()) {
		case HBAttackData::RIGHT: {
			animation_node_name = StringName("HitRight");
			transition = HBAgentConstants::MOVEMENT_COMBAT_HIT_RIGHT;
		} break;
		case HBAttackData::ATTACK_DIR_MAX: {
		} break;
	}

	root_motion_args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = animation_node_name;
	root_motion_args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = transition;
	root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->combat_move_state;

	Dictionary combat_move_args;

	root_motion_args[HBAgentRootMotionState::PARAM_NEXT_STATE_ARGS] = combat_move_args;

	root_motion_args[HBAgentRootMotionState::PARAM_COLLIDE] = true;

	get_inertialization_node()->inertialize(0.05f);

	HBAgentRootMotionState::enter(root_motion_args);
}

void HBAgentCombatHitState::enter(const Dictionary &p_args) {
	DEV_ASSERT(p_args.has(PARAM_ATTACK));
	attack = p_args.get(PARAM_ATTACK, Variant());
	DEV_ASSERT(attack.is_valid());

	DEV_ASSERT(p_args.has(PARAM_ATTACKER));
	attacker = Object::cast_to<HBAgent>(p_args[PARAM_ATTACKER]);
	DEV_ASSERT(attacker);

	hit_stop_solver.instantiate();
	Vector3 forward = get_graphics_node()->get_global_basis().xform(Vector3(0.0f, 0.0f, -1.0f));
	forward.y = 0.0f;
	forward.normalize();
	hit_stop_solver->start(forward, attack->get_hitstop_duration());

	setup_attack_reception();
	look_towards_attacker();
	setup_animation();
}

void HBAgentCombatHitState::exit() {
	remove_attack_reception();
	Node3D *gn = get_graphics_node();
	gn->set_position(Vector3());
}

void HBAgentCombatHitState::physics_process(float p_delta) {
	HBAgentRootMotionState::physics_process(p_delta);

	if (handle_parrying()) {
		return;
	}

	if (!hit_stop_solver->is_done()) {
		hit_stop_solver->advance(1.0f, p_delta);

		Node3D *gn = get_graphics_node();
		gn->set_position(hit_stop_solver->get_offset());
	}
}

void HBAgentDeadState::enter(const Dictionary &p_args) {
	get_epas_controller()->set_playback_process_mode(EPASController::MANUAL);
	const Vector3 death_force = p_args.get(PARAM_DEATH_FORCE, Vector3());
	// TODO: Make this configurable
	Skeleton3D *skel = get_skeleton();
	skel->physical_bones_start_simulation_on(TypedArray<StringName>());
	for (int i = 0; i < skel->get_child_count(); i++) {
		PhysicalBone3D *pb = Object::cast_to<PhysicalBone3D>(skel->get_child(i));
		if (!pb) {
			continue;
		}
		if (pb->get_bone_name() == "head") {
			pb->apply_central_impulse(death_force);
			break;
		}
	}
}
