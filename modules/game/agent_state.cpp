#include "agent_state.h"
#include "agent_parkour.h"
#include "animation_system/epas_animation_node.h"
#include "modules/game/animation_system/epas_lookat_node.h"

#ifdef DEBUG_ENABLED
#include "implot.h"
#include "modules/imgui/godot_imgui.h"
#endif
#include "modules/tracy/tracy.gen.h"
#include "physics_layers.h"
#include "scene/resources/cylinder_shape_3d.h"
#include "springs.h"

#include "agent_string_names.h"

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

bool HBAgentState::find_facing_wall(PhysicsDirectSpaceState3D::RayResult &p_result) {
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

	debug_draw_raycast(ray_params, Color(1.0f, 0.0f, 0.0f));

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

bool HBAgentState::find_lege_wall_sweep(const Vector3 &p_from, const Vector3 &p_to, const Vector3 &p_offset, Transform3D &p_out_edge_trf, HBAgentParkourLedge **p_out_ledge, int p_iterations) {
	*p_out_ledge = nullptr;
	PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	ray_params.hit_back_faces = false;
	PhysicsDirectSpaceState3D::RayResult ray_result;
	bool has_result = false;
	const float floor_max_angle = get_agent()->get_floor_max_angle();
	for (int i = 0; i < p_iterations; i++) {
		float t = i / (float)(p_iterations - 1);
		ray_params.from = p_from + p_offset * t;
		ray_params.to = p_to + p_offset * t;
		PhysicsDirectSpaceState3D::RayResult ray_result_tmp;
		debug_draw_raycast(ray_params);
		if (!dss->intersect_ray(ray_params, ray_result_tmp) || !is_wall(ray_result_tmp.normal, floor_max_angle)) {
			continue;
		}

		has_result = true;
		ray_result = ray_result_tmp;
		break;
	}
	ray_params.hit_back_faces = true;

	if (!has_result) {
		return false;
	}

	ray_params.from = ray_result.position + ray_result.normal * -0.01;
	ray_params.to = ray_params.from;
	ray_params.from.y += get_agent()->get_height();

	Transform3D edge_trf;
	edge_trf.origin = ray_result.position;
	edge_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), ray_result.normal);

	debug_draw_raycast(ray_params, Color("RED"));
	if (!dss->intersect_ray(ray_params, ray_result) || !is_floor(ray_result.normal, floor_max_angle)) {
		return false;
	}

	edge_trf.origin.y = ray_result.position.y;

	p_out_edge_trf = edge_trf;

	PhysicsDirectSpaceState3D::ShapeParameters params;
	Ref<SphereShape3D> sphere;
	sphere.instantiate();
	sphere->set_radius(0.1f);
	params.shape_rid = sphere->get_rid();
	params.collide_with_bodies = false;
	params.collide_with_areas = true;
	params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
	params.transform.origin = p_out_edge_trf.origin;
	const int MAX_RESULTS = 10;
	Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
	results.resize(MAX_RESULTS);

	HBAgentParkourLedge *ledge_r = nullptr;
	int result_count = dss->intersect_shape(params, results.ptrw(), MAX_RESULTS);
	for (int i = 0; i < result_count; i++) {
		HBAgentParkourLedge *ledge = Object::cast_to<HBAgentParkourLedge>(results[i].collider);
		if (!ledge) {
			continue;
		}
		Vector3 test_pos = ledge->to_local(p_out_edge_trf.origin);
		Transform3D ref_trf = ledge->get_curve()->sample_baked_with_rotation(ledge->get_curve()->get_closest_offset(test_pos));
		Vector3 ledge_normal = ref_trf.basis.get_rotation_quaternion().xform(Vector3(-1.0f, 0.0f, 0.0f));

		if (ledge_normal.angle_to(p_out_edge_trf.basis.get_rotation_quaternion().xform(Vector3(0.0f, 0.0f, -1.0f))) > Math::deg_to_rad(15.0f)) {
			continue;
		}

		ledge_r = ledge;
		break;
	}

	if (!ledge_r) {
		return false;
	}

	*p_out_ledge = ledge_r;

	return true;
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
	return get_agent()->_get_epas_controller();
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
	ERR_FAIL_COND_V(get_epas_controller() == nullptr, Ref<EPASInertializationNode>());
	return get_epas_controller()->get_epas_node(SNAME("MovementWheel"));
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
	get_wheel_locomotion_node()->set_x_blend(x_blend);
	get_wheel_locomotion_node()->reset_foot_ik();
	get_inertialization_node()->inertialize(p_args.get(MoveStateParams::PARAM_TRANSITION_DURATION, 0.2f));
	get_agent()->connect("stopped_at_edge", callable_mp(this, &HBAgentMoveState::_on_agent_edge_hit));
	Ref<EPASLookatNode> torso_lookat_node = get_epas_controller()->get_epas_node("TorsoLookAt");
	Ref<EPASLookatNode> head_lookat_node = get_epas_controller()->get_epas_node("HeadLookAt");
	torso_lookat_node->set_influence(0.0f);
	head_lookat_node->set_influence(0.0f);
	get_softness_node()->set_influence(1.0f);
	get_wheel_locomotion_node()->set_use_foot_ik(true);
}

void HBAgentMoveState::exit() {
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
	HBAgent *agent = get_agent();
	Ref<EPASIKNode> left_foot_ik_node = get_epas_controller()->get_epas_node(ASN()->left_foot_ik_node);
	Ref<EPASIKNode> right_foot_ik_node = get_epas_controller()->get_epas_node(ASN()->right_foot_ik_node);

	left_foot_ik_node->set_ik_influence(1.0f);
	right_foot_ik_node->set_ik_influence(1.0f);

	debug_draw_clear();
	Vector3 parkour_dir = agent->get_desired_movement_input_transformed().normalized();

	if (parkour_dir.is_normalized()) {
		if (_handle_parkour()) {
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

	Ref<EPASLookatNode> torso_lookat_node = get_epas_controller()->get_epas_node("TorsoLookAt");
	Ref<EPASLookatNode> head_lookat_node = get_epas_controller()->get_epas_node("HeadLookAt");

	if (movement_input.length() > 0.0) {
		torso_lookat_node->set_target_world(get_agent()->get_global_position() + movement_input.normalized());
		head_lookat_node->set_target_world(get_agent()->get_global_position() + movement_input.normalized());
		if (torso_lookat_node->get_influence() == 0.0f) {
			torso_lookat_node->set_influence(1.0f);
			head_lookat_node->set_influence(1.0f);
			head_lookat_node->reset();
			torso_lookat_node->reset();
		}
	} else {
		if (torso_lookat_node->get_influence() == 1.0f) {
			torso_lookat_node->set_influence(0.0f);
			head_lookat_node->set_influence(0.0f);
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
			dict[HBAgentParkourBeamWalk::ParkourBeamWalkParams::PARAM_PREV_POSITION] = agent->get_previous_position();
			state_machine->transition_to(ASN()->beam_walk_state, dict);
		}
	}
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
bool HBAgentMoveState::_try_vault_over_obstacle() {
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

	debug_draw_raycast(ray_params, Color(0.0f, 1.0f, 0.0f));

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
	debug_draw_raycast(ray_params, Color(0.0f, 0.0f, 1.0f));

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
	debug_draw_raycast(ray_params, Color(1.0f, 1.0f, 0.0f));

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

bool HBAgentMoveState::_handle_parkour_up() {
	HBAgent *agent = get_agent();
	Ref<Shape3D> agent_shape = agent->get_collision_shape();
	const float agent_height = agent->get_height();
	// Find a wall in front
	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::ShapeParameters shape_params;
	shape_params.shape_rid = agent_shape->get_rid();
	shape_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	shape_params.transform.origin = get_agent()->get_global_position();
	shape_params.transform.origin.y += agent_height * 1.5f;

	const Vector3 input_forward = get_agent()->get_desired_movement_input_transformed().normalized();
	shape_params.motion = input_forward;

	real_t closest_safe, closest_unsafe;
	dss->cast_motion(shape_params, closest_safe, closest_unsafe);
	bool hit = closest_safe != 1.0f && closest_unsafe != 1.0f;

	debug_draw_cast_motion(agent_shape, shape_params, Color("GREEN"));
	if (hit) {
		//TODO: Check if there's a wall in front of us
		Vector3 base_wp_origin = shape_params.transform.origin + shape_params.motion * closest_safe + input_forward * agent->get_radius();
		base_wp_origin.y -= agent_height * 1.5f;

		Ref<CylinderShape3D> cyl_shape;
		cyl_shape.instantiate();
		cyl_shape->set_radius(agent->get_radius());
		const float reachable_height = agent_height * 3.0f;
		cyl_shape->set_height(reachable_height);
		shape_params.transform.origin = shape_params.transform.origin + shape_params.motion * closest_safe;
		shape_params.collide_with_areas = true;
		shape_params.collide_with_bodies = false;
		shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
		shape_params.motion = Vector3();
		shape_params.shape_rid = cyl_shape->get_rid();

		static const int constexpr MAX_RESULTS = 5;
		Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
		results.resize(MAX_RESULTS);
		int result_count = dss->intersect_shape(shape_params, results.ptrw(), MAX_RESULTS);
		for (int i = 0; i < result_count; i++) {
			HBAgentParkourLedge *ledge = Object::cast_to<HBAgentParkourLedge>(results[i].collider);
			const float offset = ledge->get_closest_offset(shape_params.transform.origin);
			const Transform3D ledge_trf = ledge->get_ledge_transform_at_offset(offset);

			const Vector3 a_l = agent->get_global_position().direction_to(ledge_trf.origin);

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

bool HBAgentMoveState::_handle_parkour_mid() {
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
				}

				_transition_to_short_hop(landing_point, next_state, next_state_args);

				return true;
			}
		}
	}

	// Second alternative, a short hop up
	// We first cast our shape forward a bit, colliding with areas
	// If we hit a ledge it means there's a point we can jump up to without having to grab the ledge first
	static const int constexpr STEPS = 2;

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

		int result_count = dss->intersect_shape(shape_params, shape_results.ptrw(), MAX_RESULTS);

		// We must make sure the ledge is roughly pointing towards us
		for (int result_i = 0; result_i < result_count; result_i++) {
			HBAgentParkourLedge *ledge_candidate = Object::cast_to<HBAgentParkourLedge>(shape_results[result_i].collider);
			if (!ledge_candidate) {
				continue;
			}
			const float offset = ledge_candidate->get_closest_offset(shape_params.transform.origin);
			Transform3D ledge_trf = ledge_candidate->get_ledge_transform_at_offset(offset);
			Vector3 ledge_forward = ledge_trf.basis.xform(Vector3(0.0f, 0.0f, 1.0f));
			if (input_forward.dot(ledge_forward) < 0.0f) {
				// finally, we find the character's landing position
				PhysicsDirectSpaceState3D::ShapeParameters landing_params;
				landing_params.shape_rid = agent_shape->get_rid();
				landing_params.transform.origin = ledge_trf.origin + (ledge_forward * -agent->get_radius() * 2.0f);
				landing_params.transform.origin.y += agent->get_height();
				landing_params.motion.y -= agent->get_height() * 1.5f;

				real_t closest_safe, closest_unsafe;

				dss->cast_motion(landing_params, closest_safe, closest_unsafe);

				if (closest_safe != 1.0f && closest_unsafe != 1.0f) {
					Vector3 landing_point = landing_params.transform.origin + landing_params.motion * closest_unsafe;
					landing_point.y -= agent->get_height() * 0.5f;
					if (landing_point.y - agent->get_global_position().y > agent->get_height() * 0.5f) {
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
	if (_handle_jump_to_ledge(box_shape, shape_trf)) {
		return true;
	}
	box_shape.instantiate();
	box_shape->set_size(Vector3(agent->get_radius() * 2.0f, agent->get_height(), STRAIGHT_TO_LEDGE_LONG_REACH));

	// This jump should only happen at edges
	shape_trf.origin = agent->get_global_position() + input_forward * (STRAIGHT_TO_LEDGE_LONG_REACH)*0.5f;
	// add a margin so we don't hit ledges at our height
	shape_trf.origin.y -= (box_shape->get_size().y * 0.5f) - agent->get_walk_stairs_step_up().y;

	if (is_at_edge && _handle_jump_to_ledge(box_shape, shape_trf)) {
		return true;
	}

	shape_trf.origin.y += box_shape->get_size().y;

	if (_handle_jump_to_parkour_point(box_shape, shape_trf)) {
		return true;
	}

	return false;
}

bool HBAgentMoveState::_handle_jump_to_ledge(Ref<BoxShape3D> p_shape, const Transform3D &p_shape_trf) {
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

bool HBAgentMoveState::_handle_jump_to_parkour_point(Ref<BoxShape3D> p_shape, const Transform3D &p_shape_trf) {
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
			next_state_args[HBAgentWallParkourState::PARAM_TARGET_PARKOUR_NODE] = point;
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

bool HBAgentMoveState::_handle_parkour() {
	HBAgent *agent = get_agent();
	bool is_parkour_down_held = get_agent()->is_action_pressed(HBAgent::INPUT_ACTION_PARKOUR_DOWN);
	bool is_parkour_up_held = get_agent()->is_action_pressed(HBAgent::INPUT_ACTION_PARKOUR_UP);
	bool is_run_held = get_agent()->is_action_pressed(HBAgent::INPUT_ACTION_RUN);
	if (is_parkour_down_held && _handle_parkour_down()) {
		return true;
	}
	if (is_parkour_down_held) {
		return false;
	}
	if (is_parkour_up_held && _handle_parkour_up()) {
		return true;
	}
	if (is_run_held && _handle_parkour_mid()) {
		return true;
	}
	return false;
}

bool HBAgentMoveState::_handle_parkour_down() {
	HBAgent *agent = get_agent();

	if (_try_vault_over_obstacle()) {
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

				Basis animation_basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), -a_l);

				Transform3D surface_anim_trf;
				surface_anim_trf.origin = ledge_slide_down_ground_position;
				surface_anim_trf.basis = animation_basis;

				Transform3D ledge_anim_trf;
				ledge_anim_trf.origin = ledge_trf.origin;
				ledge_anim_trf.basis = animation_basis;

				warp_points[StringName("ledge")] = ledge_anim_trf;
				warp_points[StringName("surface")] = surface_anim_trf;
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

	animation_node = get_epas_controller()->get_epas_node("TurnRight");
	ERR_FAIL_COND(!animation_node.is_valid());

	Node3D *gn = get_graphics_node();

	get_agent()->set_movement_mode(HBAgent::MOVE_MANUAL);
	animation_node->set_root_motion_starting_transform(get_skeleton()->get_global_transform());
	animation_node->set_root_motion_forward(Vector3(0.0f, 0.0f, 1.0f));
	get_movement_transition_node()->transition_to(HBAgentConstants::MOVEMENT_TURN180_R);
	animation_node->play();

	Vector3 movement_input = get_agent()->get_desired_movement_input_transformed();

	Vector3 forward = gn->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
	forward.y = 0.0f;
	forward.normalize();

	Vector3 rot_axis = forward.cross(movement_input).normalized();
	float angle_diff = movement_input.angle_to(forward);

	// Guard against exact 180 degree turns
	if (forward.dot(movement_input) == -1.0f) {
		angle_diff = Math_PI;
		rot_axis = Vector3(0.0f, 1.0f, 0.0f);
	}

	get_agent()->set_velocity(Vector3());

	starting_rot = gn->get_quaternion();
	target_rot = gn->get_quaternion() * Quaternion(Vector3(0.0, SIGN(rot_axis.y), 0.0), angle_diff);
	anim_length = animation_node->get_animation()->get_length() * (angle_diff / Math_PI);
	time = 0.0f;
	animation_node->connect("playback_finished", callable_mp(this, &HBAgentTurnState::_on_animation_finished), CONNECT_ONE_SHOT);
}

void HBAgentTurnState::process(float p_delta) {
	time += p_delta;
	if (animation_node.is_valid()) {
		get_agent()->apply_root_motion(animation_node, p_delta);
		Node3D *gn = get_graphics_node();
		if (gn) {
			float w = CLAMP(Math::inverse_lerp(0.0f, anim_length, time), 0.0f, 1.0f);
			gn->set_quaternion(starting_rot.slerp(target_rot, w));
		}
		if (!animation_node->is_playing()) {
			state_machine->transition_to(ASN()->move_state);
		}
	}
}

/**********************
	Ledge grabbed state
***********************/

void HBAgentLedgeGrabbedStateNew::enter(const Dictionary &p_args) {
	add_child(controller);
	controller->set_as_top_level(true);
	DEV_ASSERT(p_args.has(PARAM_LEDGE));
	ledge = Object::cast_to<HBAgentParkourLedge>(p_args[PARAM_LEDGE]);
	controller->move_to_ledge(ledge, ledge->get_closest_offset_agent(get_agent()->get_global_position()));
	animator.restart();

	for (int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		animator_options.starting_limb_transforms[i] = controller->get_limb_transform(static_cast<AgentProceduralAnimator::AgentLimb>(i));
		animator_options.target_limb_transforms[i] = animator_options.starting_limb_transforms[i];
		animator_options.limb_position_spring_halflifes[i] = 0.05f;
		animator_options.limb_rotation_spring_halflifes[i] = 0.15f;
	}

	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_LEFT_HAND][0] = 0.5f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_LEFT_HAND][1] = 0.9f;
	animator_options.limb_peak_position[AgentProceduralAnimator::LIMB_LEFT_HAND] = Vector3(0.0, 0.05f, 0.0f);

	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_RIGHT_HAND][0] = 0.0f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_RIGHT_HAND][1] = 0.4f;
	animator_options.limb_peak_position[AgentProceduralAnimator::LIMB_RIGHT_HAND] = Vector3(0.0, 0.05f, 0.0f);

	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_LEFT_FOOT][0] = 0.6f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_LEFT_FOOT][1] = 1.0f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_RIGHT_FOOT][0] = 0.1f;
	animator_options.limb_animation_timings[AgentProceduralAnimator::LIMB_RIGHT_FOOT][1] = 0.5f;

	animator_options.limb_peak_position[AgentProceduralAnimator::LIMB_LEFT_FOOT] = Vector3(0.0, 0.05f, 0.05f);
	animator_options.limb_peak_position[AgentProceduralAnimator::LIMB_RIGHT_FOOT] = Vector3(0.0, 0.05f, 0.05f);


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

	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		limbs[i].ik_node->set_ik_influence(1.0f);
		limbs[i].dangle_status = false;
	}

	get_movement_transition_node()->transition_to(HBAgentConstants::MOVEMENT_WALLGRABBED);
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
	controller->calculate_pose(p_ledge, p_offset, p_pose);
	//_update_ik_transforms(p_pose);
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
	animator_options.playback_direction = controller->get_movement_velocity() != 0.0f ? SIGN(controller->get_movement_velocity()) : animator_options.playback_direction;
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

	animator.process(animator_options, p_delta * 1.0f);

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
		animator_options.playback_direction = SIGN(input.x);
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
	ray_params.from = ledge_position + gn->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -0.01f));
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

	// We cast the agent shape down at the hit position to get the correct, as we might not fit otherwise
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
			beam_arg_dict[HBAgentParkourBeamWalk::PARAM_PREV_POSITION] = get_agent()->get_global_position();
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
			animator_options.target_limb_transforms[limb] = controller->get_limb_transform_predicted(limb);
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

// Modifies the output pose with updated values to be used with IK
void HBAgentLedgeGrabbedStateNew::_update_ik_transforms(AgentProceduralAnimator::AgentProceduralPose &p_pose) {
	Vector3 skeleton_position_offset = p_pose.skeleton_position_offset;
	skeleton_position_offset.y = 0.0f;

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

	Vector3 out_agent_position = p_pose.skeleton_trf.origin + Vector3(0.0, -get_agent()->get_height() * 0.5f, 0.0f);
	Basis out_graphics_rotation = p_pose.skeleton_trf.basis;

	p_pose.skeleton_trf.origin = out_agent_position;
	p_pose.skeleton_trf.basis = out_graphics_rotation;

	Ref<EPASAnimationNode> anim_node = get_epas_controller()->get_epas_node("WallGrabbed");
	Ref<EPASPose> ref_pose = anim_node->get_animation()->get_keyframe(0)->get_pose();

	AgentProceduralAnimator::AgentProceduralPose pose;
	animator.get_output_pose(pose);
	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		Transform3D bone_skel_trf = ref_pose->calculate_bone_global_transform(limbs[i].bone_name, get_skeleton(), get_epas_controller()->get_base_pose());
		Quaternion bone_skel_rot = bone_skel_trf.basis.get_rotation_quaternion();
		Vector3 controller_forward = pose.ik_targets[i].basis.get_rotation_quaternion().xform(Vector3(0.0f, 0.0f, -1.0f));
		Vector3 our_forward = Vector3(0.0f, 0.0f, 1.0f);
		Transform3D trf = pose.ik_targets[i];
		trf.basis = Quaternion(our_forward, controller_forward) * bone_skel_rot;
		// NOTE: Our reference wall is 0.35 meters forward of the character in the animation data, this allows us to figure out an offset for it
		Vector3 foot_offset = controller_forward * (bone_skel_trf.origin.z - 0.35f);
		trf.origin += foot_offset;

		Vector3 magnet_pos = ref_pose->get_bone_position(limbs[i].magnet_name);
		p_pose.ik_magnet_positions[i] = get_agent()->get_global_position() + Quaternion(our_forward, p_pose.skeleton_trf.basis.get_rotation_quaternion().xform(Vector3(0.0f, 0.0f, -1.0f))).normalized().xform(magnet_pos);
		p_pose.ik_targets[i] = trf;
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

		debug_draw_sphere(p_pose.skeleton_trf.origin, 0.05f, Color("GREEN"));
		debug_draw_sphere(controller->get_global_position(), 0.05f, Color("RED"));
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

/**********************
	Wall Parkour State
***********************/

Transform3D HBAgentWallParkourState::_calculate_limb_target_transform(const Quaternion &p_target_graphics_rot, const Transform3D &p_target_point_trf, const Transform3D &p_bone_base_trf, Vector3 p_visual_offset) {
	Vector3 base_position = p_target_point_trf.origin;
	Vector3 forward = p_target_point_trf.basis.xform(Vector3(0.0f, 0.0f, 1.0f));

	Transform3D trf;
	trf.origin = base_position;
	if (p_visual_offset.length_squared() > 0) {
		Quaternion rot = Quaternion(Vector3(0.0f, 0.0f, -1.0f), forward);

		Vector3 visual_offset = p_visual_offset;
		if (rot.get_axis().is_normalized()) {
			visual_offset = rot.xform(p_visual_offset);
		}
		trf.origin += visual_offset;
	}

	Basis target_skel_global_basis = p_target_graphics_rot * (get_graphics_node()->get_global_transform().basis.inverse() * get_skeleton()->get_global_transform().basis);

	trf.basis = target_skel_global_basis * p_bone_base_trf.basis;

	return trf;
}

Vector3 HBAgentWallParkourState::_calculate_limb_current_position(const WallParkourLimb &p_limb, bool p_use_visual_offset) {
	Vector3 base_position = p_limb.get_current_transform().origin;
	Vector3 forward;

	switch (p_limb.current.type) {
		case WallParkourTargetType::TARGET_PARKOUR_NODE: {
			HBAgentParkourPoint *point = p_limb.current.parkour_node;
			forward = point->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, 1.0f));
		} break;
		case WallParkourTargetType::TARGET_LOCATION: {
			forward = get_graphics_node()->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
		} break;
	}
	Quaternion rot = Quaternion(Vector3(0.0f, 0.0f, -1.0f), forward);

	Vector3 pos = base_position;
	if (p_use_visual_offset) {
		Vector3 visual_offset = p_limb.visual_offset;

		if (rot.get_axis().is_normalized()) {
			visual_offset = rot.xform(visual_offset);
		}
		pos += visual_offset;
	}

	return pos;
}

Quaternion HBAgentWallParkourState::_calculate_target_graphics_node_rotation(const Transform3D &left_hand_target_transform, const Transform3D &right_hand_target_transform) const {
	Vector3 left_hand_forward = left_hand_target_transform.basis.xform(Vector3(0.0f, 0.0f, -1.0f));
	Vector3 right_hand_forward = right_hand_target_transform.basis.xform(Vector3(0.0f, 0.0f, -1.0f));

	Vector3 average_parkour_point_normal = (left_hand_forward + right_hand_forward) * 0.5f;
	average_parkour_point_normal.y = 0.0f;
	average_parkour_point_normal.normalize();

	Basis out_basis;
	if (average_parkour_point_normal.is_normalized()) {
		out_basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), -average_parkour_point_normal);
	} else {
		// In some cases the average normal might be 0ed out, so we just use nothing as a fallback
		ERR_FAIL_V(Basis());
	}

	return out_basis;
}

void HBAgentWallParkourState::_handle_horizontal_parkour_movement(const Vector3 &p_movement_input) {
	HBAgent *agent = get_agent();
	Vector3 movement_input = p_movement_input;

	int limb_side_to_move = SIGN(p_movement_input.x);

	HBAgentParkourPoint *target_point = nullptr;

	if (parkour_limbs[AgentIKLimbType::HAND_RIGHT].target.parkour_node != parkour_limbs[AgentIKLimbType::HAND_LEFT].target.parkour_node) {
		// When the parkour node of one limb != the one used by the other limb it means we have to find a new point to grab onto
		limb_side_to_move *= -1;
		int target_i = SIGN(movement_input.x) == 1 ? AgentIKLimbType::HAND_RIGHT : AgentIKLimbType::HAND_LEFT;
		target_point = parkour_limbs[target_i].target.parkour_node;
	}

	AgentIKLimbType limb_arm_i = limb_side_to_move == 1 ? AgentIKLimbType::HAND_RIGHT : AgentIKLimbType::HAND_LEFT;
	AgentIKLimbType limb_foot_i = limb_side_to_move == 1 ? AgentIKLimbType::FOOT_RIGHT : AgentIKLimbType::FOOT_LEFT;
	WallParkourLimb *limb_arm = &parkour_limbs[limb_arm_i];
	WallParkourLimb *limb_foot = &parkour_limbs[limb_foot_i];
	WallParkourLimb *limb_arm_opposite = &parkour_limbs[limb_side_to_move == -1 ? AgentIKLimbType::HAND_RIGHT : AgentIKLimbType::HAND_LEFT];
	WallParkourLimb *limb_foot_opposite = &parkour_limbs[limb_side_to_move == -1 ? AgentIKLimbType::FOOT_RIGHT : AgentIKLimbType::FOOT_LEFT];

	limb_arm->animation_start = 0.15f;
	limb_arm->animation_end = 1.0f;
	limb_foot->animation_start = 0.0f;
	limb_foot->animation_end = 0.75f;

	if (parkour_limbs[AgentIKLimbType::HAND_RIGHT].target.parkour_node != parkour_limbs[AgentIKLimbType::HAND_LEFT].target.parkour_node) {
		// When closing in, we should move the arm first
		limb_arm->animation_start = 0.0f;
		limb_arm->animation_end = 0.5f;
		limb_foot->animation_start = 0.5f;
		limb_foot->animation_end = 1.0f;
	}

	// Setup spring for lateral movement, which should be fast
	limb_foot->position_spring_halflife = 0.05f;
	limb_arm->position_spring_halflife = 0.05f;

	// When moving horizontally it's very important to make the other foot reach
	// its target if its moving quickly, otherwise it looks weird
	limb_foot_opposite->position_spring_halflife = 0.025f;

	limb_arm_opposite->animation_start = -1.0f;
	limb_arm_opposite->animation_end = -1.0f;
	limb_foot_opposite->animation_start = -1.0f;
	limb_foot_opposite->animation_end = -1.0f;

	Vector3 movement_input_trf = limb_arm->current.parkour_node->get_global_transform().basis.xform(movement_input * Vector3(-1.0f, 1.0f, 1.0f));

	Transform3D shape_trf;
	shape_trf.origin = limb_arm->current.parkour_node->get_global_position() + movement_input_trf.normalized() * dir_check_mesh->get_height() * 0.5f;
	shape_trf.basis = Quaternion(Vector3(0.0f, 1.0f, 0.0f), movement_input_trf.normalized());

	if (!target_point) {
		PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
		PhysicsDirectSpaceState3D::ShapeParameters shape_params;
		shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
		shape_params.exclude.insert(limb_arm->current.parkour_node->get_rid());
		shape_params.shape_rid = dir_check_mesh->get_rid();
		shape_params.transform = shape_trf;
		const int MAX_RESULTS = 16;
		PhysicsDirectSpaceState3D::ShapeResult shape_results[MAX_RESULTS];
		debug_draw_cast_motion(dir_check_mesh, shape_params);
		int result_count = dss->intersect_shape(shape_params, shape_results, MAX_RESULTS);
		if (result_count > 0) {
			float closest_node_distance = 0.0f;
			target_point = Object::cast_to<HBAgentParkourPoint>(shape_results[0].collider);
			ERR_FAIL_COND(!target_point);
			closest_node_distance = target_point->get_global_position().distance_to(limb_arm->target.parkour_node->get_global_position());
			for (int i = 0; i < result_count; i++) {
				HBAgentParkourPoint *candidate = Object::cast_to<HBAgentParkourPoint>(shape_results[i].collider);
				if (!candidate) {
					continue;
				}
				float candidate_node_distance = candidate->get_global_position().distance_to(limb_arm->target.parkour_node->get_global_position());
				if (candidate_node_distance < closest_node_distance) {
					target_point = candidate;
					closest_node_distance = candidate_node_distance;
				}
			}
		}
	}

	if (!target_point) {
		return;
	}

	parkour_limbs[limb_arm_i].target.parkour_node = target_point;
	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

	Node3D *gn = get_graphics_node();
	ERR_FAIL_COND(!gn);
	Transform3D target_gn_trf;
	target_gn_trf.origin = agent_position_target;
	target_gn_trf.basis = graphics_rotation_spring_target;

	Vector3 from = target_gn_trf.xform(parkour_limbs[limb_foot_i].fallback_location_raycast_start);
	Vector3 to = target_gn_trf.xform(parkour_limbs[limb_foot_i].fallback_location_raycast_end);
	from.y = target_point->get_global_position().y - target_leg_height_from_node;
	to.y = target_point->get_global_position().y - target_leg_height_from_node;

	ray_params.from = from;
	ray_params.to = to;

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

	PhysicsDirectSpaceState3D::RayResult ray_result;

	if (!dss->intersect_ray(ray_params, ray_result)) {
		parkour_limbs[limb_foot_i].target.transform.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), ray_result.normal);
		parkour_limbs[limb_foot_i].target.transform.origin = ray_result.position;
	} else {
		parkour_limbs[limb_foot_i].target.transform.origin = target_point->get_global_position() + Vector3(0.0f, -target_leg_height_from_node, 0.0f);
		parkour_limbs[limb_foot_i].target.transform.basis = limb_arm->get_target_transform().basis;
	}

	Quaternion graphics_node_target_rot = _calculate_target_graphics_node_rotation(parkour_limbs[HAND_LEFT].get_target_transform(), parkour_limbs[HAND_RIGHT].get_target_transform());
	limb_arm->target_ik_transform = _calculate_limb_target_transform(graphics_node_target_rot, limb_arm->get_target_transform(), limb_arm->bone_base_trf, limb_arm->visual_offset);
	limb_foot->target_ik_transform = _calculate_limb_target_transform(graphics_node_target_rot, limb_foot->get_target_transform(), limb_foot->bone_base_trf, limb_foot->visual_offset);

	animation_time = 0.0f;
}

bool HBAgentWallParkourState::_handle_vertical_parkour_movement(const Vector3 &p_movement_input) {
	int vertical_dir = SIGN(p_movement_input.y);

	const int LEFT = -1;
	const int RIGHT = 1;
	const int UP = 1;
	int side_to_move = RIGHT;

	HBAgentParkourPoint *parkour_node_hand_l = parkour_limbs[AgentIKLimbType::HAND_LEFT].target.parkour_node;
	HBAgentParkourPoint *parkour_node_hand_r = parkour_limbs[AgentIKLimbType::HAND_RIGHT].target.parkour_node;
	ERR_FAIL_COND_V(!parkour_node_hand_l, false);
	ERR_FAIL_COND_V(!parkour_node_hand_r, false);

	// This is what we use to check if both hands are at the same height.
	const float SAME_HEIGHT_EPS = 0.1f;
	if (parkour_node_hand_l != parkour_node_hand_r && Math::abs(parkour_node_hand_l->get_global_position().y - parkour_node_hand_r->get_global_position().y) > SAME_HEIGHT_EPS) {
		// When going up, the lowest side is the one that's moved
		// When going down, the highest side is the one that gets moved
		side_to_move = parkour_node_hand_l->get_global_position().y > parkour_node_hand_r->get_global_position().y ? RIGHT : LEFT;
		side_to_move *= vertical_dir == UP ? 1 : -1;
	}

	// For vertical climbing we fire the collision cast from the center of both hands
	Vector3 hands_center = parkour_node_hand_l->get_global_position() + parkour_node_hand_r->get_global_position();
	hands_center *= 0.5f;

	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(!agent, false);

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::ShapeParameters params;
	const int MAX_RESULTS = 16;
	PhysicsDirectSpaceState3D::ShapeResult results[MAX_RESULTS];

	params.shape_rid = dir_check_mesh->get_rid();
	params.transform.origin = hands_center + Vector3(0.0f, dir_check_mesh->get_height() * 0.5f * vertical_dir, 0.0f);
	params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;

	debug_draw_cast_motion(dir_check_mesh, params);
	int result_count = dss->intersect_shape(params, results, MAX_RESULTS);

	WallParkourLimb *hand_to_move = &parkour_limbs[side_to_move == 1 ? AgentIKLimbType::HAND_RIGHT : AgentIKLimbType::HAND_LEFT];
	WallParkourLimb *foot_to_move = &parkour_limbs[side_to_move == 1 ? AgentIKLimbType::FOOT_RIGHT : AgentIKLimbType::FOOT_LEFT];
	WallParkourLimb *hand_to_stay = &parkour_limbs[side_to_move == -1 ? AgentIKLimbType::HAND_RIGHT : AgentIKLimbType::HAND_LEFT];
	WallParkourLimb *foot_to_stay = &parkour_limbs[side_to_move == -1 ? AgentIKLimbType::FOOT_RIGHT : AgentIKLimbType::FOOT_LEFT];

	hand_to_stay->animation_start = -1.0f;
	hand_to_stay->animation_end = -1.0f;
	foot_to_stay->animation_start = -1.0f;
	foot_to_stay->animation_end = -1.0f;

	HBAgentParkourPoint *node_to_move_to = nullptr;
	if (result_count > 0) {
		// Find the closest node to us that isn't the other hand's
		float node_to_move_to_dist = 0.0f;

		for (int i = 0; i < result_count; i++) {
			HBAgentParkourPoint *node = Object::cast_to<HBAgentParkourPoint>(results[i].collider);
			ERR_FAIL_COND_V(!node, false);
			// We try not to use the other hand's node, at least not at first
			if (node == hand_to_stay->target.parkour_node) {
				continue;
			}
			float height_diff = node->get_global_position().y - hand_to_stay->target.parkour_node->get_global_position().y;
			// Make sure the node is somewhat below or above us, depending on the direction of travel
			if (SIGN(height_diff) != vertical_dir || Math::abs(height_diff) < SAME_HEIGHT_EPS) {
				continue;
			}
			float candidate_node_dist = node->get_global_position().distance_to(hand_to_move->target.parkour_node->get_global_position());
			if (!node_to_move_to || candidate_node_dist < node_to_move_to_dist) {
				node_to_move_to = node;
				node_to_move_to_dist = candidate_node_dist;
			}
		}
	}
	// We didn't a find a node to move towards, so we will try to reach a ledge
	if (!node_to_move_to && vertical_dir == UP) {
		if (_try_reach_ledge(hand_to_move, foot_to_move)) {
			return true;
		}
	}

	// Since we can't reach a ledge, let's just put both hands on the same spot.
	if (!node_to_move_to) {
		node_to_move_to = hand_to_stay->target.parkour_node;
	}

	if (node_to_move_to == hand_to_move->target.parkour_node) {
		// Can't move, abort
		return false;
	}

	// If both hands are on the same node, we should use whichever hand is on the side of the node
	if (parkour_node_hand_l == parkour_node_hand_r) {
		int new_dir = SIGN(get_graphics_node()->get_global_transform().xform_inv(node_to_move_to->get_global_position()).x);
		if (new_dir != 0 && new_dir != side_to_move) {
			SWAP(hand_to_move, hand_to_stay);
			SWAP(foot_to_move, foot_to_stay);
		}
	}

	// Now we should just animate all of this
	if (vertical_dir == UP) {
		// When going up, it makes sense that both the foot and the hand move simultaneously
		// however, the hand should let go first, this is because
		// you would usually use your legs to give you an impulse and then grab onto the next point
		hand_to_move->animation_start = 0.0f;
		hand_to_move->animation_end = 0.8f;
		foot_to_move->animation_start = 0.15f;
		foot_to_move->animation_end = 1.0f;
	} else {
		// For going down just invert it
		hand_to_move->animation_start = 0.0f;
		hand_to_move->animation_end = 0.8f;
		foot_to_move->animation_start = 0.0f;
		foot_to_move->animation_end = 1.0f;
	}

	// Setup spring for vertical movement, which should be slower than normal to
	// prevent overstretching on the legs
	foot_to_move->position_spring_halflife = 0.09f;
	hand_to_move->position_spring_halflife = 0.1f;

	hand_to_move->target.parkour_node = node_to_move_to;
	// TODO: Make legs try to find a point to support themselves
	foot_to_move->target.transform.origin = node_to_move_to->get_global_position() - Vector3(0.0f, target_leg_height_from_node, 0.0f);
	// TODO: replace this too with the correct orientation
	foot_to_move->target.transform.basis = hand_to_move->get_target_transform().basis;
	Quaternion graphics_node_target_rot = _calculate_target_graphics_node_rotation(parkour_limbs[HAND_LEFT].get_target_transform(), parkour_limbs[HAND_RIGHT].get_target_transform());
	foot_to_move->target_ik_transform = _calculate_limb_target_transform(graphics_node_target_rot, foot_to_move->get_target_transform(), foot_to_move->bone_base_trf, foot_to_move->visual_offset);
	hand_to_move->target_ik_transform = _calculate_limb_target_transform(graphics_node_target_rot, hand_to_move->get_target_transform(), hand_to_move->bone_base_trf, hand_to_move->visual_offset);

	animation_time = 0.0f;

	return false;
}

// Will try to start the ledge reaching animation, will return true if successful.
bool HBAgentWallParkourState::_try_reach_ledge(WallParkourLimb *p_hand_to_move, WallParkourLimb *p_foot_to_move) {
	HBAgent *agent = get_agent();
	Ref<BoxShape3D> box_shape;
	box_shape.instantiate();
	float box_shape_height = get_agent()->get_height() * 1.5;
	box_shape->set_size(Vector3(agent->get_radius()*2, box_shape_height, agent->get_radius()*2));
	PhysicsDirectSpaceState3D::ShapeParameters shape_params;
	shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
	shape_params.transform.origin = agent->get_global_position();
	shape_params.transform.origin.y += box_shape_height;
	shape_params.transform.origin += agent->get_graphics_rotation().xform(Vector3(0.0, 0.0f, -1.0f) * agent->get_radius());
	shape_params.collide_with_areas = true;
	shape_params.collide_with_bodies = false;
	shape_params.shape_rid = box_shape->get_rid();

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

	constexpr static int MAX_RESULTS = 10;

	Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
	results.resize(MAX_RESULTS);

	int result_count = dss->intersect_shape(shape_params, results.ptrw(), MAX_RESULTS);
	debug_draw_shape(box_shape, shape_params.transform.origin);
	for (int i = 0; i < result_count; i++) {
		HBAgentParkourLedge *ledge = Object::cast_to<HBAgentParkourLedge>(results[i].collider);
		if (!ledge) {
			continue;
		}

		Dictionary arg;
		arg[HBAgentLedgeGrabbedStateNew::PARAM_LEDGE] = ledge;
		state_machine->transition_to(ASN()->ledge_grabbed_state, arg);
		return true;		
	}



	return false;
	/*
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(!agent, false);

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	Node3D *gn = get_graphics_node();
	Vector3 agent_normal = gn->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
	Vector3 ledge_raycast_start_pos = gn->get_global_position();

	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	PhysicsDirectSpaceState3D::RayResult ray_result;

	// Find right ledge grab point
	Vector3 from = ledge_raycast_start_pos - agent_normal * 0.5f;
	Vector3 to = ledge_raycast_start_pos + agent_normal * 0.5f;
	Transform3D ledge_transform;

	if (!find_lege_wall_sweep(from, to, Vector3(0.0f, 1.0f, 0.0f), ledge_transform)) {
		return false;
	}

	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		parkour_limbs[i].target.type = WallParkourTargetType::TARGET_LOCATION;
		if (i == FOOT_LEFT || i == FOOT_RIGHT) {
			parkour_limbs[i].position_spring_halflife = 0.09f;
		} else {
			parkour_limbs[i].position_spring_halflife = 0.1f;
		}
	}

	// Prepare this transforms already for TO_LEDGE_2
	HBAgentLedgeGrabbedState *wall_grabbed_state = Object::cast_to<HBAgentLedgeGrabbedState>(state_machine->get_state(ASN()->ledge_grabbed_state));
	ERR_FAIL_COND_V(!wall_grabbed_state, false);

	HBAgentLedgeGrabbedState::LedgeAgentIKPose initial_pose;
	HBAgentLedgeGrabbedState::WallGrabbedStateInitialPoseParams pose_params;
	pose_params.ledge_transform = ledge_transform;

	if (!wall_grabbed_state->find_initial_pose(initial_pose, pose_params)) {
		return false;
	}

	target_ledge_trf = ledge_transform;
	ledge_transition_agent_target = initial_pose.pose.actor_transform;
	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		parkour_limbs[i].local_magnet_pos = initial_pose.pose.magnet_positions[i];
		parkour_limbs[i].target.transform = initial_pose.ledge_transforms[i];
		parkour_limbs[i].current_ik_transform = parkour_limbs[i].target_ik_transform;
		parkour_limbs[i].target_ik_transform = initial_pose.pose.target_transforms[i];
	}

	p_hand_to_move->animation_start = 0.0f;
	p_hand_to_move->animation_end = 0.8f;
	p_foot_to_move->animation_start = 0.0f;
	p_foot_to_move->animation_end = 1.0f;
	animation_time = 0.0f;
	parkour_stage = ParkourStage::TO_LEDGE_1;

	return true;
	*/
	// TODO: Fix this
}

void HBAgentWallParkourState::enter(const Dictionary &p_args) {
	ERR_FAIL_COND(!p_args.has(PARAM_TARGET_PARKOUR_NODE));
	HBAgentParkourPoint *starting_parkour_node = Object::cast_to<HBAgentParkourPoint>(p_args.get(PARAM_TARGET_PARKOUR_NODE, Variant()));
	ERR_FAIL_COND(!starting_parkour_node);
	EPASController *epas_controller = get_epas_controller();
	ERR_FAIL_COND(!epas_controller);

	Ref<EPASTransitionNode> movement_transition = get_movement_transition_node();
	ERR_FAIL_COND(!movement_transition.is_valid());

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);

	Ref<EPASAnimationNode> animation_node = epas_controller->get_epas_node("WallParkourCat");
	ERR_FAIL_COND(!animation_node.is_valid());
	Ref<EPASPose> reference_pose = animation_node->get_animation()->get_keyframe(0)->get_pose();
	back_straightness_blend_node = epas_controller->get_epas_node("WallParkourCatBlend");
	ERR_FAIL_COND(!back_straightness_blend_node.is_valid());

	Ref<EPASLookatNode> lookat = get_epas_controller()->get_epas_node("HeadLookAt");
	lookat->set_influence(0.0f);
	
	lookat = get_epas_controller()->get_epas_node("TorsoLookAt");
	lookat->set_influence(0.0f);

	if (!limbs_init) {
		// Initialize limb structures
		limbs_init = true;

		StringName ik_node_names[LIMB_TYPE_MAX] = {
			"LeftHandIK",
			"RightHandIK",
			"LeftFootIK",
			"RightFootIK"
		};
		StringName ik_magnet_node_names[LIMB_TYPE_MAX] = {
			"IK.Magnet.hand.L",
			"IK.Magnet.hand.R",
			"IK.Magnet.foot.L",
			"IK.Magnet.foot.R"
		};

		for (int i = 0; i < LIMB_TYPE_MAX; i++) {
			parkour_limbs[i].ik_node = epas_controller->get_epas_node(ik_node_names[i]);
			ERR_FAIL_COND(!parkour_limbs[i].ik_node.is_valid());
			parkour_limbs[i].bone_name = parkour_limbs[i].ik_node->get_ik_end();

			StringName a_bone_name = parkour_limbs[i].ik_node->get_ik_end();
			int b_bone_idx = skel->get_bone_parent(skel->find_bone(parkour_limbs[i].bone_name));
			StringName b_bone_name = skel->get_bone_name(b_bone_idx);
			StringName c_bone_name = skel->get_bone_name(skel->get_bone_parent(b_bone_idx));

			Ref<EPASPose> base_pose = animation_node->get_animation()->get_keyframe(0)->get_pose();

			Transform3D a_bone_trf = base_pose->calculate_bone_global_transform(a_bone_name, skel, epas_controller->get_base_pose());

			// Again, in skeleton space z=+1.0 is forward
			Quaternion rot = Quaternion(Vector3(0.0f, 0.0f, 1.0f), Vector3(0.0f, 0.0f, -1.0f));
			parkour_limbs[i].fallback_location_raycast_start = rot.xform(a_bone_trf.origin);
			parkour_limbs[i].fallback_location_raycast_start.z = 0.75f;
			parkour_limbs[i].fallback_location_raycast_end = parkour_limbs[i].fallback_location_raycast_start;
			parkour_limbs[i].fallback_location_raycast_end.z = -0.75f * 2;

			// Magnet is already in global space, so this should be fine, assuming root is at 0,0 that is...
			parkour_limbs[i].local_magnet_pos = base_pose->get_bone_position(ik_magnet_node_names[i]);
			parkour_limbs[i].default_magnet_pos = parkour_limbs[i].local_magnet_pos;
		}
	}

	parkour_limbs[AgentIKLimbType::HAND_LEFT].target.type = WallParkourTargetType::TARGET_PARKOUR_NODE;
	parkour_limbs[AgentIKLimbType::HAND_LEFT].current.type = WallParkourTargetType::TARGET_PARKOUR_NODE;
	parkour_limbs[AgentIKLimbType::HAND_LEFT].target.parkour_node = starting_parkour_node;
	parkour_limbs[AgentIKLimbType::HAND_LEFT].current.parkour_node = starting_parkour_node;
	parkour_limbs[AgentIKLimbType::HAND_LEFT].animating_peak_offset = Vector3(0.0, 0.0, -0.1f);
	parkour_limbs[AgentIKLimbType::HAND_LEFT].visual_offset = Vector3(-0.05f, 0.05f, 0.125f);

	parkour_limbs[AgentIKLimbType::HAND_RIGHT].target.type = WallParkourTargetType::TARGET_PARKOUR_NODE;
	parkour_limbs[AgentIKLimbType::HAND_RIGHT].current.type = WallParkourTargetType::TARGET_PARKOUR_NODE;
	parkour_limbs[AgentIKLimbType::HAND_RIGHT].target.parkour_node = starting_parkour_node;
	parkour_limbs[AgentIKLimbType::HAND_RIGHT].current.parkour_node = starting_parkour_node;
	parkour_limbs[AgentIKLimbType::HAND_RIGHT].animating_peak_offset = Vector3(0.0, 0.0, -0.1f);
	parkour_limbs[AgentIKLimbType::HAND_RIGHT].visual_offset = Vector3(0.05f, 0.05f, 0.125f);

	parkour_limbs[AgentIKLimbType::FOOT_LEFT].visual_offset = Vector3(-0.1f, 0.0f, 0.150f);
	parkour_limbs[AgentIKLimbType::FOOT_RIGHT].visual_offset = Vector3(0.1f, 0.0f, 0.150f);
	parkour_limbs[AgentIKLimbType::FOOT_LEFT].animating_peak_offset = Vector3(0.0, 0.0, -0.1f);
	parkour_limbs[AgentIKLimbType::FOOT_RIGHT].animating_peak_offset = Vector3(0.0, 0.0, -0.1f);

	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);

	Node3D *gn = get_graphics_node();
	ERR_FAIL_COND(!gn);

	graphics_rotation_spring_target = gn->get_global_transform().basis;

	const float floor_max_angle = agent->get_floor_max_angle();
	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		if (parkour_limbs[i].target.type == TARGET_LOCATION) {
			PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

			PhysicsDirectSpaceState3D::RayParameters ray_params;
			PhysicsDirectSpaceState3D::RayResult ray_result;

			ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
			ray_params.from = starting_parkour_node->get_global_transform().xform(Vector3(0.0f, -target_leg_height_from_node, -0.75f));
			ray_params.to = starting_parkour_node->get_global_transform().xform(Vector3(0.0f, -target_leg_height_from_node, 0.75f));

			if (dss->intersect_ray(ray_params, ray_result)) {
				ERR_FAIL_COND_MSG(!is_wall(ray_result.normal, floor_max_angle), "Failed to get wall for feet, check level geometry");
				parkour_limbs[i].target.transform.origin = ray_result.position;
				parkour_limbs[i].target.transform.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), ray_result.normal);
				parkour_limbs[i].current.transform = parkour_limbs[i].target.transform;
				parkour_limbs[i].current.type = WallParkourTargetType::TARGET_LOCATION;
			} else {
				parkour_limbs[i].dangling = true;
			}
		}
		// Setup magnet (common for both target types)
		parkour_limbs[i].ik_node->set_use_magnet(true);
		parkour_limbs[i].ik_node->set_ik_influence(1.0f);
		parkour_limbs[i].local_magnet_pos = parkour_limbs[i].default_magnet_pos;
		Vector3 magnet = skel->to_global(parkour_limbs[i].local_magnet_pos);
		parkour_limbs[i].ik_node->set_magnet_position(magnet);
		parkour_limbs[i].bone_base_trf = reference_pose->calculate_bone_global_transform(parkour_limbs[i].bone_name, skel, epas_controller->get_base_pose());
		Transform3D target_trf = _calculate_limb_target_transform(graphics_rotation_spring_target, parkour_limbs[i].get_target_transform(), parkour_limbs[i].bone_base_trf, parkour_limbs[i].visual_offset);
		parkour_limbs[i].ik_node->set_target_transform(target_trf);
		parkour_limbs[i].target_ik_transform = target_trf;
		parkour_limbs[i].current_ik_transform = target_trf;
		parkour_limbs[i].ik_node->set_use_hinge(true);
		parkour_limbs[i].position_spring_velocity = Vector3();
	}

	if (!dir_check_mesh.is_valid()) {
		dir_check_mesh.instantiate();
		dir_check_mesh->set_height(1.0f);
		dir_check_mesh->set_radius(0.5f);
	}

	agent->set_movement_mode(HBAgent::MovementMode::MOVE_MANUAL);

	Vector3 limb_center;

	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		limb_center += parkour_limbs[i].target_ik_transform.origin;
	}

	limb_center = limb_center / ((float)AgentIKLimbType::LIMB_TYPE_MAX);

	Vector3 center = agent->get_global_position() - limb_center;
	agent_offset_base = Vector3(-0, -0.60059, 0.212675) + agent_offset_adjustment;
	agent_offset_target = agent_offset_base;
	// Reset springs
	agent_position_spring_velocity = Vector3();
	graphics_rotation_spring_velocity = Vector3();
	agent_position_target = agent->get_global_position();

	movement_transition->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_PARKOUR_CAT);

	animation_time = -1.0f;
	parkour_stage = ParkourStage::NORMAL;
}

void HBAgentWallParkourState::exit() {
	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		parkour_limbs[i].ik_node->set_ik_influence(0.0f);
	}

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);
	skel->set_position(Vector3());
}

void HBAgentWallParkourState::physics_process(float p_delta) {
	debug_draw_clear();
	if (get_inertialization_node()->is_inertializing()) {
		return;
	}
	HBAgent *agent = get_agent();

	if (animation_time == -1.0f && agent->is_action_pressed(HBAgent::AgentInputAction::INPUT_ACTION_PARKOUR_DOWN)) {
		state_machine->transition_to(StringName("Fall"));
		return;
	}

	Vector3 movement_input = agent->get_movement_input();
	movement_input.y = movement_input.z * -1.0f;
	movement_input.z = 0.0f;

	Node3D *gn = get_graphics_node();

	Vector<float> weights;
	weights.resize(AgentIKLimbType::LIMB_TYPE_MAX);
	weights.fill(1.0f);
	Vector<Transform3D> ik_targets;
	ik_targets.resize(AgentIKLimbType::LIMB_TYPE_MAX);

	float total_weight = AgentIKLimbType::LIMB_TYPE_MAX;

	Skeleton3D *skel = get_skeleton();

	if (animation_time >= 0.0f) {
		animation_time = CLAMP(animation_time + p_delta * (1.0f / ANIMATION_DURATION), 0.0f, 1.0f);
		total_weight = 0.0f;

		for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
			if (parkour_limbs[i].animation_start == -1.0f) {
				ik_targets.write[i] = parkour_limbs[i].current_ik_transform;
				total_weight += 1.0f;
				weights.write[i] = 1.0f;
				continue;
			}

			Transform3D target_trf;
			float blend_t = Math::inverse_lerp(parkour_limbs[i].animation_start, parkour_limbs[i].animation_end, animation_time);
			blend_t = CLAMP(blend_t, 0.0f, 1.0f);

			weights.write[i] = 1.0f - sin(blend_t * Math_PI);

			target_trf = parkour_limbs[i].current_ik_transform.interpolate_with(parkour_limbs[i].target_ik_transform, blend_t);
			target_trf.origin += skel->get_global_transform().basis.xform(parkour_limbs[i].animating_peak_offset) * sin(blend_t * Math_PI);

			ik_targets.write[i] = target_trf;
			total_weight += weights[i];
		}
	} else {
		for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
			ik_targets.write[i] = parkour_limbs[i].current_ik_transform;
		}
	}

	// Back eject is only possible when both feet aren't dangling
	// TODO: reimplement back ejects

	if (parkour_stage == ParkourStage::NORMAL && animation_time == -1.0f && movement_input.length_squared() != 0.0f) {
		if (Math::abs(movement_input.y) > Math::abs(movement_input.x)) {
			_handle_vertical_parkour_movement(movement_input);
		} else {
			_handle_horizontal_parkour_movement(movement_input);
		}
	}

	Vector3 ik_position_weighted; // This is used as the center point for the body movement
	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		if (parkour_limbs[i].dangling && (i == AgentIKLimbType::FOOT_LEFT || i == AgentIKLimbType::FOOT_RIGHT)) {
			AgentIKLimbType hand = i == AgentIKLimbType::FOOT_LEFT ? AgentIKLimbType::HAND_LEFT : AgentIKLimbType::HAND_RIGHT;
			Vector3 target = ik_targets[hand].origin;
			target.y -= parkour_limbs[hand].fallback_location_raycast_start.y;
			ik_position_weighted += target * (weights[i] / total_weight);
			parkour_limbs[i].target.transform.origin = target;
			continue;
		}
		ik_position_weighted += ik_targets[i].origin * (weights[i] / total_weight);

		Transform3D new_trf = parkour_limbs[i].ik_node->get_target_transform();
		Vector3 target_origin = parkour_limbs[i].current_ik_transform.origin.lerp(ik_targets[i].origin, animation_time);
		// Funny thing here, the movement of the agent is based on the target unsprung
		// positions, however the limbs themselves are driven by springs, this is a crude
		// way to add even more of a feel of inertia and weight transfer
		HBSprings::critical_spring_damper_exact_vector3(
				new_trf.origin,
				parkour_limbs[i].position_spring_velocity,
				target_origin,
				parkour_limbs[i].position_spring_halflife, p_delta);
		new_trf.basis = ik_targets[i].basis;
		parkour_limbs[i].ik_node->set_target_transform(new_trf);
	}

	float curr_leg_height_difference = parkour_limbs[AgentIKLimbType::FOOT_LEFT].ik_node->get_target_transform().origin.y;
	curr_leg_height_difference -= parkour_limbs[AgentIKLimbType::FOOT_RIGHT].ik_node->get_target_transform().origin.y;

	float curr_hand_height_difference = parkour_limbs[AgentIKLimbType::HAND_LEFT].ik_node->get_target_transform().origin.y;
	curr_hand_height_difference -= parkour_limbs[AgentIKLimbType::HAND_RIGHT].ik_node->get_target_transform().origin.y;

	float diff = Math::abs(curr_hand_height_difference) / 0.5f;
	diff = CLAMP(diff, 0.0f, 1.0f);
	back_straightness_blend_node->set_blend_amount(diff);
	// TODO: Make these constant
	agent_offset_target.z = agent_offset_base.z - diff * 0.1f;
	agent_offset_target.y = agent_offset_base.y + Math::abs(curr_hand_height_difference) - Math::abs(curr_leg_height_difference * 0.85f);

	debug_draw_sphere(ik_position_weighted, 0.05f, Color("PURPLE"));

	agent_position_target = ik_position_weighted + get_graphics_node()->get_global_transform().basis.xform(agent_offset_target);

	if (parkour_stage == ParkourStage::TO_LEDGE_2) {
		if (animation_time != -1.0f) {
			agent_position_target = agent_position_target.lerp(ledge_transition_agent_target.origin, animation_time);
		}
		debug_draw_sphere(target_ledge_trf.origin, 0.05f, Color("Yellow"));
	}

	Vector3 new_pos = get_agent()->get_global_position();

	HBSprings::spring_damper_exact_ratio_vector3(
			new_pos,
			agent_position_spring_velocity,
			agent_position_target,
			Vector3(), 0.7f, 0.2f, p_delta);

	get_agent()->set_global_position(new_pos);

	Transform3D new_gn_trf = gn->get_global_transform();
	Quaternion new_rot = new_gn_trf.basis.get_rotation_quaternion();
	graphics_rotation_spring_target = _calculate_target_graphics_node_rotation(parkour_limbs[AgentIKLimbType::HAND_LEFT].get_target_transform(), parkour_limbs[AgentIKLimbType::HAND_RIGHT].get_target_transform());

	HBSprings::simple_spring_damper_exact_quat(new_rot, graphics_rotation_spring_velocity, graphics_rotation_spring_target, graphics_rotation_spring_halflife, p_delta);

	new_gn_trf.basis = new_rot;
	gn->set_global_transform(new_gn_trf);

	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		// Update magnet position
		Vector3 magnet = skel->to_global(parkour_limbs[i].local_magnet_pos);
		parkour_limbs[i].ik_node->set_magnet_position(magnet);

		debug_draw_sphere(magnet);
	}
	if (animation_time >= 1.0f) {
		animation_time = -1.0f;

		for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
			if (parkour_limbs[i].animation_start == -1.0 && parkour_limbs[i].animation_end == -1.0f) {
				continue;
			}
			parkour_limbs[i].current_ik_transform = parkour_limbs[i].target_ik_transform;
			parkour_limbs[i].current.type = parkour_limbs[i].target.type;
			switch (parkour_limbs[i].target.type) {
				case WallParkourTargetType::TARGET_PARKOUR_NODE: {
					parkour_limbs[i].current.parkour_node = parkour_limbs[i].target.parkour_node;
				} break;
				case WallParkourTargetType::TARGET_LOCATION: {
					parkour_limbs[i].current.transform = parkour_limbs[i].target.transform;
				} break;
			}
		}

		if (parkour_stage == ParkourStage::TO_LEDGE_1) {
			// Use swap instead of hardcoding for consistency
			SWAP(parkour_limbs[HAND_LEFT].animation_start, parkour_limbs[HAND_RIGHT].animation_start);
			SWAP(parkour_limbs[HAND_LEFT].animation_end, parkour_limbs[HAND_RIGHT].animation_end);
			SWAP(parkour_limbs[FOOT_LEFT].animation_start, parkour_limbs[FOOT_RIGHT].animation_start);
			SWAP(parkour_limbs[FOOT_LEFT].animation_end, parkour_limbs[FOOT_RIGHT].animation_end);
			animation_time = 0.0f;
			parkour_stage = ParkourStage::TO_LEDGE_2;
		} else if (parkour_stage == ParkourStage::TO_LEDGE_2) {
			Dictionary args;
			// TODO: Fix this
			//args[HBAgentLedgeGrabbedState::PARAM_LEDGE_TRF] = target_ledge_trf;
			state_machine->transition_to(ASN()->ledge_grabbed_state, args);
		}
	}
}

#ifdef DEBUG_ENABLED
void HBAgentWallParkourState::debug_ui_draw() {
	HBAgentState::debug_ui_draw();
	ImGui::InputFloat3("Left leg magnet pos", (float *)&parkour_limbs[AgentIKLimbType::FOOT_LEFT].local_magnet_pos.coord);
	ImGui::InputFloat3("Right leg magnet pos", (float *)&parkour_limbs[AgentIKLimbType::FOOT_RIGHT].local_magnet_pos.coord);
}
#endif

void HBAgentWallParkourState::_notification(int p_what) {
}

HBAgentWallParkourState::HBAgentWallParkourState() {
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

	if (get_agent()->is_action_pressed(HBAgent::INPUT_ACTION_RUN) && !movement_goes_through_beam) {
		if (_handle_parkour_mid()) {
			return;
		}
	}

	if (get_agent()->is_action_pressed(HBAgent::INPUT_ACTION_PARKOUR_DOWN)) {
		if (_handle_parkour_down()) {
			return;
		}
	}

	if (movement_input.length_squared() > 0) {
		if (movement_goes_through_beam) {
			desired_movement_input = movement_input.project(forward);
			desired_movement_input = desired_movement_input.normalized() * movement_input.length();
		}
	}

	Ref<HBAgentConstants> agent_constants = get_agent()->get_agent_constants();

	Vector3 desired_velocity = desired_movement_input * agent_constants->get_max_move_velocity();
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
	if (curve_offset != clamped_offset) {
		// When the offset goes beyond our clamped offset this means we've gone out of the beam
		curve_offset = clamped_offset;
		agent_global_position = beam->get_global_transform().xform(beam->get_curve()->sample_baked(curve_offset));
		Dictionary args;
		args[HBAgentMoveState::PARAM_TRANSITION_DURATION] = 0.5f;
		state_machine->transition_to(ASN()->move_state, args);
		get_agent()->reset_desired_input_velocity_to(get_agent()->get_linear_velocity());
		return;
	}

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
	ERR_FAIL_COND(!animation_node->get_animation().is_valid());
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
}

void HBAgentRootMotionState::physics_process(float p_delta) {
	if (hack) {
		return;
	}
	debug_draw_sphere(animation_node->get_root_motion_starting_transform().origin, 0.05f, Color("GREEN"));
	get_agent()->apply_root_motion(animation_node, p_delta);
	
	if (!inertialization_init) {
		Transform3D trf = get_agent()->get_global_transform();
		trf.basis = get_agent()->get_graphics_rotation();
		get_agent()->inertialize_graphics_transform(trf, 0.25f);
		inertialization_init = true;
	}

	if (!animation_node->is_playing()) {
		switch (velocity_mode) {
			case ANIMATION_DRIVEN: {
				get_agent()->reset_desired_input_velocity_to((get_agent()->get_global_position() - prev_pos) / p_delta);
			} break;
			case CONSERVE: {
				get_agent()->set_velocity(get_agent()->get_desired_movement_input_transformed() * get_agent()->get_agent_constants()->get_max_move_velocity());
				get_agent()->reset_desired_input_velocity_to(get_agent()->get_desired_movement_input_transformed() * get_agent()->get_agent_constants()->get_max_move_velocity());
			} break;
		}
		state_machine->transition_to(next_state, next_state_args);
	}
	prev_pos = get_agent()->get_global_position();
}

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
		shape_params.motion = input * EJECT_REACH * ((i+1) / ((float)ITERS));
		real_t closest_safe, closest_unsafe;
		debug_draw_cast_motion(shape, shape_params, Color("RED"));
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

/**********************
	WALL PARKOUR NEW
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
		ik_nodes[i]->set_ik_influence(1.0f);
		ik_nodes[i]->set_magnet_position(p_pose.ik_magnet_positions[i]);
		ik_nodes[i]->set_target_transform(p_pose.ik_targets[i]);
	}

	HBAgent *agent = get_agent();

	agent->set_global_position(p_pose.skeleton_trf.origin + p_pose.skeleton_position_offset);
	agent->set_graphics_rotation(p_pose.skeleton_trf.basis);
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

	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		StringName bone_name = AgentProceduralAnimator::limb_to_bone_name(static_cast<AgentProceduralAnimator::AgentLimb>(i));
		Transform3D hand_trf_local = reference_pose->calculate_bone_global_transform(bone_name, get_skeleton(), get_epas_controller()->get_base_pose());
		hand_trf_local = wp_trf.affine_inverse() * hand_trf_local;
		p_pose.pose.ik_targets[i] = p_point->get_global_transform() * hand_trf_local;
		if (i <= AgentProceduralAnimator::LIMB_RIGHT_HAND) {
			p_pose.pose.ik_magnet_positions[i] = Vector3(0.0f, -1.0f, 0.0f);
		} else {
			p_pose.pose.ik_magnet_positions[i] = Vector3(0.0f, -1.0f, 0.0f);
		}
		p_pose.parkour_points[i] = const_cast<HBAgentParkourPoint*>(p_point);
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
	animator_options.target_skeleton_transform.basis = Quaternion(Vector3(0.0, 0.0, -1.0f), target_pose.pose.skeleton_trf.basis.xform(Vector3(0.0, 0.0, 1.0f)));
	animator_options.target_skeleton_transform.origin = target_pose.pose.skeleton_trf.origin;

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
	animator_options.target_skeleton_transform.basis = Quaternion(Vector3(0.0, 0.0, -1.0f), target_pose.pose.skeleton_trf.basis.xform(Vector3(0.0, 0.0, 1.0f)));
	animator_options.target_skeleton_transform.origin = target_pose.pose.skeleton_trf.origin;
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
			p_pose.ik_magnet_positions[i].y -= 3.0f;
		} else {
			// Feet
			p_pose.ik_magnet_positions[i] = p_pose.ik_targets[i].xform(Vector3(0.0f, 3.0f, 0.0f));
			p_pose.ik_magnet_positions[i].y += 3.0f;
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
	new_graphics_trf.basis = Quaternion(Vector3(0.0, 0.0, -1.0f), new_graphics_trf.basis.xform(Vector3(0.0, 0.0, 1.0f)));
	get_agent()->inertialize_graphics_transform(new_graphics_trf, 0.25f);
	Ref<EPASLookatNode> lookat_node = get_epas_controller()->get_epas_node(ASN()->lookat_head_node_name);
	lookat_node->set_influence(0.0f);
	lookat_node = get_epas_controller()->get_epas_node(ASN()->lookat_torso_node_name);
	lookat_node->set_influence(0.0f);

	update_animator_initial();

	for (int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		animator_options.limb_position_spring_halflifes[i] = 0.1f;
	}
	animator_options.skeleton_position_spring_halflife = 0.15f;
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
	HBSprings::critical_spring_damper_exact(&back_straightness, &back_straightness_velocity, back_straightness_target, 0.5f, p_delta);
	back_straightness_blend_node->set_blend_amount(back_straightness);

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

	for(int i = 0; i < AgentProceduralAnimator::AgentLimb::LIMB_MAX; i++) {
		debug_draw_sphere(pose.ik_magnet_positions[i], 0.05f, Color("RED"));
	}

	bool animator_just_finished = !animator_was_finished && animator.is_done();
	bool are_hands_together = target_pose.parkour_points[AgentProceduralAnimator::LIMB_LEFT_HAND] == target_pose.parkour_points[AgentProceduralAnimator::LIMB_RIGHT_HAND];

	int vertical_component = 0;
	int horizontal_component = 0;

	if (!input.is_zero_approx()){
		const float snapped_angle = Math::snapped(input.signed_angle_to(Vector3(0.0, 1.0, 0.0), Vector3(0.0f, 0.0f, -1.0f)), Math::deg_to_rad(45.0f));
		const Vector3 snapped_untransformed_dir = Vector3(0.0f, 1.0f, 0.0f).rotated(Vector3(0.0f, 0.0f, 1.0f), snapped_angle);
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

		const float SHORT_GRAB_REACH = 0.5f;

		HBAgentParkourPoint *point_to_reach = find_reachable_parkour_point(sampling_source_limb, snapped_dir, SHORT_GRAB_REACH);

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
			animator.reset();
			current_parkour_point_target = point_to_reach;
			current_limb = limb_to_move;
		} else {
			// Try if we can find a point to long jump to
			const float LONG_GRAB_REACH = 1.5f;
			HBAgentParkourPoint* long_grab_point = find_reachable_parkour_point(sampling_source_limb, snapped_dir, LONG_GRAB_REACH);

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
		shape_params.motion = input * EJECT_REACH * ((i+1) / ((float)ITERS));
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
HBAgentParkourPoint* HBAgentWallParkourStateNew::find_reachable_parkour_point(const AgentProceduralAnimator::AgentLimb p_sampling_limb, const Vector3 &p_direction, const float &p_reach) const {
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
		
		if (point_to_reach) {
			// Perhaps we should project the hit nodes onto the input direction to use as a distance...
			const float distance_to_current_point = sampling_source_trf.origin.distance_to(point_to_reach->get_global_position());
			const float distance_to_candidate_point = sampling_source_trf.origin.distance_to(pp->get_global_position());
			if (distance_to_candidate_point > distance_to_current_point) {
				continue;
			}
		}

		point_to_reach = pp;
		skip_iter: ;
	}
	return point_to_reach;
}
HBAgentWallParkourStateNew::HBAgentWallParkourStateNew() {
}
