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

bool HBAgentState::_try_slide_edge(Vector3 p_dir) {
	if (!get_agent()->is_at_edge(p_dir)) {
		return false;
	}

	const int SLIDE_DOWN_FLOOR_ITERATIONS = 6;
	Vector3 from = get_agent()->get_global_position();
	from += p_dir * 2.0f;
	Vector3 to = get_agent()->get_global_position();

	Transform3D edge_trf;

	if (!find_lege_wall_sweep(from, to, Vector3(0.0f, get_agent()->get_height() * -0.25f, 0.0f), edge_trf)) {
		return false;
	}

	PhysicsDirectSpaceState3D::RayParameters params;
	params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	PhysicsDirectSpaceState3D::RayResult result;
	PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();
	bool got_result = false;
	for (int i = 0; i < SLIDE_DOWN_FLOOR_ITERATIONS; i++) {
		params.from = get_agent()->get_global_position() + p_dir * get_agent()->get_radius() * 2.0f;
		params.from += p_dir * (i / ((float)SLIDE_DOWN_FLOOR_ITERATIONS - 1)) * 1.5f;
		params.to = params.from;
		params.to.y -= get_agent()->get_height() * 0.75f;
		PhysicsDirectSpaceState3D::RayResult temp_result;
		debug_draw_raycast(params);
		if (!dss->intersect_ray(params, temp_result) || !is_floor(temp_result.normal, get_agent()->get_floor_max_angle())) {
			break;
		}
		got_result = true;
		result = temp_result;
		break;
	}
	if (!got_result) {
		return false;
	}
	edge_trf.basis = Quaternion(edge_trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f)), edge_trf.basis.xform(Vector3(0.0f, 0.0f, 1.0f))) * edge_trf.basis;

	Dictionary warp_points;
	warp_points[StringName("ledge")] = edge_trf;

	Transform3D surface_wp;
	surface_wp.origin = result.position;
	surface_wp.basis = Quaternion(Vector3(0.0f, 0.0f, 1.0), p_dir);
	warp_points[StringName("surface")] = surface_wp;

	Dictionary args;
	args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = StringName("LedgeToGround");
	args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->move_state;
	args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MovementTransitionInputs::MOVEMENT_LEDGE_TO_GROUND;
	args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;

	state_machine->transition_to(ASN()->root_motion_state, args);

	return true;
}

bool HBAgentState::_try_find_parkour_point(Vector3 p_dir, float p_length, float p_radius, HBAgentParkourPoint **p_out_parkour_point) {
	*p_out_parkour_point = nullptr;
	PhysicsDirectSpaceState3D::ShapeParameters shape_params;

	Ref<CylinderShape3D> cylinder_shape;
	cylinder_shape.instantiate();
	cylinder_shape->set_height(p_length);
	cylinder_shape->set_radius(p_radius);
	shape_params.transform.origin = get_agent()->get_global_position();
	shape_params.transform.origin += p_dir * (cylinder_shape->get_height() * 0.5f);
	shape_params.transform.origin.y += p_radius;
	shape_params.transform.basis = Quaternion(Vector3(0.0f, 1.0f, 0.0f), p_dir);
	shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
	shape_params.shape_rid = cylinder_shape->get_rid();

	const int MAX_RESULTS = 8;
	Vector<PhysicsDirectSpaceState3D::ShapeResult> shape_results;
	shape_results.resize(8);

	PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();

	HBAgentParkourPoint *target_point = nullptr;

	get_debug_geometry()->debug_cast_motion(cylinder_shape, shape_params);
	for (int i = 0; i < dss->intersect_shape(shape_params, shape_results.ptrw(), MAX_RESULTS); i++) {
		HBAgentParkourPoint *point = Object::cast_to<HBAgentParkourPoint>(shape_results[i].collider);

		if (!point) {
			continue;
		}

		Vector3 point_normal = point->get_global_basis().xform(Vector3(0.0f, 0.0f, -1.0f));

		if (point_normal.dot(p_dir) >= 0.0f) {
			continue;
		}

		target_point = point;
		break;
	}

	if (!target_point) {
		return false;
	}

	*p_out_parkour_point = target_point;

	return true;
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

bool HBAgentState::find_ledge(const Vector3 &p_wall_base_point, const Vector3 &p_wall_normal, Transform3D &p_ledge_transform) {
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(!agent, false);

	// wallrun constantss
	const float max_ledge_height = agent->get_agent_constants()->get_parkour_max_ledge_height();
	const float floor_max_angle = agent->get_floor_max_angle();

	PhysicsDirectSpaceState3D::RayResult ray_result;
	Ref<Shape3D> agent_shape = agent->get_collision_shape();

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	ray_params.from = p_wall_base_point - p_wall_normal * 0.01f;
	ray_params.from.y += max_ledge_height;
	ray_params.to = p_wall_base_point - p_wall_normal * 0.01f;

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

	// Try to find a roof
	debug_draw_raycast(ray_params, Color(0.0f, 0.0f, 1.0f));
	if (!dss->intersect_ray(ray_params, ray_result)) {
		return false;
	}

	// Make sure its a roof
	if (!is_floor(ray_result.normal, floor_max_angle)) {
		return false;
	}

	p_ledge_transform.origin = p_wall_base_point;
	p_ledge_transform.origin.y = ray_result.position.y;
	p_ledge_transform.basis = Basis::looking_at(p_wall_normal);

	return true;
}

bool HBAgentState::find_lege_wall_sweep(const Vector3 &p_from, const Vector3 &p_to, const Vector3 &p_offset, Transform3D &p_out_edge_trf, int p_iterations) {
	PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
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

	if (!has_result) {
		return false;
	}

	ray_params.from = ray_result.position + ray_result.normal * -0.01;
	ray_params.to = ray_params.from;
	ray_params.from.y += get_agent()->get_height();

	Transform3D edge_trf;
	edge_trf.origin = ray_result.position;
	edge_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), ray_result.normal);

	debug_draw_raycast(ray_params);
	if (!dss->intersect_ray(ray_params, ray_result) || !is_floor(ray_result.normal, floor_max_angle)) {
		return false;
	}

	edge_trf.origin.y = ray_result.position.y;

	p_out_edge_trf = edge_trf;

	return true;
}

HBDebugGeometry *HBAgentState::get_debug_geometry() {
	return _get_debug_geo();
}

bool HBAgentState::handle_autojump_mid(Vector3 p_dir, bool p_needs_gap) {
	HBAgentParkourAutoJumpState *autojump_state = Object::cast_to<HBAgentParkourAutoJumpState>(state_machine->get_state(ASN()->autojump_state));
	HBAgentParkourPoint *point = nullptr;
	if (_try_find_parkour_point(p_dir, 10.0f, 1.5f, &point)) {
		ParkourAutojump::AutojumpSettings settings;
		settings.needs_gap = p_needs_gap;

		Vector3 target_pos = point->get_global_position();
		target_pos.y -= get_agent()->get_height() * 0.66666f;
		target_pos += point->get_global_basis().xform(Vector3(0.0f, 0.0f, -1.0f)) * get_agent()->get_radius() * 2.0f;
		ParkourAutojump::AgentParkourAutojumpData data = settings.find_autojump_from_to(
				get_agent(),
				get_agent()->get_global_position(),
				target_pos);
		if (data.found) {
			Dictionary transition_dict;
			Dictionary ledge_state_args;
			ledge_state_args[HBAgentWallParkourState::PARAM_TARGET_PARKOUR_NODE] = point;
			transition_dict[HBAgentParkourAutoJumpState::PARAM_TYPE] = HBAgentParkourAutoJumpState::AUTOJUMP_TO_STATE;
			transition_dict[HBAgentParkourAutoJumpState::PARAM_TARGET_STATE_ARGS] = ledge_state_args;
			transition_dict[HBAgentParkourAutoJumpState::PARAM_TARGET_STATE_NAME] = ASN()->wall_parkour_state;
			autojump_state->set_autojump_data(data);
			state_machine->transition_to(ASN()->autojump_state, transition_dict);
			return true;
		}
	}

	if (!autojump_state) {
		return false;
	}
	if (p_dir.length_squared() == 0) {
		return false;
	}
	ParkourAutojump::AutojumpSettings settings;
	settings.needs_gap = p_needs_gap;

	ParkourAutojump::AgentParkourAutojumpData data = settings.find_autojump(
			get_agent(),
			p_dir.normalized(),
			get_debug_geometry());

	if (data.found) {
		autojump_state->set_autojump_data(data);
		state_machine->transition_to(ASN()->autojump_state);
		return true;
	}

	Vector3 from = get_agent()->get_global_position();
	Vector3 to = get_agent()->get_global_position() + p_dir * 3.0f;
	Transform3D out_edge_trf;
	if (!find_lege_wall_sweep(from, to, Vector3(0.0f, -1.0f, 0.0f), out_edge_trf)) {
		return false;
	}
	Dictionary args;
	HBAgentLedgeGrabbedState *ledge_grabbed_state = Object::cast_to<HBAgentLedgeGrabbedState>(state_machine->get_state(ASN()->ledge_grabbed_state));
	Transform3D ledge_agent_trf = ledge_grabbed_state->get_ledge_agent_trf(out_edge_trf);

	data = settings.find_autojump_from_to(get_agent(), from, ledge_agent_trf.origin);
	if (!data.found) {
		args[HBAgentParkourAutoJumpState::PARAM_LEDGE_TRF] = out_edge_trf;
		args[HBAgentParkourAutoJumpState::PARAM_TYPE] = HBAgentParkourAutoJumpState::AUTOJUMP_LEDGE;

		autojump_state->set_autojump_data(data);
		state_machine->transition_to(ASN()->autojump_state, args);
		return true;
	}

	PhysicsDirectSpaceState3D::ShapeParameters shape_params;

	Ref<CylinderShape3D> cylinder_shape;
	cylinder_shape.instantiate();
	cylinder_shape->set_height(3.0f);
	cylinder_shape->set_radius(0.25f);
	shape_params.transform.origin = get_agent()->get_global_position();
	shape_params.transform.origin += p_dir * (cylinder_shape->get_height() * 0.5f);
	shape_params.transform.basis = Quaternion(Vector3(0.0f, 1.0f, 0.0f), p_dir);
	shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
	shape_params.shape_rid = cylinder_shape->get_rid();

	const int MAX_RESULTS = 8;
	Vector<PhysicsDirectSpaceState3D::ShapeResult> shape_results;
	shape_results.resize(8);

	PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();

	HBAgentParkourPoint *target_point = nullptr;

	get_debug_geometry()->debug_cast_motion(cylinder_shape, shape_params);
	for (int i = 0; i < dss->intersect_shape(shape_params, shape_results.ptrw(), MAX_RESULTS); i++) {
		HBAgentParkourPoint *point = Object::cast_to<HBAgentParkourPoint>(shape_results[i].collider);

		if (!point) {
			continue;
		}

		Vector3 point_normal;

		if (point_normal.dot(p_dir) >= 0.0f) {
			continue;
		}

		target_point = point;
		break;
	}

	if (target_point) {
		Dictionary transition_dict;
		transition_dict[HBAgentWallParkourState::PARAM_TARGET_PARKOUR_NODE] = target_point;
		state_machine->transition_to(ASN()->wall_parkour_state, transition_dict);
		return true;
	}

	return false;
}

bool HBAgentState::handle_autojump_down(Vector3 p_dir, bool p_needs_gap) {
	if (get_agent()->is_on_floor() && _try_slide_edge(p_dir)) {
		return true;
	}

	if (p_dir.length_squared() == 0) {
		return false;
	}

	ParkourAutojump::AutojumpSettings settings;
	settings.needs_gap = p_needs_gap;
	settings.ray_down_offset = get_agent()->get_height() * -1.0;

	HBAgentParkourAutoJumpState *autojump_state = Object::cast_to<HBAgentParkourAutoJumpState>(state_machine->get_state(ASN()->autojump_state));
	DEV_ASSERT(autojump_state);

	Vector3 from = get_agent()->get_global_position();
	Vector3 to = get_agent()->get_global_position() + p_dir * 3.0f;
	Transform3D out_edge_trf;

	if (find_lege_wall_sweep(from, to, Vector3(0.0f, -1.0f, 0.0f), out_edge_trf)) {
		Dictionary args;
		HBAgentLedgeGrabbedState *ledge_grabbed_state = Object::cast_to<HBAgentLedgeGrabbedState>(state_machine->get_state(ASN()->ledge_grabbed_state));
		Transform3D ledge_agent_trf = ledge_grabbed_state->get_ledge_agent_trf(out_edge_trf);

		ParkourAutojump::AgentParkourAutojumpData data = settings.find_autojump_from_to(get_agent(), from, ledge_agent_trf.origin);
		if (data.found) {
			args[HBAgentParkourAutoJumpState::PARAM_LEDGE_TRF] = out_edge_trf;
			args[HBAgentParkourAutoJumpState::PARAM_TYPE] = HBAgentParkourAutoJumpState::AUTOJUMP_LEDGE;

			autojump_state->set_autojump_data(data);
			state_machine->transition_to(ASN()->autojump_state, args);
			return true;
		}
	}

	ParkourAutojump::AgentParkourAutojumpData data = settings.find_autojump(
			get_agent(),
			p_dir.normalized(),
			get_debug_geometry());
	if (!data.found) {
		return false;
	}
	autojump_state->set_autojump_data(data);
	state_machine->transition_to(ASN()->autojump_state);
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

void HBAgentState::debug_ui_draw() {
#ifdef DEBUG_ENABLED
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
#endif
}

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
	get_inertialization_node()->inertialize(p_args.get(MoveStateParams::PARAM_TRANSITION_DURATION, 0.2f));
	get_agent()->inertialize_graphics_rotation(get_graphics_node()->get_global_transform().basis, true);
	get_agent()->connect("stopped_at_edge", callable_mp(this, &HBAgentMoveState::_on_agent_edge_hit));
	Ref<EPASLookatNode> torso_lookat_node = get_epas_controller()->get_epas_node("TorsoLookAt");
	Ref<EPASLookatNode> head_lookat_node = get_epas_controller()->get_epas_node("HeadLookAt");
	torso_lookat_node->set_influence(0.0f);
	head_lookat_node->set_influence(0.0f);
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
}

void HBAgentMoveState::physics_process(float p_delta) {
	ZoneScopedN("HBAgentMoveState physics process");
	HBAgent *agent = get_agent();
	Ref<EPASIKNode> left_foot_ik_node = get_epas_controller()->get_epas_node(ASN()->left_foot_ik_node);
	Ref<EPASIKNode> right_foot_ik_node = get_epas_controller()->get_epas_node(ASN()->right_foot_ik_node);

	left_foot_ik_node->set_ik_influence(1.0f);
	right_foot_ik_node->set_ik_influence(1.0f);

	bool is_parkour_down_held = agent->is_action_pressed(HBAgent::INPUT_ACTION_PARKOUR_DOWN);
	bool is_parkour_up_held = agent->is_action_pressed(HBAgent::INPUT_ACTION_PARKOUR_UP);
	bool is_run_held = agent->is_action_pressed(HBAgent::INPUT_ACTION_RUN);
	debug_draw_clear();
	Vector3 parkour_dir = agent->get_desired_movement_input_transformed().normalized();

	if (parkour_dir.is_normalized() && agent->is_at_edge(parkour_dir)) {
		if (is_parkour_down_held && handle_autojump_down(parkour_dir)) {
			return;
		} else if ((is_parkour_down_held || is_run_held) && _handle_parkour_mid()) {
			return;
		}
	}
	if (is_parkour_down_held) {
		if (_try_vault_over_obstacle()) {
			return;
		}
	} else if (is_parkour_up_held) {
		if (_handle_parkour_up()) {
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
			get_inertialization_node()->inertialize(0.25f);
		}
	} else {
		if (torso_lookat_node->get_influence() == 1.0f) {
			get_inertialization_node()->inertialize(0.5f);
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
			//if (movement_input.angle_to(curve_forward) > Math::deg_to_rad(90.0f) && movement_input.angle_to(-curve_forward) > Math::deg_to_rad(90.0f)) {
			if (movement_input.angle_to(get_agent()->get_global_position().direction_to(curve_sample.origin)) > Math::deg_to_rad(90.0f)) {
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
	Dictionary args;
	Transform3D temp_trf;

	temp_trf.origin = vault_base_near;
	temp_trf.basis = Basis().looking_at(-movement_input_dir);
	args[SNAME("VaultBaseNear")] = temp_trf;
	temp_trf.origin = vault_edge_near;
	args[SNAME("VaultEdgeNear")] = temp_trf;

	temp_trf.basis = Basis().looking_at(-movement_input_dir);
	temp_trf.origin = vault_edge_far;
	args[SNAME("VaultEdgeFar")] = temp_trf;
	temp_trf.origin = vault_base_far;
	args[SNAME("VaultBaseFar")] = temp_trf;

	state_machine->transition_to(ASN()->vault_state, args);

	return true;
}

bool HBAgentMoveState::_handle_parkour_up() {
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(agent == nullptr, false);

	// wallrun constants
	const float max_ledge_height = agent->get_agent_constants()->get_parkour_max_ledge_height();
	const float floor_max_angle = agent->get_floor_max_angle();

	Transform3D base_trf;

	PhysicsDirectSpaceState3D::RayResult ray_result;
	// Find a wall in front of us
	if (!find_facing_wall(ray_result)) {
		return false;
	}

	Transform3D ledge_trf;
	base_trf.origin = ray_result.position;
	base_trf.origin.y = agent->get_global_position().y;
	base_trf.basis = Basis::looking_at(ray_result.normal);

	if (find_ledge(ray_result.position, ray_result.normal, ledge_trf)) {
		if (ledge_trf.origin.y - agent->get_global_position().y <= 0.7f) {
			// Short hop up
			Dictionary transition_dict;
			transition_dict[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MovementTransitionInputs::MOVEMENT_LEDGE_HOP_UP;
			transition_dict[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = "HopUp";
			transition_dict[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->move_state;

			Vector3 dir_to_player = base_trf.origin.direction_to(agent->get_global_position());
			dir_to_player.y = 0.0f;
			dir_to_player.normalize();

			if (dir_to_player != Vector3(0.0f, 0.0f, -1.0f)) {
				base_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), dir_to_player);
			}
			ledge_trf.basis = base_trf.basis;

			Dictionary warp_points;
			warp_points[StringName("base")] = base_trf;
			warp_points[StringName("edge")] = ledge_trf;

			transition_dict[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;

			state_machine->transition_to(ASN()->root_motion_state, transition_dict);
		} else {
			// We found a ledge, so climb up to it
			Dictionary transition_dict;
			transition_dict[HBAgentWallrunState::WallrunParams::PARAM_BASE] = base_trf;
			transition_dict[HBAgentWallrunState::WallrunParams::PARAM_EDGE] = ledge_trf;
			transition_dict[HBAgentWallrunState::WallrunParams::PARAM_WALLRUN_TYPE] = HBAgentWallrunState::TO_LEDGE;
			state_machine->transition_to(ASN()->wallrun_state, transition_dict);
		}

		return true;
	}

	Vector3 base = base_trf.origin;
	Vector3 wall_normal = base_trf.basis.xform(Vector3(0.0, 0.0, -1.0f));

	// We didn't find any ledge to grab onto, let's try to find some parkour nodes
	PhysicsDirectSpaceState3D::ShapeParameters shape_params;
	PhysicsDirectSpaceState3D::ShapeResult shape_result;

	Ref<CylinderShape3D> cylinder_shape;
	cylinder_shape.instantiate();
	cylinder_shape->set_height(max_ledge_height);
	cylinder_shape->set_radius(0.25f);

	shape_params.shape_rid = cylinder_shape->get_rid();
	shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
	shape_params.transform.origin = base;
	shape_params.transform.origin.y += max_ledge_height * 0.5f;

	debug_draw_shape(cylinder_shape, shape_params.transform.origin);

	const int MAX_RESULTS = 32;
	Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
	results.resize(MAX_RESULTS);

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
	int result_count = dss->intersect_shape(shape_params, results.ptrw(), MAX_RESULTS);

	HBAgentParkourPoint *target_point = nullptr;

	Vector3 wallrun_top_position = base;
	wallrun_top_position.y += max_ledge_height;

	for (int i = 0; i < result_count; i++) {
		HBAgentParkourPoint *point = Object::cast_to<HBAgentParkourPoint>(results[i].collider);

		if (!point) {
			continue;
		}

		Vector3 point_forward = point->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
		// Ensure the parkour point points roughly in the same direction
		if (point_forward.angle_to(wall_normal) > Math::deg_to_rad(15.0f)) {
			continue;
		}
		// We don't have any parkour points yet to compare distances, so just use this one
		if (!target_point) {
			target_point = point;
			continue;
		}

		float curr_dist = target_point->get_global_position().distance_to(wallrun_top_position);
		float point_dist = point->get_global_position().distance_to(wallrun_top_position);
		target_point = point_dist < curr_dist ? point : target_point;
	}

	if (target_point) {
		Dictionary transition_dict;
		Transform3D temp_trf;
		temp_trf.basis = Basis().looking_at(wall_normal);
		temp_trf.origin = base;
		transition_dict[HBAgentWallrunState::WallrunParams::PARAM_BASE] = temp_trf;
		transition_dict[HBAgentWallrunState::WallrunParams::PARAM_EDGE] = target_point->get_global_transform();
		transition_dict[HBAgentWallrunState::WallrunParams::PARAM_WALLRUN_TYPE] = HBAgentWallrunState::TO_PARKOUR_POINT;
		transition_dict[HBAgentWallrunState::WallrunParams::PARAM_TARGET_PARKOUR_NODE] = target_point;
		state_machine->transition_to(ASN()->wallrun_state, transition_dict);
		return true;
	}

	// Nothing to grab onto, try to do an empty wallrun

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.from = base;
	ray_params.from.y += max_ledge_height;
	ray_params.to = ray_params.from;
	ray_params.to -= wall_normal;
	ray_params.from += wall_normal;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

	debug_draw_raycast(ray_params, Color("GREEN"));

	if (!dss->intersect_ray(ray_params, ray_result)) {
		return false;
	}
	if (!is_wall(wall_normal, floor_max_angle)) {
		return false;
	}

	Dictionary transition_dict;
	Transform3D temp_trf;
	temp_trf.basis = Basis().looking_at(wall_normal);
	temp_trf.origin = base;
	transition_dict[HBAgentWallrunState::WallrunParams::PARAM_BASE] = temp_trf;
	temp_trf.origin = ray_result.position;
	transition_dict[HBAgentWallrunState::WallrunParams::PARAM_EDGE] = temp_trf;
	transition_dict[HBAgentWallrunState::WallrunParams::PARAM_WALLRUN_TYPE] = HBAgentWallrunState::EMPTY_CLIMB;
	state_machine->transition_to(ASN()->wallrun_state, transition_dict);

	return true;
}

bool HBAgentMoveState::_handle_parkour_mid() {
	Vector3 dir = get_agent()->get_desired_movement_input_transformed().normalized();
	return handle_autojump_mid(dir, true);
}

/**********************
	VAULT STATE
***********************/

void HBAgentVaultState::_on_animation_finished() {
	Vector3 pos = animation_node->get_root_motion_starting_transform().xform(animation_node->get_root_motion_transform().origin);
	get_agent()->apply_root_motion(animation_node, 0.0f);
	get_agent()->set_velocity((pos - prev_position).normalized() * get_agent()->get_agent_constants()->get_max_move_velocity());
	state_machine->transition_to(ASN()->move_state);
}

void HBAgentVaultState::enter(const Dictionary &p_args) {
	Ref<EPASTransitionNode> transition_node = get_movement_transition_node();
	EPASController *epas_controller = get_epas_controller();
	ERR_FAIL_COND(epas_controller == nullptr);
	ERR_FAIL_COND(!transition_node.is_valid());

	transition_node->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_VAULT);
	animation_node = epas_controller->get_epas_node("Vault");
	if (animation_node.is_valid()) {
		animation_node->set_root_motion_forward(Vector3(0.0f, 0.0, 1.0f));
		animation_node->play_with_warp_points(p_args);
		animation_node->connect("playback_finished", callable_mp(this, &HBAgentVaultState::_on_animation_finished), CONNECT_ONE_SHOT);
	}

	get_agent()->set_movement_mode(HBAgent::MOVE_MANUAL);
	get_agent()->set_velocity(Vector3());
}

void HBAgentVaultState::process(float p_delta) {
	if (animation_node.is_valid()) {
		if (animation_node->is_playing()) {
			get_agent()->apply_root_motion(animation_node, p_delta);
			prev_position = animation_node->get_root_motion_starting_transform().xform(animation_node->get_root_motion_transform().origin);
		}
	}
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
	Wallrun state
***********************/

void HBAgentWallrunState::enter(const Dictionary &p_args) {
	ERR_FAIL_COND(!p_args.has(WallrunParams::PARAM_BASE));
	ERR_FAIL_COND(!p_args.has(WallrunParams::PARAM_EDGE));
	wallrun_type = p_args.get(WallrunParams::PARAM_WALLRUN_TYPE, WallrunType::EMPTY_CLIMB);
	parkour_point_target = Object::cast_to<StaticBody3D>(p_args.get(WallrunParams::PARAM_TARGET_PARKOUR_NODE, Variant()));
	autojump_queued = false;
	Ref<EPASTransitionNode> transition_node = get_movement_transition_node();
	Skeleton3D *skel = get_skeleton();
	EPASController *epas_controller = get_epas_controller();
	ERR_FAIL_COND(epas_controller == nullptr);
	ERR_FAIL_COND(skel == nullptr);
	ERR_FAIL_COND(!transition_node.is_valid());

	animation_node = epas_controller->get_epas_node("Wallrun");
	ERR_FAIL_COND(!animation_node.is_valid());

	transition_node->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_WALLRUN);
	animation_node->set_root_motion_starting_transform(skel->get_global_transform());
	if (animation_node.is_valid()) {
		animation_node->set_warp_point_transform("WallrunBase", p_args.get(WallrunParams::PARAM_BASE, Transform3D()));
		ledge_transform = p_args.get(WallrunParams::PARAM_EDGE, Transform3D());
		animation_node->set_warp_point_transform("WallrunEdge", ledge_transform);
		animation_node->set_root_motion_forward(Vector3(0.0f, 0.0, 1.0f));
		animation_node->play();
	}

	get_agent()->set_movement_mode(HBAgent::MOVE_MANUAL);
	get_agent()->set_velocity(Vector3());
	get_agent()->root_motion_begin(animation_node, get_physics_process_delta_time());

	if (!dbg) {
		dbg = memnew(EPASOneshotAnimationNodeDebug);
		add_child(dbg);
		dbg->set_animation_node(animation_node);
	}
}

void HBAgentWallrunState::physics_process(float p_delta) {
	get_agent()->apply_root_motion(animation_node, p_delta);
	debug_draw_clear();
	Vector3 movement_input = get_graphics_node()->get_global_transform().basis.xform(get_agent()->get_movement_input());

	if (get_agent()->is_action_just_pressed(HBAgent::INPUT_ACTION_PARKOUR_UP)) {
		if (movement_input.length_squared() > 0) {
			autojump_dir = movement_input.normalized();
		} else {
			autojump_dir = ledge_transform.basis.xform(Vector3(0.0, 0.0, -1.0f));
		}
		autojump_queued = true;
	}

	if (animation_node->is_playing()) {
		return;
	}

	if (autojump_queued) {
		if (handle_autojump_mid(autojump_dir) || handle_autojump_down(autojump_dir)) {
			return;
		}
	}

	switch (wallrun_type) {
		case WallrunType::EMPTY_CLIMB: {
			state_machine->transition_to(ASN()->fall_state);
		} break;
		case WallrunType::TO_LEDGE: {
			Dictionary args;
			args[HBAgentLedgeGrabbedState::WallGrabbedParams::PARAM_LEDGE_TRF] = ledge_transform;
			state_machine->transition_to(ASN()->ledge_grabbed_state, args);
		} break;
		case WallrunType::TO_PARKOUR_POINT: {
			Dictionary transition_dict;
			transition_dict[HBAgentWallParkourState::WallParkourParams::PARAM_TARGET_PARKOUR_NODE] = parkour_point_target;
			state_machine->transition_to(ASN()->wall_parkour_state, transition_dict);
		};
	}
	//dbg->update();
}

/**********************
	Wallgrabbed State
***********************/

// Returns the target grab point's transform
Transform3D HBAgentLedgeGrabbedState::_get_ledge_point_target_trf(const Transform3D &p_graphics_trf, const Vector3 &p_limb_position_world, const Vector3 &p_wall_normal) const {
	Transform3D trf;
	trf.origin = p_limb_position_world;
	Ref<EPASAnimationNode> animation_node = get_epas_controller()->get_epas_node("WallGrabbed");

	Ref<EPASAnimation> animation = animation_node->get_animation();
	Ref<EPASPose> reference_pose = animation->get_keyframe(0)->get_pose();
	trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), p_wall_normal);

	return trf;
}

Transform3D HBAgentLedgeGrabbedState::_get_limb_ik_target_trf(const Transform3D &p_graphics_trf, const Transform3D &p_ledge_trf, const Transform3D &p_bone_base_trf, const Vector3 &p_bone_offset) const {
	Transform3D skel_global_trf = p_graphics_trf * (get_graphics_node()->get_global_transform().affine_inverse() * get_skeleton()->get_global_transform());
	Vector3 bone_offset = skel_global_trf.basis.xform(p_bone_offset);
	Transform3D bone_target_trf;
	bone_target_trf.origin = p_ledge_trf.origin + bone_offset;
	bone_target_trf.basis = skel_global_trf.basis * p_bone_base_trf.basis;
	return bone_target_trf;
}

Transform3D HBAgentLedgeGrabbedState::get_ledge_agent_trf(Transform3D p_world_ledge_trf) const {
	EPASController *epas_controller = get_epas_controller();
	Ref<EPASAnimationNode> animation_node = epas_controller->get_epas_node("WallGrabbed");
	Ref<EPASAnimation> animation = animation_node->get_animation();
	Ref<EPASPose> reference_pose = animation->get_keyframe(0)->get_pose();
	Ref<EPASWarpPoint> ledge_warp_point = animation->get_warp_point(animation->find_warp_point("Ledge"));

	Transform3D target_skeleton_trf_global;
	Transform3D wp_trf = ledge_warp_point->get_transform();
	// We use skeleton space 0,0,0 to calculate the local transform of the origin skeleton in respect to the warp point
	// then, we use that local transform to find the new world transform of the skeleton in world space
	Transform3D root_trf = reference_pose->calculate_bone_global_transform("root", get_skeleton(), epas_controller->get_base_pose());

	Transform3D local_root_trf = target_skeleton_trf_global = wp_trf.affine_inverse() * root_trf;

	target_skeleton_trf_global = p_world_ledge_trf * local_root_trf;
	Vector3 skel_forward = target_skeleton_trf_global.basis.xform(Vector3(0.0f, 0.0f, 1.0f));
	target_skeleton_trf_global.basis = Quaternion(skel_forward, -skel_forward);
	return target_skeleton_trf_global;
}

bool HBAgentLedgeGrabbedState::find_initial_pose(LedgeAgentIKPose &p_agent_pose, const WallGrabbedStateInitialPoseParams &p_params) const {
	AgentIKPose ik_pose;
	Transform3D world_ledge_trf = p_params.ledge_transform;

	EPASController *epas_controller = get_epas_controller();

	Ref<EPASAnimationNode> animation_node = epas_controller->get_epas_node("WallGrabbed");
	Ref<EPASAnimation> animation = animation_node->get_animation();
	Ref<EPASPose> reference_pose = animation->get_keyframe(0)->get_pose();
	Ref<EPASWarpPoint> ledge_warp_point = animation->get_warp_point(animation->find_warp_point("Ledge"));

	Skeleton3D *skel = get_skeleton();

	Transform3D target_skeleton_trf_global;
	{
		Transform3D wp_trf = ledge_warp_point->get_transform();
		// We use skeleton space 0,0,0 to calculate the local transform of the origin skeleton in respect to the warp point
		// then, we use that local transform to find the new world transform of the skeleton in world space
		target_skeleton_trf_global = wp_trf.affine_inverse() * Transform3D();
		target_skeleton_trf_global = world_ledge_trf * target_skeleton_trf_global;
	}

	Transform3D target_graphics_node_trf_global;
	target_graphics_node_trf_global.origin = target_skeleton_trf_global.origin;
	// Calculate the current transform of the graphics node relative to the skeleton
	// then, use this information to figure out the new graphics node transform relative to the target skeleton transform
	target_graphics_node_trf_global = skel->get_global_transform().affine_inverse() * get_graphics_node()->get_global_transform();
	target_graphics_node_trf_global = target_skeleton_trf_global * target_graphics_node_trf_global;

	if (ledge_ik_points.size() == 0) {
		// Lazily init IK if it isn't initialized already
		const_cast<HBAgentLedgeGrabbedState *>(this)->_init_ik_points();
	}

	// Now do our typical pose calculation routine

	Quaternion offset_rot = Quaternion(Vector3(0.0f, 0.0f, 1.0f), world_ledge_trf.basis.xform(Vector3(0.0f, 0.0f, 1.0f)));
	Vector3 new_target_agent_position = world_ledge_trf.origin + offset_rot.xform(animation_root_offset);

	target_graphics_node_trf_global.origin = new_target_agent_position;
	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		ik_pose.dangling[i] = false;
		// Setup magnet, first we calculate the elbow/knee position in skeleton space
		Vector3 magnet = reference_pose->calculate_bone_global_transform(skel->get_bone_name(skel->get_bone_parent(ledge_ik_points[i].bone_idx)), skel, epas_controller->get_base_pose()).origin;
		magnet.y += ledge_ik_points[i].raycast_type == RAYCAST_FOOT ? 1.5f : -1.5f;
		if (ledge_ik_points[i].raycast_type == RAYCAST_FOOT) {
			magnet.x += SIGN(magnet.x);
		}
		ik_pose.magnet_positions[i] = magnet;
		// Setup initial IK position
		Vector3 from = target_graphics_node_trf_global.xform(ledge_ik_points[i].raycast_origin);
		Vector3 to = target_graphics_node_trf_global.xform(ledge_ik_points[i].raycast_target);
		bool found_point;
		Vector3 position, wall_normal, ledge_normal;
		if (ledge_ik_points[i].raycast_type == LedgeIKPointRaycastType::RAYCAST_HAND) {
			Vector3 sweep_offset = Vector3(0.0f, get_agent()->get_height(), 0.0f);
			found_point = _find_ledge_sweep(from, to, position, wall_normal, ledge_normal, ledge_ik_points[i].debug_color, sweep_offset);
		} else {
			found_point = _find_wall_point(from, to, position, wall_normal, ledge_ik_points[i].debug_color);
			ik_pose.dangling[i] = !found_point;
		}
		if (ledge_ik_points[i].raycast_type != LedgeIKPointRaycastType::RAYCAST_FOOT) {
			ERR_FAIL_COND_V_MSG(!found_point, false, vformat("Failed to find initial point for limb %d", i));
		}
		p_agent_pose.ledge_transforms[i] = _get_ledge_point_target_trf(target_graphics_node_trf_global.basis, position, wall_normal);
		p_agent_pose.wall_normals[i] = wall_normal;
		p_agent_pose.ledge_normals[i] = ledge_normal;
		ik_pose.target_transforms[i] = _get_limb_ik_target_trf(target_graphics_node_trf_global, p_agent_pose.ledge_transforms[i], ledge_ik_points[i].bone_base_trf, ledge_ik_points[i].target_offset);
	}

	ik_pose.actor_transform = target_graphics_node_trf_global;
	p_agent_pose.pose = ik_pose;

	return true;
}

bool HBAgentLedgeGrabbedState::_find_ledge(const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_wall_normal, Vector3 &p_out_ledge_normal, const Color &p_debug_color) const {
	HBAgent *agent = get_agent();

	ERR_FAIL_COND_V(!agent, false);

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	PhysicsDirectSpaceState3D::RayResult ray_result;

	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
	ERR_FAIL_COND_V(!dss, false);

	Vector3 pos = p_from;
	Vector3 pos_target = p_to;

	// First we find the wall by firing towards it

	ray_params.to = pos_target;
	ray_params.from = pos;

	if (ik_debug_info.show_limb_raycasts) {
		const_cast<HBAgentLedgeGrabbedState *>(this)->debug_draw_raycast(ray_params, p_debug_color);
	}

	if (!dss->intersect_ray(ray_params, ray_result)) {
		return false;
	}

	if (!is_wall(ray_result.normal, agent->get_floor_max_angle())) {
		return false;
	}

	Vector3 wall_normal = ray_result.normal;

	Vector3 ledge_point = ray_result.position;

	// Then we try to find the ledge top point

	ray_params.to = ray_result.position - ray_result.normal * 0.01;
	ray_params.from = ray_params.to + Vector3(0.0f, 1.75f, 0.0f);

	if (ik_debug_info.show_limb_raycasts) {
		const_cast<HBAgentLedgeGrabbedState *>(this)->debug_draw_raycast(ray_params, p_debug_color);
	}
	if (!dss->intersect_ray(ray_params, ray_result)) {
		return false;
	}

	if (!is_floor(ray_result.normal, agent->get_floor_max_angle())) {
		return false;
	}

	p_out_wall_normal = wall_normal;
	p_out_ledge_normal = ray_result.normal;

	p_out = ray_result.position;
	ledge_point.y = ray_result.position.y;

	p_out = ledge_point;

	return true;
}

bool HBAgentLedgeGrabbedState::_find_ledge_sweep(const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_wall_normal, Vector3 &p_out_ledge_normal, const Color &p_debug_color, Vector3 p_sweep_offset, int p_sweep_iterations) const {
	for (int i = 0; i < p_sweep_iterations; i++) {
		Vector3 offset = p_sweep_offset * (i / ((float)p_sweep_iterations - 1.0f));

		bool result = _find_ledge(p_from + offset, p_to + offset, p_out, p_out_wall_normal, p_out_ledge_normal, p_debug_color);
		if (result) {
			return true;
		}
	}

	return false;
}

bool HBAgentLedgeGrabbedState::_find_wall_point(const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_normal, const Color &p_debug_color) const {
	HBAgent *agent = get_agent();

	ERR_FAIL_COND_V(!agent, false);

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	PhysicsDirectSpaceState3D::RayResult ray_result;

	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
	ERR_FAIL_COND_V(!dss, false);

	Vector3 pos = p_from;
	Vector3 pos_target = p_to;

	// First we find the wall by firing towards it

	ray_params.to = pos_target;
	ray_params.from = pos;

	if (ik_debug_info.show_limb_raycasts) {
		const_cast<HBAgentLedgeGrabbedState *>(this)->debug_draw_raycast(ray_params, p_debug_color);
	}

	if (!dss->intersect_ray(ray_params, ray_result)) {
		return false;
	}

	if (!is_wall(ray_result.normal, agent->get_floor_max_angle())) {
		return false;
	}

	p_out = ray_result.position;
	p_out_normal = ray_result.normal;

	return true;
}

void HBAgentLedgeGrabbedState::_init_ik_points() {
	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);

	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);

	EPASController *epas_controller = get_epas_controller();
	ERR_FAIL_COND(!epas_controller);

	// Not very pretty but lets initialize IK points
	struct ik_points {
		String bone_name;
		StringName epas_node_name;
	} ik_points[AgentIKLimbType::LIMB_TYPE_MAX];

	Vector3 hand_raycast_origin = Vector3(0.0f, agent->get_height() * 0.25f, 0.5f);
	Vector3 hand_raycast_target_base = Vector3(0.0f, agent->get_height() * 0.25f, -1.0f);

	// Left hand
	ik_points[AgentIKLimbType::HAND_LEFT].bone_name = "hand.L";
	ik_points[AgentIKLimbType::HAND_LEFT].epas_node_name = "LeftHandIK";

	// Right hand
	ik_points[AgentIKLimbType::HAND_RIGHT].bone_name = "hand.R";
	ik_points[AgentIKLimbType::HAND_RIGHT].epas_node_name = "RightHandIK";

	Ref<EPASAnimationNode> animation_node = epas_controller->get_epas_node("WallGrabbed");
	ERR_FAIL_COND(!animation_node.is_valid());
	Ref<EPASAnimation> animation = animation_node->get_animation();
	ERR_FAIL_COND(!animation.is_valid());
	ERR_FAIL_COND(animation->get_keyframe_count() == 0);

	Transform3D foot_trf = animation->get_keyframe(0)->get_pose()->calculate_bone_global_transform("foot.R", skel, get_epas_controller()->get_base_pose());
	Transform3D foot_knee_trf = animation->get_keyframe(0)->get_pose()->calculate_bone_global_transform("shin.R", skel, get_epas_controller()->get_base_pose());
	Transform3D hand_elbow_trf = animation->get_keyframe(0)->get_pose()->calculate_bone_global_transform("hand.R", skel, get_epas_controller()->get_base_pose());

	animation_root_offset = calculate_animation_root_offset();

	foot_trf.rotate(Vector3(0.0f, 1.0f, 0.0f), Math::deg_to_rad(180.0f));

	// Skeleton forward is +1.0 instead of -z for blender reasons, so we correct for that
	Vector3 foot_trg = foot_trf.origin;
	foot_trg.z -= 1.0f;

	// Left foot
	ik_points[AgentIKLimbType::FOOT_LEFT].bone_name = "foot.L";
	ik_points[AgentIKLimbType::FOOT_LEFT].epas_node_name = "LeftFootIK";

	// Right foot
	ik_points[AgentIKLimbType::FOOT_RIGHT].bone_name = "foot.R";
	ik_points[AgentIKLimbType::FOOT_RIGHT].epas_node_name = "RightFootIK";

	ledge_ik_points.resize_zeroed(AgentIKLimbType::LIMB_TYPE_MAX);

	Ref<EPASPose> reference_pos = animation->get_keyframe(0)->get_pose();
	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		ledge_ik_points.write[i].bone_idx = skel->find_bone(ik_points[i].bone_name);
		ledge_ik_points.write[i].ik_node = epas_controller->get_epas_node(ik_points[i].epas_node_name);
		ERR_FAIL_COND_MSG(ledge_ik_points[i].bone_idx == -1, vformat("Error getting bone %s from skeleton.", ik_points[i].bone_name));
		ERR_FAIL_COND(!ledge_ik_points[i].ik_node.is_valid());
		ledge_ik_points.write[i].target_offset = Vector3(0.0f, 0.0f, -0.05f);
		ledge_ik_points.write[i].raycast_origin = hand_raycast_origin;

		// Calculate the bone's base transform
		StringName bone_name = ik_points[i].bone_name;
		ledge_ik_points.write[i].bone_base_trf = reference_pos->calculate_bone_global_transform(bone_name, skel, epas_controller->get_base_pose());
	}

	// Left Hand
	ledge_ik_points.write[AgentIKLimbType::HAND_LEFT].raycast_target = hand_raycast_target_base + Vector3(-0.2f, 0.0f, 0.0f);
	ledge_ik_points.write[AgentIKLimbType::HAND_LEFT].raycast_origin += Vector3(-0.2f, 0.0f, 0.0f);
	ledge_ik_points.write[AgentIKLimbType::HAND_LEFT].debug_color = Color("RED");
	ledge_ik_points.write[AgentIKLimbType::HAND_LEFT].start_time = 0.5f;
	ledge_ik_points.write[AgentIKLimbType::HAND_LEFT].end_time = 0.9f;
	ledge_ik_points.write[AgentIKLimbType::HAND_LEFT].raycast_type = LedgeIKPointRaycastType::RAYCAST_HAND;
	ledge_ik_points.write[AgentIKLimbType::HAND_LEFT].magnet_position = hand_elbow_trf.origin + Vector3(0.0f, -1.0f, 0.0f);
	ledge_ik_points.write[AgentIKLimbType::HAND_LEFT].magnet_position.x *= -1.0f;

	// Right Hand
	ledge_ik_points.write[AgentIKLimbType::HAND_RIGHT].raycast_target = hand_raycast_target_base + Vector3(0.2f, 0.0f, 0.0f);
	ledge_ik_points.write[AgentIKLimbType::HAND_RIGHT].raycast_origin += Vector3(0.2f, 0.0f, 0.0f);
	ledge_ik_points.write[AgentIKLimbType::HAND_RIGHT].debug_color = Color("GREEN");
	ledge_ik_points.write[AgentIKLimbType::HAND_RIGHT].start_time = 0.0f;
	ledge_ik_points.write[AgentIKLimbType::HAND_RIGHT].end_time = 0.4f;
	ledge_ik_points.write[AgentIKLimbType::HAND_RIGHT].raycast_type = LedgeIKPointRaycastType::RAYCAST_HAND;
	ledge_ik_points.write[AgentIKLimbType::HAND_RIGHT].magnet_position = hand_elbow_trf.origin + Vector3(0.0f, -1.0f, 0.0f);

	// Left Foot
	ledge_ik_points.write[AgentIKLimbType::FOOT_LEFT].raycast_target = foot_trg * Vector3(-1.0f, 1.0f, 1.0f);
	ledge_ik_points.write[AgentIKLimbType::FOOT_LEFT].raycast_origin = foot_trg * Vector3(-1.0f, 1.0f, 1.0f);
	ledge_ik_points.write[AgentIKLimbType::FOOT_LEFT].raycast_origin.z = 0.5f;
	ledge_ik_points.write[AgentIKLimbType::FOOT_LEFT].debug_color = Color("BLACK");
	ledge_ik_points.write[AgentIKLimbType::FOOT_LEFT].start_time = 0.6f;
	ledge_ik_points.write[AgentIKLimbType::FOOT_LEFT].end_time = 1.0f;
	ledge_ik_points.write[AgentIKLimbType::FOOT_LEFT].raycast_type = LedgeIKPointRaycastType::RAYCAST_FOOT;
	ledge_ik_points.write[AgentIKLimbType::FOOT_LEFT].target_offset = Vector3(0.0f, 0.0f, -0.15f);
	ledge_ik_points.write[AgentIKLimbType::FOOT_LEFT].magnet_position = foot_knee_trf.origin * Vector3(-1.0f, 1.0f, 1.0f);

	// Right foot
	ledge_ik_points.write[AgentIKLimbType::FOOT_RIGHT].raycast_origin = foot_trg;
	ledge_ik_points.write[AgentIKLimbType::FOOT_RIGHT].raycast_origin.z = 0.5f;
	ledge_ik_points.write[AgentIKLimbType::FOOT_RIGHT].raycast_target = foot_trg;
	ledge_ik_points.write[AgentIKLimbType::FOOT_RIGHT].debug_color = Color("WHITE");
	ledge_ik_points.write[AgentIKLimbType::FOOT_RIGHT].start_time = 0.1f;
	ledge_ik_points.write[AgentIKLimbType::FOOT_RIGHT].end_time = 0.5f;
	ledge_ik_points.write[AgentIKLimbType::FOOT_RIGHT].raycast_type = LedgeIKPointRaycastType::RAYCAST_FOOT;
	ledge_ik_points.write[AgentIKLimbType::FOOT_RIGHT].target_offset = Vector3(0.0f, 0.0f, -0.15f);
	ledge_ik_points.write[AgentIKLimbType::FOOT_RIGHT].magnet_position = foot_knee_trf.origin;
}

bool HBAgentLedgeGrabbedState::_handle_getup() {
	Node3D *gn = get_graphics_node();
	ERR_FAIL_COND_V(!gn, false);
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(!agent, false);

	Vector3 origin = ledge_ik_points[HAND_LEFT].raycast_origin + ledge_ik_points[HAND_RIGHT].raycast_origin;
	origin *= 0.5f;
	origin.x = 0.0f;
	origin = gn->get_global_transform().xform(origin);

	Vector3 target = ledge_ik_points[HAND_LEFT].raycast_target + ledge_ik_points[HAND_RIGHT].raycast_target;
	target *= 0.5f;
	target.x = 0.0f;
	target = gn->get_global_transform().xform(target);

	Vector3 ledge_position, wall_normal, ledge_normal;
	// We need to find the center of both hands, this is because we don't have any information on the
	// height of the ledge at the middle, only at the sides
	if (!_find_ledge_sweep(origin, target, ledge_position, wall_normal, ledge_normal, Color("purple"), Vector3(0.0f, agent->get_height(), 0.0f))) {
		// No ledge found? abort
		return false;
	}

	// Now we find the position where the character will end up after getup is finished
	PhysicsDirectSpaceState3D::RayParameters ray_params;
	PhysicsDirectSpaceState3D::RayResult ray_result;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	ray_params.from = ledge_position + gn->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -0.01f));
	ray_params.from.y += 0.5f;
	ray_params.to = ray_params.from;
	ray_params.to.y -= 1.0f;

	debug_draw_raycast(ray_params, Color("purple"));
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
	if (!agent->get_world_3d()->get_direct_space_state()->cast_motion(shape_params, closest_safe, closest_unsafe)) {
		// ??? This should never happen
		ERR_FAIL_V_MSG(false, "Something clearly went wrong with the getup logic, call a programmer");
	}

	Vector3 safe_pos = shape_params.transform.origin + shape_params.motion * closest_safe - Vector3(0.0f, agent->get_height() * 0.5f, 0.0f);

	Dictionary warp_points;
	Transform3D temp_trf;
	// root motion basis forward is reversed
	temp_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), wall_normal);
	temp_trf.origin = ledge_position;

	warp_points[StringName("Ledge")] = temp_trf;
	temp_trf.origin = safe_pos;
	warp_points[StringName("GetUpTarget")] = temp_trf;

	Dictionary args;
	args[HBAgentRootMotionState::PARAM_WARP_POINTS] = warp_points;
	args[HBAgentRootMotionState::PARAM_ANIMATION_NODE_NAME] = "LedgeGetUp";
	args[HBAgentRootMotionState::PARAM_TRANSITION_NODE_INDEX] = HBAgentConstants::MovementTransitionInputs::MOVEMENT_LEDGE_GETUP;
	args[HBAgentRootMotionState::PARAM_NEXT_STATE] = ASN()->move_state;
	args[HBAgentRootMotionState::PARAM_VELOCITY_MODE] = HBAgentRootMotionState::VelocityMode::ANIMATION_DRIVEN;

	ray_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
	ray_params.collide_with_areas = true;
	ray_params.collide_with_bodies = false;
	debug_draw_raycast(ray_params, Color("ORANGE"));

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
		}
	}

	state_machine->transition_to(ASN()->root_motion_state, args);
	return true;
}

void HBAgentLedgeGrabbedState::_debug_init_settings() {
	if (!ik_debug_info.ui_config_init) {
		ik_debug_info.show_center_raycast = GodotImGui::get_singleton()->get_config_value(state_machine, String(get_name()) + "/show_center_raycast", false);
		ik_debug_info.show_limb_raycasts = GodotImGui::get_singleton()->get_config_value(state_machine, String(get_name()) + "/show_limb_raycasts", false);
		ik_debug_info.ui_config_init = true;
	}
}

bool HBAgentLedgeGrabbedState::_handle_transition_inputs() {
	// Handles inputs that transition to other states, such as getup or dropping
	HBAgent *agent = get_agent();

	bool both_feet_dangling = ledge_ik_points[AgentIKLimbType::FOOT_LEFT].is_dangling || ledge_ik_points[AgentIKLimbType::FOOT_RIGHT].is_dangling;
	if (!both_feet_dangling) {
		// Back eject is only possible when both feet aren't dangling
		if (agent->is_action_just_released(HBAgent::INPUT_ACTION_PARKOUR_UP)) {
			Vector3 target_back_dir = animation_direction == 1 ? ledge_ik_points[AgentIKLimbType::HAND_RIGHT].wall_normal : ledge_ik_points[AgentIKLimbType::HAND_LEFT].wall_normal;
			Quaternion target_rot = Quaternion(Vector3(0.0f, 0.0f, 1.0f), target_back_dir);
			Vector3 input_transformed = agent->get_desired_movement_input_transformed().normalized();
			Vector3 autojump_dir = target_back_dir;
			if (input_transformed.length_squared() > 0) {
				Vector3 right = target_rot.xform(Vector3(-1.0f, 0.0f, 0.0f));
				Vector3 sideways_project = input_transformed.project(right);
				Vector3 backwards_project = input_transformed.project(target_back_dir);

				if (sideways_project.length() > backwards_project.length()) {
					autojump_dir = sideways_project.normalized();
				} else {
					autojump_dir = backwards_project.normalized();
				}
			}
			DEV_ASSERT(autojump_dir.length_squared() > 0);
			if (handle_autojump_down(autojump_dir)) {
				return true;
			} else if (handle_autojump_mid(autojump_dir)) {
				return true;
			}
		}
	}

	// Handle getting up from the ledge
	if (agent->get_movement_input().length_squared() > 0 && agent->get_movement_input().angle_to(Vector3(0.0, 0.0, -1.0f)) < Math::deg_to_rad(45.0f)) {
		if (_handle_getup()) {
			return true;
		}
	}

	// Handle trying to grab to parkour nodes below
	if (agent->get_movement_input().length_squared() > 0 && agent->get_movement_input().angle_to(Vector3(0.0f, 0.0f, 1.0f)) < Math::deg_to_rad(15.0f)) {
		Ref<CylinderShape3D> cylinder_shape;
		cylinder_shape.instantiate();
		cylinder_shape->set_height(agent->get_height());
		cylinder_shape->set_radius(0.5f);

		PhysicsDirectSpaceState3D::ShapeParameters params;
		params.transform.origin = agent->get_global_position();
		params.transform.origin.y += agent->get_height() * 0.5f;

		const int RESULTS_MAX = 5;

		Vector<PhysicsDirectSpaceState3D::ShapeResult> shape_results;
		shape_results.resize(RESULTS_MAX);
		PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
		params.shape_rid = cylinder_shape->get_rid();
		int results = dss->intersect_shape(params, shape_results.ptrw(), RESULTS_MAX);
		HBAgentParkourPoint *target_point = nullptr;
		float target_point_dist_to_agent = 0.0f;

		debug_draw_cast_motion(cylinder_shape, params);

		Vector3 agent_top = agent->get_global_position() + Vector3(0.0f, agent->get_height(), 0.0f);

		for (int i = 0; i < results; i++) {
			HBAgentParkourPoint *point = Object::cast_to<HBAgentParkourPoint>(shape_results[i].collider);
			if (!point) {
				continue;
			}
			float point_dist_to_agent = agent_top.distance_to(point->get_global_position());
			if (!target_point) {
				target_point = point;
				target_point_dist_to_agent = point_dist_to_agent;
				continue;
			}
			if (point_dist_to_agent < target_point_dist_to_agent) {
				target_point = point;
				target_point_dist_to_agent = point_dist_to_agent;
			}
		}

		// We found a node, time to transition
		if (target_point) {
			Dictionary transition_dict;
			transition_dict[HBAgentWallParkourState::WallParkourParams::PARAM_TARGET_PARKOUR_NODE] = target_point;
			state_machine->transition_to(ASN()->wall_parkour_state, transition_dict);
			return true;
		}
	}
	if (agent->is_action_pressed(HBAgent::AgentInputAction::INPUT_ACTION_PARKOUR_DOWN)) {
		state_machine->transition_to(ASN()->fall_state);
		return true;
	}
	return false;
}

void HBAgentLedgeGrabbedState::enter(const Dictionary &p_args) {
	_debug_init_settings();
	debug_draw_clear();
	ERR_FAIL_COND(!p_args.has(PARAM_LEDGE_TRF));

	Ref<EPASTransitionNode> movement_transition = get_movement_transition_node();
	ERR_FAIL_COND(!movement_transition.is_valid());

	movement_transition->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_WALLGRABBED);
	// Force inertialization to ocurr
	get_inertialization_node()->inertialize(0.25f);

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);

	EPASController *epas_controller = get_epas_controller();

	ERR_FAIL_COND(!epas_controller);

	Node3D *gn = get_graphics_node();
	ERR_FAIL_COND(!gn);

	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);

	if (ledge_ik_points.size() == 0) {
		_init_ik_points();
	}
	Transform3D ledge_trf = p_args.get(PARAM_LEDGE_TRF, Transform3D());

	Transform3D target_gn_trf = gn->get_global_transform();
	target_gn_trf.origin = target_agent_position;

	LedgeAgentIKPose initial_pose;
	WallGrabbedStateInitialPoseParams params;
	params.ledge_transform = ledge_trf;

	ERR_FAIL_COND(!find_initial_pose(initial_pose, params));

	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		// Setup magnet
		ledge_ik_points.write[i].ik_node->set_use_hinge(true);
		ledge_ik_points.write[i].ik_node->set_magnet_position(skel->to_global(initial_pose.pose.magnet_positions[i]));
		ledge_ik_points.write[i].ik_node->set_use_magnet(true);

		// Setup initial IK transform
		ledge_ik_points.write[i].ik_node->set_ik_influence(1.0f);
		ledge_ik_points.write[i].position = initial_pose.ledge_transforms[i].origin;
		ledge_ik_points.write[i].target_position = initial_pose.ledge_transforms[i].origin;
		ledge_ik_points.write[i].ik_node->set_target_transform(initial_pose.pose.target_transforms[i]);
		ledge_ik_points.write[i].wall_normal = initial_pose.wall_normals[i];
		ledge_ik_points.write[i].ledge_normal = initial_pose.ledge_normals[i];
		ledge_ik_points.write[i].is_dangling = initial_pose.pose.dangling[i];
	}

	Ref<EPASBlendNode> dangle_blend_node = get_epas_controller()->get_epas_node("WallDangleBlend");
	dangle_blend_node->set_blend_amount(0.0f);
	if (ledge_ik_points[AgentIKLimbType::FOOT_LEFT].is_dangling || ledge_ik_points[AgentIKLimbType::FOOT_LEFT].is_dangling) {
		TypedArray<StringName> dangling_bones;
		get_skeleton()->set_animate_physical_bones(true);
		if (ledge_ik_points[AgentIKLimbType::FOOT_LEFT].is_dangling) {
			dangling_bones.push_back("thigh.L");
			dangling_bones.push_back("shin.L");
			dangling_bones.push_back("foot.L");
			ledge_ik_points[FOOT_LEFT].ik_node->set_ik_influence(0.0f);
		}
		if (ledge_ik_points[AgentIKLimbType::FOOT_RIGHT].is_dangling) {
			dangling_bones.push_back("thigh.R");
			dangling_bones.push_back("shin.R");
			dangling_bones.push_back("foot.R");
			ledge_ik_points[FOOT_RIGHT].ik_node->set_ik_influence(0.0f);
		}
		dangle_blend_node->set_blend_amount(1.0f);
	}

	agent->inertialize_graphics_rotation(Quaternion(Vector3(0.0f, 0.0f, 1.0f), ledge_trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f))), true);

	// Remember, animations are +Z forward!!!
	// also, ledges point TOWARDS the player
	// which means that we have to use its +Z as the new forward vector of the graphics node
	target_agent_position = initial_pose.pose.actor_transform.origin;

	animation_time = 0.0f;
	ledge_movement_acceleration = 0.0f;
	ledge_movement_velocity = 0.0f;
}

void HBAgentLedgeGrabbedState::exit() {
	ledge_ik_points.write[AgentIKLimbType::FOOT_LEFT].ik_node->set_ik_influence(0.0f);
	ledge_ik_points.write[AgentIKLimbType::FOOT_RIGHT].ik_node->set_ik_influence(0.0f);
	ledge_ik_points.write[AgentIKLimbType::HAND_LEFT].ik_node->set_ik_influence(0.0f);
	ledge_ik_points.write[AgentIKLimbType::HAND_RIGHT].ik_node->set_ik_influence(0.0f);

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);
	skel->set_position(Vector3());
	skel->physical_bones_stop_simulation();
	get_epas_controller()->clear_ignored_bones();
}

void HBAgentLedgeGrabbedState::physics_process(float p_delta) {
	ZoneScopedN("HBAgentWallGrabbedState physics process");
	ERR_FAIL_COND(!get_inertialization_node().is_valid());
	if (!get_inertialization_node()->is_inertializing() && !inertialization_finished) {
		inertialization_finished = true;

		// Dangle feet if needded after inertialization is done
		if (ledge_ik_points[AgentIKLimbType::FOOT_LEFT].is_dangling || ledge_ik_points[AgentIKLimbType::FOOT_LEFT].is_dangling) {
			Ref<EPASBlendNode> dangle_blend_node = get_epas_controller()->get_epas_node("WallDangleBlend");
			TypedArray<StringName> dangling_bones;
			if (ledge_ik_points[AgentIKLimbType::FOOT_LEFT].is_dangling) {
				dangling_bones.push_back("thigh.L");
				dangling_bones.push_back("shin.L");
				dangling_bones.push_back("foot.L");
			}
			if (ledge_ik_points[AgentIKLimbType::FOOT_RIGHT].is_dangling) {
				dangling_bones.push_back("thigh.R");
				dangling_bones.push_back("shin.R");
				dangling_bones.push_back("foot.R");
			}
			dangle_blend_node->set_blend_amount(1.0f);
			get_skeleton()->physical_bones_start_simulation_on(dangling_bones);
		}
	}

	debug_draw_clear();
	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);
	debug_draw_sphere(agent->get_global_position(), 0.05f, Color("Yellow"));

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);

	if (_handle_transition_inputs()) {
		return;
	}

	// move the agent
	Node3D *gn = get_graphics_node();
	ERR_FAIL_COND(!gn);

	Vector3 forward = get_graphics_node()->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
	forward.y = 0.0f;
	forward.normalize();
	Plane plane = Plane(forward);
	Vector3 movement_input_transformed = Quaternion(Vector3(0.0f, 0.0f, -1.0f), forward).xform(agent->get_desired_movement_input());
	Vector3 lateral_movement_input = plane.project(movement_input_transformed);
	ik_debug_info.transformed_movement_input = Vector2(lateral_movement_input.x, lateral_movement_input.z);

	const float ledge_movement_max_vel = agent->get_agent_constants()->get_ledge_movement_velocity();

	float velocity_goal = gn->get_global_transform().basis.xform_inv(lateral_movement_input).x * ledge_movement_max_vel;

	if (SIGN(agent->get_movement_input().x) != 0) {
		AgentIKLimbType ledge_point_to_test = SIGN(agent->get_movement_input().x) == -1 ? AgentIKLimbType::HAND_LEFT : AgentIKLimbType::FOOT_RIGHT;
		Vector3 raycast_start = Vector3(0.5f * SIGN(agent->get_movement_input().x), agent->get_height() * 0.25f, 0.5f);
		Vector3 raycast_end = raycast_start + Vector3(0.0f, 0.0f, -1.5f);
		raycast_start = gn->get_global_transform().xform(raycast_start);
		raycast_end = gn->get_global_transform().xform(raycast_end);

		Vector3 out_position, wall_normal, ledge_normal;

		// If we are moving in a direction we can't get to we should start slowing down
		if (!_find_ledge_sweep(raycast_start, raycast_end, out_position, wall_normal, ledge_normal, ledge_ik_points[ledge_point_to_test].debug_color, Vector3(0.0f, get_agent()->get_height(), 0.0f))) {
			velocity_goal = 0.0f;
		}
	}

	HBSprings::velocity_spring(
			&ledge_movement_velocity,
			&ledge_movement_acceleration,
			velocity_goal,
			0.175f,
			p_delta);

	bool is_moving = !Math::is_zero_approx(ledge_movement_velocity);

	if (animation_time == 0.0f && is_moving) {
		// Set the animation direction when initiating a move
		animation_direction = SIGN(ledge_movement_velocity);
	}

	if (is_moving || animation_time > 0.0f) {
		animation_time += p_delta + (Math::abs(ledge_movement_velocity) * 1.25f * p_delta);
	}

	// When the animation ends if we are still moving, wrap around, otherwise set it to 0
	if (animation_time >= 1.0f) {
		if (is_moving) {
			float prev_anim_time = animation_time;
			animation_time = Math::fposmod(animation_time, 1.0f);
			//  We wrapped around, we can update the animation direction
			if (animation_time < prev_anim_time) {
				animation_direction = SIGN(ledge_movement_velocity);
			}
		} else {
			animation_time = 0.0f;
		}
		// Always apply the target positions
		for (int i = 0; i < ledge_ik_points.size(); i++) {
			ledge_ik_points.write[i].position = ledge_ik_points[i].target_position;
		}
	}

	Vector3 prev_agent_pos = agent->get_global_position();

	/*if (animation_direction != 0) {
		Vector3 curr_graphics_forward = gn->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
		Vector3 new_graphics_forward = animation_direction == 1 ? ledge_ik_points[AgentIKLimbType::HAND_RIGHT].wall_normal : ledge_ik_points[AgentIKLimbType::HAND_LEFT].wall_normal;
		new_graphics_forward *= -1.0f;
		float angle = curr_graphics_forward.angle_to(new_graphics_forward);
		Vector3 axis = curr_graphics_forward.cross(new_graphics_forward).normalized();

		if (axis.is_normalized()) {
			float angle_mul = Math::abs(ledge_movement_velocity) / ledge_movement_max_vel;
			new_graphics_forward = curr_graphics_forward.rotated(axis, (angle * p_delta * angle_mul));
		}

		Transform3D gn_trf_target = gn->get_global_transform();
		gn_trf_target.origin = target_agent_position;

		Transform3D gn_trf = gn->get_global_transform();
		gn_trf.basis = Quaternion(curr_graphics_forward, new_graphics_forward) * gn_trf.basis.get_rotation_quaternion();
		gn->set_global_transform(gn_trf);

		Vector3 average_pos = ledge_ik_points[AgentIKLimbType::HAND_RIGHT].position + ledge_ik_points[AgentIKLimbType::HAND_LEFT].position;
		average_pos *= 0.5f;

		PhysicsDirectSpaceState3D::RayParameters ray_params;
		ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
		Vector3 ray_dir = gn_trf_target.origin;
		ray_dir.y = average_pos.y;
		ray_dir = ray_dir.direction_to(average_pos);

		ray_params.from = gn_trf_target.origin - new_graphics_forward;
		ray_params.to = gn_trf_target.origin + new_graphics_forward * 2.0f;

		PhysicsDirectSpaceState3D::RayResult ray_result;

		PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

		// Try to aim graphics forward
		debug_draw_raycast(ray_params);
		}
	}*/

	static struct {
		Vector3 position;
		Vector3 wall_normal;
		Vector3 ledge_normal;
	} new_ledge_info[AgentIKLimbType::LIMB_TYPE_MAX];

	// First, we figure out if any ledge doesn't have a potential target ledge position, this means
	// that we moved into a side that doesn't have more corner for us to grab to
	Transform3D predicted_gn_trf = gn->get_global_transform();
	predicted_gn_trf.origin += target_agent_position - agent->get_global_position();
	if (is_moving) {
		for (int i = 0; i < ledge_ik_points.size(); i++) {
			LedgeIKLimb &ik_point = ledge_ik_points.write[i];
			Vector3 new_position;
			Vector3 wall_normal;
			Vector3 ledge_normal;
			Vector3 from = predicted_gn_trf.xform(ik_point.raycast_origin);
			Vector3 to = predicted_gn_trf.xform(ik_point.raycast_target);

			bool result;

			if (ik_point.raycast_type == LedgeIKPointRaycastType::RAYCAST_FOOT) {
				result = _find_wall_point(from, to, new_position, wall_normal, ledge_ik_points[i].debug_color);
				if (!result) {
					continue;
				}
			} else {
				Vector3 sweep_offset = Vector3(0.0f, get_agent()->get_height(), 0.0f);
				result = _find_ledge_sweep(from, to, new_position, wall_normal, ledge_normal, ledge_ik_points[i].debug_color, sweep_offset);
			}
			if (!result) {
				// Ledge detection failed, which means we must stop moving and go back
				ledge_movement_acceleration = 0.0f;
				ledge_movement_velocity = 0.0f;
				target_agent_spring_velocity = Vector3();
				is_moving = false;
				agent->set_global_position(prev_agent_pos);
				break;
			} else {
				new_ledge_info[i].position = new_position;
				new_ledge_info[i].wall_normal = wall_normal;
				new_ledge_info[i].ledge_normal = ledge_normal;
				debug_draw_sphere(new_position, 0.05f, ik_point.debug_color);
			}
		}
	}

	// Average position of all IK positions
	Vector3 ik_position_avg;
	// Weights used for a weighted average of the ik positions
	// this is used to calculate the hip offset
	float weights[AgentIKLimbType::LIMB_TYPE_MAX];
	float total_weight = 0.0f;
	int non_dangling_limbs = 0;
	Vector3 ik_handle_positions[AgentIKLimbType::LIMB_TYPE_MAX];

	float anim_mul = Math::abs(ledge_movement_velocity) / ledge_movement_max_vel;
	AgentIKPose pose;
	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		LedgeIKLimb &ik_point = ledge_ik_points.write[i];
		weights[i] = 0.0f;
		if (ik_point.is_dangling) {
			continue;
		}
		non_dangling_limbs++;

		float start_time = ik_point.start_time;
		float end_time = ik_point.end_time;
		// Invert the animation if moving to the left
		if (animation_direction == -1) {
			start_time = 1.0f - start_time;
			end_time = 1.0f - end_time;
			SWAP(start_time, end_time);
		}

		float animation_pos = CLAMP(Math::inverse_lerp(start_time, end_time, animation_time), 0.0f, 1.0f);

		// The ik point only moves when we are within the animation, otherwise we make it stay in place
		if (is_moving && animation_time <= end_time && animation_time >= start_time) {
			ik_point.target_position = new_ledge_info[i].position;
			ik_point.wall_normal = new_ledge_info[i].wall_normal;
			ik_point.ledge_normal = new_ledge_info[i].ledge_normal;
		}
		Vector3 new_pos = ik_point.position.lerp(ik_point.target_position, animation_pos);
		new_pos.y += Math::sin(animation_pos * Math_PI) * 0.05f * anim_mul;

		Transform3D ledge_trf = _get_ledge_point_target_trf(predicted_gn_trf, new_pos, new_ledge_info[i].wall_normal);
		Transform3D ik_trf = _get_limb_ik_target_trf(gn->get_global_transform(), ledge_trf, ik_point.bone_base_trf, ik_point.target_offset);
		ledge_ik_points.write[i].ik_node->set_target_transform(ik_trf);

		ik_position_avg += ik_trf.origin;
		weights[i] = ik_point.get_weight(animation_pos);
		total_weight += weights[i];
		ik_handle_positions[i] = ik_trf.origin;
	}

	Vector3 ik_position_weighted; // The hip position after
	ik_position_avg /= (float)non_dangling_limbs;

	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		if (ledge_ik_points[i].is_dangling) {
			continue;
		}
		ik_position_weighted += ik_handle_positions[i] * (weights[i] / total_weight);
	}

	// Project into a plane that matches the wall normal
	ik_position_weighted = plane.project(ik_position_weighted);
	ik_position_avg = plane.project(ik_position_avg);

	// Move along the wall
	Vector3 movement_vector = Vector3(ledge_movement_velocity, 0.0f, 0.0f);
	Vector3 movement_dir = ledge_ik_points[AgentIKLimbType::HAND_LEFT].ledge_normal.cross(ledge_ik_points[AgentIKLimbType::HAND_LEFT].wall_normal).normalized();

	movement_vector = Quaternion(Vector3(1.0f, 0.0f, 0.0f), movement_dir).xform(movement_vector);
	target_agent_position = target_agent_position + movement_vector * p_delta;

	// Make the movement lag behind visually a bit, to make limbs look better
	Vector3 graphics_position = agent->get_global_position();
	HBSprings::critical_spring_damper_exact_vector3(graphics_position, target_agent_spring_velocity, target_agent_position, target_spring_halflife, p_delta);
	agent->set_global_position(graphics_position);

	float anim_mul_sq = anim_mul * anim_mul;
	Vector3 offset = skel->get_global_transform().basis.xform_inv(ik_position_weighted - ik_position_avg);
	offset.y = 0.0f;
	// TODO: make these magic numbers into constants
	Vector3 skel_pos = offset * anim_mul_sq * 0.4f;
	skel_pos.y = 0.2f * anim_mul_sq; // Perhaps these two should be using a spring to make them less sudden
	skel_pos.z = 0.1f * anim_mul_sq;
	skel->set_position(skel_pos);

	// Update debug info
	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		ik_debug_info.last_tip_weights[i] = weights[i];
	}
	ik_debug_info.last_hip_offset_x = skel_pos.x;
	ik_debug_info.physics_time_delta += p_delta;
	ik_debug_info.last_data_is_valid = true;

	for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
		Vector3 magnet = skel->to_global(ledge_ik_points[i].magnet_position);
		ledge_ik_points.write[i].ik_node->set_magnet_position(magnet);
	}
}

void HBAgentLedgeGrabbedState::debug_ui_draw() {
	HBAgentState::debug_ui_draw();

	_debug_init_settings();

	if (ImGui::Checkbox("Show limb raycasts", &ik_debug_info.show_limb_raycasts)) {
		GodotImGui::get_singleton()->set_config_value(state_machine, String(get_name()) + "/show_limb_raycasts", ik_debug_info.show_limb_raycasts);
	}
	if (ImGui::Checkbox("Show center raycast", &ik_debug_info.show_center_raycast)) {
		GodotImGui::get_singleton()->set_config_value(state_machine, String(get_name()) + "/show_center_raycast", ik_debug_info.show_center_raycast);
	}

	ImGui::Text("Animation time: %.2f", animation_time);
	ImGui::SliderFloat("Velocity spring halflife", &target_spring_halflife, 0.0f, 1.0f);

	GodotImGui::DrawJoystick(ik_debug_info.transformed_movement_input, 50.0f);

	if (ik_debug_info.last_data_is_valid) {
		ik_debug_info.plot_time += ik_debug_info.physics_time_delta;
		// Rollback timelines
		for (int i = 1; i < IKDebugInfo::IK_OFFSET_PLOT_SIZE; i++) {
			ik_debug_info.ik_hip_offset_graph[i - 1] = ik_debug_info.ik_hip_offset_graph[i];
			ik_debug_info.graph_x_time[i - 1] = ik_debug_info.graph_x_time[i];
			for (int j = 0; j < AgentIKLimbType::LIMB_TYPE_MAX; j++) {
				ik_debug_info.ik_tip_influence_y_graph[j][i - 1] = ik_debug_info.ik_tip_influence_y_graph[j][i];
			}
		}
		// Apply new values
		for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
			ik_debug_info.ik_tip_influence_y_graph[i][IKDebugInfo::IK_OFFSET_PLOT_SIZE - 1] = ik_debug_info.last_tip_weights[i];
		}
		ik_debug_info.ik_hip_offset_graph[IKDebugInfo::IK_OFFSET_PLOT_SIZE - 1] = ik_debug_info.last_hip_offset_x;
		ik_debug_info.graph_x_time[IKDebugInfo::IK_OFFSET_PLOT_SIZE - 1] = ik_debug_info.plot_time;

		// Reset state
		ik_debug_info.physics_time_delta = 0.0f;
		ik_debug_info.last_data_is_valid = false;
	}

	if (ImPlot::BeginPlot("Tip influences (weights)", ImVec2(-1, 200.0f), ImPlotFlags_CanvasOnly & ~(ImPlotFlags_NoLegend | ImPlotFlags_NoTitle))) {
		ImPlot::SetupAxes(nullptr, nullptr);
		ImPlot::SetupAxisLimits(ImAxis_X1, ik_debug_info.graph_x_time[0], ik_debug_info.graph_x_time[IKDebugInfo::IK_OFFSET_PLOT_SIZE - 1], ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, -1.1f, 1.1f);
		for (int i = 0; i < AgentIKLimbType::LIMB_TYPE_MAX; i++) {
			ImPlot::PlotLine(vformat("weight %d", i).utf8().get_data(), ik_debug_info.graph_x_time, ik_debug_info.ik_tip_influence_y_graph[i], IKDebugInfo::IK_OFFSET_PLOT_SIZE);
		}
		ImPlot::EndPlot();
	}
	if (ImPlot::BeginPlot("IK hip proc offset X", ImVec2(-1, 200.0f), ImPlotFlags_CanvasOnly & ~(ImPlotFlags_NoLegend | ImPlotFlags_NoTitle))) {
		ImPlot::SetupAxes(nullptr, nullptr);
		ImPlot::SetupAxisLimits(ImAxis_X1, ik_debug_info.graph_x_time[0], ik_debug_info.graph_x_time[IKDebugInfo::IK_OFFSET_PLOT_SIZE - 1], ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, -0.25f, 0.25f);
		ImPlot::PlotLine(vformat("x offset").utf8().get_data(), ik_debug_info.graph_x_time, ik_debug_info.ik_hip_offset_graph, IKDebugInfo::IK_OFFSET_PLOT_SIZE);
		ImPlot::EndPlot();
	}
}

Vector3 HBAgentLedgeGrabbedState::calculate_animation_root_offset() {
	EPASController *epas_controller = get_epas_controller();
	ERR_FAIL_COND_V(!epas_controller, Vector3());

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND_V(!skel, Vector3());

	Ref<EPASAnimationNode> animation_node = epas_controller->get_epas_node("WallGrabbed");
	ERR_FAIL_COND_V(!animation_node.is_valid(), Vector3());

	Ref<EPASAnimation> animation = animation_node->get_animation();
	ERR_FAIL_COND_V(!animation.is_valid(), Vector3());
	ERR_FAIL_COND_V(animation->get_keyframe_count() == 0, Vector3());
	Vector3 root_offset = animation->get_keyframe(0)->get_pose()->calculate_bone_global_transform("root", skel, epas_controller->get_base_pose()).origin;
	int warp_point_idx = animation->find_warp_point("Ledge");
	ERR_FAIL_COND_V(warp_point_idx == -1, Vector3());
	root_offset -= animation->get_warp_point(warp_point_idx)->get_transform().origin;
	return root_offset;
}

/**********************
	Fall State
***********************/

void HBAgentFallState::enter(const Dictionary &p_args) {
	Ref<EPASTransitionNode> movement_transition = get_movement_transition_node();
	ERR_FAIL_COND(!movement_transition.is_valid());

	movement_transition->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_FALL);

	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);
	agent->set_movement_mode(HBAgent::MovementMode::MOVE_FALL);
	agent->set_velocity(Vector3());
}

void HBAgentFallState::physics_process(float p_delta) {
	HBAgent *agent = get_agent();
	if (agent->is_on_floor()) {
		state_machine->transition_to(ASN()->move_state);
	}
}

/**********************
	Ledge Getup State
***********************/

void HBAgentLedgeGetUpState::_on_animation_finished() {
	Vector3 pos = animation_node->get_root_motion_starting_transform().xform(animation_node->get_root_motion_transform().origin);
	get_agent()->apply_root_motion(animation_node, 0.0f);
	Vector3 new_vel = (pos - prev_position);
	new_vel.y = 0.0f;
	new_vel.normalize();
	new_vel = new_vel * get_agent()->get_desired_movement_input_transformed().length() * get_agent()->get_agent_constants()->get_max_move_velocity();

	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);

	get_agent()->set_velocity(new_vel);
	state_machine->transition_to(target_state, target_state_args);
}

void HBAgentLedgeGetUpState::enter(const Dictionary &p_args) {
	EPASController *epas_controller = get_epas_controller();
	ERR_FAIL_COND(!epas_controller);

	Ref<EPASTransitionNode> movement_transition = get_movement_transition_node();
	ERR_FAIL_COND(!movement_transition.is_valid());

	animation_node = epas_controller->get_epas_node("LedgeGetUp");
	ERR_FAIL_COND(!animation_node.is_valid());

	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);

	agent->set_movement_mode(HBAgent::MOVE_MANUAL);

	movement_transition->transition_to(HBAgentConstants::MOVEMENT_LEDGE_GETUP);
	animation_node->play_with_warp_points(p_args);

	target_state = p_args.get(LedgeGetUpParams::TARGET_STATE, "Move");
	target_state_args = p_args.get(LedgeGetUpParams::TARGET_STATE_ARGS, Dictionary());

	animation_node->connect("playback_finished", callable_mp(this, &HBAgentLedgeGetUpState::_on_animation_finished), CONNECT_ONE_SHOT);
}

void HBAgentLedgeGetUpState::process(float p_delta) {
	if (animation_node->is_playing()) {
		get_agent()->apply_root_motion(animation_node, p_delta);
		prev_position = animation_node->get_root_motion_starting_transform().xform(animation_node->get_root_motion_transform().origin);
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

	get_softness_node()->set_influence(0.0f);

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

	debug_draw_sphere(agent->get_global_position(), 0.05f, Color("Yellow"));

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
	if (agent->is_action_just_released(HBAgent::INPUT_ACTION_PARKOUR_UP)) {
		Vector3 target_back_dir = graphics_rotation_spring_target.xform(Vector3(0.0f, 0.0f, 1.0f));
		Quaternion target_rot = Quaternion(Vector3(0.0f, 0.0f, 1.0f), target_back_dir);
		Vector3 input_transformed = agent->get_desired_movement_input_transformed().normalized();
		Vector3 autojump_dir = target_back_dir;
		if (input_transformed.length_squared() > 0) {
			Vector3 right = target_rot.xform(Vector3(-1.0f, 0.0f, 0.0f));
			Vector3 sideways_project = input_transformed.project(right);
			Vector3 backwards_project = input_transformed.project(target_back_dir);

			if (sideways_project.length() > backwards_project.length()) {
				autojump_dir = sideways_project.normalized();
			} else {
				autojump_dir = backwards_project.normalized();
			}
		}
		DEV_ASSERT(autojump_dir.length_squared() > 0);
		if (handle_autojump_down(autojump_dir)) {
			return;
		} else if (handle_autojump_mid(autojump_dir)) {
			return;
		}
	}

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
			args[HBAgentLedgeGrabbedState::PARAM_LEDGE_TRF] = target_ledge_trf;
			state_machine->transition_to(ASN()->ledge_grabbed_state, args);
		}
	}
}

void HBAgentWallParkourState::debug_ui_draw() {
	HBAgentState::debug_ui_draw();
	ImGui::InputFloat3("Left leg magnet pos", (float *)&parkour_limbs[AgentIKLimbType::FOOT_LEFT].local_magnet_pos.coord);
	ImGui::InputFloat3("Right leg magnet pos", (float *)&parkour_limbs[AgentIKLimbType::FOOT_RIGHT].local_magnet_pos.coord);
}

void HBAgentWallParkourState::_notification(int p_what) {
}

HBAgentWallParkourState::HBAgentWallParkourState() {
}

void HBAgentParkourAutoJumpState::set_autojump_data(ParkourAutojump::AgentParkourAutojumpData p_autojump_data) {
	autojump_data = p_autojump_data;
}

void HBAgentParkourAutoJumpState::enter(const Dictionary &p_args) {
	type = (AutoJumpType)(int)p_args.get((int)AutoJumpParams::PARAM_TYPE, (int)AutoJumpType::AUTOJUMP_WORLD);
	Vector3 dir = autojump_data.start.direction_to(autojump_data.end);
	dir.y = 0.0f;
	dir.normalize();
	if (type == AutoJumpType::AUTOJUMP_LEDGE) {
		ERR_FAIL_COND_MSG(!p_args.has(AutoJumpParams::PARAM_LEDGE_TRF), "When autojumping to a ledge you need to provider the ledge transform");
		ledge_trf = p_args.get(AutoJumpParams::PARAM_LEDGE_TRF, Transform3D());
		Quaternion target_rot = Quaternion(Vector3(0.0f, 0.0f, -1.0f), ledge_trf.basis.get_rotation_quaternion().xform(Vector3(0.0f, 0.0f, 1.0f)));
		get_agent()->inertialize_graphics_rotation(target_rot, true);
	} else if (dir.length_squared() > 0) {
		get_agent()->inertialize_graphics_rotation(Quaternion(Vector3(0.0f, 0.0f, -1.0f), dir), true);
	}
	target_state_args = p_args.get(PARAM_TARGET_STATE_ARGS, Dictionary());
	target_state_name = p_args.get(PARAM_TARGET_STATE_NAME, "");
	get_movement_transition_node()->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_FALL);
	time = 0.0f;
	get_agent()->set_movement_mode(HBAgent::MOVE_MANUAL);
}

void HBAgentParkourAutoJumpState::physics_process(float p_delta) {
	debug_draw_clear();
	time += p_delta;
	time = MIN(time, autojump_data.parabola_t_max);

	for (int i = 1; i < 20; i++) {
		float t = i / (20.0f - 1.0f);
		float t_prev = (i - 1) / (20.0f - 1.0f);
		t *= autojump_data.parabola_t_max;
		t_prev *= autojump_data.parabola_t_max;
		Vector3 pos = autojump_data.get_position(t);
		Vector3 pos_prev = autojump_data.get_position(t_prev);
		debug_draw_line(pos_prev, pos);
	}

	debug_draw_sphere(autojump_data.start);
	debug_draw_sphere(autojump_data.end);

	debug_draw_sphere(ledge_trf.origin, 0.1f, Color("GREEN"));

	Vector3 velocity = autojump_data.get_position(time) - autojump_data.get_position(time - p_delta);
	velocity /= p_delta;

	get_agent()->set_velocity(velocity);
	get_agent()->set_global_position(autojump_data.get_position(time));

	if (time < autojump_data.parabola_t_max) {
		return;
	}

	if (type == AutoJumpType::AUTOJUMP_LEDGE) {
		Dictionary args;
		args[HBAgentLedgeGrabbedState::PARAM_LEDGE_TRF] = ledge_trf;
		state_machine->transition_to(ASN()->ledge_grabbed_state, args);
		return;
	}

	if (type == AUTOJUMP_TO_STATE) {
		state_machine->transition_to(target_state_name, target_state_args);
		return;
	}

	Ref<Shape3D> col_shape = get_agent()->get_collision_shape();

	PhysicsDirectSpaceState3D *dss = get_agent()->get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::ShapeParameters shape_params;
	shape_params.shape_rid = col_shape->get_rid();
	shape_params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;
	shape_params.collide_with_bodies = false;
	shape_params.collide_with_areas = true;
	shape_params.transform.origin = get_agent()->get_global_position();
	const int MAX_RESULTS = 16;
	Vector<PhysicsDirectSpaceState3D::ShapeResult> shape_results;
	shape_results.resize(MAX_RESULTS);

	int results = dss->intersect_shape(shape_params, shape_results.ptrw(), MAX_RESULTS);

	for (int i = 0; i < results; i++) {
		HBAgentParkourBeam *beam = Object::cast_to<HBAgentParkourBeam>(shape_results[i].collider);
		if (!beam) {
			continue;
		}
		Dictionary beam_args;
		beam_args[HBAgentParkourBeamWalk::PARAM_BEAM_NODE] = beam;
		beam_args[HBAgentParkourBeamWalk::PARAM_PREV_POSITION] = get_agent()->get_previous_position();
		state_machine->transition_to(ASN()->beam_walk_state, beam_args);
		return;
	}

	state_machine->transition_to(ASN()->move_state);
}

void HBAgentParkourBeamWalk::enter(const Dictionary &p_args) {
	beam = Object::cast_to<HBAgentParkourBeam>(p_args.get(PARAM_BEAM_NODE, Variant()));
	Vector3 prev_position = p_args.get(PARAM_PREV_POSITION, Variant());

	DEV_ASSERT(beam != nullptr);
	DEV_ASSERT(beam->get_curve().is_valid());

	curve_offset = beam->get_curve()->get_closest_offset(beam->to_local(get_agent()->get_global_position()));

	Transform3D global_curve_trf = beam->get_global_transform() * beam->get_curve()->sample_baked_with_rotation(curve_offset);
	Vector3 global_curve_point = global_curve_trf.origin;

	pos_inertializer = pos_inertializer->create(prev_position, get_agent()->get_global_position(), global_curve_point, 1.0f, get_process_delta_time());
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
		if (handle_autojump_mid(movement_input, true)) {
			return;
		}
	}

	if (get_agent()->is_action_pressed(HBAgent::INPUT_ACTION_PARKOUR_DOWN)) {
		if (movement_input.length_squared() == 0 && try_ledge_drop()) {
			return;
		}
		if (!movement_goes_through_beam && handle_autojump_down(rotation_spring_goal.xform(Vector3(0.0f, 0.0f, -1.0f)))) {
			return;
		}
	}

	if (movement_input.length_squared() > 0) {
		if (movement_goes_through_beam) {
			desired_movement_input = movement_input.project(forward);
			desired_movement_input = desired_movement_input.normalized() * movement_input.length();
		}
	}

	Node3D *gn = get_graphics_node();

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
		return;
	}

	agent_global_position = beam->get_global_transform().xform(beam->get_curve()->sample_baked(curve_offset));

	Vector3 curr_agent_pos = agent_global_position;
	if (!pos_inertializer->is_done()) {
		curr_agent_pos += pos_inertializer->advance(p_delta);
	}

	Transform3D global_gn_trf = gn->get_global_transform();
	if (movement_input.length_squared() > 0) {
		Vector3 desired_look_dir = forward;
		desired_look_dir.rotate(Vector3(0.0f, 1.0f, 0.0f), Math::snapped(forward.normalized().signed_angle_to(movement_input.normalized(), Vector3(0.0f, 1.0f, 0.0f)), Math::deg_to_rad(90.0f)));
		if (desired_look_dir.length_squared() > 0) {
			rotation_spring_goal = Quaternion(Vector3(0.0f, 0.0f, -1.0f), desired_look_dir);
		}
	}

	Quaternion rot = global_gn_trf.basis;
	HBSprings::simple_spring_damper_exact_quat(rot, rotation_spring_vel, rotation_spring_goal, 0.05f, p_delta);
	global_gn_trf.basis = rot;

	gn->set_global_transform(global_gn_trf);

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
	ledge_args[HBAgentLedgeGrabbedState::PARAM_LEDGE_TRF] = ledge_trf;

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
		debug_draw_sphere(trf.origin);
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
}

void HBAgentRootMotionState::physics_process(float p_delta) {
	get_agent()->apply_root_motion(animation_node, p_delta);
	if (!animation_node->is_playing()) {
		switch (velocity_mode) {
			case ANIMATION_DRIVEN: {
				get_agent()->set_velocity((get_agent()->get_global_position() - prev_pos) / p_delta);
			} break;
			case CONSERVE: {
				get_agent()->set_velocity(starting_velocity);
			} break;
		}
		state_machine->transition_to(next_state, next_state_args);
	}
	prev_pos = get_agent()->get_global_position();
}
