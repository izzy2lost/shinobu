#include "agent_state.h"
#include "agent_parkour.h"
#include "animation_system/epas_animation_node.h"

#ifdef DEBUG_ENABLED
#include "implot.h"
#include "modules/imgui/godot_imgui.h"
#endif
#include "modules/tracy/tracy.gen.h"
#include "physics_layers.h"
#include "scene/resources/cylinder_shape_3d.h"
#include "scene/resources/sphere_shape_3d.h"
#include "springs.h"

HBDebugGeometry *HBAgentState::_get_debug_geo() {
	if (debug_geo == nullptr) {
		_init_ui_settings_if_needed();
		debug_geo = memnew(HBDebugGeometry);
		debug_geo->set_visible(draw_debug_geometry);
		debug_geo->set_as_top_level(true);
		add_child(debug_geo, false, INTERNAL_MODE_FRONT);
		debug_geo->set_position(Vector3());
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

Node3D *HBAgentState::get_graphics_node() {
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(!agent, nullptr);
	return agent->_get_graphics_node();
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
}

bool is_wall(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) > p_floor_max_angle;
}

bool is_floor(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) <= p_floor_max_angle;
}

void HBAgentMoveState::physics_process(float p_delta) {
	ZoneScopedN("HBAgentMoveState physics process");
	HBAgent *agent = get_agent();
	if (agent) {
		bool is_parkour_down_held = agent->is_action_pressed(HBAgent::INPUT_ACTION_PARKOUR_DOWN);
		bool is_parkour_up_held = agent->is_action_pressed(HBAgent::INPUT_ACTION_PARKOUR_UP);
		bool is_run_held = agent->is_action_pressed(HBAgent::INPUT_ACTION_RUN);
		debug_draw_clear();
		if (is_run_held) {
			if (is_parkour_down_held) {
				if (_handle_parkour_down()) {
					return;
				}
			} else if (is_parkour_up_held) {
				if (_handle_parkour_up()) {
					return;
				}
			}
		}

		Node3D *gn = get_graphics_node();
		Vector3 movement_input = agent->get_desired_movement_input_transformed();
		if (gn && movement_input.length() > 0.0f) {
			Vector3 forward = gn->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
			forward.y = 0.0f;
			forward.normalize();

			float turn_threshold = Math::deg_to_rad(agent->get_agent_constants()->get_turn_animation_threshold_degrees());
			if (movement_input.angle_to(forward) > turn_threshold && agent->get_linear_velocity().length() < 0.1f) {
				//state_machine->transition_to("Turn");
			}
		}
	}
}

/*
 * Vault points:
 *
 * Edge near --> |-----| <-- Edge far
 *               |     |
 *               |     |
 * Base near --> |-----| <-- Base far
 */
bool HBAgentMoveState::_handle_parkour_down() {
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(agent == nullptr, false);

	Vector3 movement_input_dir = agent->get_desired_movement_input().normalized();

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

	Ref<Shape3D> body_shape = agent->get_collision_shape();

	float mid_height = get_agent()->get_height() * 0.5f;
	float floor_max_angle = agent->get_floor_max_angle();

	// Vault constants
	const float vault_max_obstacle_width = agent->get_agent_constants()->get_vault_max_obstacle_width();

	PhysicsDirectSpaceState3D::RayResult ray_result;

	// Check if there's a wall in front of us
	if (!_check_wall(ray_result)) {
		return false;
	}

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

	Vector3 vault_base_near = ray_result.position;
	vault_base_near.y -= mid_height;

	// Find vaultable near edge
	ray_params.from = ray_result.position;
	ray_params.from += movement_input_dir * 0.01f;
	ray_params.from.y += mid_height * 0.5f;
	ray_params.to = ray_result.position + movement_input_dir * 0.01f;

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
	shape_query_params.transform.origin = vault_base_far * movement_input_dir * (agent->get_radius() + 0.5f);
	shape_query_params.collision_mask = ray_params.collision_mask;
	PhysicsDirectSpaceState3D::ShapeResult shape_query_result;

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

	state_machine->transition_to("Vault", args);

	return true;
}

bool HBAgentMoveState::_handle_parkour_up() {
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(agent == nullptr, false);

	// wallrun constants
	const float max_ledge_height = agent->get_agent_constants()->get_parkour_max_ledge_height();
	const float floor_max_angle = agent->get_floor_max_angle();

	PhysicsDirectSpaceState3D::RayResult ray_result;

	Ref<Shape3D> agent_shape = agent->get_collision_shape();

	// Check if we have a wall in front
	if (!_check_wall(ray_result)) {
		return false;
	}

	Vector3 wall_normal = ray_result.normal;

	Vector3 base = ray_result.position;
	base.y = agent->get_global_position().y;

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	ray_params.from = ray_result.position - ray_result.normal * 0.01f;
	ray_params.from.y += max_ledge_height;
	ray_params.to = ray_result.position - ray_result.normal * 0.01f;

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

	// Try to find a roof
	debug_draw_raycast(ray_params);
	bool found_ledge = true;
	if (!dss->intersect_ray(ray_params, ray_result)) {
		found_ledge = false;
	}

	// Make sure its a roof
	if (found_ledge && !is_floor(ray_result.normal, floor_max_angle)) {
		found_ledge = false;
	}

	if (found_ledge) {
		// We found a ledge, so climb up to it
		Vector3 edge = base;
		edge.y = ray_result.position.y;

		Dictionary transition_dict;
		Transform3D temp_trf;
		temp_trf.basis = Basis().looking_at(wall_normal);
		temp_trf.origin = base;
		transition_dict[HBAgentWallrunState::WallrunParams::PARAM_BASE] = temp_trf;
		temp_trf.origin = edge;
		transition_dict[HBAgentWallrunState::WallrunParams::PARAM_EDGE] = temp_trf;
		transition_dict[HBAgentWallrunState::WallrunParams::PARAM_WALLRUN_TYPE] = HBAgentWallrunState::TO_LEDGE;

		state_machine->transition_to("Wallrun", transition_dict);
		return true;
	}

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
		state_machine->transition_to("Wallrun", transition_dict);
		return true;
	}

	// Nothing to grab onto, try to do an empty wallrun
	ray_params.from += wall_normal;
	ray_params.to = ray_params.from - wall_normal * 1.5f;

	debug_draw_raycast(ray_params);

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
	state_machine->transition_to("Wallrun", transition_dict);

	return true;
}

bool HBAgentMoveState::_check_wall(PhysicsDirectSpaceState3D::RayResult &p_result) {
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(agent == nullptr, false);

	if (agent->get_desired_movement_input_transformed().length() == 0.0f) {
		return false;
	}

	const float parkour_wall_check_distance = agent->get_agent_constants()->get_parkour_wall_check_distance();
	const float parkour_max_wall_facing_angle_degrees = Math::deg_to_rad(agent->get_agent_constants()->get_parkour_max_wall_facing_angle_degrees());
	const float floor_max_angle = agent->get_floor_max_angle();

	Ref<Shape3D> player_shape = agent->get_collision_shape();
	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

	float mid_height = get_agent()->get_height() * 0.5f;

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

/**********************
	VAULT STATE
***********************/

void HBAgentVaultState::_on_animation_finished() {
	Vector3 pos = animation_node->get_root_motion_starting_transform().xform(animation_node->get_root_motion_transform().origin);
	get_agent()->apply_root_motion(animation_node);
	get_agent()->set_velocity((pos - prev_position).normalized() * get_agent()->get_agent_constants()->get_max_move_velocity());
	state_machine->transition_to("Move");
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
			get_agent()->apply_root_motion(animation_node);
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
		get_agent()->apply_root_motion(animation_node);
		Node3D *gn = get_graphics_node();
		if (gn) {
			float w = CLAMP(Math::inverse_lerp(0.0f, anim_length, time), 0.0f, 1.0f);
			gn->set_quaternion(starting_rot.slerp(target_rot, w));
		}
		if (!animation_node->is_playing()) {
			state_machine->transition_to("Move");
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
		animation_node->set_warp_point_transform("WallrunEdge", p_args.get(WallrunParams::PARAM_EDGE, Transform3D()));
		animation_node->set_root_motion_forward(Vector3(0.0f, 0.0, 1.0f));
		animation_node->play();
	}

	get_agent()->set_movement_mode(HBAgent::MOVE_MANUAL);
	get_agent()->set_velocity(Vector3());

	if (!dbg) {
		dbg = memnew(EPASOneshotAnimationNodeDebug);
		add_child(dbg);
		dbg->set_animation_node(animation_node);
	}
}

void HBAgentWallrunState::process(float p_delta) {
	get_agent()->apply_root_motion(animation_node);
	if (!animation_node->is_playing()) {
		switch (wallrun_type) {
			case WallrunType::EMPTY_CLIMB: {
				state_machine->transition_to("Fall");
			} break;
			case WallrunType::TO_LEDGE: {
				state_machine->transition_to("WallGrabbed");
			} break;
			case WallrunType::TO_PARKOUR_POINT: {
				Dictionary transition_dict;
				transition_dict[HBAgentWallParkourState::WallParkourParams::PARAM_TARGET_PARKOUR_NODE] = parkour_point_target;
				state_machine->transition_to("WallParkour", transition_dict);
			};
		}
	}
	//dbg->update();
}

/**********************
	Wallgrabbed State
***********************/

// Returns the actual target transform, including the offset to the bones
Transform3D HBAgentWallGrabbedState::_get_ledge_point_target_trf(int p_ledge_point, const Vector3 &p_position) {
	ERR_FAIL_INDEX_V(p_ledge_point, ledge_ik_points.size(), Transform3D());
	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND_V(!skel, Transform3D());
	Transform3D trf;
	Node3D *gn = get_graphics_node();
	ERR_FAIL_COND_V(!gn, Transform3D());
	trf.origin = p_position + gn->get_global_transform().basis.xform(ledge_ik_points[p_ledge_point].target_offset);
	trf.basis = skel->get_global_transform().basis * skel->get_bone_global_pose(skel->find_bone(ledge_ik_points[p_ledge_point].ik_node->get_ik_end())).basis;
	return trf;
}

bool HBAgentWallGrabbedState::_find_ledge(const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_wall_normal, Vector3 &p_out_ledge_normal, const Color &p_debug_color) {
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
		debug_draw_raycast(ray_params, p_debug_color);
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
	ray_params.from = ray_params.to + Vector3(0.0f, 0.75f, 0.0f);

	if (ik_debug_info.show_limb_raycasts) {
		debug_draw_raycast(ray_params, p_debug_color);
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

bool HBAgentWallGrabbedState::_find_wall_point(const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_normal, const Color &p_debug_color) {
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
		debug_draw_raycast(ray_params, p_debug_color);
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

void HBAgentWallGrabbedState::_init_ik_points() {
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
	} ik_points[LedgePoint::LEDGE_POINT_MAX];

	Vector3 hand_raycast_origin = Vector3(0.0f, agent->get_height() * 0.25f, 0.5f);
	Vector3 hand_raycast_target_base = Vector3(0.0f, agent->get_height() * 0.25f, -1.0f);

	// Left hand
	ik_points[LedgePoint::LEDGE_POINT_LEFT_HAND].bone_name = "hand.L";
	ik_points[LedgePoint::LEDGE_POINT_LEFT_HAND].epas_node_name = "LeftHandIK";

	// Right hand
	ik_points[LedgePoint::LEDGE_POINT_RIGHT_HAND].bone_name = "hand.R";
	ik_points[LedgePoint::LEDGE_POINT_RIGHT_HAND].epas_node_name = "RightHandIK";

	Ref<EPASAnimationNode> animation_node = epas_controller->get_epas_node("WallGrabbed");
	ERR_FAIL_COND(!animation_node.is_valid());
	Ref<EPASAnimation> animation = animation_node->get_animation();
	ERR_FAIL_COND(!animation.is_valid());
	ERR_FAIL_COND(animation->get_keyframe_count() == 0);

	Transform3D foot_trf = animation->get_keyframe(0)->get_pose()->calculate_bone_global_transform("foot.R", skel, get_epas_controller()->get_base_pose());
	Transform3D foot_knee_trf = animation->get_keyframe(0)->get_pose()->calculate_bone_global_transform("shin.R", skel, get_epas_controller()->get_base_pose());
	Transform3D hand_elbow_trf = animation->get_keyframe(0)->get_pose()->calculate_bone_global_transform("hand.R", skel, get_epas_controller()->get_base_pose());

	foot_trf.rotate(Vector3(0.0f, 1.0f, 0.0f), Math::deg_to_rad(180.0f));

	// Skeleton forward is +1.0 instead of -z for blender reasons, so we correct for that
	Vector3 foot_trg = foot_trf.origin;
	foot_trg.z -= 1.0f;

	// Left foot
	ik_points[LedgePoint::LEDGE_POINT_LEFT_FOOT].bone_name = "foot.L";
	ik_points[LedgePoint::LEDGE_POINT_LEFT_FOOT].epas_node_name = "LeftFootIK";

	// Right foot
	ik_points[LedgePoint::LEDGE_POINT_RIGHT_FOOT].bone_name = "foot.R";
	ik_points[LedgePoint::LEDGE_POINT_RIGHT_FOOT].epas_node_name = "RightFootIK";

	ledge_ik_points.resize_zeroed(LedgePoint::LEDGE_POINT_MAX);

	for (int i = 0; i < LedgePoint::LEDGE_POINT_MAX; i++) {
		ledge_ik_points.write[i].bone_idx = skel->find_bone(ik_points[i].bone_name);
		ledge_ik_points.write[i].ik_node = epas_controller->get_epas_node(ik_points[i].epas_node_name);
		ERR_FAIL_COND_MSG(ledge_ik_points[i].bone_idx == -1, vformat("Error getting bone %s from skeleton.", ik_points[i].bone_name));
		ERR_FAIL_COND(!ledge_ik_points[i].ik_node.is_valid());
		ledge_ik_points.write[i].ik_node->set_ik_influence(1.0f);
		ledge_ik_points.write[i].target_offset = Vector3(0.0f, 0.0f, 0.05f);
		ledge_ik_points.write[i].raycast_origin = hand_raycast_origin;
	}

	// Left Hand
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].raycast_target = hand_raycast_target_base + Vector3(-0.2f, 0.0f, 0.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].raycast_origin += Vector3(-0.2f, 0.0f, 0.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].debug_color = Color("RED");
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].start_time = 0.5f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].end_time = 0.9f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].raycast_type = LedgeIKPointRaycastType::RAYCAST_HAND;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].magnet_position = hand_elbow_trf.origin + Vector3(0.0f, -1.0f, 0.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].magnet_position.x *= -1.0f;

	// Right Hand
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].raycast_target = hand_raycast_target_base + Vector3(0.2f, 0.0f, 0.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].raycast_origin += Vector3(0.2f, 0.0f, 0.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].debug_color = Color("GREEN");
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].start_time = 0.0f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].end_time = 0.4f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].raycast_type = LedgeIKPointRaycastType::RAYCAST_HAND;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].magnet_position = hand_elbow_trf.origin + Vector3(0.0f, -1.0f, 0.0f);

	// Left Foot
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].raycast_target = foot_trg * Vector3(-1.0f, 1.0f, 1.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].raycast_origin = foot_trg * Vector3(-1.0f, 1.0f, 1.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].raycast_origin.z = 0.5f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].debug_color = Color("BLACK");
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].start_time = 0.6f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].end_time = 1.0f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].raycast_type = LedgeIKPointRaycastType::RAYCAST_FOOT;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].target_offset = Vector3(0.0f, 0.0f, 0.15f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].magnet_position = foot_knee_trf.origin * Vector3(-1.0f, 1.0f, 1.0f);

	// Right foot
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].raycast_origin = foot_trg;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].raycast_origin.z = 0.5f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].raycast_target = foot_trg;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].debug_color = Color("WHITE");
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].start_time = 0.1f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].end_time = 0.5f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].raycast_type = LedgeIKPointRaycastType::RAYCAST_FOOT;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].target_offset = Vector3(0.0f, 0.0f, 0.15f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].magnet_position = foot_knee_trf.origin;
}

bool HBAgentWallGrabbedState::_handle_getup() {
	Node3D *gn = get_graphics_node();
	ERR_FAIL_COND_V(!gn, false);
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(!agent, false);

	Vector3 origin = ledge_ik_points[LEDGE_POINT_LEFT_HAND].raycast_origin + ledge_ik_points[LEDGE_POINT_RIGHT_HAND].raycast_origin;
	origin *= 0.5f;
	origin.x = 0.0f;
	origin = gn->get_global_transform().xform(origin);

	Vector3 target = ledge_ik_points[LEDGE_POINT_LEFT_HAND].raycast_target + ledge_ik_points[LEDGE_POINT_RIGHT_HAND].raycast_target;
	target *= 0.5f;
	target.x = 0.0f;
	target = gn->get_global_transform().xform(target);

	Vector3 ledge_position, wall_normal, ledge_normal;
	// We need to find the center of both hands, this is because we don't have any information on the
	// height of the ledge at the middle, only at the sides
	if (!_find_ledge(origin, target, ledge_position, wall_normal, ledge_normal, Color("purple"))) {
		// No ledge found? abort
		return false;
	}

	// Now we find the position where the character will end up after getup is finished
	PhysicsDirectSpaceState3D::RayParameters ray_params;
	PhysicsDirectSpaceState3D::RayResult ray_result;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	ray_params.from = ledge_position + gn->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -0.25f));
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

	Dictionary args;
	Transform3D temp_trf;
	// root motion basis forward is reversed
	temp_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), wall_normal);
	temp_trf.origin = ledge_position;
	args[StringName("Ledge")] = temp_trf;
	temp_trf.origin = safe_pos;
	args[StringName("GetUpTarget")] = temp_trf;
	state_machine->transition_to("LedgeGetUp", args);
	return true;
}

void HBAgentWallGrabbedState::_debug_init_settings() {
	if (!ik_debug_info.ui_config_init) {
		ik_debug_info.show_center_raycast = GodotImGui::get_singleton()->get_config_value(state_machine, String(get_name()) + "/show_center_raycast", false);
		ik_debug_info.show_limb_raycasts = GodotImGui::get_singleton()->get_config_value(state_machine, String(get_name()) + "/show_limb_raycasts", false);
		ik_debug_info.ui_config_init = true;
	}
}

void HBAgentWallGrabbedState::enter(const Dictionary &p_args) {
	_debug_init_settings();
	debug_draw_clear();
	Ref<EPASTransitionNode> movement_transition = get_movement_transition_node();
	ERR_FAIL_COND(!movement_transition.is_valid());

	movement_transition->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_WALLGRABBED);

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

	for (int i = 0; i < LedgePoint::LEDGE_POINT_MAX; i++) {
		// Setup magnet
		Vector3 magnet = skel->get_bone_global_pose(skel->get_bone_parent(ledge_ik_points[i].bone_idx)).origin;
		magnet.y += ledge_ik_points[i].raycast_type == RAYCAST_FOOT ? 1.5f : -1.5f;
		if (ledge_ik_points[i].raycast_type == RAYCAST_FOOT) {
			magnet.x += SIGN(magnet.x);
		}
		ledge_ik_points.write[i].ik_node->set_use_hinge(true);
		ledge_ik_points.write[i].ik_node->set_magnet_position(skel->to_global(ledge_ik_points.write[i].magnet_position));
		ledge_ik_points.write[i].ik_node->set_use_magnet(true);

		// Setup initial IK position
		Vector3 from = gn->to_global(ledge_ik_points[i].raycast_origin);
		Vector3 to = gn->to_global(ledge_ik_points[i].raycast_target);
		if (ledge_ik_points[i].raycast_type == LedgeIKPointRaycastType::RAYCAST_HAND) {
			_find_ledge(from, to, ledge_ik_points.write[i].position, ledge_ik_points.write[i].wall_normal, ledge_ik_points.write[i].ledge_normal, ledge_ik_points[i].debug_color);
		} else {
			_find_wall_point(from, to, ledge_ik_points.write[i].position, ledge_ik_points.write[i].wall_normal, ledge_ik_points[i].debug_color);
		}
		ledge_ik_points.write[i].ik_node->set_ik_influence(1.0f);
		ledge_ik_points.write[i].target_position = ledge_ik_points[i].position;
		ledge_ik_points.write[i].ik_node->set_target_transform(_get_ledge_point_target_trf(i, ledge_ik_points[i].position));
	}

	target_agent_position = agent->get_global_position();

	animation_time = 0.0f;
	ledge_movement_acceleration = 0.0f;
	ledge_movement_velocity = 0.0f;
}

void HBAgentWallGrabbedState::exit() {
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].ik_node->set_ik_influence(0.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].ik_node->set_ik_influence(0.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].ik_node->set_ik_influence(0.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].ik_node->set_ik_influence(0.0f);

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);
	skel->set_position(Vector3());
}

void HBAgentWallGrabbedState::physics_process(float p_delta) {
	ZoneScopedN("HBAgentWallGrabbedState physics process");
	ERR_FAIL_COND(!get_inertialization_node().is_valid());
	if (get_inertialization_node()->is_inertializing()) {
		return;
	}
	debug_draw_clear();
	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);

	if (agent->is_action_pressed(HBAgent::AgentInputAction::INPUT_ACTION_PARKOUR_DOWN)) {
		state_machine->transition_to("Fall");
		return;
	}

	// move the agent
	Node3D *gn = get_graphics_node();
	ERR_FAIL_COND(!gn);
	if (agent->get_movement_input().length_squared() > 0 && agent->get_movement_input().angle_to(Vector3(0.0, 0.0, -1.0f)) < Math::deg_to_rad(45.0f)) {
		if (_handle_getup()) {
			return;
		}
	}

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
		LedgePoint ledge_point_to_test = SIGN(agent->get_movement_input().x) == -1 ? LedgePoint::LEDGE_POINT_LEFT_HAND : LedgePoint::LEDGE_POINT_RIGHT_FOOT;
		Vector3 raycast_start = Vector3(0.5f * SIGN(agent->get_movement_input().x), agent->get_height() * 0.25f, 0.5f);
		Vector3 raycast_end = raycast_start + Vector3(0.0f, 0.0f, -1.5f);
		raycast_start = gn->get_global_transform().xform(raycast_start);
		raycast_end = gn->get_global_transform().xform(raycast_end);

		Vector3 out_position, wall_normal, ledge_normal;

		// If we are moving in a direction we can't get to start slowing down
		if (!_find_ledge(raycast_start, raycast_end, out_position, wall_normal, ledge_normal, ledge_ik_points[ledge_point_to_test].debug_color)) {
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

	if (animation_direction != 0) {
		Vector3 curr_graphics_forward = gn->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
		Vector3 new_graphics_forward = animation_direction == 1 ? ledge_ik_points[LedgePoint::LEDGE_POINT_RIGHT_HAND].wall_normal : ledge_ik_points[LedgePoint::LEDGE_POINT_LEFT_HAND].wall_normal;
		new_graphics_forward *= -1.0f;
		float angle = curr_graphics_forward.angle_to(new_graphics_forward);
		Vector3 axis = curr_graphics_forward.cross(new_graphics_forward).normalized();

		if (axis.is_normalized()) {
			float angle_mul = Math::abs(ledge_movement_velocity) / ledge_movement_max_vel;
			new_graphics_forward = curr_graphics_forward.rotated(axis, (angle * p_delta * angle_mul));
		}

		Transform3D gn_trf = gn->get_global_transform();
		gn_trf.basis = Quaternion(curr_graphics_forward, new_graphics_forward) * gn_trf.basis.get_rotation_quaternion();

		Vector3 average_pos = ledge_ik_points[LedgePoint::LEDGE_POINT_RIGHT_HAND].position + ledge_ik_points[LedgePoint::LEDGE_POINT_LEFT_HAND].position;
		average_pos *= 0.5f;

		PhysicsDirectSpaceState3D::RayParameters ray_params;
		ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
		Vector3 ray_dir = gn_trf.origin;
		ray_dir.y = average_pos.y;
		ray_dir = ray_dir.direction_to(average_pos);

		ray_params.from = gn_trf.origin - new_graphics_forward;
		ray_params.to = gn_trf.origin + new_graphics_forward;

		PhysicsDirectSpaceState3D::RayResult ray_result;

		PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

		gn->set_global_transform(gn_trf);

		// Try to aim graphics forward
		if (dss->intersect_ray(ray_params, ray_result)) {
			Vector3 new_pos = ray_result.position + ray_result.position.direction_to(agent->get_global_position()) * 0.35f;
			agent->set_global_position(new_pos);
		} else {
			ERR_FAIL_MSG("ERROR AIMING FORWARD, CHECK MAP GEOMETRY OR CALL EIREXE!!!!");
		}
	}

	static struct {
		Vector3 position;
		Vector3 wall_normal;
		Vector3 ledge_normal;
	} new_ledge_info[LedgePoint::LEDGE_POINT_MAX];

	// First, we figure out if any ledge doesn't have a potential target ledge position, this means
	// that we moved into a side that doesn't have more corner for us to grab to
	Transform3D predicted_gn_trf = gn->get_global_transform();
	predicted_gn_trf.origin += target_agent_position - agent->get_global_position();
	if (is_moving) {
		for (int i = 0; i < ledge_ik_points.size(); i++) {
			LedgeIKPoint &ik_point = ledge_ik_points.write[i];
			Vector3 new_position;
			Vector3 wall_normal;
			Vector3 ledge_normal;
			Vector3 from = predicted_gn_trf.xform(ik_point.raycast_origin);
			Vector3 to = predicted_gn_trf.xform(ik_point.raycast_target);

			bool result;

			if (ik_point.raycast_type == LedgeIKPointRaycastType::RAYCAST_FOOT) {
				result = _find_wall_point(from, to, new_position, wall_normal, ledge_ik_points[i].debug_color);
			} else {
				result = _find_ledge(from, to, new_position, wall_normal, ledge_normal, ledge_ik_points[i].debug_color);
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
	float weights[LedgePoint::LEDGE_POINT_MAX];
	float total_weight = 0.0f;
	Vector3 ik_handle_positions[LedgePoint::LEDGE_POINT_MAX];

	float anim_mul = Math::abs(ledge_movement_velocity) / ledge_movement_max_vel;
	for (int i = 0; i < LedgePoint::LEDGE_POINT_MAX; i++) {
		LedgeIKPoint &ik_point = ledge_ik_points.write[i];

		// Do interpolation, if needed
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

		Transform3D trf = _get_ledge_point_target_trf(i, new_pos);
		ledge_ik_points.write[i].ik_node->set_target_transform(trf);

		ik_position_avg += trf.origin;
		weights[i] = ik_point.get_weight(animation_pos);
		total_weight += weights[i];
		ik_handle_positions[i] = trf.origin;
	}

	Vector3 ik_position_weighted; // The hip position after
	ik_position_avg /= (float)LedgePoint::LEDGE_POINT_MAX;

	for (int i = 0; i < LedgePoint::LEDGE_POINT_MAX; i++) {
		ik_position_weighted += ik_handle_positions[i] * (weights[i] / total_weight);
	}

	// Project into a plane that matches the wall normal
	ik_position_weighted = plane.project(ik_position_weighted);
	ik_position_avg = plane.project(ik_position_avg);

	// Move along the wall
	Vector3 movement_vector = Vector3(ledge_movement_velocity, 0.0f, 0.0f);
	Vector3 movement_dir = ledge_ik_points[LedgePoint::LEDGE_POINT_LEFT_HAND].ledge_normal.cross(ledge_ik_points[LedgePoint::LEDGE_POINT_LEFT_HAND].wall_normal).normalized();

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
	for (int i = 0; i < LedgePoint::LEDGE_POINT_MAX; i++) {
		ik_debug_info.last_tip_weights[i] = weights[i];
	}
	ik_debug_info.last_hip_offset_x = skel_pos.x;
	ik_debug_info.physics_time_delta += p_delta;
	ik_debug_info.last_data_is_valid = true;

	debug_draw_sphere(skel->get_global_position() + (ik_position_weighted - ik_position_avg), 0.05f, Color("red"));

	for (int i = 0; i < LedgePoint::LEDGE_POINT_MAX; i++) {
		Vector3 magnet = skel->to_global(ledge_ik_points[i].magnet_position);
		ledge_ik_points.write[i].ik_node->set_magnet_position(magnet);
		debug_draw_sphere(magnet, 0.05f, ledge_ik_points[i].debug_color);
	}
}

void HBAgentWallGrabbedState::debug_ui_draw() {
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
			for (int j = 0; j < LedgePoint::LEDGE_POINT_MAX; j++) {
				ik_debug_info.ik_tip_influence_y_graph[j][i - 1] = ik_debug_info.ik_tip_influence_y_graph[j][i];
			}
		}
		// Apply new values
		for (int i = 0; i < LedgePoint::LEDGE_POINT_MAX; i++) {
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
		for (int i = 0; i < LedgePoint::LEDGE_POINT_MAX; i++) {
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
		state_machine->transition_to("Move");
	}
}

/**********************
	Ledge Getup State
***********************/

void HBAgentLedgeGetUpState::_on_animation_finished() {
	Vector3 pos = animation_node->get_root_motion_starting_transform().xform(animation_node->get_root_motion_transform().origin);
	get_agent()->apply_root_motion(animation_node);
	Vector3 new_vel = (pos - prev_position);
	new_vel.y = 0.0f;
	new_vel.normalize();
	new_vel = new_vel * get_agent()->get_desired_movement_input_transformed().length() * get_agent()->get_agent_constants()->get_max_move_velocity();

	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);

	get_agent()->set_velocity(new_vel);
	state_machine->transition_to("Move");
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

	animation_node->connect("playback_finished", callable_mp(this, &HBAgentLedgeGetUpState::_on_animation_finished), CONNECT_ONE_SHOT);
}

void HBAgentLedgeGetUpState::process(float p_delta) {
	if (animation_node->is_playing()) {
		get_agent()->apply_root_motion(animation_node);
		prev_position = animation_node->get_root_motion_starting_transform().xform(animation_node->get_root_motion_transform().origin);
	}
}

/**********************
	Wall Parkour State
***********************/

Transform3D HBAgentWallParkourState::_calculate_limb_target_transform(const WallParkourLimb &p_limb) {
	Vector3 base_position;
	Vector3 forward;

	switch (p_limb.target_type) {
		case WallParkourTargetType::TARGET_PARKOUR_NODE: {
			base_position = p_limb.target.parkour_node->get_global_position();
			HBAgentParkourPoint *point = p_limb.target.parkour_node;
			forward = point->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, 1.0f));
		} break;
		case WallParkourTargetType::TARGET_LOCATION: {
			base_position = p_limb.target.location;
			forward = get_graphics_node()->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
		} break;
	}
	Quaternion rot = Quaternion(Vector3(0.0f, 0.0f, -1.0f), forward);

	Vector3 visual_offset = p_limb.visual_offset;

	if (!rot.get_axis().is_normalized()) {
		visual_offset = rot.xform(visual_offset);
	}

	Transform3D trf;
	trf.origin = base_position + visual_offset;

	return trf;
}

Transform3D HBAgentWallParkourState::_calculate_limb_current_transform(const WallParkourLimb &p_limb) {
	Vector3 base_position;
	Vector3 forward;

	switch (p_limb.target_type) {
		case WallParkourTargetType::TARGET_PARKOUR_NODE: {
			base_position = p_limb.current.parkour_node->get_global_position();
			HBAgentParkourPoint *point = p_limb.current.parkour_node;
			forward = point->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, 1.0f));
		} break;
		case WallParkourTargetType::TARGET_LOCATION: {
			base_position = p_limb.current.location;
			forward = get_graphics_node()->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
		} break;
	}
	Quaternion rot = Quaternion(Vector3(0.0f, 0.0f, -1.0f), forward);

	Vector3 visual_offset = p_limb.visual_offset;

	if (!rot.get_axis().is_normalized()) {
		visual_offset = rot.xform(visual_offset);
	}

	Transform3D trf;
	trf.origin = base_position + visual_offset;

	return trf;
}

void HBAgentWallParkourState::_handle_horizontal_parkour_movement(const Vector3 &p_movement_input) {
	HBAgent *agent = get_agent();
	Vector3 movement_input = p_movement_input;

	int limb_side_to_move = SIGN(p_movement_input.x);

	HBAgentParkourPoint *target_point = nullptr;

	if (parkour_limbs[WallParkourLimbType::HAND_RIGHT].target.parkour_node != parkour_limbs[WallParkourLimbType::HAND_LEFT].target.parkour_node) {
		// When the parkour node of one limb != the one used by the other limb it means we have to find a new point to grab onto
		limb_side_to_move *= -1;
		int target_i = SIGN(movement_input.x) == 1 ? WallParkourLimbType::HAND_RIGHT : WallParkourLimbType::HAND_LEFT;
		target_point = parkour_limbs[target_i].target.parkour_node;
	}

	WallParkourLimbType limb_arm_i = limb_side_to_move == 1 ? WallParkourLimbType::HAND_RIGHT : WallParkourLimbType::HAND_LEFT;
	WallParkourLimbType limb_foot_i = limb_side_to_move == 1 ? WallParkourLimbType::FOOT_RIGHT : WallParkourLimbType::FOOT_LEFT;
	WallParkourLimb *limb_arm = &parkour_limbs[limb_arm_i];
	WallParkourLimb *limb_foot = &parkour_limbs[limb_foot_i];
	WallParkourLimb *limb_arm_opposite = &parkour_limbs[limb_side_to_move == -1 ? WallParkourLimbType::HAND_RIGHT : WallParkourLimbType::HAND_LEFT];
	WallParkourLimb *limb_foot_opposite = &parkour_limbs[limb_side_to_move == -1 ? WallParkourLimbType::FOOT_RIGHT : WallParkourLimbType::FOOT_LEFT];

	limb_arm->animation_start = 0.25f;
	limb_arm->animation_end = 1.0f;
	limb_foot->animation_start = 0.0f;
	limb_foot->animation_end = 0.75f;

	if (parkour_limbs[WallParkourLimbType::HAND_RIGHT].target.parkour_node != parkour_limbs[WallParkourLimbType::HAND_LEFT].target.parkour_node) {
		// When closing in, we should move the arm first
		limb_arm->animation_start = 0.0f;
		limb_arm->animation_end = 0.5f;
		limb_foot->animation_start = 0.5f;
		limb_foot->animation_end = 1.0f;
	}

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
	parkour_limbs[limb_foot_i].target.location = target_point->get_global_position() + Vector3(0.0f, -target_leg_height_from_node, 0.0f);

	animation_time = 0.0f;
}

void HBAgentWallParkourState::_handle_vertical_parkour_movement(const Vector3 &p_movement_input) {
	int vertical_dir = SIGN(p_movement_input.y);

	const int LEFT = -1;
	const int RIGHT = 1;
	const int UP = 1;
	int side_to_move = RIGHT;

	HBAgentParkourPoint *parkour_node_hand_l = parkour_limbs[WallParkourLimbType::HAND_LEFT].target.parkour_node;
	HBAgentParkourPoint *parkour_node_hand_r = parkour_limbs[WallParkourLimbType::HAND_RIGHT].target.parkour_node;
	ERR_FAIL_COND(!parkour_node_hand_l);
	ERR_FAIL_COND(!parkour_node_hand_r);

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
	ERR_FAIL_COND(!agent);

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::ShapeParameters params;
	const int MAX_RESULTS = 16;
	PhysicsDirectSpaceState3D::ShapeResult results[MAX_RESULTS];

	params.shape_rid = dir_check_mesh->get_rid();
	params.transform.origin = hands_center + Vector3(0.0f, dir_check_mesh->get_height() * 0.5f * vertical_dir, 0.0f);
	params.collision_mask = HBPhysicsLayers::LAYER_PARKOUR_NODES;

	int result_count = dss->intersect_shape(params, results, MAX_RESULTS);

	WallParkourLimb *hand_to_move = &parkour_limbs[side_to_move == 1 ? WallParkourLimbType::HAND_RIGHT : WallParkourLimbType::HAND_LEFT];
	WallParkourLimb *foot_to_move = &parkour_limbs[side_to_move == 1 ? WallParkourLimbType::FOOT_RIGHT : WallParkourLimbType::FOOT_LEFT];
	WallParkourLimb *hand_to_stay = &parkour_limbs[side_to_move == -1 ? WallParkourLimbType::HAND_RIGHT : WallParkourLimbType::HAND_LEFT];
	WallParkourLimb *foot_to_stay = &parkour_limbs[side_to_move == -1 ? WallParkourLimbType::FOOT_RIGHT : WallParkourLimbType::FOOT_LEFT];

	HBAgentParkourPoint *node_to_move_to = nullptr;
	if (result_count > 0) {
		// Find the closest node to us that isn't the other hand's
		float node_to_move_to_dist = 0.0f;

		for (int i = 0; i < result_count; i++) {
			HBAgentParkourPoint *node = Object::cast_to<HBAgentParkourPoint>(results[i].collider);
			ERR_FAIL_COND(!node);
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
	// We didn't a find a node to move towards, so we will use the other hand's
	if (!node_to_move_to) {
		node_to_move_to = hand_to_stay->target.parkour_node;
	}

	if (node_to_move_to == hand_to_move->target.parkour_node) {
		// Can't move, abort
		return;
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
	if (vertical_dir == 1) {
		// When going up, it makes sense that both the foot and the hand move mostly simultaneously
		// however, the hand should let go first, this is because
		// you would usually use your legs to give you an impulse and then grab onto the next point
		hand_to_move->animation_start = 0.0f;
		hand_to_move->animation_end = 0.8f;
		foot_to_move->animation_start = 0.0f;
		foot_to_move->animation_end = 1.0f;
	} else {
		// For going down just invert it
		hand_to_move->animation_start = 0.0f;
		hand_to_move->animation_end = 0.8f;
		foot_to_move->animation_start = 0.0f;
		foot_to_move->animation_end = 1.0f;
	}

	hand_to_stay->animation_start = -1.0f;
	hand_to_stay->animation_end = -1.0f;
	foot_to_stay->animation_start = -1.0f;
	foot_to_stay->animation_end = -1.0f;

	hand_to_move->target.parkour_node = node_to_move_to;
	// TODO: Make legs try to find a point to support themselves
	foot_to_move->target.location = node_to_move_to->get_global_position() - Vector3(0.0f, target_leg_height_from_node, 0.0f);

	animation_time = 0.0f;
}

void HBAgentWallParkourState::enter(const Dictionary &p_args) {
	parkour_node = Object::cast_to<StaticBody3D>(p_args.get(PARAM_TARGET_PARKOUR_NODE, Variant()));
	ERR_FAIL_COND(!parkour_node);
	ERR_FAIL_COND(!Object::cast_to<HBAgentParkourPoint>(parkour_node));
	EPASController *epas_controller = get_epas_controller();
	ERR_FAIL_COND(!epas_controller);

	Ref<EPASTransitionNode> movement_transition = get_movement_transition_node();
	ERR_FAIL_COND(!movement_transition.is_valid());

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);

	Ref<EPASAnimationNode> animation_node = epas_controller->get_epas_node("WallParkourCat");
	ERR_FAIL_COND(!animation_node.is_valid());
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
		}
	}

	HBAgentParkourPoint *starting_parkour_node = Object::cast_to<HBAgentParkourPoint>(parkour_node);
	ERR_FAIL_COND(!starting_parkour_node);

	parkour_limbs[WallParkourLimbType::HAND_LEFT].target_type = WallParkourTargetType::TARGET_PARKOUR_NODE;
	parkour_limbs[WallParkourLimbType::HAND_LEFT].target.parkour_node = starting_parkour_node;
	parkour_limbs[WallParkourLimbType::HAND_LEFT].current.parkour_node = starting_parkour_node;
	parkour_limbs[WallParkourLimbType::HAND_LEFT].animating_peak_offset = Vector3(0.0, 0.0, -0.1f);
	parkour_limbs[WallParkourLimbType::HAND_LEFT].visual_offset = Vector3(-0.05f, 0.05f, 0.125f);

	parkour_limbs[WallParkourLimbType::HAND_RIGHT].target_type = WallParkourTargetType::TARGET_PARKOUR_NODE;
	parkour_limbs[WallParkourLimbType::HAND_RIGHT].target.parkour_node = starting_parkour_node;
	parkour_limbs[WallParkourLimbType::HAND_RIGHT].current.parkour_node = starting_parkour_node;
	parkour_limbs[WallParkourLimbType::HAND_RIGHT].animating_peak_offset = Vector3(0.0, 0.0, -0.1f);
	parkour_limbs[WallParkourLimbType::HAND_RIGHT].visual_offset = Vector3(0.05f, 0.05f, 0.125f);

	parkour_limbs[WallParkourLimbType::FOOT_LEFT].visual_offset = Vector3(-0.05f, 0.0f, 0.125f);
	parkour_limbs[WallParkourLimbType::FOOT_RIGHT].visual_offset = Vector3(0.05f, 0.0f, 0.125f);
	parkour_limbs[WallParkourLimbType::FOOT_LEFT].animating_peak_offset = Vector3(0.0, 0.0, -0.1f);
	parkour_limbs[WallParkourLimbType::FOOT_RIGHT].animating_peak_offset = Vector3(0.0, 0.0, -0.1f);

	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);

	Node3D *gn = get_graphics_node();
	ERR_FAIL_COND(!gn);

	const float floor_max_angle = agent->get_floor_max_angle();
	for (int i = 0; i < WallParkourLimbType::LIMB_TYPE_MAX; i++) {
		if (parkour_limbs[i].target_type == TARGET_LOCATION) {
			PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

			PhysicsDirectSpaceState3D::RayParameters ray_params;
			PhysicsDirectSpaceState3D::RayResult ray_result;

			ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
			ray_params.from = starting_parkour_node->get_global_transform().xform(Vector3(0.0f, -target_leg_height_from_node, -0.75f));
			ray_params.to = starting_parkour_node->get_global_transform().xform(Vector3(0.0f, -target_leg_height_from_node, 0.75f));

			if (dss->intersect_ray(ray_params, ray_result)) {
				ERR_FAIL_COND_MSG(!is_wall(ray_result.normal, floor_max_angle), "Failed to get wall for feet, check level geometry");
				parkour_limbs[i].target.location = ray_result.position;
				parkour_limbs[i].current.location = parkour_limbs[i].target.location;
			}
		}
		// Setup magnet (common for both target types)
		parkour_limbs[i].ik_node->set_use_magnet(true);
		parkour_limbs[i].ik_node->set_ik_influence(1.0f);
		Vector3 magnet = skel->to_global(parkour_limbs[i].local_magnet_pos);
		parkour_limbs[i].ik_node->set_magnet_position(magnet);
		parkour_limbs[i].ik_node->set_target_transform(_calculate_limb_target_transform(parkour_limbs[i]));
		parkour_limbs[i].ik_node->set_use_hinge(true);
	}

	if (!dir_check_mesh.is_valid()) {
		dir_check_mesh.instantiate();
		dir_check_mesh->set_height(1.0f);
		dir_check_mesh->set_radius(0.5f);
	}

	agent->set_movement_mode(HBAgent::MovementMode::MOVE_MANUAL);

	Vector3 limb_center;

	for (int i = 0; i < WallParkourLimbType::LIMB_TYPE_MAX; i++) {
		limb_center += _calculate_limb_target_transform(parkour_limbs[i]).origin;
	}

	limb_center = limb_center / ((float)WallParkourLimbType::LIMB_TYPE_MAX);

	Vector3 center = agent->get_global_position() - limb_center;
	agent_offset = get_graphics_node()->get_global_transform().basis.xform_inv(center) + agent_offset_adjustment;
	agent_offset_target = agent_offset;

	agent_position_spring_velocity = Vector3();
	agent_position_target = agent->get_global_position();

	movement_transition->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_PARKOUR_CAT);

	animation_time = -1.0f;
}

void HBAgentWallParkourState::exit() {
	for (int i = 0; i < WallParkourLimbType::LIMB_TYPE_MAX; i++) {
		parkour_limbs[i].ik_node->set_ik_influence(0.0f);
	}

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);
	skel->set_position(Vector3());
}

void HBAgentWallParkourState::physics_process(float p_delta) {
	debug_draw_clear();
	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);

	if (agent->is_action_pressed(HBAgent::AgentInputAction::INPUT_ACTION_PARKOUR_DOWN)) {
		state_machine->transition_to(StringName("Fall"));
		return;
	}

	Vector3 movement_input = agent->get_movement_input();
	movement_input.y = movement_input.z * -1.0f;
	movement_input.z = 0.0f;

	Node3D *gn = get_graphics_node();
	ERR_FAIL_COND(!gn);

	Vector<float> weights;
	weights.resize(WallParkourLimbType::LIMB_TYPE_MAX);
	weights.fill(1.0f);
	Vector<Vector3> ik_positions;
	ik_positions.resize(WallParkourLimbType::LIMB_TYPE_MAX);

	float total_weight = WallParkourLimbType::LIMB_TYPE_MAX;

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);

	if (animation_time >= 0.0f) {
		animation_time = CLAMP(animation_time + p_delta * (1.0f / ANIMATION_DURATION), 0.0f, 1.0f);
		total_weight = 0.0f;

		for (int i = 0; i < WallParkourLimbType::LIMB_TYPE_MAX; i++) {
			Vector3 current_position;
			Vector3 target_position;

			if (parkour_limbs[i].animation_start == -1.0f) {
				ik_positions.write[i] = _calculate_limb_target_transform(parkour_limbs[i]).origin;
				total_weight += 1.0f;
				weights.write[i] = 1.0f;
				continue;
			}

			current_position = _calculate_limb_current_transform(parkour_limbs[i]).origin;
			target_position = _calculate_limb_target_transform(parkour_limbs[i]).origin;

			Transform3D target_trf;
			float blend_t = Math::inverse_lerp(parkour_limbs[i].animation_start, parkour_limbs[i].animation_end, animation_time);
			blend_t = CLAMP(blend_t, 0.0f, 1.0f);

			weights.write[i] = 1.0f - sin(blend_t * Math_PI);

			target_trf.origin = current_position.lerp(target_position, blend_t);
			target_trf.origin += skel->get_global_transform().basis.xform(parkour_limbs[i].animating_peak_offset) * sin(blend_t * Math_PI);
			parkour_limbs[i].ik_node->set_target_transform(target_trf);
			ik_positions.write[i] = target_trf.origin;
			total_weight += weights[i];
		}
	} else {
		for (int i = 0; i < WallParkourLimbType::LIMB_TYPE_MAX; i++) {
			ik_positions.write[i] = _calculate_limb_target_transform(parkour_limbs[i]).origin;
		}
	}

	if (animation_time == -1.0f && movement_input.length_squared() != 0.0f) {
		if (Math::abs(movement_input.y) > Math::abs(movement_input.x)) {
			_handle_vertical_parkour_movement(movement_input);
		} else {
			_handle_horizontal_parkour_movement(movement_input);
		}
	}

	Vector3 ik_position_weighted; // This is used as the center point for the body movement
	int highest_limb = 0;
	int lowest_limb = 0;
	for (int i = 0; i < WallParkourLimbType::LIMB_TYPE_MAX; i++) {
		ik_position_weighted += ik_positions[i] * (weights[i] / total_weight);
		highest_limb = ik_positions[i].y > ik_positions[highest_limb].y ? i : highest_limb;
		lowest_limb = ik_positions[i].y < ik_positions[lowest_limb].y ? i : lowest_limb;
	}

	float target_hand_height_difference = parkour_limbs[WallParkourLimbType::HAND_LEFT].target.parkour_node->get_global_position().y;
	float curr_hand_height_difference = parkour_limbs[WallParkourLimbType::HAND_LEFT].current.parkour_node->get_global_position().y;
	target_hand_height_difference -= parkour_limbs[WallParkourLimbType::HAND_RIGHT].target.parkour_node->get_global_position().y;
	curr_hand_height_difference -= parkour_limbs[WallParkourLimbType::HAND_RIGHT].current.parkour_node->get_global_position().y;
	float target_leg_height_difference = parkour_limbs[WallParkourLimbType::FOOT_LEFT].target.location.y;
	target_leg_height_difference -= parkour_limbs[WallParkourLimbType::FOOT_RIGHT].target.location.y;
	if (animation_time != -1.0f) {
		float diff = Math::lerp(Math::abs(curr_hand_height_difference), Math::abs(target_hand_height_difference), animation_time) / 0.5f;
		diff = CLAMP(diff, 0.0f, 1.0f);
		back_straightness_blend_node->set_blend_amount(diff);
		// TODO: Make these constant
		agent_offset_target.z = agent_offset.z - diff * 0.1f;
		agent_offset_target.y = agent_offset.y + Math::abs(target_hand_height_difference) - Math::abs(target_leg_height_difference * 0.85f);
	}

	debug_draw_sphere(ik_position_weighted, 0.05f, Color("PURPLE"));

	agent_position_target = ik_position_weighted + get_graphics_node()->get_global_transform().basis.xform(agent_offset_target);

	Vector3 new_pos = get_agent()->get_global_position();

	HBSprings::spring_damper_exact_ratio_vector3(
			new_pos,
			agent_position_spring_velocity,
			agent_position_target,
			Vector3(), 0.7f, 0.2f, p_delta);

	get_agent()->set_global_position(new_pos);

	for (int i = 0; i < WallParkourLimbType::LIMB_TYPE_MAX; i++) {
		// Update magnet position
		Vector3 magnet = skel->to_global(parkour_limbs[i].local_magnet_pos);
		parkour_limbs[i].ik_node->set_magnet_position(magnet);
		if (i == WallParkourLimbType::FOOT_LEFT) {
			debug_draw_sphere(magnet, 0.05f, Color("RED"));
		} else if (i == WallParkourLimbType::FOOT_RIGHT) {
			debug_draw_sphere(magnet, 0.05f, Color("GREEN"));
		}
	}

	if (animation_time >= 1.0f) {
		animation_time = -1.0f;

		for (int i = 0; i < WallParkourLimbType::LIMB_TYPE_MAX; i++) {
			switch (parkour_limbs[i].target_type) {
				case WallParkourTargetType::TARGET_PARKOUR_NODE: {
					parkour_limbs[i].current.parkour_node = parkour_limbs[i].target.parkour_node;
				} break;
				case WallParkourTargetType::TARGET_LOCATION: {
					parkour_limbs[i].current.location = parkour_limbs[i].target.location;
				} break;
			}
		}
	}
}

void HBAgentWallParkourState::debug_ui_draw() {
	HBAgentState::debug_ui_draw();
	ImGui::InputFloat3("Left leg magnet pos", (float *)&parkour_limbs[WallParkourLimbType::FOOT_LEFT].local_magnet_pos.coord);
	ImGui::InputFloat3("Right leg magnet pos", (float *)&parkour_limbs[WallParkourLimbType::FOOT_RIGHT].local_magnet_pos.coord);
}

HBAgentWallParkourState::HBAgentWallParkourState() {
}
