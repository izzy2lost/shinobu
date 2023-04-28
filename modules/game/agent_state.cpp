#include "agent_state.h"

#include "modules/imgui/godot_imgui.h"
#include "physics_layers.h"
#include "scene/resources/cylinder_shape_3d.h"
#include "scene/resources/sphere_shape_3d.h"
#include "utils.h"

HBDebugGeometry *HBAgentState::_get_debug_geo() {
	if (debug_geo == nullptr) {
		debug_geo = memnew(HBDebugGeometry);
		debug_geo->set_visible(draw_debug_geometry);
		debug_geo->set_as_top_level(true);
		add_child(debug_geo, false, INTERNAL_MODE_FRONT);
		debug_geo->set_position(Vector3());
	}
	return debug_geo;
}

void HBAgentState::debug_draw_shape(Ref<Shape3D> p_shape, const Vector3 &p_position, const Color &p_color) {
	HBDebugGeometry *dbg_geo = _get_debug_geo();
	if (dbg_geo) {
		dbg_geo->debug_shape(p_shape, p_position, p_color);
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

void HBAgentState::debug_ui_draw() const {
#ifdef DEBUG_ENABLED
	if (ImGui::Checkbox("Draw debug geometry", &const_cast<HBAgentState *>(this)->draw_debug_geometry)) {
		if (debug_geo) {
			debug_geo->set_visible(draw_debug_geometry);
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
}

bool is_wall(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) > p_floor_max_angle;
}

bool is_floor(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) <= p_floor_max_angle;
}

void HBAgentMoveState::physics_process(float p_delta) {
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
		if (gn && agent->get_desired_movement_input().length() > 0.0f) {
			Vector3 forward = gn->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
			forward.y = 0.0f;
			forward.normalize();

			float turn_threshold = Math::deg_to_rad(agent->get_agent_constants()->get_turn_animation_threshold_degrees());
			if (agent->get_desired_movement_input().angle_to(forward) > turn_threshold && agent->get_linear_velocity().length() < 0.1f) {
				state_machine->transition_to("Turn");
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

	if (dss->intersect_shape(shape_query_params, &shape_query_result, 1)) {
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

	PhysicsDirectSpaceState3D::ShapeResult shape_result;
	PhysicsDirectSpaceState3D::ShapeParameters shape_params;

	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

	// Try to find a roof
	debug_draw_raycast(ray_params);
	if (!dss->intersect_ray(ray_params, ray_result)) {
		return false;
	}

	// Make sure its a roof
	if (!is_floor(ray_result.normal, floor_max_angle)) {
		return false;
	}

	Vector3 edge = base;
	edge.y = ray_result.position.y;

	Dictionary transition_dict;
	Transform3D temp_trf;
	temp_trf.basis = Basis().looking_at(wall_normal);
	temp_trf.origin = base;
	transition_dict[StringName("WallrunBase")] = temp_trf;
	temp_trf.origin = edge;
	transition_dict[StringName("WallrunEdge")] = temp_trf;

	state_machine->transition_to("Wallrun", transition_dict);

	return true;
}

bool HBAgentMoveState::_check_wall(PhysicsDirectSpaceState3D::RayResult &p_result) {
	HBAgent *agent = get_agent();
	ERR_FAIL_COND_V(agent == nullptr, false);

	const float parkour_wall_check_distance = agent->get_agent_constants()->get_parkour_wall_check_distance();
	const float parkour_max_wall_facing_angle_degrees = Math::deg_to_rad(agent->get_agent_constants()->get_parkour_max_wall_facing_angle_degrees());
	const float floor_max_angle = agent->get_floor_max_angle();

	Ref<Shape3D> player_shape = agent->get_collision_shape();
	PhysicsDirectSpaceState3D *dss = agent->get_world_3d()->get_direct_space_state();

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

	float mid_height = get_agent()->get_height() * 0.5f;

	ray_params.from = agent->get_global_position() + Vector3(0.0f, mid_height, 0.0f);
	Vector3 movement_input_dir = agent->get_desired_movement_input().normalized();
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
	const StringName wp_names[4] = {
		"VaultBaseFar",
		"VaultBaseNear",
		"VaultEdgeNear",
		"VaultEdgeFar"
	};
	Array arr;
	for (int i = 0; i < 4; i++) {
		arr.push_back(wp_names[i]);
	}

	ERR_FAIL_COND_MSG(!p_args.has_all(arr), "Missing vault warp points");

	Ref<EPASTransitionNode> transition_node = get_movement_transition_node();
	Skeleton3D *skel = get_skeleton();
	EPASController *epas_controller = get_epas_controller();
	ERR_FAIL_COND(epas_controller == nullptr);
	ERR_FAIL_COND(skel == nullptr);
	ERR_FAIL_COND(!transition_node.is_valid());

	transition_node->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_VAULT);
	animation_node = epas_controller->get_epas_node("Vault");
	animation_node->set_root_motion_starting_transform(skel->get_global_transform());
	if (animation_node.is_valid()) {
		for (int i = 0; i < 4; i++) {
			ERR_FAIL_COND_MSG(p_args[wp_names[i]].get_type() != Variant::TRANSFORM3D, "Warp point transition must be Transform3D");
			animation_node->set_warp_point_transform(wp_names[i], p_args[wp_names[i]]);
		}
		animation_node->set_root_motion_forward(Vector3(0.0f, 0.0, 1.0f));
		animation_node->play();
		animation_node->connect("playback_finished", callable_mp(this, &HBAgentVaultState::_on_animation_finished));
	}

	get_agent()->set_movement_mode(HBAgent::MOVE_MANUAL);
	get_agent()->set_velocity(Vector3());
}

void HBAgentVaultState::exit() {
	if (animation_node.is_valid()) {
		animation_node->disconnect("playback_finished", callable_mp(this, &HBAgentVaultState::_on_animation_finished));
	}
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

	Vector3 movement_input = get_agent()->get_desired_movement_input().normalized();

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
	animation_node->connect("playback_finished", callable_mp(this, &HBAgentTurnState::_on_animation_finished));
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

void HBAgentTurnState::exit() {
	if (animation_node.is_valid()) {
		animation_node->disconnect("playback_finished", callable_mp(this, &HBAgentTurnState::_on_animation_finished));
	}
}

/**********************
	Wallrun state
***********************/

void HBAgentWallrunState::enter(const Dictionary &p_args) {
	const StringName wp_names[2] = {
		"WallrunBase",
		"WallrunEdge",
	};
	Array arr;
	for (int i = 0; i < 2; i++) {
		arr.push_back(wp_names[i]);
	}

	ERR_FAIL_COND_MSG(!p_args.has_all(arr), "Missing wallrun warp points");

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
		for (int i = 0; i < 2; i++) {
			ERR_FAIL_COND_MSG(p_args[wp_names[i]].get_type() != Variant::TRANSFORM3D, "Warp point transform must be Transform3D");
			animation_node->set_warp_point_transform(wp_names[i], p_args[wp_names[i]]);
		}
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
		state_machine->transition_to("WallGrabbed");
	}
	dbg->update();
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

bool HBAgentWallGrabbedState::_find_ledge(const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_wall_normal, const Color &p_debug_color) {
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

	debug_draw_raycast(ray_params, Color(1.0f, 0.0f, 0.0f));

	if (!dss->intersect_ray(ray_params, ray_result)) {
		return false;
	}

	if (!is_wall(ray_result.normal, agent->get_floor_max_angle())) {
		return false;
	}

	Vector3 ledge_point = ray_result.position;

	// Then we try to find the ledge top point

	ray_params.to = ray_result.position - ray_result.normal * 0.01;
	ray_params.from = ray_params.to + Vector3(0.0f, 0.5f, 0.0f);

	debug_draw_raycast(ray_params, p_debug_color);
	if (!dss->intersect_ray(ray_params, ray_result)) {
		return false;
	}

	if (!is_floor(ray_result.normal, agent->get_floor_max_angle())) {
		return false;
	}

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

	debug_draw_raycast(ray_params, p_debug_color);

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

void HBAgentWallGrabbedState::enter(const Dictionary &p_args) {
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
		// Not very pretty but lets initialize IK points
		struct ik_points {
			String bone_name;
			StringName epas_node_name;
		} ik_points[LedgePoint::LEDGE_POINT_MAX];

		Vector3 raycast_origin = Vector3();
		Vector3 raycast_target_base = Vector3(0.0f, agent->get_height() * 0.5f, -0.5f);

		// Left hand
		ik_points[LedgePoint::LEDGE_POINT_LEFT_HAND].bone_name = "hand.L";
		ik_points[LedgePoint::LEDGE_POINT_LEFT_HAND].epas_node_name = "LeftHandIK";

		// Right hand
		ik_points[LedgePoint::LEDGE_POINT_RIGHT_HAND].bone_name = "hand.R";
		ik_points[LedgePoint::LEDGE_POINT_RIGHT_HAND].epas_node_name = "RightHandIK";

		Vector3 foot_trg = Vector3(0.05f, 0.225f, -0.2f).normalized() * 0.75f;

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
			ledge_ik_points.write[i].raycast_origin = raycast_origin;
			ledge_ik_points.write[i].ik_node->set_use_magnet(false);
		}

		// Left Hand
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].raycast_target = raycast_target_base + Vector3(-0.2f, 0.0f, 0.0f);
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].debug_color = Color("RED");
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].start_time = 0.6f;
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].end_time = 1.0f;
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].raycast_type = LedgeIKPointRaycastType::RAYCAST_HAND;

		// Right Hand
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].raycast_target = raycast_target_base + Vector3(0.2f, 0.0f, 0.0f);
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].debug_color = Color("GREEN");
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].start_time = 0.1f;
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].end_time = 0.6f;
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].raycast_type = LedgeIKPointRaycastType::RAYCAST_HAND;

		// Left Foot
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].raycast_target = foot_trg * Vector3(-1.0f, 1.0f, 1.0f);
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].debug_color = Color("DARK_RED");
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].start_time = 0.65f;
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].end_time = 1.0f;
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].raycast_type = LedgeIKPointRaycastType::RAYCAST_FOOT;
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].target_offset = Vector3(0.0f, 0.0f, 0.15f);

		// Right foot
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].raycast_target = foot_trg;
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].debug_color = Color("DARK_GREEN");
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].start_time = 0.2f;
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].end_time = 0.7f;
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].raycast_type = LedgeIKPointRaycastType::RAYCAST_FOOT;
		ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].target_offset = Vector3(0.0f, 0.0f, 0.15f);
	}

	int hand_bone_l_idx = ledge_ik_points[LedgePoint::LEDGE_POINT_LEFT_HAND].bone_idx;
	int hand_bone_r_idx = ledge_ik_points[LedgePoint::LEDGE_POINT_RIGHT_HAND].bone_idx;
	Ref<EPASIKNode> left_hand_ik_node = ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].ik_node;
	Ref<EPASIKNode> right_hand_ik_node = ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].ik_node;

	LedgeIKPoint &ik_point_left = ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND];
	LedgeIKPoint &ik_point_right = ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND];

	ik_point_left.start_time = 0.6f;
	ik_point_left.end_time = 1.0f;
	ik_point_right.start_time = 0.1f;
	ik_point_right.end_time = 0.6f;

	for (int i = 0; i < ledge_ik_points.size(); i++) {
		Vector3 from = gn->to_global(ledge_ik_points[i].raycast_origin);
		Vector3 to = gn->to_global(ledge_ik_points[i].raycast_target);
		if (ledge_ik_points[i].raycast_type == LedgeIKPointRaycastType::RAYCAST_HAND) {
			_find_ledge(from, to, ledge_ik_points.write[i].position, ledge_ik_points.write[i].wall_normal, ledge_ik_points[i].debug_color);
		} else {
			_find_wall_point(from, to, ledge_ik_points.write[i].position, ledge_ik_points.write[i].wall_normal, ledge_ik_points[i].debug_color);
		}
		ledge_ik_points.write[i].target_position = ledge_ik_points[i].position;
	}

	Vector3 left_magnet = skel->to_global(skel->get_bone_global_pose(skel->get_bone_parent(hand_bone_l_idx)).origin);
	Vector3 right_magnet = skel->to_global(skel->get_bone_global_pose(skel->get_bone_parent(hand_bone_r_idx)).origin);
	left_magnet.y -= 1.5f;
	right_magnet.y -= 1.5f;

	Transform3D temp_trf;
	left_hand_ik_node->set_target_transform(_get_ledge_point_target_trf(LedgePoint::LEDGE_POINT_LEFT_HAND, ik_point_left.position));
	left_hand_ik_node->set_use_magnet(true);
	left_hand_ik_node->set_magnet_position(left_magnet);

	right_hand_ik_node->set_target_transform(_get_ledge_point_target_trf(LedgePoint::LEDGE_POINT_RIGHT_HAND, ik_point_left.position));
	right_hand_ik_node->set_use_magnet(true);
	right_hand_ik_node->set_magnet_position(right_magnet);
}

void HBAgentWallGrabbedState::physics_process(float p_delta) {
	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);

	// move the agent
	Node3D *gn = get_graphics_node();
	ERR_FAIL_COND(!gn);

	Vector3 forward = get_graphics_node()->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
	Plane plane = Plane(forward);
	Vector3 movement_input = plane.project(agent->get_movement_input());

	HBUtils::velocity_spring(
			&ledge_movement_velocity,
			&ledge_movement_acceleration,
			gn->get_global_transform().basis.xform_inv(movement_input).x * 0.5f,
			0.175f,
			p_delta);

	bool is_moving = Math::abs(ledge_movement_velocity) > 0.05f;

	Vector3 prev_pos = agent->get_global_position();
	agent->set_global_position(agent->get_global_position() + gn->get_global_transform().basis.xform(Vector3(ledge_movement_velocity, 0.0f, 0.0f)) * p_delta);

	// Now deal with IK

	if (animation_time != 0.0f || is_moving) {
		animation_direction = SIGN(ledge_movement_velocity);
		animation_time += p_delta + (Math::abs(ledge_movement_velocity) * 2.0f * p_delta);
	}

	// If the animation ends and we are still moving, wrap around, otherwise we set it to 0
	if (animation_time >= 1.0f) {
		if (is_moving) {
			animation_time = Math::fposmod(animation_time, 1.0f);
			animation_direction = SIGN(ledge_movement_velocity);
		} else {
			animation_time = 0.0f;
		}
		// Always apply the target positions
		for (int i = 0; i < ledge_ik_points.size(); i++) {
			ledge_ik_points.write[i].position = ledge_ik_points[i].target_position;
		}
	}

	// First, we figure out if any ledge doesn't have a potential target ledge position, this means
	// that we moved into a side that doesn't have more corner for us to grab to
	Vector<Vector3> new_ledge_positions;
	new_ledge_positions.resize(ledge_ik_points.size());
	debug_draw_clear();
	if (is_moving) {
		for (int i = 0; i < ledge_ik_points.size(); i++) {
			LedgeIKPoint &ik_point = ledge_ik_points.write[i];
			Vector3 new_position;
			Vector3 wall_normal;
			Vector3 from = gn->to_global(ik_point.raycast_origin);
			Vector3 to = gn->to_global(ik_point.raycast_target);

			bool result;

			if (ik_point.raycast_type == LedgeIKPointRaycastType::RAYCAST_FOOT) {
				result = _find_wall_point(from, to, new_position, wall_normal, ledge_ik_points[i].debug_color);
				debug_draw_sphere(new_position, 0.05f, ledge_ik_points[i].debug_color);
				debug_draw_sphere(new_position + ik_point.target_offset, 0.05f, ledge_ik_points[i].debug_color);

			} else {
				result = _find_ledge(from, to, new_position, wall_normal, ledge_ik_points[i].debug_color);
			}

			if (!result) {
				// Ledge detection failed, which means we must end and undo the previous movement
				agent->set_global_position(prev_pos);
				ledge_movement_acceleration = 0.0f;
				ledge_movement_velocity = 0.0f;
				is_moving = false;
				break;
			} else {
				new_ledge_positions.write[i] = new_position;
			}
		}
	}

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);

	for (int i = 0; i < LedgePoint::LEDGE_POINT_MAX; i++) {
		Transform3D trf;

		LedgeIKPoint &ik_point = ledge_ik_points.write[i];
		if (is_moving) {
			ik_point.target_position = new_ledge_positions[i];
		}
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
		Vector3 new_pos = ik_point.position.lerp(ik_point.target_position, animation_pos);
		new_pos.y += Math::sin(animation_pos * Math_PI) * 0.05f;
		ledge_ik_points.write[i].ik_node->set_target_transform(_get_ledge_point_target_trf(i, new_pos));
	}

	ERR_FAIL_COND(!agent);
	skel->set_position(Vector3(0.0f, 0.0f, 0.0f) * (Math::abs(ledge_movement_velocity) / 0.5f));
}
void HBAgentWallGrabbedState::debug_ui_draw() const {
	HBAgentState::debug_ui_draw();
	ImGui::Text("%.2f", animation_time);
}
