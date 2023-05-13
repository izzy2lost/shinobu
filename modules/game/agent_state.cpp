#include "agent_state.h"

#ifdef DEBUG_ENABLED
#include "implot.h"
#include "modules/imgui/godot_imgui.h"
#endif
#include "modules/tracy/tracy.gen.h"
#include "physics_layers.h"
#include "scene/resources/cylinder_shape_3d.h"
#include "scene/resources/sphere_shape_3d.h"
#include "utils.h"

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

	debug_draw_raycast(ray_params, p_debug_color);

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

	debug_draw_raycast(ray_params, p_debug_color);
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

	Vector3 raycast_origin = Vector3(0.0f, agent->get_height() * 0.25f, 0.5f);
	Vector3 raycast_target_base = Vector3(0.0f, agent->get_height() * 0.25f, -1.0f);

	// Left hand
	ik_points[LedgePoint::LEDGE_POINT_LEFT_HAND].bone_name = "hand.L";
	ik_points[LedgePoint::LEDGE_POINT_LEFT_HAND].epas_node_name = "LeftHandIK";

	// Right hand
	ik_points[LedgePoint::LEDGE_POINT_RIGHT_HAND].bone_name = "hand.R";
	ik_points[LedgePoint::LEDGE_POINT_RIGHT_HAND].epas_node_name = "RightHandIK";

	Vector3 foot_trg = Vector3(0.02f, 0.1, -0.5f).normalized();

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
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].raycast_origin += Vector3(-0.2f, 0.0f, 0.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].debug_color = Color("RED");
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].start_time = 0.5f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].end_time = 0.9f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_HAND].raycast_type = LedgeIKPointRaycastType::RAYCAST_HAND;

	// Right Hand
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].raycast_target = raycast_target_base + Vector3(0.2f, 0.0f, 0.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].raycast_origin += Vector3(0.2f, 0.0f, 0.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].debug_color = Color("GREEN");
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].start_time = 0.0f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].end_time = 0.4f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_HAND].raycast_type = LedgeIKPointRaycastType::RAYCAST_HAND;

	// Left Foot
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].raycast_target = foot_trg * Vector3(-1.0f, 1.0f, 1.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].raycast_origin = foot_trg * Vector3(-1.0f, 1.0f, 1.0f);
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].raycast_origin.z = 0.0f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].debug_color = Color("DARK_RED");
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].start_time = 0.6f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].end_time = 1.0f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].raycast_type = LedgeIKPointRaycastType::RAYCAST_FOOT;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_LEFT_FOOT].target_offset = Vector3(0.0f, 0.0f, 0.15f);

	// Right foot
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].raycast_origin = foot_trg;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].raycast_origin.z = 0.0f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].raycast_target = foot_trg;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].debug_color = Color("DARK_GREEN");
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].start_time = 0.1f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].end_time = 0.5f;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].raycast_type = LedgeIKPointRaycastType::RAYCAST_FOOT;
	ledge_ik_points.write[LedgePoint::LEDGE_POINT_RIGHT_FOOT].target_offset = Vector3(0.0f, 0.0f, 0.15f);
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
		_init_ik_points();
	}

	for (int i = 0; i < LedgePoint::LEDGE_POINT_MAX; i++) {
		// Setup magnet
		Vector3 magnet = skel->get_bone_global_pose(skel->get_bone_parent(ledge_ik_points[i].bone_idx)).origin;
		magnet.y += ledge_ik_points[i].raycast_type == RAYCAST_FOOT ? 1.5f : -1.5f;
		if (ledge_ik_points[i].raycast_type == RAYCAST_FOOT) {
			magnet.x += SIGN(magnet.x);
		}
		ledge_ik_points.write[i].magnet_position = magnet;
		ledge_ik_points.write[i].ik_node->set_use_hinge(true);
		ledge_ik_points.write[i].ik_node->set_magnet_position(skel->to_global(magnet));
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
	debug_draw_clear();
	HBAgent *agent = get_agent();
	ERR_FAIL_COND(!agent);

	Skeleton3D *skel = get_skeleton();
	ERR_FAIL_COND(!skel);

	if (agent->is_action_just_pressed(HBAgent::AgentInputAction::INPUT_ACTION_PARKOUR_DOWN)) {
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

	HBUtils::velocity_spring(
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
		animation_time += p_delta + (Math::abs(ledge_movement_velocity) * 1.5f * p_delta);
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

		if (dss->intersect_ray(ray_params, ray_result)) {
			Vector3 new_pos = ray_result.position + ray_result.position.direction_to(agent->get_global_position()) * 0.35f;
			agent->set_global_position(new_pos);
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
	HBUtils::critical_spring_damper_exact_vector3(graphics_position, target_agent_spring_velocity, target_agent_position, target_spring_halflife, p_delta);
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
	}
}

void HBAgentWallGrabbedState::debug_ui_draw() {
	HBAgentState::debug_ui_draw();
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
	get_agent()->apply_root_motion(animation_node);

	if (!animation_node->is_playing()) {
		state_machine->transition_to("Move");
	}
}
