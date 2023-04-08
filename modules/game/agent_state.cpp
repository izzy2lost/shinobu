#include "agent_state.h"

#include "modules/imgui/godot_imgui.h"
#include "physics_layers.h"
#include "scene/resources/cylinder_shape_3d.h"

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

/**********************
	MOVE STATE
***********************/

void HBAgentMoveState::enter(const Dictionary &p_args) {
	get_agent()->set_movement_mode(HBAgent::MovementMode::MOVE_GROUNDED);
	Ref<EPASTransitionNode> transition_node = get_movement_transition_node();
	ERR_FAIL_COND(!transition_node.is_valid());
	transition_node->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_MOVE);
}

void HBAgentMoveState::debug_draw() const {
#ifdef DEBUG_ENABLED
	if (ImGui::Checkbox("Show raycast debug", &const_cast<HBAgentMoveState *>(this)->show_debug_geometry)) {
		debug_geo->set_visible(show_debug_geometry);
	}
#endif
}

HBAgentMoveState::HBAgentMoveState() {
#ifdef DEBUG_ENABLED
	debug_geo = memnew(HBDebugGeometry);
	debug_geo->hide();
	add_child(debug_geo, false, INTERNAL_MODE_BACK);
#endif
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
		debug_geo->clear();
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

	debug_geo->debug_raycast(ray_params, Color(0.0f, 1.0f, 0.0f));

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
	debug_geo->debug_raycast(ray_params, Color(0.0f, 0.0f, 1.0f));

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
	debug_geo->debug_raycast(ray_params, Color(1.0f, 1.0f, 0.0f));

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
		print_line("WALLCHECK_FAILED");
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
	debug_geo->debug_raycast(ray_params);
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

	debug_geo->debug_raycast(ray_params, Color(1.0f, 0.0f, 0.0f));

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

void HBAgentWallGrabbedState::enter(const Dictionary &p_args) {
	Ref<EPASTransitionNode> movement_transition = get_movement_transition_node();
	ERR_FAIL_COND(!movement_transition.is_valid());

	movement_transition->transition_to(HBAgentConstants::MovementTransitionInputs::MOVEMENT_WALLGRABBED);
}
