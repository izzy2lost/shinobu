#include "agent.h"
#include "core/math/transform_3d.h"
#include "modules/game/agent_parkour.h"
#include "physics_layers.h"
#include "springs.h"

#ifdef DEBUG_ENABLED
#include "implot.h"
#include "modules/imgui/godot_imgui.h"
#endif

#include "modules/imgui/godot_imgui_macros.h"

#include "core/config/project_settings.h"
#include "scene/resources/3d/cylinder_shape_3d.h"

HBAgent::HBAgent() :
		JoltCharacterBody3D() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_physics_process(true);
		set_process_internal(true);
		agent_constants.instantiate();
	}
	String outline_material_path = GLOBAL_DEF(PropertyInfo(Variant::STRING, "game/agent/outline_material", PROPERTY_HINT_FILE, "*.tres,*.res,*.theme"), "");
	if (!outline_material_path.is_empty() && !Engine::get_singleton()->is_editor_hint()) {
		outline_material = ResourceLoader::load(outline_material_path);
	}
#ifdef DEBUG_ENABLED
	velocity_plot_lines_y.resize_zeroed(VELOCITY_PLOT_SIZE);
	velocity_plot_lines_x.resize_zeroed(VELOCITY_PLOT_SIZE);
	desired_velocity_plot_lines_y.resize_zeroed(VELOCITY_PLOT_SIZE);
#endif
}

HBAgent::~HBAgent() {
}

bool HBAgent::get_is_player_controlled() const {
	return is_player_controlled;
}

void HBAgent::set_is_player_controlled(bool p_is_player_controlled) {
	is_player_controlled = p_is_player_controlled;
}

HBAgent *HBAgent::get_target() const {
	return Object::cast_to<HBAgent>(ObjectDB::get_instance(target_agent));
}

void HBAgent::set_target(HBAgent *p_target) {
	if (p_target == nullptr) {
		target_agent = ObjectID();
		return;
	}
	target_agent = p_target->get_instance_id();
}

Ref<HBAttackData> HBAgent::get_attack_data(const StringName &p_name) {
	ERR_FAIL_COND_V_MSG(!attack_datas.has(p_name), Ref<HBAttackData>(), vformat("Attack %s did not exist", p_name));
	return attack_datas[p_name];
}

void HBAgent::receive_attack(HBAgent *p_attacker, Ref<HBAttackData> p_attack_data) {
	emit_signal("attack_received", p_attacker, p_attack_data);
}

void HBAgent::set_outline_mode(AgentOutlineMode p_outline_mode) {
	Skeleton3D *skel = get_skeleton_node();

	Color color;
	switch (p_outline_mode) {
		case SOFT_TARGET: {
			color = Color(1.0f, 0.64f, 0.0f);
		} break;
		case TARGET_ASSASSINATE: {
			color = Color(1.0f, 0.0f, 0.0f);
		} break;
		case TARGET_COMBAT: {
			color = Color(1.0f, 1.0f, 1.0f);
		} break;
		default:;
	}

	for (int i = 0; i < skel->get_child_count(); i++) {
		MeshInstance3D *mi = Object::cast_to<MeshInstance3D>(skel->get_child(i));
		if (!mi) {
			continue;
		}

		if (p_outline_mode == AgentOutlineMode::DISABLED) {
			mi->set_material_overlay(Ref<Material>());
			continue;
		}

		mi->set_material_overlay(outline_material);
		outline_material->set_shader_parameter("fresnel_color", color);
	}
}

bool HBAgent::get_is_invulnerable() const {
	return is_invulnerable;
}

void HBAgent::set_is_invulnerable(bool p_is_invulnerable) {
	is_invulnerable = p_is_invulnerable;
}

Quaternion HBAgent::get_graphics_rotation() const { return graphics_rotation; }

void HBAgent::set_graphics_rotation(const Quaternion &p_graphics_rotation) { graphics_rotation = p_graphics_rotation; }

Vector<HBAgent *> HBAgent::find_nearby_agents(float p_radius) const {
	Ref<SphereShape3D> sphere;
	sphere.instantiate();
	sphere->set_radius(p_radius);

	PhysicsDirectSpaceState3D *dss = get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::ShapeParameters params;
	params.shape_rid = sphere->get_rid();
	params.transform.origin = get_global_position();
	params.collision_mask = HBPhysicsLayers::LAYER_PLAYER;
	params.exclude.insert(get_rid());

	const int RESULT_COUNT = 5;
	PhysicsDirectSpaceState3D::ShapeResult results[RESULT_COUNT];
	int result_count = dss->intersect_shape(params, results, RESULT_COUNT);

	Vector<HBAgent *> agents;

	for (int i = 0; i < result_count; i++) {
		HBAgent *agent = Object::cast_to<HBAgent>(results[i].collider);
		if (agent) {
			agents.push_back(agent);
		}
	}

	return agents;
}

void HBAgent::set_ledge_detector_node(const NodePath &ledge_detector_path_) { ledge_detector_path = ledge_detector_path_; }

void HBAgent::add_attack(const Ref<HBAttackData> &p_attack_data) {
	ERR_FAIL_COND(attack_datas.has(p_attack_data->get_name()));
	attack_datas.insert(p_attack_data->get_name(), p_attack_data);
}

NodePath HBAgent::get_ledge_detector_node() const { return ledge_detector_path; }

Ref<HBAgentConstants> HBAgent::get_agent_constants() const {
	return agent_constants;
}

void HBAgent::set_agent_constants(const Ref<HBAgentConstants> &p_agent_constants) {
	agent_constants = p_agent_constants;
}

void HBAgent::apply_root_motion(const Ref<EPASOneshotAnimationNode> &p_animation_node, bool p_collide, float p_delta) {
	ERR_FAIL_COND(!p_animation_node.is_valid());
	// os_node.root_motion_starting_transform * os_node.get_root_motion_transform()
	Transform3D root_trf = p_animation_node->get_root_motion_starting_transform() * p_animation_node->get_root_motion_transform();
	Node3D *gn = _get_graphics_node();

	Transform3D trf = gn->get_global_transform();
	trf.basis = root_trf.basis;
	trf.basis.rotate(Vector3(0.0, 1.0, 0.0), Math::deg_to_rad(180.0f));
	if (graphics_position_intertializer.is_valid() && !graphics_position_intertializer->is_done()) {
		trf.origin = get_global_position() + graphics_position_intertializer->advance(p_delta);
	}
	graphics_rotation = trf.basis.get_rotation_quaternion();

	if (p_collide) {
		set_velocity((root_trf.origin - get_global_position()) / p_delta);
	} else {
		set_global_position(root_trf.origin);
	}
}

float HBAgent::get_height() const {
	return 1.40f;
}

float HBAgent::get_radius() const {
	return 0.2f;
}

void HBAgent::_start_inertialize_graphics_position(const Vector3 &p_prev_prev, const Vector3 &p_prev, const Vector3 &p_target, float p_delta, float p_duration) {
	graphics_position_intertializer = PositionInertializer::create(p_prev_prev, p_prev, p_target, p_duration, p_delta);
}

void HBAgent::inertialize_graphics_transform(const Transform3D &p_target, float p_duration) {
	inertialization_duration = p_duration;
	inertialization_queued = true;

	Vector3 current_pos = _get_graphics_node()->get_global_position();
	Vector3 target_pos = p_target.origin;
	Quaternion current_rot = _get_graphics_node()->get_global_basis();
	Quaternion target_rot = p_target.basis.get_rotation_quaternion();

	_start_inertialize_graphics_position(prev_graphics_position, current_pos, target_pos, get_physics_process_delta_time());
	set_global_position(p_target.origin);
	graphics_rotation_intertializer = RotationInertializer::create(prev_graphics_rotation, current_rot, target_rot, p_duration, get_physics_process_delta_time());
	graphics_rotation = p_target.basis.get_rotation_quaternion();
	_get_graphics_node()->set_global_position(target_pos + process_graphics_position_inertialization(0.0f));
	_get_graphics_node()->set_global_basis(process_graphics_rotation_inertialization(0.0f) * target_rot);
}

Vector3 HBAgent::process_graphics_position_inertialization(const float &p_delta) {
	if (graphics_position_intertializer.is_valid() && !graphics_position_intertializer->is_done()) {
		return graphics_position_intertializer->advance(p_delta);
	}
	return Vector3();
}

Quaternion HBAgent::process_graphics_rotation_inertialization(const float &p_delta) {
	if (graphics_rotation_intertializer.is_valid() && !graphics_rotation_intertializer->is_done()) {
		return graphics_rotation_intertializer->advance(p_delta);
	}
	return Quaternion();
}

void HBAgent::root_motion_begin(Ref<EPASOneshotAnimationNode> p_animation_node, float p_delta) {
	Transform3D root_trf = p_animation_node->get_root_motion_starting_transform() * p_animation_node->get_root_motion_transform();
	graphics_position_intertializer = PositionInertializer::create(prev_graphics_position, _get_graphics_node()->get_global_position(), root_trf.origin, 0.25f, p_delta);
}

Ref<Shape3D> HBAgent::get_collision_shape() {
	Ref<CylinderShape3D> body_shape;
	body_shape.instantiate();
	body_shape->set_height(get_height());
	body_shape->set_radius(get_radius());
	return body_shape;
}

void HBAgent::_bind_methods() {
	NODE_CACHE_BIND(attack_trail, MeshInstance3D, HBAgent);

	ClassDB::bind_method(D_METHOD("set_graphics_node", "path"), &HBAgent::set_graphics_node);
	ClassDB::bind_method(D_METHOD("get_graphics_node"), &HBAgent::get_graphics_node);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "graphics_node", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_graphics_node", "get_graphics_node");

	NODE_CACHE_BIND(tilt_node, Node3D, HBAgent);

	ClassDB::bind_method(D_METHOD("get_agent_constants"), &HBAgent::get_agent_constants);
	ClassDB::bind_method(D_METHOD("set_agent_constants", "agent_constants"), &HBAgent::set_agent_constants);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "agent_constants", PROPERTY_HINT_RESOURCE_TYPE, "HBAgentConstants"), "set_agent_constants", "get_agent_constants");

	NODE_CACHE_BIND(epas_controller, EPASController, HBAgent);
	NODE_CACHE_BIND(skeleton, Skeleton3D, HBAgent);

	ClassDB::bind_method(D_METHOD("set_input_action_state", "event", "state"), &HBAgent::set_input_action_state);

	ClassDB::bind_method(D_METHOD("get_desired_movement_input"), &HBAgent::get_desired_movement_input);
	ClassDB::bind_method(D_METHOD("get_desired_movement_input_transformed"), &HBAgent::get_desired_movement_input_transformed);
	ClassDB::bind_method(D_METHOD("set_movement_input", "movement_input"), &HBAgent::set_movement_input);

	ClassDB::bind_method(D_METHOD("set_movement_input_rotation", "movement_input_rotation"), &HBAgent::set_movement_input_rotation);
	ClassDB::bind_method(D_METHOD("get_movement_input_rotation"), &HBAgent::get_movement_input_rotation);
	ADD_PROPERTY(PropertyInfo(Variant::QUATERNION, "movement_input_rotation", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "set_movement_input_rotation", "get_movement_input_rotation");

	ClassDB::bind_method(D_METHOD("flush_inputs"), &HBAgent::flush_inputs);

	ADD_SIGNAL(MethodInfo("stopped_at_edge"));
	ADD_SIGNAL(MethodInfo("emit_sword_sparks"));
	ADD_SIGNAL(MethodInfo("parried"));
	ADD_SIGNAL(MethodInfo("died"));
	ADD_SIGNAL(MethodInfo(
			"attack_received",
			PropertyInfo(Variant::OBJECT, "attacker", PROPERTY_HINT_NODE_TYPE, "HBAgent"),
			PropertyInfo(Variant::OBJECT, "attack_data", PROPERTY_HINT_RESOURCE_TYPE, "HBAttackData")));

	ADD_SIGNAL(MethodInfo(
			"damage_received",
			PropertyInfo(Variant::OBJECT, "old_health"),
			PropertyInfo(Variant::OBJECT, "new_health")));

	ADD_SIGNAL(MethodInfo("attack_connected"));
	ADD_SIGNAL(MethodInfo("attack_aborted"));

	BIND_ENUM_CONSTANT(INPUT_ACTION_RUN);
	BIND_ENUM_CONSTANT(INPUT_ACTION_PARKOUR_DOWN);
	BIND_ENUM_CONSTANT(INPUT_ACTION_PARKOUR_UP);
	BIND_ENUM_CONSTANT(INPUT_ACTION_MAX);

	ClassDB::bind_method(D_METHOD("set_ledge_detector_node", "path"), &HBAgent::set_ledge_detector_node);
	ClassDB::bind_method(D_METHOD("get_ledge_detector_node"), &HBAgent::get_ledge_detector_node);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "ledge_detector_node", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Area3D"), "set_ledge_detector_node", "get_ledge_detector_node");

	ClassDB::bind_method(D_METHOD("set_starting_heading", "path"), &HBAgent::set_starting_heading);
	ClassDB::bind_method(D_METHOD("get_starting_heading"), &HBAgent::get_starting_heading);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "starting_heading"), "set_starting_heading", "get_starting_heading");

	ClassDB::bind_method(D_METHOD("add_attack", "attack"), &HBAgent::add_attack);
}

void HBAgent::_rotate_towards_velocity(float p_delta) {
	Vector3 movement_input = get_movement_input();
	Vector3 dir = get_effective_velocity();
	dir.y = 0.0f;
	dir.normalize();

	if (dir.is_normalized()) {
		const float angle = get_graphics_rotation().xform(Vector3(0.0f, 0.0f, -1.0f)).angle_to(dir);

		if (movement_input.length() > 0) {
			Vector3 planar_vel = get_effective_velocity();
			planar_vel.y = 0.0f;
			planar_vel.normalize();
			if (planar_vel.is_normalized()) {
				graphics_rotation = Quaternion(Vector3(0.0f, 0.0f, -1.0f), planar_vel).normalized();
			}
		}
	}
}

static Vector3 prev_velocities[2];

void HBAgent::_tilt_towards_acceleration(float p_delta) {
	prev_velocities[0] = prev_velocities[1];
	prev_velocities[1] = get_effective_velocity();
}

void HBAgent::_physics_process(float p_delta) {
	ERR_FAIL_COND(!agent_constants.is_valid());

	switch (movement_mode) {
		case MovementMode::MOVE_FALL: {
			set_desired_velocity(Vector3());
			handle_input(Vector3(0.0f, 0.0f, 0.0f), p_delta);
			update(p_delta);
		} break;
		case MovementMode::MOVE_GROUNDED: {
			/*Vector3 input_vector = get_desired_movement_input_transformed();
			Vector3 target_desired_velocity = input_vector * agent_constants->get_max_move_velocity();

			Vector3 prev_horiz_dir = get_velocity();
			prev_horiz_dir.y = 0.0f;
			prev_horiz_dir.normalize();
			if (prev_horiz_dir.length_squared() == 0.0f) {
				prev_horiz_dir = get_desired_movement_input_transformed().normalized();
			}

			bool was_at_edge = prev_horiz_dir.is_normalized() && is_at_edge(prev_horiz_dir);

			set_desired_velocity(desired_velocity);

			Vector3 prev_vel_horiz = get_velocity();
			prev_vel_horiz.y = 0.0f;

			Vector3 prev_pos = get_global_position();

			handle_input(desired_velocity, p_delta);
			update(p_delta);

			Vector3 horiz_dir = get_velocity();
			horiz_dir.y = 0.0f;
			horiz_dir.normalize();
			if (horiz_dir.length_squared() == 0.0f) {
				horiz_dir = get_desired_movement_input_transformed().normalized();
			}
			if (horiz_dir.length_squared() > 0 && is_at_edge(horiz_dir)) {
				desired_velocity = Vector3();
				velocity_spring_acceleration = Vector3();
				set_velocity(Vector3());
				if (was_at_edge) {
					set_global_position(prev_pos);
				}
				if (!was_at_edge) {
					emit_signal(SNAME("stopped_at_edge"));
				}
			}
			_rotate_towards_velocity(p_delta);*/
			//_tilt_towards_acceleration(p_delta);
		} break;
		case MovementMode::MOVE_MANUAL: {
			set_desired_velocity(Vector3());
			tilt_spring_velocity = Vector3();
		} break;
	}

	Vector3 pos_offset = process_graphics_position_inertialization(p_delta);
	Quaternion rot_offset = process_graphics_rotation_inertialization(p_delta);
	_get_graphics_node()->set_global_position(get_global_position() + pos_offset);
	_get_graphics_node()->set_global_basis(rot_offset * graphics_rotation);
	prev_graphics_position = _get_graphics_node()->get_global_position();
	prev_graphics_rotation = _get_graphics_node()->get_global_basis().get_rotation_quaternion();
}

bool HBAgent::is_at_edge(Vector3 p_direction) {
	ERR_FAIL_COND_V(!p_direction.is_normalized(), false);
	PhysicsDirectSpaceState3D *dss = get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::RayParameters ray_params;
	PhysicsDirectSpaceState3D::RayResult ray_result;
	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

	ray_params.from = get_global_position() + p_direction * get_radius();
	ray_params.to = ray_params.from;
	ray_params.to.y -= get_height() * 0.25f;
	ray_params.from.y += get_height();

	return !dss->intersect_ray(ray_params, ray_result);
}

void HBAgent::reset_desired_input_velocity_to(const Vector3 &p_new_vel) {
	desired_velocity = p_new_vel;
}

Area3D *HBAgent::get_ledge_detector() const {
	Area3D *ledge = Object::cast_to<Area3D>(get_node(ledge_detector_path));
	ERR_FAIL_NULL_V(ledge, nullptr);
	return ledge;
}

LocalVector<HBAgentParkourLedge *> HBAgent::get_overlapping_ledges() const {
	LocalVector<HBAgentParkourLedge *> ledges;
	Area3D *ledge_detector = get_ledge_detector();
	ERR_FAIL_NULL_V(ledge_detector, ledges);
	TypedArray<Area3D> overlapping_areas = ledge_detector->get_overlapping_areas();
	for (int i = 0; i < overlapping_areas.size(); i++) {
		HBAgentParkourLedge *ledge = Object::cast_to<HBAgentParkourLedge>(overlapping_areas[i]);
		if (ledge) {
			ledges.push_back(ledge);
		}
	}
	return ledges;
}

bool HBAgent::is_action_pressed(AgentInputAction p_action) const {
	return current_input_state.action_states[p_action];
}

bool HBAgent::is_action_just_pressed(AgentInputAction p_action) const {
	return current_input_state.action_states[p_action] && !prev_input_state.action_states[p_action];
}

bool HBAgent::is_action_just_released(AgentInputAction p_action) const {
	return !current_input_state.action_states[p_action] && prev_input_state.action_states[p_action];
}

uint64_t HBAgent::get_last_action_press_time(AgentInputAction p_action) const {
	return current_input_state.last_action_press_times[p_action];
}

uint64_t HBAgent::get_last_action_release_time(AgentInputAction p_action) const {
	return current_input_state.last_action_release_times[p_action];
}

void HBAgent::flush_inputs() {
	prev_input_state = current_input_state;
}

void HBAgent::_update_graphics_node_cache() {
	graphics_node_cache = ObjectID();

	if (has_node(graphics_node)) {
		Node *node = get_node(graphics_node);
		ERR_FAIL_COND_MSG(!node, "Cannot update actor graphics cache: Node cannot be found!");

		// Ensure its a Node3D
		Node3D *nd = Object::cast_to<Node3D>(node);
		ERR_FAIL_COND_MSG(!nd, "Cannot update actor graphics cache: NodePath does not point to a Node3D node!");

		graphics_node_cache = nd->get_instance_id();
	}
}

void HBAgent::set_graphics_node(NodePath p_path) {
	graphics_node = p_path;
	_update_graphics_node_cache();
}

NodePath HBAgent::get_graphics_node() const {
	return graphics_node;
}

Node3D *HBAgent::_get_graphics_node() {
	if (graphics_node_cache.is_valid()) {
		return Object::cast_to<Node3D>(ObjectDB::get_instance(graphics_node_cache));
	} else {
		_update_graphics_node_cache();
		if (graphics_node_cache.is_valid()) {
			return Object::cast_to<Node3D>(ObjectDB::get_instance(graphics_node_cache));
		}
	}

	return nullptr;
}

void HBAgent::set_movement_mode(MovementMode p_movement_mode) {
	if (p_movement_mode == MovementMode::MOVE_MANUAL) {
		if (get_tilt_node_node()) {
			// HACK-ish, should probably inertialize this
			get_tilt_node_node()->set_transform(Transform3D());
		}
		rotation_spring_velocity = Vector3();
	}
	movement_mode = p_movement_mode;
}

HBAgent::MovementMode HBAgent::get_movement_mode() const {
	return movement_mode;
}

void HBAgent::receive_damage(const int p_damage) {
	const int old_health = health;
	health = MAX(health - p_damage, 0);
	emit_signal("damage_received", old_health, health);
	if (health == 0) {
		emit_signal("died");
	}
}

void HBAgent::_notification(int p_what) {
#ifdef TOOLS_ENABLED
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
#endif
	switch (p_what) {
		case NOTIFICATION_READY: {
			if (Engine::get_singleton()->is_editor_hint()) {
				return;
			}
			Node3D *gn = _get_graphics_node();
			set_graphics_rotation(Basis::from_euler(Vector3(0.0f, starting_heading, 0.0f)));
			if (gn) {
				current_rotation = gn->get_global_transform().basis.get_rotation_quaternion();
				gn->set_global_basis(graphics_rotation);
				last_last_rotation = current_rotation;
				last_rotation = current_rotation;
				rotation_spring_target = gn->get_global_transform().basis.get_rotation_quaternion();
			}
		} break;
		case NOTIFICATION_PHYSICS_PROCESS: {
			_physics_process(get_physics_process_delta_time());
		} break;
#ifdef DEBUG_ENABLED
		case NOTIFICATION_ENTER_TREE: {
			REGISTER_DEBUG(this);
		} break;
		case NOTIFICATION_EXIT_TREE: {
			UNREGISTER_DEBUG(this);
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
			GodotImGui *gim = GodotImGui::get_singleton();
			if (gim && gim->is_debug_enabled(this)) {
				if (gim->begin_debug_window(this)) {
					ImGui::Text("Supported: %s", is_on_floor() ? "true" : "false");
					ImGui::Text("pos %s", String(get_global_position()).utf8().get_data());
					ImGui::Text("Velocity %s %.2f", String(get_effective_velocity()).utf8().get_data(), get_effective_velocity().length());
					ImGui::Text("Desired Velocity %s", String(_get_desired_velocity()).utf8().get_data());
					ImGui::Text("gn rot %s", String(_get_graphics_node()->get_rotation_degrees()).utf8().get_data());
					plot_t += get_process_delta_time();

					{
						for (int i = 1; i < VELOCITY_PLOT_SIZE; i++) {
							velocity_plot_lines_x.set(i - 1, velocity_plot_lines_x[i]);
							velocity_plot_lines_y.set(i - 1, velocity_plot_lines_y[i]);
							desired_velocity_plot_lines_y.set(i - 1, desired_velocity_plot_lines_y[i]);
						}
						velocity_plot_lines_x.set(VELOCITY_PLOT_SIZE - 1, plot_t);
						velocity_plot_lines_y.set(VELOCITY_PLOT_SIZE - 1, get_velocity().length());
						desired_velocity_plot_lines_y.set(VELOCITY_PLOT_SIZE - 1, desired_velocity.length());

						String title = vformat("Velocity\n%.2f m/s", get_velocity().length());
						if (ImPlot::BeginPlot(title.utf8().get_data(), ImVec2(-1, 200.0f), ImPlotFlags_CanvasOnly & ~(ImPlotFlags_NoTitle | ImPlotFlags_NoLegend))) {
							ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoTickLabels);
							ImPlot::SetupAxisLimits(ImAxis_X1, velocity_plot_lines_x[0], velocity_plot_lines_x[velocity_plot_lines_x.size() - 1], ImGuiCond_Always);
							ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0f, 4.0f);
							ImPlot::PlotLine("Velocity", velocity_plot_lines_x.ptr(), velocity_plot_lines_y.ptr(), VELOCITY_PLOT_SIZE);
							ImPlot::PlotLine("Desired velocity", velocity_plot_lines_x.ptr(), desired_velocity_plot_lines_y.ptr(), VELOCITY_PLOT_SIZE);
							ImPlot::EndPlot();
						}
					}

					//ImGui::Separator();
					//ImGui::Text("Tilt spring velocity %s", String(get_global_position()).utf8().ptr());
				}
				ImGui::End();
			}
		} break;
#endif
	}
}

Ref<HBAgentConstants> HBAgent::_get_agent_constants() const {
	// same as get_agent_constants but with checking
	ERR_FAIL_COND_V(agent_constants.is_null(), nullptr);
	return agent_constants;
}

Vector3 HBAgent::_get_desired_velocity() const {
	Vector3 input_vector = get_desired_movement_input_transformed();
	Vector3 desvel = input_vector * agent_constants->get_max_move_velocity();
	return desvel;
}

void HBAgent::set_input_action_state(AgentInputAction p_event, bool p_state) {
	if (p_state != current_input_state.action_states[p_event]) {
		if (p_state) {
			current_input_state.last_action_press_times[p_event] = OS::get_singleton()->get_ticks_usec();
		} else {
			current_input_state.last_action_release_times[p_event] = OS::get_singleton()->get_ticks_usec();
		}
	}
	current_input_state.action_states[p_event] = p_state;
}

void HBAgent::set_movement_input(Vector3 p_movement_input) {
	ERR_FAIL_COND(p_movement_input.y != 0.0f);
	ERR_FAIL_COND(p_movement_input.length_squared() > 1.0f + UNIT_EPSILON);
	current_input_state.movement = p_movement_input;
}

Vector3 HBAgent::get_movement_input() const {
	return current_input_state.movement;
}

void HBAgent::set_movement_input_rotation(Quaternion p_movement_input_rotation) {
	ERR_FAIL_COND(!p_movement_input_rotation.is_normalized());
	current_input_state.movement_input_rotation = p_movement_input_rotation;
}

Quaternion HBAgent::get_movement_input_rotation() const {
	return current_input_state.movement_input_rotation;
}

Vector3 HBAgent::get_desired_movement_input() const {
	float movement_mul = is_action_pressed(AgentInputAction::INPUT_ACTION_RUN) ? 1.0f : 0.5f;
	return current_input_state.movement * movement_mul;
}
Vector3 HBAgent::get_desired_movement_input_transformed() const {
	float movement_mul = is_action_pressed(AgentInputAction::INPUT_ACTION_RUN) ? 1.0f : 0.5f;
	return current_input_state.movement_input_rotation.xform(current_input_state.movement * movement_mul);
}

void HBAttackData::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_name"), &HBAttackData::get_name);
	ClassDB::bind_method(D_METHOD("set_name", "name"), &HBAttackData::set_name);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "name"), "set_name", "get_name");

	ClassDB::bind_method(D_METHOD("get_animation"), &HBAttackData::get_animation);
	ClassDB::bind_method(D_METHOD("set_animation", "animation"), &HBAttackData::set_animation);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "animation", PROPERTY_HINT_RESOURCE_TYPE, "EPASAnimation"), "set_animation", "get_animation");

	ClassDB::bind_method(D_METHOD("get_mesh"), &HBAttackData::get_mesh);
	ClassDB::bind_method(D_METHOD("set_mesh", "mesh"), &HBAttackData::set_mesh);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "mesh", PROPERTY_HINT_RESOURCE_TYPE, "Mesh"), "set_mesh", "get_mesh");

	ClassDB::bind_method(D_METHOD("get_transition_index"), &HBAttackData::get_transition_index);
	ClassDB::bind_method(D_METHOD("set_transition_index", "transition_index"), &HBAttackData::set_transition_index);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "transition_index"), "set_transition_index", "get_transition_index");

	ClassDB::bind_method(D_METHOD("get_hit_time"), &HBAttackData::get_hit_time);
	ClassDB::bind_method(D_METHOD("set_hit_time", "hit_time"), &HBAttackData::set_hit_time);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "hit_time"), "set_hit_time", "get_hit_time");

	ClassDB::bind_method(D_METHOD("get_hitstop_duration"), &HBAttackData::get_hitstop_duration);
	ClassDB::bind_method(D_METHOD("set_hitstop_duration", "hit_stop_duration"), &HBAttackData::set_hitstop_duration);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "hitstop_duration"), "set_hitstop_duration", "get_hitstop_duration");

	ClassDB::bind_method(D_METHOD("get_attack_direction"), &HBAttackData::get_attack_direction);
	ClassDB::bind_method(D_METHOD("set_attack_direction", "attack_direction"), &HBAttackData::set_attack_direction);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "attack_direction", PROPERTY_HINT_ENUM, "Right"), "set_attack_direction", "get_attack_direction");

	ClassDB::bind_method(D_METHOD("get_cancel_time"), &HBAttackData::get_cancel_time);
	ClassDB::bind_method(D_METHOD("set_cancel_time", "cancel_time"), &HBAttackData::set_cancel_time);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cancel_time"), "set_cancel_time", "get_cancel_time");

	ClassDB::bind_method(D_METHOD("get_next_attack"), &HBAttackData::get_next_attack);
	ClassDB::bind_method(D_METHOD("set_next_attack", "next_attack"), &HBAttackData::set_next_attack);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "next_attack"), "set_next_attack", "get_next_attack");
}

float HBAttackData::get_cancel_time() const { return cancel_time; }

void HBAttackData::set_cancel_time(float p_cancel_time) { cancel_time = p_cancel_time; }
