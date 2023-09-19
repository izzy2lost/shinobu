#include "agent.h"
#include "physics_layers.h"
#include "springs.h"

#ifdef DEBUG_ENABLED
#include "implot.h"
#include "modules/imgui/godot_imgui.h"
#endif

#include "modules/imgui/godot_imgui_macros.h"

#include "core/config/project_settings.h"
#include "scene/resources/cylinder_shape_3d.h"

HBAgent::HBAgent() :
		JoltCharacterBody3D() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_physics_process(true);
		set_process_internal(true);
		agent_constants.instantiate();
	}
#ifdef DEBUG_ENABLED
	velocity_plot_lines_y.resize_zeroed(VELOCITY_PLOT_SIZE);
	velocity_plot_lines_x.resize_zeroed(VELOCITY_PLOT_SIZE);
	desired_velocity_plot_lines_y.resize_zeroed(VELOCITY_PLOT_SIZE);
	acceleration_plot_lines_x.resize_zeroed(VELOCITY_PLOT_SIZE);
	acceleration_plot_lines_y.resize_zeroed(VELOCITY_PLOT_SIZE);
#endif
}

HBAgent::~HBAgent() {
}

Ref<HBAgentConstants> HBAgent::get_agent_constants() const {
	return agent_constants;
}

void HBAgent::set_agent_constants(const Ref<HBAgentConstants> &p_agent_constants) {
	agent_constants = p_agent_constants;
}

void HBAgent::apply_root_motion(const Ref<EPASOneshotAnimationNode> &p_animation_node, float p_delta) {
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
	gn->set_global_transform(trf);
	set_global_position(root_trf.origin);
}

float HBAgent::get_height() const {
	return 1.40f;
}

float HBAgent::get_radius() const {
	return 0.2f;
}

void HBAgent::inertialize_graphics_position(float p_duration) {
	graphics_inertialization_queued = true;
	graphics_inertialization_duration = p_duration;
}

void HBAgent::root_motion_begin(Ref<EPASOneshotAnimationNode> p_animation_node, float p_delta) {
	Transform3D root_trf = p_animation_node->get_root_motion_starting_transform() * p_animation_node->get_root_motion_transform();
	print_line(root_trf.origin, get_global_position(), root_trf.origin - get_global_position());
	graphics_position_intertializer = PositionInertializer::create(prev_graphics_position, _get_graphics_node()->get_global_position(), root_trf.origin, 0.25f, p_delta);
}

Ref<Shape3D> HBAgent::get_collision_shape() {
	Ref<CylinderShape3D> body_shape;
	body_shape.instantiate();
	body_shape->set_height(get_height());
	body_shape->set_radius(get_radius());
	return body_shape;
}

void HBAgent::inertialize_graphics_rotation(Quaternion p_target_rot, bool p_now) {
	rotation_inertialization_queued = true;
	queued_rotation_inertialization_target = p_target_rot;
	if (p_now) {
		_rotation_inertialization_process(get_physics_process_delta_time());
	}
}

void HBAgent::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_graphics_node", "path"), &HBAgent::set_graphics_node);
	ClassDB::bind_method(D_METHOD("get_graphics_node"), &HBAgent::get_graphics_node);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "graphics_node", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_graphics_node", "get_graphics_node");
	ClassDB::bind_method(D_METHOD("set_tilt_node", "path"), &HBAgent::set_tilt_node);
	ClassDB::bind_method(D_METHOD("get_tilt_node"), &HBAgent::get_tilt_node);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "tilt_node", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_tilt_node", "get_tilt_node");

	ClassDB::bind_method(D_METHOD("get_agent_constants"), &HBAgent::get_agent_constants);
	ClassDB::bind_method(D_METHOD("set_agent_constants", "agent_constants"), &HBAgent::set_agent_constants);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "agent_constants", PROPERTY_HINT_RESOURCE_TYPE, "HBAgentConstants"), "set_agent_constants", "get_agent_constants");

	ClassDB::bind_method(D_METHOD("set_epas_controller_node", "epas_controller_node"), &HBAgent::set_epas_controller_node);
	ClassDB::bind_method(D_METHOD("get_epas_controller_node"), &HBAgent::get_epas_controller_node);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "epas_controller_node", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "EPASController"), "set_epas_controller_node", "get_epas_controller_node");

	ClassDB::bind_method(D_METHOD("set_input_action_state", "event", "state"), &HBAgent::set_input_action_state);

	ClassDB::bind_method(D_METHOD("get_desired_movement_input"), &HBAgent::get_desired_movement_input);
	ClassDB::bind_method(D_METHOD("get_desired_movement_input_transformed"), &HBAgent::get_desired_movement_input_transformed);
	ClassDB::bind_method(D_METHOD("set_movement_input", "movement_input"), &HBAgent::set_movement_input);

	ClassDB::bind_method(D_METHOD("set_movement_input_rotation", "movement_input_rotation"), &HBAgent::set_movement_input_rotation);
	ClassDB::bind_method(D_METHOD("get_movement_input_rotation"), &HBAgent::get_movement_input_rotation);
	ADD_PROPERTY(PropertyInfo(Variant::QUATERNION, "movement_input_rotation", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "set_movement_input_rotation", "get_movement_input_rotation");

	ClassDB::bind_method(D_METHOD("flush_inputs"), &HBAgent::flush_inputs);

	ADD_SIGNAL(MethodInfo("stopped_at_edge"));

	BIND_ENUM_CONSTANT(INPUT_ACTION_RUN);
	BIND_ENUM_CONSTANT(INPUT_ACTION_PARKOUR_DOWN);
	BIND_ENUM_CONSTANT(INPUT_ACTION_PARKOUR_UP);
	BIND_ENUM_CONSTANT(INPUT_ACTION_MAX);
}

void HBAgent::_rotate_towards_velocity(float p_delta) {
	Vector3 input_vector = get_desired_movement_input_transformed();
	Node3D *gn = _get_graphics_node();
	Vector3 gn_forward = gn->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, -1.0f));
	if (graphics_inertialization_queued) {
		gn_forward = queued_rotation_inertialization_target.xform(Vector3(0.0f, 0.0f, -1.0f));
	} else if (rot_inertializer.is_valid() && !rot_inertializer->is_done()) {
		gn_forward = inertialization_target.xform(Vector3(0.0f, 0.0f, -1.0f));
	}
	Vector3 horizontal_vel = get_velocity();
	horizontal_vel.y = 0.0f;

	if (horizontal_vel.length_squared() > 0) {
		float angle = gn_forward.angle_to(horizontal_vel.normalized());
		if (rot_inertializer.is_valid() && !rot_inertializer->is_done() && _get_desired_velocity().length_squared() > 0) {
			angle = gn_forward.angle_to(_get_desired_velocity().normalized());
		}
		if (angle > Math::deg_to_rad(40.0f)) {
			Quaternion target_rot = Quaternion(Vector3(0.0f, 0.0f, -1.0f), horizontal_vel.normalized());

			inertialize_graphics_rotation(target_rot);
			print_line("ANGLE", Math::rad_to_deg(angle));
			//_rotation_inertialization_process(p_delta);
			//return;
		}
	}

	if (gn && agent_constants.is_valid()) {
		if (input_vector.length_squared() > 0 && get_velocity().length_squared() > 0) {
			Vector3 new_dir = get_velocity();
			new_dir.y = 0.0f;
			new_dir.normalize();
			if (new_dir.is_normalized()) {
				Quaternion rot_target = Quaternion(Vector3(0.0f, 0.0f, -1.0f), new_dir).normalized();
				rotation_spring_target = rot_target;
			}
		}
	}

	HBSprings::simple_spring_damper_exact_quat(
			current_rotation,
			rotation_spring_velocity,
			rotation_spring_target,
			0.05f,
			p_delta);

	Transform3D new_trf = gn->get_global_transform();
	new_trf.basis = current_rotation;
	gn->set_global_transform(new_trf);
}

void HBAgent::_tilt_towards_acceleration(float p_delta) {
	Node3D *tn = _get_tilt_node();

	if (tn && agent_constants.is_valid()) {
		// Handle acceleration tilting of the actor
		Vector3 accel = velocity_spring_acceleration;
		float alpha = 0.2;
		// Smooth accel using a low pass filter
		smoothed_accel = smoothed_accel * (1 - alpha) + accel * alpha;
		accel = smoothed_accel;
		accel.y = 0.0f;
		Vector3 tilt_axis = accel.cross(Vector3(0.0f, 1.0f, 0.0));
		tilt_axis.normalize();
		// rough calculation of maximum tilt, seems to work ok
		float max_accel = agent_constants->get_max_move_velocity() / agent_constants->get_velocity_spring_halflife();
		float max_tilt_angle_degrees = agent_constants->get_tilt_max_angle_degrees();
		real_t angle = Math::deg_to_rad(CLAMP(-accel.length() / max_accel, -1.0f, 1.0f) * max_tilt_angle_degrees);

		Transform3D global_trf = tn->get_global_transform();

		if (!tilt_axis.is_normalized()) {
			tilt_axis = global_trf.get_basis().get_column(Vector3::AXIS_X);
		}

		Quaternion rotation_goal(tilt_axis, angle);

		Quaternion current_rot = tn->get_global_transform().basis.get_rotation_quaternion();
		rotation_goal = rotation_goal * tn->get_parent_node_3d()->get_global_transform().get_basis().get_rotation_quaternion();
		HBSprings::simple_spring_damper_exact_quat(current_rot, tilt_spring_velocity, rotation_goal, agent_constants->get_tilt_spring_halflife(), p_delta);
		global_trf.basis = Basis(current_rot);
		tn->set_global_transform(global_trf);
	}
}

void HBAgent::_physics_process(float p_delta) {
	ERR_FAIL_COND(!agent_constants.is_valid());
	prev_position = get_global_position();
	switch (movement_mode) {
		case MovementMode::MOVE_FALL: {
			set_desired_velocity(Vector3());
			handle_input(Vector3(0.0f, 0.0f, 0.0f), p_delta);
		} break;
		case MovementMode::MOVE_GROUNDED: {
			Vector3 input_vector = get_desired_movement_input_transformed();
			Vector3 target_desired_velocity = input_vector * agent_constants->get_max_move_velocity();

			Vector3 prev_horiz_dir = get_velocity();
			prev_horiz_dir.y = 0.0f;
			prev_horiz_dir.normalize();
			if (prev_horiz_dir.length_squared() == 0.0f) {
				prev_horiz_dir = get_desired_movement_input_transformed().normalized();
			}

			bool was_at_edge = prev_horiz_dir.is_normalized() && is_at_edge(prev_horiz_dir);

			HBSprings::velocity_spring_vector3(
					desired_velocity,
					velocity_spring_acceleration,
					target_desired_velocity,
					agent_constants->get_velocity_spring_halflife(),
					p_delta);
			set_desired_velocity(desired_velocity);

			Vector3 prev_vel_horiz = get_velocity();
			prev_vel_horiz.y = 0.0f;

			Vector3 prev_pos = get_global_position();

			handle_input(input_vector, p_delta);
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
			_rotate_towards_velocity(p_delta);
			_tilt_towards_acceleration(p_delta);
		} break;
		case MovementMode::MOVE_MANUAL: {
			set_desired_velocity(Vector3());
			velocity_spring_acceleration = Vector3();
			tilt_spring_velocity = Vector3();
		} break;
	}
	_rotation_inertialization_process(p_delta);
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

void HBAgent::_rotation_inertialization_process(float p_delta) {
	Node3D *gn = _get_graphics_node();

	if (rotation_inertialization_queued) {
		Quaternion target_rot = queued_rotation_inertialization_target;
		rot_inertializer = RotationInertializer::create(last_last_rotation, last_rotation, target_rot, 0.15f, p_delta);
		inertialization_target = target_rot;
		rotation_spring_target = target_rot;
		current_rotation = target_rot;
		rotation_spring_velocity = Vector3();
		rotation_inertialization_queued = false;
		p_delta = 0.0f;
	}

	if (rot_inertializer.is_valid() && !rot_inertializer->is_done()) {
		Quaternion rot_offset = rot_inertializer->advance(p_delta);
		Transform3D new_trf = gn->get_global_transform();
		new_trf.basis = current_rotation * rot_offset;
		gn->set_global_transform(new_trf);
	}
	last_last_rotation = last_rotation;
	last_rotation = gn->get_global_transform().basis.get_rotation_quaternion();
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

void HBAgent::_update_tilt_node_cache() {
	tilt_node_cache = ObjectID();

	if (has_node(tilt_node)) {
		Node *node = get_node(tilt_node);
		ERR_FAIL_COND_MSG(!node, "Cannot update actor tilt node cache: Node cannot be found!");

		// Ensure its a Node3D
		Node3D *nd = Object::cast_to<Node3D>(node);
		ERR_FAIL_COND_MSG(!nd, "Cannot update actor tilt node cache: Node3D Nodepath does not point to a Node3D node!");

		tilt_node_cache = nd->get_instance_id();
	}
}

void HBAgent::_update_epas_controller_cache() {
	epas_controller_cache = ObjectID();

	if (has_node(epas_controller_node)) {
		Node *node = get_node(epas_controller_node);
		ERR_FAIL_COND_MSG(!node, "Cannot update actor tilt node cache: Node cannot be found!");

		// Ensure its an EPASController node
		EPASController *nd = Object::cast_to<EPASController>(node);
		ERR_FAIL_COND_MSG(!nd, "Cannot update actor tilt node cache: Nodepath does not point to a EPASController node!");

		epas_controller_cache = nd->get_instance_id();
	}
}

void HBAgent::set_graphics_node(NodePath p_path) {
	graphics_node = p_path;
	_update_graphics_node_cache();
}

NodePath HBAgent::get_graphics_node() const {
	return graphics_node;
}

NodePath HBAgent::get_tilt_node() const {
	return tilt_node;
}

void HBAgent::set_tilt_node(NodePath p_path) {
	tilt_node = p_path;
	_update_tilt_node_cache();
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

Node3D *HBAgent::_get_tilt_node() {
	if (tilt_node_cache.is_valid()) {
		return Object::cast_to<Node3D>(ObjectDB::get_instance(tilt_node_cache));
	} else {
		_update_tilt_node_cache();
		if (tilt_node_cache.is_valid()) {
			return Object::cast_to<Node3D>(ObjectDB::get_instance(tilt_node_cache));
		}
	}

	return nullptr;
}

EPASController *HBAgent::_get_epas_controller() {
	if (epas_controller_cache.is_valid()) {
		return Object::cast_to<EPASController>(ObjectDB::get_instance(epas_controller_cache));
	} else {
		_update_epas_controller_cache();
		if (epas_controller_cache.is_valid()) {
			return Object::cast_to<EPASController>(ObjectDB::get_instance(epas_controller_cache));
		}
	}

	return nullptr;
}

void HBAgent::set_movement_mode(MovementMode p_movement_mode) {
	if (p_movement_mode == MovementMode::MOVE_MANUAL) {
		if (_get_tilt_node()) {
			// HACK-ish, should probably inertialize this
			_get_tilt_node()->set_transform(Transform3D());
		}
		velocity_spring_acceleration = Vector3();
		rotation_spring_velocity = Vector3();
	}
	movement_mode = p_movement_mode;
}

HBAgent::MovementMode HBAgent::get_movement_mode() const {
	return movement_mode;
}

NodePath HBAgent::get_epas_controller_node() const {
	return epas_controller_node;
}

void HBAgent::set_epas_controller_node(const NodePath &p_epas_controller_node) {
	epas_controller_node = p_epas_controller_node;
	_update_epas_controller_cache();
}

Vector3 HBAgent::get_previous_position() const {
	return prev_position;
}

void HBAgent::_notification(int p_what) {
#ifdef TOOLS_ENABLED
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
#endif
	switch (p_what) {
		case NOTIFICATION_READY: {
			Node3D *gn = _get_graphics_node();
			if (gn) {
				current_rotation = gn->get_global_transform().basis.get_rotation_quaternion();
				last_last_rotation = current_rotation;
				last_rotation = current_rotation;
				rotation_spring_target = gn->get_global_transform().basis.get_rotation_quaternion();
			}
		} break;
		case NOTIFICATION_PHYSICS_PROCESS: {
			_physics_process(get_physics_process_delta_time());
		} break;
		case NOTIFICATION_ENTER_TREE: {
			REGISTER_DEBUG(this);
		} break;
		case NOTIFICATION_EXIT_TREE: {
			UNREGISTER_DEBUG(this);
		} break;
#ifdef DEBUG_ENABLED
		case NOTIFICATION_INTERNAL_PROCESS: {
			GodotImGui *gim = GodotImGui::get_singleton();
			if (gim && gim->is_debug_enabled(this)) {
				if (gim->begin_debug_window(this)) {
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
						desired_velocity_plot_lines_y.set(VELOCITY_PLOT_SIZE - 1, _get_desired_velocity().length());

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

					{
						for (int i = 1; i < VELOCITY_PLOT_SIZE; i++) {
							acceleration_plot_lines_x.set(i - 1, acceleration_plot_lines_x[i]);
							acceleration_plot_lines_y.set(i - 1, acceleration_plot_lines_y[i]);
						}
						float accel_number = Math::abs(get_velocity().angle_to(velocity_spring_acceleration));
						accel_number = Math::remap(accel_number, 0.0f, Math::deg_to_rad(180.0f), 1.0f, -1.0f);
						accel_number *= velocity_spring_acceleration.length();
						acceleration_plot_lines_x.set(VELOCITY_PLOT_SIZE - 1, plot_t);
						acceleration_plot_lines_y.set(VELOCITY_PLOT_SIZE - 1, accel_number);

						String title = vformat("Acceleration\n%.2f m/s2", accel_number);
						if (ImPlot::BeginPlot(title.utf8().get_data(), ImVec2(-1, 200.0f), ImPlotFlags_CanvasOnly & ~ImPlotFlags_NoTitle)) {
							ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoTickLabels);
							ImPlot::SetupAxisLimits(ImAxis_X1, acceleration_plot_lines_x[0], acceleration_plot_lines_x[VELOCITY_PLOT_SIZE - 1], ImGuiCond_Always);
							ImPlot::SetupAxisLimits(ImAxis_Y1, -10.0f, 10.0f);
							ImPlot::PlotLine("Acceleration", acceleration_plot_lines_x.ptr(), acceleration_plot_lines_y.ptr(), VELOCITY_PLOT_SIZE);
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
