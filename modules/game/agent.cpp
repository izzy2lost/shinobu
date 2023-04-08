#include "agent.h"
#include "utils.h"

#ifdef DEBUG_ENABLED
#include "implot.h"
#include "modules/imgui/godot_imgui.h"
#endif

#include "modules/imgui/godot_imgui_macros.h"

#include "core/config/project_settings.h"
#include "scene/resources/cylinder_shape_3d.h"

HBAgent::HBAgent() :
		CharacterBody3D() {
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
	REGISTER_DEBUG(this);
}

HBAgent::~HBAgent() {
	UNREGISTER_DEBUG(this);
}

Ref<HBAgentConstants> HBAgent::get_agent_constants() const {
	return agent_constants;
}

void HBAgent::set_agent_constants(const Ref<HBAgentConstants> &p_agent_constants) {
	agent_constants = p_agent_constants;
}

void HBAgent::apply_root_motion(const Ref<EPASOneshotAnimationNode> &p_animation_node) {
	ERR_FAIL_COND(!p_animation_node.is_valid());
	// os_node.root_motion_starting_transform * os_node.get_root_motion_transform()
	Transform3D root_trf = p_animation_node->get_root_motion_starting_transform() * p_animation_node->get_root_motion_transform();
	Node3D *gn = _get_graphics_node();
	if (gn != nullptr) {
		Transform3D trf = gn->get_global_transform();
		trf.basis = root_trf.basis;
		trf.basis.rotate(Vector3(0.0, 1.0, 0.0), Math::deg_to_rad(180.0f));
		gn->set_global_transform(trf);
	}
	set_global_position(root_trf.origin);
}

float HBAgent::get_height() const {
	return 1.40f;
}

float HBAgent::get_radius() const {
	return 0.2f;
}

Ref<Shape3D> HBAgent::get_collision_shape() {
	Ref<CylinderShape3D> body_shape;
	body_shape.instantiate();
	body_shape->set_height(get_height());
	body_shape->set_radius(get_radius());
	return body_shape;
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
	ClassDB::bind_method(D_METHOD("set_movement_input", "movement_input"), &HBAgent::set_movement_input);

	BIND_ENUM_CONSTANT(INPUT_ACTION_RUN);
}

void HBAgent::_rotate_towards_velocity(float p_delta) {
	Vector3 input_vector = get_desired_movement_input();
	Node3D *gn = _get_graphics_node();

	if (gn && agent_constants.is_valid()) {
		// Handle horizontal rotation of the actor
		if (input_vector.length() > 0.0) {
			Vector3 desired_look_normal = get_velocity();
			desired_look_normal.y = 0.0f;
			desired_look_normal.normalize();
			if (desired_look_normal.length() > 0.0f) {
				Vector3 forward = Vector3(0, 0, -1);
				Vector3 current_forward = gn->get_global_transform().basis.xform(Vector3(0.0, 0.0, -1.0));
				current_forward.y = 0.0f;
				current_forward.normalize();
				Vector3 up = Vector3(0, 1.0f, 0);

				float angle_diff = current_forward.signed_angle_to(desired_look_normal, up);

				float angle_delta = SIGN(angle_diff) * MIN(Math::abs(angle_diff), p_delta * Math::deg_to_rad(360.0f));
				Transform3D global_trf = gn->get_global_transform();
				Quaternion rot = Quaternion(up, angle_delta) * global_trf.basis.get_rotation_quaternion();
				global_trf.basis.set_quaternion(rot);
				gn->set_global_transform(global_trf);
			}
		}
	}

	last_last_rotation = last_rotation;
	last_rotation = gn->get_global_transform().basis.get_rotation_quaternion();
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
		HBUtils::simple_spring_damper_exact_quat(current_rot, tilt_spring_velocity, rotation_goal, agent_constants->get_tilt_spring_halflife(), p_delta);
		global_trf.basis = Basis(current_rot);
		tn->set_global_transform(global_trf);
	}
}

void HBAgent::_physics_process(float p_delta) {
	Vector3 vel = get_velocity();
	ERR_FAIL_COND(!agent_constants.is_valid());
	switch (movement_mode) {
		case MovementMode::MOVE_GROUNDED: {
			Vector3 input_vector = get_desired_movement_input();
			Vector3 desired_velocity = input_vector * agent_constants->get_max_move_velocity();
			HBUtils::velocity_spring_vector3(
					vel,
					velocity_spring_acceleration,
					desired_velocity,
					agent_constants->get_velocity_spring_halflife(),
					p_delta);
			vel += agent_constants->get_gravity() * p_delta;

			set_velocity(vel);

			move_and_slide();
			_rotate_towards_velocity(p_delta);
			_tilt_towards_acceleration(p_delta);
		} break;
		case MovementMode::MOVE_MANUAL: {
			velocity_spring_acceleration = Vector3();
			tilt_spring_velocity = Vector3();
		} break;
	}

	prev_input_state = current_input_state;
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

void HBAgent::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PHYSICS_PROCESS: {
			_physics_process(get_physics_process_delta_time());
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
#ifdef DEBUG_ENABLED
			GodotImGui *gim = GodotImGui::get_singleton();
			if (gim && gim->is_debug_enabled(this)) {
				if (gim->begin_debug_window(this)) {
					ImGui::Text("Velocity %s", String(get_linear_velocity()).utf8().get_data());
					ImGui::Text("gn rot %s", String(_get_graphics_node()->get_rotation_degrees()).utf8().get_data());
					plot_t += get_process_delta_time();

					{
						for (int i = 1; i < VELOCITY_PLOT_SIZE; i++) {
							velocity_plot_lines_x.set(i - 1, velocity_plot_lines_x[i]);
							velocity_plot_lines_y.set(i - 1, velocity_plot_lines_y[i]);
							desired_velocity_plot_lines_y.set(i - 1, desired_velocity_plot_lines_y[i]);
						}
						velocity_plot_lines_x.set(VELOCITY_PLOT_SIZE - 1, plot_t);
						velocity_plot_lines_y.set(VELOCITY_PLOT_SIZE - 1, get_linear_velocity().length());
						desired_velocity_plot_lines_y.set(VELOCITY_PLOT_SIZE - 1, _get_desired_velocity().length());

						String title = vformat("Velocity\n%.2f m/s", get_linear_velocity().length());
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
						float accel_number = Math::abs(get_linear_velocity().angle_to(velocity_spring_acceleration));
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
#endif
		} break;
	}
}

Ref<HBAgentConstants> HBAgent::_get_agent_constants() const {
	// same as get_agent_constants but with checking
	ERR_FAIL_COND_V(agent_constants.is_null(), nullptr);
	return agent_constants;
}

Vector3 HBAgent::_get_desired_velocity() const {
	Vector3 input_vector = get_desired_movement_input();
	Vector3 desired_velocity = input_vector * agent_constants->get_max_move_velocity();
	return desired_velocity;
}

void HBAgent::set_input_action_state(AgentInputAction p_event, bool p_state) {
	current_input_state.action_states[p_event] = p_state;
}

void HBAgent::set_movement_input(Vector3 p_movement_input) {
	ERR_FAIL_COND(p_movement_input.y != 0.0f);
	current_input_state.movement = p_movement_input;
}

Vector3 HBAgent::get_desired_movement_input() const {
	float movement_mul = is_action_pressed(AgentInputAction::INPUT_ACTION_RUN) ? 1.0f : 0.5f;
	return current_input_state.movement * movement_mul;
}
