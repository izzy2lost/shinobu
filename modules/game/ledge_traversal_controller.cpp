#include "ledge_traversal_controller.h"
#include "physics_layers.h"
#include "scene/3d/collision_shape_3d.h"
#include "scene/resources/immediate_mesh.h"
#include "scene/resources/sphere_shape_3d.h"
#include "springs.h"

bool is_wall_n(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) > p_floor_max_angle;
}

bool is_floor_n(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) <= p_floor_max_angle;
}

bool _find_ledge(CharacterBody3D *p_body, const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_wall_normal, Vector3 &p_out_ledge_normal, const Color &p_debug_color) {
	ERR_FAIL_COND_V(!p_body, false);

	PhysicsDirectSpaceState3D::RayParameters ray_params;
	PhysicsDirectSpaceState3D::RayResult ray_result;

	ray_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

	PhysicsDirectSpaceState3D *dss = p_body->get_world_3d()->get_direct_space_state();
	ERR_FAIL_COND_V(!dss, false);

	Vector3 pos = p_from;
	Vector3 pos_target = p_to;

	// First we find the wall by firing towards it

	ray_params.to = pos_target;
	ray_params.from = pos;
	ray_params.exclude.insert(p_body->get_rid());

	if (!dss->intersect_ray(ray_params, ray_result)) {
		return false;
	}

	if (!is_wall_n(ray_result.normal, p_body->get_floor_max_angle())) {
		return false;
	}

	Vector3 wall_normal = ray_result.normal;

	Vector3 ledge_point = ray_result.position;

	// Then we try to find the ledge top point

	ray_params.to = ray_result.position - ray_result.normal * 0.01;
	ray_params.from = ray_params.to + Vector3(0.0f, 1.75f, 0.0f);

	if (!dss->intersect_ray(ray_params, ray_result)) {
		return false;
	}

	if (!is_floor_n(ray_result.normal, p_body->get_floor_max_angle())) {
		return false;
	}

	p_out_wall_normal = wall_normal;
	p_out_ledge_normal = ray_result.normal;

	p_out = ray_result.position;
	ledge_point.y = ray_result.position.y;

	p_out = ledge_point;

	return true;
}

bool _find_ledge_sweep(CharacterBody3D *p_body, const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_wall_normal, Vector3 &p_out_ledge_normal, const Color &p_debug_color, Vector3 p_sweep_offset, int p_sweep_iterations) {
	for (int i = 0; i < p_sweep_iterations; i++) {
		Vector3 offset = p_sweep_offset * (i / ((float)p_sweep_iterations - 1.0f));

		bool result = _find_ledge(p_body, p_from + offset, p_to + offset, p_out, p_out_wall_normal, p_out_ledge_normal, p_debug_color);
		if (result) {
			return true;
		}
	}

	return false;
}

void HBLedgeTraversalController::move_to_ledge(const Transform3D &p_ledge_transform) {
	Vector3 normal = p_ledge_transform.basis.xform(Vector3(0.0f, 0.0f, -1.0f));
	Vector3 new_pos = p_ledge_transform.origin;
	new_pos += normal * radius;
	new_pos.y -= 0.01f;
	out_rotation = Quaternion(Vector3(0.0f, 0.0f, -1.0f), -normal);
	current_wall = p_ledge_transform;
	set_global_position(new_pos);
	is_turning_in_place = false;
	_update_movement_dir_debug_graphic();
	_handle_limbs();
}

void HBLedgeTraversalController::update(float p_delta) {
	if (!debug_geo) {
		debug_geo = memnew(HBDebugGeometry);
		add_child(debug_geo);
		debug_geo->set_as_top_level(true);
		debug_geo->set_global_position(Vector3());
	}

	debug_geo->clear();
	debug_geo->debug_sphere(current_wall.origin, 0.05f, Color("RED"));
	debug_geo->debug_sphere(wall_being_rotated_towards.origin, 0.05f, Color("GREEN"));
	debug_geo->debug_line(current_wall.origin, current_wall.origin + current_wall.basis.xform(Vector3(0.0f, 0.0f, -1.0f)), Color("RED"));
	debug_geo->debug_line(wall_being_rotated_towards.origin, wall_being_rotated_towards.origin + wall_being_rotated_towards.basis.xform(Vector3(0.0f, 0.0f, -1.0f)), Color("GREEN"));
	if (is_turning_in_place) {
		_handle_turn_in_place(p_delta);
		return;
	}

	set_max_slides(1);

	// Try one, try to move in the direction of the wall
	movement_velocity_target = movement_input.x * max_movement_velocity;
	HBSprings::velocity_spring(&movement_velocity, &movement_acceleration, movement_velocity_target, 0.175f, p_delta);
	Vector3 vel = out_rotation.xform(Vector3(movement_velocity_target, 0.0f, 0.0f));
	if (vel.is_zero_approx()) {
		return;
	}
	Vector3 original_position = get_global_position();
	Vector3 movement_starting_position = original_position;
	set_velocity(vel);

	bool collided = move_and_slide();

	// Try two, try to move into the wall from our new position
	Vector3 current_wall_normal = current_wall.basis.xform(Vector3(0.0, 0.0, -1.0));
	if (!collided) {
		vel = -current_wall_normal * Math::abs(movement_velocity) * 1.0f;
		set_velocity(vel);
		collided = move_and_slide();
		movement_starting_position = get_global_position();
	}

	if (!collided) {
		set_global_position(original_position);
	} else {
		_handle_collision(original_position);
	}
}

void HBLedgeTraversalController::_handle_collision(const Vector3 &p_previous_pos) {
	PhysicsServer3D::MotionCollision closest_collision = get_slide_collision(0).collisions[0];
	// Find motion that's closest in direction to our travel direction
	for (int j = 0; j < get_slide_collision_count(); j++) {
		PhysicsServer3D::MotionResult motion = get_slide_collision(j);
		Vector3 travel_dir = motion.travel.normalized();
		for (int i = 1; i < motion.collision_count; i++) {
			float dot_a = travel_dir.dot(p_previous_pos.direction_to(closest_collision.position));
			float dot_b = travel_dir.dot(p_previous_pos.direction_to(motion.collisions[i].position));
			if (dot_a < dot_b) {
				closest_collision = motion.collisions[i];
			}
		}
	}

	Vector3 new_normal = get_global_position().direction_to(closest_collision.position);
	new_normal.y = 0.0f;
	new_normal.normalize();
	Quaternion new_rotation = Quaternion(Vector3(0.0f, 0.0f, -1.0f), new_normal);

	Transform3D new_wall_trf;
	new_wall_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), closest_collision.normal);
	new_wall_trf.origin = closest_collision.position;

	Vector3 wall_normal = new_wall_trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f));
	Vector3 from = new_wall_trf.origin + wall_normal * 0.5f - Vector3(0.0, 0.01f, 0.0f);
	Vector3 to = from - wall_normal;
	Vector3 out_ledge_pos;
	Vector3 out_ledge_normal;
	Vector3 out_wall_normal;
	bool found_ledge = false;
	for (int i = 1; i > -2; i -= 2) {
		Vector3 offset = Vector3(0.0, 1.0f, 0.0f).cross(wall_normal) * 0.05f * i;
		if (!_find_ledge_sweep(this, from, to, out_ledge_pos, out_wall_normal, out_ledge_normal, Color(), offset, 5)) {
			continue;
		}
		float new_y = out_ledge_pos.y;
		Vector3 gp = get_global_position();
		gp.y = new_y - 0.01;
		set_global_position(gp);
		found_ledge = true;
		break;
	}

	if (!found_ledge) {
		set_global_position(p_previous_pos);
		return;
	}

	if (new_rotation.angle_to(out_rotation) < Math::deg_to_rad(25.0f)) {
		if (new_normal.is_normalized()) {
			out_rotation = new_rotation.normalized();
			current_wall = new_wall_trf;
		}
	} else {
		turn_in_place_values[0].basis = out_rotation;
		turn_in_place_values[1].basis = new_rotation;
		is_turning_in_place = true;
		turn_progress = 0.0f;
		wall_being_rotated_towards = new_wall_trf;
		rotation_movement_dir = SIGN(movement_input.x);
		turn_in_place_values[0].origin = p_previous_pos;
		turn_in_place_values[1].origin = get_global_position();
		set_global_position(p_previous_pos);
	}
	_handle_limbs();
	_update_movement_dir_debug_graphic();
}

void HBLedgeTraversalController::_update_movement_dir_debug_graphic() {
	if (!debug_geo) {
		debug_geo = memnew(HBDebugGeometry);
		add_child(debug_geo);
		debug_geo->set_as_top_level(true);
		debug_geo->set_global_position(Vector3());
	}
	if (!movement_dir_debug_graphic) {
		Ref<ImmediateMesh> im;
		im.instantiate();

		Ref<StandardMaterial3D> mat;
		mat.instantiate();
		mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
		mat->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
		mat->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);

		im->surface_begin(Mesh::PRIMITIVE_LINE_STRIP, mat);
		const Vector3 up = Vector3(0.0f, 1.0f, 0.0f);
		const Vector3 forward = Vector3(0.0f, 0.0f, -1.0f);
		float size = radius + 0.1;
		const int VERTICES = 16;
		for (int i = 0; i < VERTICES; i++) {
			im->surface_set_color(Color("RED"));
			if (i == 0 || i == VERTICES - 1) {
				im->surface_add_vertex(forward * 1.3f * size);
				continue;
			}
			float p = i / (float)(VERTICES - 1);
			im->surface_add_vertex(forward.rotated(up, Math_TAU * p) * size);
		}
		im->surface_end();
		movement_dir_debug_graphic = memnew(MeshInstance3D);
		movement_dir_debug_graphic->set_mesh(im);
		add_child(movement_dir_debug_graphic);
	}

	movement_dir_debug_graphic->set_global_basis(out_rotation);
}

Vector3 HBLedgeTraversalController::get_transformed_movement_input() const {
	return out_rotation.xform(movement_input);
}

Vector3 HBLedgeTraversalController::get_movement_input() const {
	return movement_input;
}

void HBLedgeTraversalController::set_movement_input(const Vector3 &p_movement_input) {
	movement_input = p_movement_input;
}

void HBLedgeTraversalController::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_movement_input", "movement_input"), &HBLedgeTraversalController::set_movement_input);
	ClassDB::bind_method(D_METHOD("get_movement_input"), &HBLedgeTraversalController::get_movement_input);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "movement_input"), "set_movement_input", "get_movement_input");
	ClassDB::bind_method(D_METHOD("update", "delta"), &HBLedgeTraversalController::update);
	ClassDB::bind_method(D_METHOD("move_to_ledge", "ledge_transform"), &HBLedgeTraversalController::move_to_ledge);
}

void HBLedgeTraversalController::_handle_turn_in_place(float p_delta) {
	if (movement_input.x != 0.0f && SIGN(movement_input.x) != rotation_movement_dir) {
		rotation_movement_dir = -rotation_movement_dir;
		SWAP(turn_in_place_values[0], turn_in_place_values[1]);
		SWAP(current_wall, wall_being_rotated_towards);
		turn_progress = 1.0f - turn_progress;
	}

	Quaternion first_rot = turn_in_place_values[0].basis.get_rotation_quaternion();
	Quaternion second_rot = turn_in_place_values[1].basis.get_rotation_quaternion();
	float controller_perimeter = 2.0f * Math_PI * radius;
	// angular velocity will always be positive because we will always be going towards 1
	float angular_velocity = (Math_TAU / controller_perimeter) * abs(movement_velocity) * abs(movement_input.x);
	float angle_curr_to_target = first_rot.angle_to(second_rot);
	turn_progress = MIN(turn_progress + ((angular_velocity * p_delta) / angle_curr_to_target), 1.0f);
	out_rotation = first_rot.slerp(second_rot, turn_progress).normalized();
	set_global_position(turn_in_place_values[0].origin.lerp(turn_in_place_values[1].origin, turn_progress));
	_update_movement_dir_debug_graphic();

	print_line(turn_progress, angular_velocity);

	//debug_geo->debug_sphere(wall_being_rotated_towards.origin, 0.05f, Color("BLUE"));
	//debug_geo->debug_sphere(current_wall.origin, 0.05f, Color("ORANGE"));

	if (turn_progress >= 1.0f) {
		is_turning_in_place = false;
		out_rotation = turn_in_place_values[1].basis.get_rotation_quaternion().normalized();
		current_wall = wall_being_rotated_towards;
	} else {
		_handle_limbs();
	}
}

void HBLedgeTraversalController::_handle_limbs() {
	Vector3 forward = out_rotation.xform(Vector3(0.0f, 0.0f, -1.0f));
	Vector3 right = out_rotation.xform(Vector3(1.0f, 0.0f, 0.0f));

	for (int i = 0; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		AgentProceduralAnimator::AgentLimb limb = static_cast<AgentProceduralAnimator::AgentLimb>(i);
		limb_dangle_status[limb] = false;
		float dir = i % 2 == 0 ? -1.0f : 1.0f;
		Vector3 from = get_global_position() + right * radius * 0.5f * dir - Vector3(0.0f, height * 0.5, 0.0f);
		Vector3 to = from + forward;

		if (limb_test_body->get_child_count() == 0) {
			Ref<SphereShape3D> sphere;
			sphere.instantiate();
			sphere->set_radius(radius * 0.5f);
			CollisionShape3D *cs = memnew(CollisionShape3D);
			cs->set_shape(sphere);
			limb_test_body->add_child(cs);
			cs->set_position(Vector3());
			limb_test_shape = sphere;
		}

		// For hands we try higher up, so we can dangle the feet
		if (i <= AgentProceduralAnimator::LIMB_RIGHT_HAND) {
			from += Vector3(0.0f, height * 0.45, 0.0f);
			to += Vector3(0.0f, height * 0.45, 0.0f);
		}

		limb_test_body->set_global_position(from);
		PhysicsServer3D::MotionParameters motion_params;
		motion_params.from.origin = from;
		motion_params.max_collisions = 1;
		motion_params.motion = to - from;
		motion_params.exclude_bodies.insert(get_rid());
		PhysicsServer3D::MotionResult mot_result;

		debug_geo->debug_shape(limb_test_shape, limb_test_body->get_global_transform());
		debug_geo->debug_line(limb_test_body->get_global_transform().origin, limb_test_body->get_global_transform().origin + motion_params.motion);

		if (!limb_test_body->move_and_collide(motion_params, mot_result, false, false)) {
			limb_dangle_status[limb] = true;
			continue;
		}

		// Feet don't need to find the ledge above
		if (i > AgentProceduralAnimator::LIMB_RIGHT_HAND) {
			limb_transforms[limb].origin = mot_result.collisions[0].position;
			limb_transforms[limb].basis = Quaternion(Vector3(0.0f, 0.0f, 1.0f), mot_result.collisions[0].normal);
			continue;
		}

		Color col = i == 0 ? Color("RED") : Color("GREEN");

		Vector3 out_ledge_pos;
		Vector3 out_wall_normal;
		Vector3 out_ledge_normal;

		Vector3 ledge_from = mot_result.collisions[0].position + mot_result.collisions[0].normal + Vector3(0.0f, -radius * 0.5f, 0.0f);
		Vector3 ledge_to = mot_result.collisions[0].position - mot_result.collisions[0].normal + Vector3(0.0f, -radius * 0.5f, 0.0f);

		bool found = _find_ledge_sweep(
				this,
				ledge_from,
				ledge_to,
				out_ledge_pos,
				out_wall_normal,
				out_ledge_normal,
				Color("RED"),
				Vector3(),
				2);
		if (!found) {
			limb_dangle_status[limb] = true;
			continue;
		}

		limb_transforms[limb].origin = out_ledge_pos;
		limb_transforms[limb].basis = Quaternion(Vector3(0.0f, 0.0f, 1.0f), mot_result.collisions[0].normal);
	}

	for (int i = 0; i < 2; i++) {
		AgentProceduralAnimator::AgentLimb hand = static_cast<AgentProceduralAnimator::AgentLimb>(i);
		Color col = i == 0 ? Color("RED") : Color("GREEN");
		//debug_geo->debug_sphere(limb_transforms[hand].origin, radius * 0.05f, col);
	}
}

Quaternion HBLedgeTraversalController::get_graphics_rotation() const {
	return out_rotation;
}

void HBLedgeTraversalController::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			limb_test_body = memnew(CharacterBody3D);
			limb_test_body->set_collision_layer(0);
			limb_test_body->set_collision_mask(HBPhysicsLayers::LAYER_WORLD_GEO);
			limb_test_body->set_as_top_level(true);
			add_child(limb_test_body);
			limb_test_body->set_global_position(Vector3());
		} break;
	}
}

Transform3D HBLedgeTraversalController::get_limb_transform(AgentProceduralAnimator::AgentLimb p_limb) const {
	return limb_transforms[p_limb];
}

bool HBLedgeTraversalController::get_limb_is_dangling(AgentProceduralAnimator::AgentLimb p_limb) const {
	return limb_dangle_status[p_limb];
}

float HBLedgeTraversalController::get_movement_velocity() const {
	return movement_velocity;
}