/**************************************************************************/
/*  ledge_traversal_controller.cpp                                        */
/**************************************************************************/
/*                         This file is part of:                          */
/*                               SWANSONG                                 */
/*                          https://eirteam.moe                           */
/**************************************************************************/
/* Copyright (c) 2023-present Álex Román Núñez (EIRTeam).                 */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "ledge_traversal_controller.h"
#include "physics_layers.h"
#include "scene/3d/physics/character_body_3d.h"
#include "scene/resources/3d/cylinder_shape_3d.h"
#include "scene/resources/3d/sphere_shape_3d.h"
#include "scene/resources/immediate_mesh.h"
#include "springs.h"

bool is_wall_n(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) > p_floor_max_angle;
}

bool is_floor_n(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) <= p_floor_max_angle;
}

bool _find_ledge(CharacterBody3D *p_body, const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_wall_pos, Vector3 &p_out_wall_normal, Vector3 &p_out_ledge_normal, const Color &p_debug_color) {
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

	p_out_wall_pos = ray_result.position;

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

bool _find_ledge_sweep(CharacterBody3D *p_body, const Vector3 &p_from, const Vector3 &p_to, Vector3 &p_out, Vector3 &p_out_wall_pos, Vector3 &p_out_wall_normal, Vector3 &p_out_ledge_normal, const Color &p_debug_color, Vector3 p_sweep_offset, int p_sweep_iterations) {
	for (int i = 0; i < p_sweep_iterations; i++) {
		Vector3 offset = p_sweep_offset * (i / ((float)p_sweep_iterations - 1.0f));

		bool result = _find_ledge(p_body, p_from + offset, p_to + offset, p_out, p_out_wall_pos, p_out_wall_normal, p_out_ledge_normal, p_debug_color);
		if (result) {
			return true;
		}
	}

	return false;
}

void HBLedgeTraversalController::update(float p_delta) {
	if (!debug_geo) {
		debug_geo = memnew(HBDebugGeometry);
		add_child(debug_geo);
		debug_geo->set_as_top_level(true);
		debug_geo->set_global_position(Vector3());
	}

	debug_geo->clear();
	// Try one, try to move in the direction of the wall
	movement_velocity_target = (-movement_input.x) * max_movement_velocity;
	HBSprings::velocity_spring(&movement_velocity, &movement_acceleration, movement_velocity_target, 0.175f, p_delta);

	float new_offset = curve_offset;
	new_offset += movement_velocity * p_delta;
	new_offset = Math::fposmod(new_offset, ledge->get_curve()->get_baked_length());

	move_to_offset(new_offset);
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
	ClassDB::bind_method(D_METHOD("move_to_ledge", "ledge_transform", "offset"), &HBLedgeTraversalController::move_to_ledge);
}

void HBLedgeTraversalController::_handle_limbs() {
	Vector3 pos = ledge->get_ledge_transform_at_offset(curve_offset).origin;
	const float inner_offset = ledge->get_closest_offset(pos);

	for (int i = 0; i < AgentProceduralAnimator::LIMB_LEFT_FOOT; i++) {
		int offset_mul = 1 - i * 2;
		Transform3D hand_trf = calculate_limb_trf(ledge, radius, offset_mul * 0.089f, inner_offset, (AgentProceduralAnimator::AgentLimb)i);
		limb_transforms[i].origin = hand_trf.origin;
		limb_transforms[i].basis = hand_trf.basis;
	}

	for (int i = 0; i < AgentProceduralAnimator::LIMB_LEFT_FOOT; i++) {
		int offset_mul = 1 - i * 2;
		float offset = movement_velocity * 0.3f + (offset_mul * 0.089f);
		Transform3D hand_trf = calculate_limb_trf(ledge, radius, offset, inner_offset, (AgentProceduralAnimator::AgentLimb)i);
		limb_transforms_predicted[i].origin = hand_trf.origin;
		limb_transforms_predicted[i].basis = hand_trf.basis;
	}

	for (int i = AgentProceduralAnimator::LIMB_LEFT_FOOT; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		limb_transforms[i] = limb_transforms[i - 2];
		limb_transforms[i].origin -= Vector3(0.0f, height * 0.4f, 0.0f);

		limb_transforms_predicted[i] = limb_transforms_predicted[i - 2];
		limb_transforms_predicted[i].origin -= Vector3(0.0f, height * 0.4f, 0.0f);
	}
}

Transform3D HBLedgeTraversalController::calculate_limb_trf(HBAgentParkourLedge *p_ledge, float p_agent_radius, float p_limb_offset_diff, float p_agent_offset, const AgentProceduralAnimator::AgentLimb p_limb) const {
	Ref<Curve3D> curve = p_ledge->get_curve();
	real_t offset_diff = p_limb_offset_diff;
	Transform3D body_trf = p_ledge->get_ledge_transform_at_offset(p_agent_offset);

	Transform3D limb_trf = curve->sample_baked_with_rotation(Math::fposmod(p_agent_offset + offset_diff, curve->get_baked_length()));
	const int MAX_ITERS = 32;
	for (int i = 0; i < MAX_ITERS; i++) {
		limb_trf = curve->sample_baked_with_rotation(Math::fposmod(p_agent_offset + offset_diff, curve->get_baked_length()));
		if (limb_trf.origin.distance_to(body_trf.origin) <= p_agent_radius) {
			break;
		}
		offset_diff = Math::move_toward(offset_diff, 0.0f, Math::abs(p_limb_offset_diff / MAX_ITERS));
	}

	Vector3 normal = limb_trf.basis.get_rotation_quaternion().xform(Vector3(-1.0f, 0.0f, 0.0f));
	limb_trf.origin = limb_trf.origin;
	limb_trf.basis = Quaternion(Vector3(0.0f, 0.0f, 1.0f), normal);
	return limb_trf;
}

void HBLedgeTraversalController::calculate_pose(HBAgentParkourLedge *p_ledge, float p_offset, AgentProceduralAnimator::AgentProceduralPose &p_pose) const {
	Ref<Curve3D> curve = p_ledge->get_curve();
	for (int i = 0; i < AgentProceduralAnimator::LIMB_LEFT_FOOT; i++) {
		int offset_mul = 1 - i * 2;
		real_t offset = p_offset + offset_mul * radius;
		Transform3D hand_trf = calculate_limb_trf(p_ledge, radius, offset_mul * radius, offset, (AgentProceduralAnimator::AgentLimb)i);
		p_pose.ik_targets[i].origin = hand_trf.origin;
		p_pose.ik_targets[i].basis = hand_trf.basis;
	}

	for (int i = AgentProceduralAnimator::LIMB_LEFT_FOOT; i < AgentProceduralAnimator::LIMB_MAX; i++) {
		p_pose.ik_targets[i] = p_pose.ik_targets[i - 2];
		p_pose.ik_targets[i].origin -= Vector3(0.0f, height * 0.4f, 0.0f);
	}

	Transform3D trf = p_pose.ik_targets[AgentProceduralAnimator::LIMB_LEFT_HAND];
	trf = trf.interpolate_with(p_pose.ik_targets[AgentProceduralAnimator::LIMB_RIGHT_HAND], 0.5f);
	Vector3 normal = trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f));
	Vector3 new_pos = trf.origin;
	new_pos += normal * radius;
	trf.basis = Quaternion(Vector3(0.0, 0.0, 1.0f), trf.basis.get_rotation_quaternion().xform(Vector3(0.0f, 0.0f, -1.0f)));

	p_pose.skeleton_trf = trf;
}

Quaternion HBLedgeTraversalController::get_graphics_rotation() const {
	return out_rotation;
}

void HBLedgeTraversalController::move_to_ledge(HBAgentParkourLedge *p_ledge, const float p_offset) {
	curve_offset = p_offset;
	//curve_offset = p_ledge->get_curve()->get_closest_offset(p_ledge->to_local(p_current_agent_position));
	ledge = p_ledge;
	current_curve = ledge->get_curve();
	move_to_offset(curve_offset);
	_update_movement_dir_debug_graphic();
	_handle_limbs();
}

void HBLedgeTraversalController::move_to_offset(float p_offset) {
	if (!debug_geo) {
		debug_geo = memnew(HBDebugGeometry);
		add_child(debug_geo);
		debug_geo->set_as_top_level(true);
		debug_geo->set_global_position(Vector3());
	}

	const Transform3D trf = ledge->get_ledge_transform_at_offset(p_offset);

	// TODO: Pretty this one up
	Ref<CylinderShape3D> cyl;
	cyl.instantiate();
	cyl->set_height(height);
	cyl->set_radius(radius);
	PhysicsDirectSpaceState3D::ShapeParameters params;
	params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	params.shape_rid = cyl->get_rid();

	bool collision_check_hit = false;

	Transform3D collision_test_trf = trf;
	Vector3 collision_test_normal = collision_test_trf.basis.xform(Vector3(0.0f, 0.0f, -1.0f));
	Vector3 collision_test_pos = collision_test_trf.origin;
	collision_test_pos += collision_test_normal * radius * 2.0f;
	params.transform.origin = collision_test_pos;
	//collision_check_hit = dss->rest_info(params, &rest_info);
	debug_geo->debug_shape(cyl, params.transform);

	if (!collision_check_hit) {
		out_rotation = trf.basis.get_rotation_quaternion();
		curve_offset = p_offset;
		set_global_position(trf.origin);
		_update_movement_dir_debug_graphic();
		_handle_limbs();
	} else {
		movement_velocity = 0.0f;
		movement_velocity_target = 0.0f;
	}
}

float HBLedgeTraversalController::get_ledge_offset() const {
	return curve_offset;
}

Transform3D HBLedgeTraversalController::get_limb_transform(AgentProceduralAnimator::AgentLimb p_limb) const {
	return limb_transforms[p_limb];
}

Transform3D HBLedgeTraversalController::get_limb_transform_predicted(AgentProceduralAnimator::AgentLimb p_limb) const {
	return limb_transforms_predicted[p_limb];
}

bool HBLedgeTraversalController::get_limb_is_dangling(AgentProceduralAnimator::AgentLimb p_limb) const {
	return limb_dangle_status[p_limb];
}

float HBLedgeTraversalController::get_movement_velocity() const {
	return movement_velocity;
}

HBLedgeTraversalController::HBLedgeTraversalController() {
}
