/**************************************************************************/
/*  epas_wheel_locomotion.h                                               */
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

#ifndef EPAS_WHEEL_LOCOMOTION_H
#define EPAS_WHEEL_LOCOMOTION_H

#include "../debug_geometry.h"
#include "../fabrik/fabrik.h"
#include "../inertialization.h"
#include "epas_animation.h"
#include "epas_ik_node.h"
#include "epas_node.h"
#include "epas_pose.h"
#include "scene/3d/decal.h"
#include "scene/3d/mesh_instance_3d.h"
#include "scene/resources/image_texture.h"
#include "scene/resources/immediate_mesh.h"

// Data that is shared between different wheel locomotions to sync them when transitioning.
class WheelSharedInfo : public RefCounted {
public:
	float wheel_angle = 0.0f;
};

class EPASWheelLocomotion : public EPASNode {
	GDCLASS(EPASWheelLocomotion, EPASNode);

public:
	HBDebugGeometry *debug_geo = nullptr;
	enum LocomotionSetType {
		WHEEL,
		CONSTANT_VELOCITY // used for idle pose that has a constant velocity
	};

private:
	Ref<ImageTexture> debug_foot_texture;
	const int MAX_DEBUG_FOOT_MESHES = 10;
	int curr_debug_foot_mesh = 0;
	Vector<Decal *> debug_foot_meshes;
	Decal *debug_foot_meshes_predict[2];
	// A wheel set contains the information of a different movement animation
	// for example, walk vs run
	struct LocomotionSet {
		float x_pos = 0.0f;
		// Step length = 1/4 the circumference of the surveyor wheel
		// Full cycle = step length * 2
		// Wheel radius = (4*step_length) / (2*PI)
		Ref<EPASAnimation> animation;
		LocomotionSetType set_type = LocomotionSetType::WHEEL;
		float step_length = 1.0f;
		float bounce_height = 0.0f;
		Vector3 hip_offset;
		Vector3 hip_offset_spring_vel;

		void interpolate(float p_amount, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_output, float p_feet_grounded[2]) const {
			// p_amount can never *actually* be 1.0 since its fed by an fmodded value
			// Remove this if we ever change that...
			ERR_FAIL_COND(set_type == LocomotionSetType::WHEEL && p_amount >= 1.0);
			if (animation.is_valid()) {
				animation->interpolate(p_amount, p_base_pose, p_output, EPASAnimation::InterpolationMethod::BICUBIC_SPLINE);
			}
			p_feet_grounded[0] = 0.0f;
			p_feet_grounded[1] = 0.0f;

			if (!animation->has_animation_curve("left_ik_grounded") || !animation->has_animation_curve("right_ik_grounded")) {
				return;
			}

			Ref<Curve> curve_left = animation->get_animation_curve("left_ik_grounded");
			Ref<Curve> curve_right = animation->get_animation_curve("right_ik_grounded");

			bool has_ik_curves = curve_left.is_valid() && curve_right.is_valid();

			if (!has_ik_curves) {
				return;
			}

			p_feet_grounded[0] = curve_left->sample(MIN(p_amount, 1.0f));
			p_feet_grounded[1] = curve_right->sample(MIN(p_amount, 1.0f));
		}
	};

	Ref<WheelSharedInfo> shared_info;
	float current_step_length = 0.0;
	struct LocomotionSetComparator {
		_FORCE_INLINE_ bool operator()(const EPASWheelLocomotion::LocomotionSet *a, const EPASWheelLocomotion::LocomotionSet *b) const { return (a->x_pos < b->x_pos); }
	};

	float x_blend = 0.0f;
	float wheel_angle = 0.0f;
	float time = 0.0f;
	// minimum distance for foot ik sliding to happen
	float foot_ik_slide_max = 0.05f;
	StringName root_bone_name;
	Vector<LocomotionSet *> locomotion_sets;
	// We need to keep sorted locomotion sets in a separate vector
	// the reason is so that the sets can be accessed by their indexed order
	// perhaps there's a better way to do this...
	Vector<LocomotionSet *> sorted_locomotion_sets;
	Vector3 linear_velocity;
	void add_set(float p_step_length);
	void _sort_sets();

	StringName hip_bone_name;
	struct FootIK {
		StringName bone_name;
		Transform3D out_ik_transform;
		Vector3 out_ik_magnet_position;
		EPASIKNode *ik_node = nullptr;
		bool pinned = false;
		Vector3 pinned_position;
		float pin_recovery_t = 0.0f;
		Vector3 prev_positions[2];
		Ref<PositionInertializer> position_inertializer;
	} foot_ik[2];
	Ref<EPASIKNode> _get_foot_ik_node(uint32_t p_idx) const;

	float foot_ik_pin_recovery_time = 0.15f;
	bool use_foot_ik = false;
	bool foot_ik_init = false;
	float max_velocity = 2.8f;

	void _ik_process_foot(LocomotionSet *p_loc_set, float p_foot_ik_grounded[2], Transform3D p_ankle_ik_targets[2], Transform3D p_ankle_pinned_ik_targets[2], const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose);
	void _ik_process(LocomotionSet *p_loc_sets[2], float p_foot_ik_grounded[2][2], Ref<EPASPose> p_target_pose, Ref<EPASPose> p_second_pose, Ref<EPASPose> p_base_pose, float p_x, float p_delta);
	float find_next_feet_ground_time(Ref<EPASAnimation> p_anim, float p_times[2]) const;
	bool process_events(LocomotionSet *p_sets[2], float p_time, float p_blend, float p_previous_time, bool p_out_prev_lock_state[2], bool p_out_lock_state[2], float p_out_prev_lock_amount[2], float p_out_lock_amount[2]);
	bool foot_predict(LocomotionSet *p_sets[2], float p_time, float p_blend);

protected:
	static void _bind_methods();

public:
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;
	float get_wheel_angle() const;
	void set_max_velocity(float p_max_velocity);
	void set_linear_velocity(const Vector3 &p_linear_velocity);
	void set_x_blend(float p_x_blend);
	float get_x_blend() const;
	float get_current_step_length() const;

	void add_locomotion_set(float p_x);
	int get_locomotion_set_count() const;
	void set_locomotion_set_step_length(int p_idx, float p_step);
	void set_locomotion_set_animation(int p_idx, Ref<EPASAnimation> p_pose);
	void set_locomotion_set_type(int p_idx, LocomotionSetType p_type);
	void set_locomotion_set_bounce_height(int p_idx, float p_bounce_height);

	StringName get_root_bone_name() const;
	void set_root_bone_name(const StringName &p_root_bone_name);

	StringName get_left_foot_bone_name() const;
	void set_left_foot_bone_name(const StringName &p_left_foot_bone_name);
	StringName get_right_foot_bone_name() const;
	void set_right_foot_bone_name(const StringName &p_right_foot_bone_name);
	bool get_use_foot_ik() const;
	void set_use_foot_ik(bool p_use_foot_ik);

	void set_left_foot_ik_node(Ref<EPASIKNode> p_ik_left_foot_ik_node);

	void set_right_foot_ik_node(Ref<EPASIKNode> p_ik_right_foot_ik_node);

	virtual ~EPASWheelLocomotion();

	StringName get_hip_bone_name() const;
	void set_hip_bone_name(const StringName &p_hip_bone_name);

	void sync_with(TypedArray<EPASWheelLocomotion> p_locomotion);
	void reset_foot_ik();

	EPASWheelLocomotion();

	friend class EPASWheelVisualizer;
};

VARIANT_ENUM_CAST(EPASWheelLocomotion::LocomotionSetType);

class EPASWheelVisualizer : public Node3D {
	GDCLASS(EPASWheelVisualizer, Node3D);
	Ref<EPASWheelLocomotion> locomotion;
	MeshInstance3D *mi;

protected:
	void _notification(int p_what) {
		switch (p_what) {
			case NOTIFICATION_PROCESS: {
				if (locomotion.is_valid()) {
					float radius = (4.0 * locomotion->get_current_step_length() / (2.0 * Math_PI));
					Transform3D trf;
					trf.rotate_basis(Vector3(1.0, 0.0, 0.0), -locomotion->get_wheel_angle());
					trf.scale(Vector3(radius, radius, radius));
					trf.origin.y = radius;
					mi->set_transform(trf);
				}
			} break;
		}
	}

	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("set_locomotion", "locomotion"), &EPASWheelVisualizer::set_locomotion);
	}

public:
	void set_locomotion(Ref<EPASWheelLocomotion> p_locomotion) {
		locomotion = p_locomotion;
	}
	Ref<EPASWheelLocomotion> get_locomotion() const {
		return locomotion;
	}

	EPASWheelVisualizer() {
		if (Engine::get_singleton()->is_editor_hint()) {
			return;
		}
		mi = memnew(MeshInstance3D);
		Ref<ImmediateMesh> mesh = memnew(ImmediateMesh);
		// Generate a wheel graphic with radius 1
		const float DEBUG_WHEEL_RADIUS = 1.0;
		mesh->surface_begin(Mesh::PrimitiveType::PRIMITIVE_LINES);
		for (int i = 0; i < 8; i++) {
			Vector3 v1 = Vector3(0.0, 0.0, 1.0);
			v1.rotate(Vector3(1.0, 0.0, 0.0), (i / 8.0) * Math_TAU);
			if (i % 2 == 0) {
				mesh->surface_add_vertex(v1 * DEBUG_WHEEL_RADIUS * 0.25);
			} else {
				mesh->surface_add_vertex(v1 * DEBUG_WHEEL_RADIUS * 0.75);
			}
			mesh->surface_add_vertex(v1 * DEBUG_WHEEL_RADIUS);
		}
		mesh->surface_add_vertex(Vector3(-DEBUG_WHEEL_RADIUS, 0.0, 0.0));
		mesh->surface_add_vertex(Vector3(DEBUG_WHEEL_RADIUS, 0.0, 0.0));
		mesh->surface_end();
		mi->set_mesh(mesh);
		Ref<StandardMaterial3D> mat = memnew(StandardMaterial3D);
		mat = Ref<StandardMaterial3D>(memnew(StandardMaterial3D));
		mat->set_shading_mode(BaseMaterial3D::ShadingMode::SHADING_MODE_UNSHADED);
		mat->set_albedo(Color(1.0, 1.0, 1.0, 1.0));
		mesh->surface_set_material(0, mat);
		add_child(mi);
		set_process(true);
	}
};
#endif // EPAS_WHEEL_LOCOMOTION_H