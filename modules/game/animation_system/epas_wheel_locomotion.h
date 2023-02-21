#ifndef EPAS_WHEEL_LOCOMOTION_H
#define EPAS_WHEEL_LOCOMOTION_H

#include "epas_animation.h"
#include "epas_node.h"
#include "epas_pose.h"
#include "scene/3d/mesh_instance_3d.h"
#include "scene/resources/immediate_mesh.h"

class EPASWheelLocomotion : public EPASNode {
	GDCLASS(EPASWheelLocomotion, EPASNode);

private:
	// A wheel set contains the information of a different movement animation
	// for example, walk vs run
	struct LocomotionSet {
		float x_pos = 0.0f;
		// Step length = 1/4 the circumference of the surveyor wheel
		// Full cycle = step length * 2
		// Wheel radius = (4*step_length) / (2*PI)
		float step_length = 1.0f;
		Ref<EPASAnimation> animation;

		void interpolate(float p_amount, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_output) const {
			// p_amount can never *actually* be 1.0 since its fed by an fmodded value
			// Remove this if we ever change that...
			ERR_FAIL_COND(p_amount >= 1.0);
			if (animation.is_valid()) {
				animation->interpolate(p_amount, p_base_pose, p_output, EPASAnimation::InterpolationMethod::BICUBIC_SPLINE);
			}
		}
	};
	float current_step_length = 0.0;
	struct LocomotionSetComparator {
		_FORCE_INLINE_ bool operator()(const EPASWheelLocomotion::LocomotionSet *a, const EPASWheelLocomotion::LocomotionSet *b) const { return (a->x_pos < b->x_pos); }
	};
	float x_blend = 0.0f;
	float wheel_angle = 0.0f;
	Vector<LocomotionSet *> locomotion_sets;
	Vector3 linear_velocity;
	void add_set(float p_step_length);
	void _sort_sets();

protected:
	static void _bind_methods();

public:
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
	virtual int get_input_count() const override;
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;
	float get_wheel_angle() const;
	void set_linear_velocity(const Vector3 &p_linear_velocity);
	void set_x_blend(float p_x_blend);
	float get_x_blend() const;
	float get_current_step_length() const;

	void add_locomotion_set(float p_x);
	int get_locomotion_set_count() const;
	void set_locomotion_set_step_length(int p_idx, float p_step);
	void set_locomotion_set_animation(int p_idx, Ref<EPASAnimation> p_pose);

	EPASWheelLocomotion() :
			EPASNode(get_input_count()) {
	}
	virtual ~EPASWheelLocomotion();
};

class EPASWheelVisualizer : public Node3D {
	GDCLASS(EPASWheelVisualizer, Node3D);
	Ref<EPASWheelLocomotion> locomotion;
	MeshInstance3D *mi;

protected:
	void _notification(int p_what) {
		switch (p_what) {
			case NOTIFICATION_PROCESS: {
				if (locomotion.is_valid()) {
					Transform3D trf;
					trf.rotate_basis(Vector3(1.0, 0.0, 0.0), -locomotion->get_wheel_angle());
					float radius = (4.0 * locomotion->get_current_step_length() / (2.0 * Math_PI));
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