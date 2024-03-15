/**************************************************************************/
/*  epas_oneshot_animation_node.h                                         */
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

#ifndef EPAS_ONESHOT_ANIMATION_NODE_H
#define EPAS_ONESHOT_ANIMATION_NODE_H

#include "epas_animation.h"
#include "epas_node.h"
#include "scene/3d/multimesh_instance_3d.h"
#include "scene/resources/3d/primitive_meshes.h"

class EPASOneshotAnimationNodeDebug;
class AudioStreamPlayer3D;

class EPASOneshotAnimationNode : public EPASNode {
	GDCLASS(EPASOneshotAnimationNode, EPASNode);

	float time = 0.0f;
	bool playing = false;
	float speed_scale = 1.0f;
	float end_time = -1.0f;
	EPASAnimation::InterpolationMethod interpolation_method = EPASAnimation::InterpolationMethod::LINEAR;
	EPASAnimationPlaybackInfo playback_info;

	Ref<EPASAnimation> animation;
	void _on_animation_event_fired(const Ref<EPASAnimationEvent> &p_event);

protected:
	static void _bind_methods();
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
public:
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;

	void play();
	bool is_playing() const;
	float get_playback_position() const;

	Ref<EPASAnimation> get_animation() const;
	void set_animation(const Ref<EPASAnimation> &p_animation);

	bool get_use_root_motion() const;
	void set_use_root_motion(bool p_use_root_motion);

	StringName get_root_bone() const;
	void set_root_bone(const StringName &p_root_bone);

	void set_root_motion_starting_transform(const Transform3D &p_root_motion_starting_transform);
	Transform3D get_root_motion_starting_transform() const;

	void set_root_motion_forward(const Vector3 &p_root_motion_forward);
	Vector3 get_root_motion_forward() const;

	void set_warp_point_transform(const StringName &p_name, const Transform3D &p_transform);

	void play_with_warp_points(const Dictionary &p_warp_points);

	Transform3D get_root_motion_transform() const;
	float get_speed_scale() const { return speed_scale; }
	void set_speed_scale(float p_speed_scale) { speed_scale = p_speed_scale; }
	float get_end_time() const { return end_time; }
	void set_end_time(float p_end_time) { end_time = p_end_time; }
	void seek(float p_time);
	friend class EPASOneshotAnimationNodeDebug;
};

class EPASOneshotAnimationNodeDebug : public Node3D {
	GDCLASS(EPASOneshotAnimationNodeDebug, Node3D);
	MultiMeshInstance3D *mmi;

	Ref<EPASOneshotAnimationNode> node;

protected:
	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("set_animation_node", "animation_node"), &EPASOneshotAnimationNodeDebug::set_animation_node);
		ClassDB::bind_method(D_METHOD("update"), &EPASOneshotAnimationNodeDebug::update);
	}

public:
	void set_animation_node(Ref<EPASOneshotAnimationNode> p_animation_node) {
		node = p_animation_node;
	}
	Ref<EPASOneshotAnimationNode> get_animation_node() const {
		return node;
	}
	void update() {
		if (node.is_valid()) {
			Ref<MultiMesh> mm = mmi->get_multimesh();
			mm->set_instance_count(node->playback_info.warp_point_transforms.size());
			int i = 0;
			for (KeyValue<StringName, Transform3D> kv : node->playback_info.warp_point_transforms) {
				mm->set_instance_transform(i, kv.value);
				i++;
			}
		}
	}
	EPASOneshotAnimationNodeDebug() {
		mmi = memnew(MultiMeshInstance3D);
		Ref<MultiMesh> mm;
		mm.instantiate();
		mm->set_transform_format(MultiMesh::TRANSFORM_3D);
		Ref<SphereMesh> sm;
		sm.instantiate();
		sm->set_radius(0.05);
		sm->set_height(0.1);

		// Forward line
		Array arrays;
		arrays.resize(Mesh::ARRAY_MAX);
		PackedVector3Array vertices;
		vertices.push_back(Vector3());
		vertices.push_back(Vector3(0, 0, -1));
		arrays[Mesh::ARRAY_VERTEX] = vertices;

		Ref<ArrayMesh> am;
		am.instantiate();
		am->add_surface_from_arrays(sm->surface_get_primitive_type(0), sm->get_mesh_arrays());
		am->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, arrays);

		mm->set_mesh(am);
		mmi->set_multimesh(mm);
		add_child(mmi);
		set_as_top_level(true);

		Ref<StandardMaterial3D> mat;
		mat.instantiate();
		mat->set_albedo(Color(1.0f, 0.0f, 1.0f, 0.5f));
		mat->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
		mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
		sm->set_material(mat);
		am->surface_set_material(0, mat);
		am->surface_set_material(1, mat);
	};
};

#endif // EPAS_ONESHOT_ANIMATION_NODE_H
