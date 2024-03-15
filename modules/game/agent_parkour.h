/**************************************************************************/
/*  agent_parkour.h                                                       */
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

#ifndef AGENT_PARKOUR_H
#define AGENT_PARKOUR_H

#include "agent.h"
#include "debug_geometry.h"
#include "modules/tbloader/src/tb_loader_singleton.h"
#include "physics_layers.h"
#include "scene/3d/physics/area_3d.h"
#include "scene/3d/physics/collision_shape_3d.h"
#include "scene/3d/physics/static_body_3d.h"
#include "scene/resources/3d/sphere_shape_3d.h"

class HBAgentParkourLedge : public Area3D, public TBLoaderEntity {
	GDCLASS(HBAgentParkourLedge, Area3D);
	Ref<Curve3D> curve;
	// Curve the agent will follow
	Ref<Curve3D> agent_curve;

	bool closed = false;

	void _on_parkour_debug_changed();

protected:
	static void _bind_methods();

public:
	Ref<Curve3D> get_curve() const;
	void set_curve(const Ref<Curve3D> &p_curve);
	void generate_colliders();
	void generate_offset_path();
	void round_path();
	float get_closest_offset(const Vector3 &p_global_pos) const;
	Transform3D get_ledge_transform_at_offset(float p_offset) const;
	Transform3D get_agent_ledge_transform_at_offset(float p_offset) const;
	bool check_agent_fits(HBAgent *p_agent, float p_offset, HBDebugGeometry *p_debug_geo = nullptr) const;

	HBAgentParkourLedge();

	static StringName get_entity_name() {
		return "func_parkour_ledge";
	}

	bool get_closed() const { return closed; }
	void set_closed(bool p_closed) { closed = p_closed; }

	void set_agent_curve(const Ref<Curve3D> &agent_curve_) { agent_curve = agent_curve_; }
	virtual void _editor_build(const EntityCompileInfo &p_info, const HashMap<StringName, EntityCompileInfo> &p_entities) override;
};

class HBAgentParkourPoint : public StaticBody3D, public TBLoaderEntity {
	GDCLASS(HBAgentParkourPoint, StaticBody3D);
	bool collision_shape_dirty = true;
	Ref<SphereShape3D> shape;
	CollisionShape3D *collision_shape = nullptr;
	void _update_collision_shape();

protected:
	void _notification(int p_what);

public:
	HBAgentParkourPoint();
	static StringName get_entity_name() {
		return "func_parkour_point";
	}
};

class HBAgentParkourBeam : public Area3D, public TBLoaderEntity {
	GDCLASS(HBAgentParkourBeam, Area3D);
	Ref<Curve3D> curve_cache;
	bool is_curve_dirty = true;
	void _update_curve();

	MeshInstance3D *debug_preview = nullptr;
	bool line_redraw_queued = false;
	void _line_redraw();

	bool is_edge = false;

	struct BeamPointData {
		bool is_edge = false;
		Vector3 position;
	};

	Vector<BeamPointData> beam_points_data;

protected:
	static void _bind_methods();
	void _notification(int p_what);
	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;

public:
	virtual void _editor_build(const EntityCompileInfo &p_info, const HashMap<StringName, EntityCompileInfo> &p_entities) override;
	Ref<Curve3D> get_curve() const;
	void _queue_line_redraw();
	static StringName get_entity_name() {
		return "func_parkour_beam";
	}

	bool get_is_edge() const;
	void set_is_edge(bool p_is_edge);

	bool is_point_edge(int p_point_idx) const;

	HBAgentParkourBeam();
};

#endif // AGENT_PARKOUR_H
