/**************************************************************************/
/*  epas_ik_node.h                                                        */
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

#ifndef EPAS_IK_NODE_H
#define EPAS_IK_NODE_H

#include "../fabrik/fabrik.h"
#include "core/object/object.h"
#include "modules/game/animation_system/epas_node.h"
#include "scene/3d/node_3d.h"

class EPASIKNode : public EPASNode {
	GDCLASS(EPASIKNode, EPASNode);

private:
	Ref<FABRIKLimbSolver> fabrik_solver;

	float ik_influence = 1.0f;
	Transform3D target_transform;
	bool use_target_basis = true;
	bool use_magnet = false;
	Vector3 magnet_position;
	StringName ik_end;
	bool length_dirty = false;

protected:
	static void _bind_methods();
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
public:
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;

	float get_ik_influence() const;
	void set_ik_influence(float p_ik_influence);
	Transform3D get_target_transform() const;
	void set_target_transform(const Transform3D &p_target_transform);
	bool get_use_magnet() const;
	void set_use_magnet(bool p_use_magnet);
	Vector3 get_magnet_position() const;
	void set_magnet_position(const Vector3 &p_magnet_position);
	void set_use_hinge(bool p_use_hinge);
	bool get_use_hinge() const;
	StringName get_ik_end() const;
	void set_ik_end(const StringName &p_ik_end);

	EPASIKNode();
};

#endif // EPAS_IK_NODE_H
