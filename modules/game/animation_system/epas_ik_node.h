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
	bool use_magnet = false;
	Vector3 magnet_position;
	StringName ik_end;
	bool length_dirty = false;

protected:
	static void _bind_methods();
	virtual void _debug_node_draw() const override;

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
