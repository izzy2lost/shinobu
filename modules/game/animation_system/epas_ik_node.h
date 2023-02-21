#ifndef EPAS_IK_NODE_H
#define EPAS_IK_NODE_H

#include "core/object/object.h"
#include "modules/game/animation_system/epas_node.h"
#include "scene/3d/node_3d.h"

class EPASIKNode : public EPASNode {
	GDCLASS(EPASIKNode, EPASNode);

private:
	float ik_influence = 1.0f;
	Vector3 target_position;
	bool use_magnet = false;
	Vector3 magnet_position;
	String ik_end;

protected:
	static void _bind_methods();

public:
	virtual int get_input_count() const override;
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;

	float get_ik_influence() const;
	void set_ik_influence(float p_ik_influence);
	Vector3 get_target_position() const;
	void set_target_position(const Vector3 &p_target_position);
	bool get_use_magnet() const;
	void set_use_magnet(bool p_use_magnet);
	Vector3 get_magnet_position() const;
	void set_magnet_position(const Vector3 &p_magnet_position);
	String get_ik_end() const;
	void set_ik_end(const String &p_ik_end);

	EPASIKNode();
};

#endif // EPAS_IK_NODE_H
