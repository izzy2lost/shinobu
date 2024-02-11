#ifndef EPAS_ORIENTATION_WARP_NODE_H
#define EPAS_ORIENTATION_WARP_NODE_H

#include "modules/game/animation_system/epas_node.h"
class EPASOrientationWarpNode : public EPASNode {
    GDCLASS(EPASOrientationWarpNode, EPASNode);
    StringName foot_ik_node_left;
    StringName foot_ik_node_right;
    StringName pelvis_bone_name;

    Vector<StringName> spine_bones;

    float orientation_spring_velocity = 0.0f;
    float _orientation_angle = 0.0f;
    float orientation_angle = 0.0f;
protected:
    static void _bind_methods();
	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
public:
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
    StringName get_foot_ik_node_left() const;
    void set_foot_ik_node_left(const StringName &p_foot_ik_node_left);

    void set_spine_bones(TypedArray<StringName> p_spine_bones) {
        spine_bones.clear();

        for(int i = 0; i < p_spine_bones.size(); i++) {
            spine_bones.push_back(p_spine_bones[i]);
        }
    };
    TypedArray<StringName> get_spine_bones() const {
        TypedArray<StringName> out;
        for(int i = 0; i < spine_bones.size(); i++) {
            out.push_back(spine_bones[i]);
        }

        return out;
    };

    StringName get_foot_ik_node_right() const;
    void set_foot_ik_node_right(const StringName &p_foot_ik_node_right);

    float get_orientation_angle() const;
    void set_orientation_angle(float p_orientation_angle);

	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;

	StringName get_pelvis_bone_name() const { return pelvis_bone_name; }
	void set_pelvis_bone_name(const StringName &pelvis_bone_name_) { pelvis_bone_name = pelvis_bone_name_; }

    EPASOrientationWarpNode();
};

#endif // EPAS_ORIENTATION_WARP_NODE_H
