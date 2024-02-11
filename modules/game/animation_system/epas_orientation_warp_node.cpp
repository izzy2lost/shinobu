#include "epas_orientation_warp_node.h"
#include "modules/game/animation_system/epas_controller.h"
#include "modules/game/animation_system/epas_ik_node.h"
#include "modules/game/debug_geometry.h"
#include "modules/game/springs.h"
void EPASOrientationWarpNode::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_spine_bones"), &EPASOrientationWarpNode::get_spine_bones);
    ClassDB::bind_method(D_METHOD("set_spine_bones", "spine_bones"), &EPASOrientationWarpNode::set_spine_bones);
    ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "spine_bones"), "set_spine_bones", "get_spine_bones");
    
    ClassDB::bind_method(D_METHOD("get_pelvis_bone_name"), &EPASOrientationWarpNode::get_pelvis_bone_name);
    ClassDB::bind_method(D_METHOD("set_pelvis_bone_name", "bone_name"), &EPASOrientationWarpNode::set_pelvis_bone_name);
    ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "pelvis_bone_name"), "set_pelvis_bone_name", "get_pelvis_bone_name");

    ClassDB::bind_method(D_METHOD("get_foot_ik_node_left"), &EPASOrientationWarpNode::get_foot_ik_node_left);
    ClassDB::bind_method(D_METHOD("set_foot_ik_node_left", "foot_ik_node"), &EPASOrientationWarpNode::set_foot_ik_node_left);
    ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "foot_ik_node_left"), "set_foot_ik_node_left", "get_foot_ik_node_left");

    ClassDB::bind_method(D_METHOD("get_foot_ik_node_right"), &EPASOrientationWarpNode::get_foot_ik_node_right);
    ClassDB::bind_method(D_METHOD("set_foot_ik_node_right", "foot_ik_node"), &EPASOrientationWarpNode::set_foot_ik_node_right);
    ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "foot_ik_node_right"), "set_foot_ik_node_right", "get_foot_ik_node_right");
}

bool EPASOrientationWarpNode::_set(const StringName &p_name, const Variant &p_value) {
	Vector<String> components = String(p_name).split("/", true, 2);
    if (components.size() == 2 && components[0].begins_with("spine_bone_") && components[0].trim_prefix("spine_bone_").is_valid_int()) {
		int index = components[0].trim_prefix("spine_bone_").to_int();
        ERR_FAIL_INDEX_V(index, spine_bones.size(), false);

        spine_bones.write[index] = p_value;

        return true;
    }
    return false;
}

bool EPASOrientationWarpNode::_get(const StringName &p_name, Variant &r_ret) const {
    Vector<String> components = String(p_name).split("/", true, 2);
	if (components.size() >= 2 && components[0].begins_with("spine_bone_") && components[0].trim_prefix("spine_bone_").is_valid_int()) {
		int index = components[0].trim_prefix("spine_bone_").to_int();
        ERR_FAIL_INDEX_V(index, spine_bones.size(), false);

        r_ret = spine_bones[index];

        return true;
    }

    return false;
}


#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
void EPASOrientationWarpNode::_debug_node_draw() const {
    ImGui::SetNextItemWidth(100);
    ImGui::SliderFloat("Orientation angle", &const_cast<EPASOrientationWarpNode*>(this)->orientation_angle, -90.0f, 90.0f);
}

#endif

static HBDebugGeometry *debug_geo = nullptr;

void EPASOrientationWarpNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
    process_input_pose(0, p_base_pose, p_target_pose, p_delta);
    HBSprings::critical_spring_damper_exact(&_orientation_angle, &orientation_spring_velocity, orientation_angle, 0.1f, p_delta);

    if (!debug_geo) {
        debug_geo = memnew(HBDebugGeometry);
        get_skeleton()->add_child(debug_geo);
        debug_geo->set_as_top_level(true);
        debug_geo->set_global_transform(Transform3D());
    }


    if (!p_target_pose->has_bone(pelvis_bone_name)) {
        p_target_pose->create_bone(pelvis_bone_name);
    }

    float offset = Math::deg_to_rad(_orientation_angle);
    const Quaternion pelvis_rot = Basis(p_target_pose->get_bone_rotation(pelvis_bone_name, p_base_pose)).rotated(Vector3(0.0f, 1.0f, 0.0f), offset).get_rotation_quaternion();
    const Vector3 pelvis_pos = p_target_pose->get_bone_position(pelvis_bone_name, p_base_pose);
    p_target_pose->set_bone_rotation(pelvis_bone_name, pelvis_rot);

    StringName prev_bone = pelvis_bone_name;
    Vector3 prev_up = Vector3(0.0f, 1.0f, 0.0f);
    for (int i = 0; i < spine_bones.size(); i++) {
        Quaternion prev_rotation = p_target_pose->get_bone_rotation(prev_bone, p_base_pose);
        prev_up = prev_rotation.xform_inv(prev_up);
        if (!p_target_pose->has_bone(spine_bones[i])) {
            p_target_pose->create_bone(spine_bones[i]);
        }

        Quaternion bone_rot = p_target_pose->get_bone_rotation(spine_bones[i], p_base_pose);
        float rot = (-offset) * (i / ((float)spine_bones.size()-1.0f));
        p_target_pose->set_bone_rotation(spine_bones[i], Basis(bone_rot).rotated(prev_up, rot).get_rotation_quaternion());
        prev_bone = spine_bones[i];
    }

    StringName ik_nodes[2] = {
        foot_ik_node_left,
        foot_ik_node_right,
    };

    debug_geo->clear();

    for(int i = 0; i < (int)std::size(ik_nodes); i++) {
        Ref<EPASIKNode> ik_node = get_epas_controller()->get_epas_node(ik_nodes[i]);
        DEV_ASSERT(ik_node.is_valid());
        Transform3D old_pelvis_trf(pelvis_rot, pelvis_pos);

        const Quaternion pelvis_rot_new = Basis(p_target_pose->get_bone_rotation(pelvis_bone_name)).rotated(Vector3(0.0f, 1.0f, 0.0f), offset).get_rotation_quaternion();
        Transform3D pelvis_trf(pelvis_rot_new, pelvis_pos);
        debug_geo->debug_sphere(ik_node->get_target_transform().origin, 0.05f, Color("RED"));
        Transform3D target_trf = (get_skeleton()->get_global_transform() * old_pelvis_trf).affine_inverse() * ik_node->get_target_transform();
        target_trf = (get_skeleton()->get_global_transform() * pelvis_trf) * target_trf;

        ik_node->set_target_transform(target_trf);

        debug_geo->debug_sphere(target_trf.origin);
    }
}

EPASOrientationWarpNode::EPASOrientationWarpNode() {
	_set_input_count(1);
}


StringName EPASOrientationWarpNode::get_foot_ik_node_left() const { return foot_ik_node_left; }
void EPASOrientationWarpNode::set_foot_ik_node_left(const StringName &p_foot_ik_node_left) { foot_ik_node_left = p_foot_ik_node_left; }

StringName EPASOrientationWarpNode::get_foot_ik_node_right() const { return foot_ik_node_right; }

void EPASOrientationWarpNode::set_foot_ik_node_right(const StringName &p_foot_ik_node_right) { foot_ik_node_right = p_foot_ik_node_right; }

float EPASOrientationWarpNode::get_orientation_angle() const { return orientation_angle; }

void EPASOrientationWarpNode::set_orientation_angle(float p_orientation_angle) { orientation_angle = p_orientation_angle; }