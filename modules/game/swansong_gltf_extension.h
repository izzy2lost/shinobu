#ifndef SWANSONG_GLTF_EXTENSION_H
#define SWANSONG_GLTF_EXTENSION_H

#include "modules/gltf/editor/editor_scene_importer_blend.h"
#include "modules/gltf/extensions/gltf_document_extension.h"
#include "modules/gltf/structures/gltf_node.h"
#include "scene/3d/node_3d.h"

class SwansongEntity {
public:
    virtual void editor_build() {};
};

class SwansongGLTFExtension : public GLTFDocumentExtension {
    GDCLASS(SwansongGLTFExtension, GLTFDocumentExtension);
    void process_nodes(Node *p_node);
protected:
    virtual Error import_post_material_parse(Ref<GLTFState> p_state) override;
    virtual Error import_node(Ref<GLTFState> p_state, Ref<GLTFNode> p_gltf_node, Dictionary &p_json, Node *p_node) override;
    virtual Error import_post(Ref<GLTFState> p_state, Node *p_root) override;
private:
    struct EntityType {
        StringName entity_name;
        StringName class_name;
    };
    HashMap<StringName, EntityType> entity_types;
public:
    template <class T>
    void register_entity_type() {
        EntityType type;
        type.entity_name = T::get_entity_name();
        type.class_name = T::get_class_static();
        ERR_FAIL_COND_MSG(!ClassDB::is_parent_class(type.class_name, "Node3D"), "Entity classes must inherit Node3D");
        entity_types.insert(type.entity_name, type);
    }
    Node3D *create_entity(StringName p_entity_name) {
        EntityType *type_info = entity_types.getptr(p_entity_name);
        ERR_FAIL_COND_V_MSG(type_info == nullptr, nullptr, vformat("Entity not found: %s", p_entity_name));
        return Object::cast_to<Node3D>(ClassDB::instantiate(type_info->class_name));
    }
};

#endif // SWANSONG_GLTF_EXTENSION_H