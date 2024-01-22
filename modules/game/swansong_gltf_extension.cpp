#include "swansong_gltf_extension.h"
#include "core/error/error_list.h"
#include "core/error/error_macros.h"
#include "core/io/resource_loader.h"
#include "core/math/transform_3d.h"
#include "core/string/print_string.h"
#include "core/string/string_name.h"
#include "core/variant/array.h"
#include "core/variant/dictionary.h"
#include "core/variant/typed_array.h"
#include "core/variant/variant.h"
#include "scene/3d/node_3d.h"
#include "scene/resources/material.h"
#include "scene/resources/packed_scene.h"

Error SwansongGLTFExtension::import_post_material_parse(Ref<GLTFState> p_state) {
    TypedArray<Material> materials = p_state->get_materials(); 
    const Dictionary json = p_state->get_json();
    Array materials_json = json.get("materials", Array());

    for (int i = 0; i < materials.size(); i++) {
        print_line("TRY LOAD MATERIAL NUMBER ", i);
        Ref<Material> mat = materials[i];
        Dictionary material_json = materials_json.get(i);
        Dictionary extras = material_json.get("extras", Dictionary());
        if (!extras.has("godot_path")) {
            continue;
        }
        String godot_path = extras.get("godot_path", "");
        ERR_CONTINUE_MSG(godot_path.is_empty(), vformat("Failed loading material number %d in %s: empty godot_path?", i, p_state->get_filename()));

        Ref<Material> new_material = ResourceLoader::load(godot_path);

        ERR_CONTINUE_MSG(new_material.is_null(), vformat("Failed to load new material at %s, does it not exist?", godot_path));

        materials[i] = new_material;
        print_line("LOADED MATERIAL NUMBER ", i, new_material->get_path());
    }

    p_state->set_materials(materials);

    return OK;
}

Error SwansongGLTFExtension::import_node(Ref<GLTFState> p_state, Ref<GLTFNode> p_gltf_node, Dictionary &p_json, Node *p_node) {
    if (!p_json.has("extras")) {
        return OK;
    }
    Dictionary extras = p_json.get("extras", Dictionary());
    String godot_path = extras.get("godot_scene_path", "");
    String entity_name = extras.get("entity_name", "");
    if (godot_path.is_empty() && entity_name.is_empty()) {
        return OK;
    }


    Node3D *original_node = Object::cast_to<Node3D>(p_node);
    
    if (!original_node) {
        return OK;
    }


    const Transform3D node_trf = original_node->get_transform();
    const StringName node_name = original_node->get_name();

    Node3D *new_node;
    if (!entity_name.is_empty()) {
        new_node = create_entity(entity_name);
        ERR_FAIL_NULL_V(new_node, ERR_BUG);
    } else {
        new_node = memnew(Node3D);
        new_node->set_scene_file_path(godot_path);
    }

    new_node->set_transform(node_trf);
    new_node->set_name(node_name);

    original_node->replace_by(new_node);
    original_node->queue_free();

    SwansongEntity *ent = Object::cast_to<SwansongEntity>(new_node);
    if (ent) {
        ent->editor_build();
    }

    return OK;
}

void SwansongGLTFExtension::process_nodes(Node *p_node) {
    for (int i = 0; i < p_node->get_child_count(); i++) {
        Node *node = p_node->get_child(i);
        if (!node->get_scene_file_path().is_empty()) {
            for (int j = node->get_child_count()-1; j >= 0; j--) {
                Node *n = node->get_child(i);
                node->remove_child(n);
                n->queue_free();
            }
        }

        process_nodes(node);
    }
}


Error SwansongGLTFExtension::import_post(Ref<GLTFState> p_state, Node *p_root) {
    process_nodes(p_root);
    return OK;
}
