#ifndef SAFE_GODOT_OBJ_ID_REF_H
#define SAFE_GODOT_OBJ_ID_REF_H
#include "core/object/object.h"


template <typename T>
class SafeGodotObjIDRef {
    ObjectID obj_id;

public:
    bool is_valid() {
        return obj_id.is_valid();
    }

    _FORCE_INLINE_ T *get_object() const {
        DEV_ASSERT(obj_id.is_valid());
        Object* obj = ObjectDB::get_instance(obj_id);
        DEV_ASSERT(obj != nullptr);
        T *obj_c = Object::cast_to<T>(obj);
        DEV_ASSERT(obj_c != nullptr);
        return obj_c;
    }

    _FORCE_INLINE_ T *operator*() const {
		return get_object();
	}

	_FORCE_INLINE_ T *operator->() const {
		return get_object();
	}

    SafeGodotObjIDRef() {
        obj_id = ObjectID();
    }

    SafeGodotObjIDRef(ObjectID p_id) {
        obj_id = p_id;
    }

    SafeGodotObjIDRef(SafeGodotObjIDRef &p_from) {
        obj_id = p_from.obj_id;
    }

	_FORCE_INLINE_ bool operator==(const SafeGodotObjIDRef<T> &p_r) const {
		return get_object() == p_r.get_object();
	}

	_FORCE_INLINE_ bool operator!=(const Ref<T> &p_r) const {
        return get_object() == p_r.get_object();
	}

    static SafeGodotObjIDRef<T> from_object(Object* p_object) {
        return SafeGodotObjIDRef<T>(p_object->get_instance_id());
    }
};

#endif // SAFE_GODOT_OBJ_ID_REF_H
