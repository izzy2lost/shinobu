/**************************************************************************/
/*  safe_godot_obj_id_ref.h                                               */
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
		Object *obj = ObjectDB::get_instance(obj_id);
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

	static SafeGodotObjIDRef<T> from_object(Object *p_object) {
		return SafeGodotObjIDRef<T>(p_object->get_instance_id());
	}
};

#endif // SAFE_GODOT_OBJ_ID_REF_H
