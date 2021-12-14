#ifndef HBNATIVE_SINGLETON_H
#define HBNATIVE_SINGLETON_H

#include "core/object.h"

class HBNativeSingleton : public Object {
	GDCLASS(HBNativeSingleton, Object)
private:
	static HBNativeSingleton *singleton;

protected:
	static void _bind_methods();

public:
	_FORCE_INLINE_ static HBNativeSingleton *get_singleton() { return singleton; }
	String romanize(String source);
	HBNativeSingleton();
};

#endif
