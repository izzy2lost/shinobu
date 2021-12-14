#include "hbnative_singleton.h"

HBNativeSingleton *HBNativeSingleton::singleton = nullptr;

HBNativeSingleton::HBNativeSingleton() {
	singleton = this;
}

void HBNativeSingleton::_bind_methods() {
	ClassDB::bind_method(D_METHOD("romanize", "string"), &HBNativeSingleton::romanize);
}

String HBNativeSingleton::romanize(String source) {
	return source;
}
