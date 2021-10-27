#include "hbnative_singleton.h"
#include "thirdparty/romaji-cpp/romaji.h"

HBNativeSingleton *HBNativeSingleton::singleton = nullptr;

HBNativeSingleton::HBNativeSingleton() {
	singleton = this;
}

void HBNativeSingleton::_bind_methods() {
	ClassDB::bind_method(D_METHOD("romanize", "string"), &HBNativeSingleton::romanize);
}

String HBNativeSingleton::romanize(String source) {
	std::wstring wstr = source.c_str();
	std::string out_str;
	japanese::utf8_kana_to_romaji(source.utf8().get_data(), out_str);
	String str_o;
	str_o.parse_utf8(out_str.c_str(), out_str.length());
	return str_o;
}