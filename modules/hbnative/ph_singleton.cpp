#include "ph_singleton.h"

PHNative *PHNative::singleton = NULL;

Ref<Process> PHNative::create_process(const String &p_path, const Vector<String> &p_arguments, const String &p_working_dir, bool p_open_stdin) {
	return Process::create(p_path, p_arguments, p_working_dir, p_open_stdin);
}

void PHNative::_bind_methods() {
	ClassDB::bind_method(D_METHOD("create_process", "path", "arguments", "working_directory", "open_stdin"), &PHNative::create_process, DEFVAL(Vector<String>()), DEFVAL(""), DEFVAL(false));
}

PHNative::PHNative() {
	singleton = this;
}