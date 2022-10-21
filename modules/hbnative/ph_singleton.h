#ifndef PH_SINGLETON_H
#define PH_SINGLETON_H

#include "process/process.h"
#include "scene/main/node.h"

class PHNative : public Node {
	GDCLASS(PHNative, Node);

	static PHNative *singleton;

protected:
	static void _bind_methods();

public:
	static PHNative *get_singleton() { return singleton; }
	Ref<Process> create_process(const String &p_path, const Vector<String> &p_arguments = Vector<String>(), const String &p_working_dir = "", bool p_open_stdin = false);

	PHNative();
};

#endif