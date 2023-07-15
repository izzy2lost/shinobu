#ifndef PH_SINGLETON_H
#define PH_SINGLETON_H

#include "modules/vorbis/audio_stream_ogg_vorbis.h"
#include "process/process.h"
#include "scene/main/node.h"

class PHNative : public Node {
	GDCLASS(PHNative, Node);

	static PHNative *singleton;
	enum {
		OGG_SYNC_BUFFER_SIZE = 8192
	};

protected:
	static void _bind_methods();

public:
	static PHNative *get_singleton() { return singleton; }
	Ref<Process> create_process(const String &p_path, const Vector<String> &p_arguments = Vector<String>(), const String &p_working_dir = "", bool p_open_stdin = false);
	void linalg_test();
	static Ref<AudioStreamOggVorbis> load_ogg_from_file(const String &p_path);
	static Ref<AudioStreamOggVorbis> load_ogg_from_buffer(const Vector<uint8_t> &p_buffer);
	PHNative();
};

#endif