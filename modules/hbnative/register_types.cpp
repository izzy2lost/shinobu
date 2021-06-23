/* register_types.cpp */

#include "register_types.h"

#include "core/class_db.h"
#include "ph_audio_stream_editor.h"
#include "ph_audio_stream_preview.h"

static PHAudioStreamPreviewGenerator *ph_ptr = NULL;

void register_hbnative_types() {
	ph_ptr = memnew(PHAudioStreamPreviewGenerator);
	ClassDB::register_class<PHAudioStreamPreviewGenerator>();
	ClassDB::register_class<PHAudioStreamEditor>();
	ClassDB::register_class<PHAudioStreamPreview>();
	Engine::get_singleton()->add_singleton(Engine::Singleton("PHAudioStreamPreviewGenerator", PHAudioStreamPreviewGenerator::get_singleton()));
}

void unregister_hbnative_types() {
	memdelete(ph_ptr);
}