/* register_types.cpp */

#include "register_types.h"

#include "core/config/engine.h"
#include "multi_spin_box.h"
#include "ph_audio_stream_editor.h"
#include "ph_audio_stream_preview.h"
#include "ph_singleton.h"

#include "process/process.h"
#include "process/process_tiny_process_lib.h"
#ifdef UNIX_ENABLED
#include "process/process_unix.h"
#endif
#ifdef WINDOWS_ENABLED
#include "process/process_windows.h"
#endif

#include "threen.h"

static PHAudioStreamPreviewGenerator *preview_generator_ptr = NULL;
static PHNative *ph_ptr = NULL;

void initialize_hbnative_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	preview_generator_ptr = memnew(PHAudioStreamPreviewGenerator);
	ph_ptr = memnew(PHNative);

#ifdef UNIX_ENABLED
	ProcessUnix::make_default();
#endif

#ifdef WINDOWS_ENABLED
	ProcessWindows::make_default();
#endif

	GDREGISTER_CLASS(PHAudioStreamPreviewGenerator);
	GDREGISTER_CLASS(PHAudioStreamEditor);
	GDREGISTER_CLASS(PHAudioStreamPreview);
	GDREGISTER_CLASS(Threen);
	GDREGISTER_ABSTRACT_CLASS(Process);
	GDREGISTER_ABSTRACT_CLASS(PHNative);
	GDREGISTER_CLASS(MultiSpinBox);
	Engine::get_singleton()->add_singleton(Engine::Singleton("PHAudioStreamPreviewGenerator", PHAudioStreamPreviewGenerator::get_singleton()));
	Engine::get_singleton()->add_singleton(Engine::Singleton("PHNative", PHNative::get_singleton()));
}

void uninitialize_hbnative_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	memdelete(preview_generator_ptr);
	memdelete(ph_ptr);
}
