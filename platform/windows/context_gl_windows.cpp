/**************************************************************************/
/*  context_gl_windows.cpp                                                */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
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

#if defined(OPENGL_ENABLED) || defined(GLES_ENABLED)

// Author: Juan Linietsky <reduzio@gmail.com>, (C) 2008

#include "context_gl_windows.h"
#include "core/project_settings.h"
#include "core/version.h"
#include "thirdparty/nvapi/nvapi.h"

#include <dwmapi.h>

#define WGL_CONTEXT_MAJOR_VERSION_ARB 0x2091
#define WGL_CONTEXT_MINOR_VERSION_ARB 0x2092
#define WGL_CONTEXT_FLAGS_ARB 0x2094
#define WGL_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB 0x00000002
#define WGL_CONTEXT_PROFILE_MASK_ARB 0x9126
#define WGL_CONTEXT_CORE_PROFILE_BIT_ARB 0x00000001

#if defined(__GNUC__)
// Workaround GCC warning from -Wcast-function-type.
#define wglGetProcAddress (void *)wglGetProcAddress
#endif

typedef HGLRC(APIENTRY *PFNWGLCREATECONTEXTATTRIBSARBPROC)(HDC, HGLRC, const int *);

void ContextGL_Windows::release_current() {
	wglMakeCurrent(hDC, NULL);
}

void ContextGL_Windows::make_current() {
	wglMakeCurrent(hDC, hRC);
}

bool ContextGL_Windows::is_offscreen_available() const {
	return hRC_offscreen != NULL;
}

void ContextGL_Windows::make_offscreen_current() {
	ERR_FAIL_COND(!wglMakeCurrent(hDC, hRC_offscreen));
}

void ContextGL_Windows::release_offscreen_current() {
	ERR_FAIL_COND(!wglMakeCurrent(hDC, NULL));
}

HDC ContextGL_Windows::get_hdc() {
	return hDC;
}

HGLRC ContextGL_Windows::get_hglrc() {
	return hRC;
}

int ContextGL_Windows::get_window_width() {
	return OS::get_singleton()->get_video_mode().width;
}

int ContextGL_Windows::get_window_height() {
	return OS::get_singleton()->get_video_mode().height;
}

bool ContextGL_Windows::should_vsync_via_compositor() {
	if (OS::get_singleton()->is_window_fullscreen() || !OS::get_singleton()->is_vsync_via_compositor_enabled()) {
		return false;
	}

	// Note: All Windows versions supported by Godot have a compositor.
	// It can be disabled on earlier Windows versions.
	BOOL dwm_enabled;

	if (SUCCEEDED(DwmIsCompositionEnabled(&dwm_enabled))) {
		return dwm_enabled;
	}

	return false;
}

void ContextGL_Windows::swap_buffers() {
	SwapBuffers(hDC);

	if (use_vsync) {
		bool vsync_via_compositor_now = should_vsync_via_compositor();

		if (vsync_via_compositor_now && wglGetSwapIntervalEXT() == 0) {
			DwmFlush();
		}

		if (vsync_via_compositor_now != vsync_via_compositor) {
			// The previous frame had a different operating mode than this
			// frame. Set the 'vsync_via_compositor' member variable and the
			// OpenGL swap interval to their proper values.
			set_use_vsync(true);
		}
	}
}

void ContextGL_Windows::set_use_vsync(bool p_use) {
	vsync_via_compositor = p_use && should_vsync_via_compositor();

	if (wglSwapIntervalEXT) {
		int swap_interval = (p_use && !vsync_via_compositor) ? 1 : 0;
		wglSwapIntervalEXT(swap_interval);
	}

	use_vsync = p_use;
}

bool ContextGL_Windows::is_using_vsync() const {
	return use_vsync;
}

#define _WGL_CONTEXT_DEBUG_BIT_ARB 0x0001

Error ContextGL_Windows::initialize() {
	_nvapi_disable_threaded_optimization();

	static PIXELFORMATDESCRIPTOR pfd = {
		sizeof(PIXELFORMATDESCRIPTOR), // Size Of This Pixel Format Descriptor
		1,
		PFD_DRAW_TO_WINDOW | // Format Must Support Window
				PFD_SUPPORT_OPENGL | // Format Must Support OpenGL
				PFD_DOUBLEBUFFER,
		(BYTE)PFD_TYPE_RGBA,
		(BYTE)(OS::get_singleton()->is_layered_allowed() ? 32 : 24),
		(BYTE)0, (BYTE)0, (BYTE)0, (BYTE)0, (BYTE)0, (BYTE)0, // Color Bits Ignored
		(BYTE)(OS::get_singleton()->is_layered_allowed() ? 8 : 0), // Alpha Buffer
		(BYTE)0, // Shift Bit Ignored
		(BYTE)0, // No Accumulation Buffer
		(BYTE)0, (BYTE)0, (BYTE)0, (BYTE)0, // Accumulation Bits Ignored
		(BYTE)24, // 24Bit Z-Buffer (Depth Buffer)
		(BYTE)0, // No Stencil Buffer
		(BYTE)0, // No Auxiliary Buffer
		(BYTE)PFD_MAIN_PLANE, // Main Drawing Layer
		(BYTE)0, // Reserved
		0, 0, 0 // Layer Masks Ignored
	};

	hDC = GetDC(hWnd);
	if (!hDC) {
		return ERR_CANT_CREATE; // Return FALSE
	}

	pixel_format = ChoosePixelFormat(hDC, &pfd);
	if (!pixel_format) // Did Windows Find A Matching Pixel Format?
	{
		return ERR_CANT_CREATE; // Return FALSE
	}

	BOOL ret = SetPixelFormat(hDC, pixel_format, &pfd);
	if (!ret) // Are We Able To Set The Pixel Format?
	{
		return ERR_CANT_CREATE; // Return FALSE
	}

	hRC = wglCreateContext(hDC);
	if (!hRC) // Are We Able To Get A Rendering Context?
	{
		return ERR_CANT_CREATE; // Return FALSE
	}

	wglMakeCurrent(hDC, hRC);

	if (opengl_3_context) {
		int attribs[] = {
			WGL_CONTEXT_MAJOR_VERSION_ARB, 3, //we want a 3.3 context
			WGL_CONTEXT_MINOR_VERSION_ARB, 3,
			//and it shall be forward compatible so that we can only use up to date functionality
			WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
			WGL_CONTEXT_FLAGS_ARB, WGL_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB /*| _WGL_CONTEXT_DEBUG_BIT_ARB*/,
			0
		}; //zero indicates the end of the array

		PFNWGLCREATECONTEXTATTRIBSARBPROC wglCreateContextAttribsARB = NULL; //pointer to the method
		wglCreateContextAttribsARB = (PFNWGLCREATECONTEXTATTRIBSARBPROC)wglGetProcAddress("wglCreateContextAttribsARB");

		if (wglCreateContextAttribsARB == NULL) //OpenGL 3.0 is not supported
		{
			wglDeleteContext(hRC);
			return ERR_CANT_CREATE;
		}

		HGLRC new_hRC = wglCreateContextAttribsARB(hDC, 0, attribs);
		if (!new_hRC) {
			wglDeleteContext(hRC);
			return ERR_CANT_CREATE; // Return false
		}
		wglMakeCurrent(hDC, NULL);
		wglDeleteContext(hRC);
		hRC = new_hRC;

		if (!wglMakeCurrent(hDC, hRC)) // Try To Activate The Rendering Context
		{
			return ERR_CANT_CREATE; // Return FALSE
		}

		hRC_offscreen = wglCreateContextAttribsARB(hDC, 0, attribs);
	}

	wglSwapIntervalEXT = (PFNWGLSWAPINTERVALEXTPROC)wglGetProcAddress("wglSwapIntervalEXT");
	wglGetSwapIntervalEXT = (PFNWGLGETSWAPINTERVALEXTPROC)wglGetProcAddress("wglGetSwapIntervalEXT");
	//glWrapperInit(wrapper_get_proc_address);

	return OK;
}

const int OGL_THREAD_CONTROL_ID = 0x20C1221E;
const int OGL_THREAD_CONTROL_DISABLE = 0x00000002;
const int OGL_THREAD_CONTROL_ENABLE = 0x00000001;

typedef int(__cdecl *NvAPI_Initialize_t)();
typedef int(__cdecl *NvAPI_Unload_t)();
typedef int(__cdecl *NvAPI_GetErrorMessage_t)(unsigned int, NvAPI_ShortString);
typedef int(__cdecl *NvAPI_DRS_CreateSession_t)(NvDRSSessionHandle *);
typedef int(__cdecl *NvAPI_DRS_DestroySession_t)(NvDRSSessionHandle);
typedef int(__cdecl *NvAPI_DRS_LoadSettings_t)(NvDRSSessionHandle);
typedef int(__cdecl *NvAPI_DRS_FindApplicationByName_t)(NvDRSSessionHandle, NvAPI_UnicodeString, NvDRSProfileHandle *, NVDRS_APPLICATION *);
typedef int(__cdecl *NvAPI_DRS_CreateProfile_t)(NvDRSSessionHandle, NVDRS_PROFILE *, NvDRSProfileHandle *);
typedef int(__cdecl *NvAPI_DRS_CreateApplication_t)(NvDRSSessionHandle, NvDRSProfileHandle, NVDRS_APPLICATION *);
typedef int(__cdecl *NvAPI_DRS_SaveSettings_t)(NvDRSSessionHandle);
typedef int(__cdecl *NvAPI_DRS_SetSetting_t)(NvDRSSessionHandle, NvDRSProfileHandle, NVDRS_SETTING *);
typedef int(__cdecl *NvAPI_DRS_FindProfileByName_t)(NvDRSSessionHandle, NvAPI_UnicodeString, NvDRSProfileHandle *);
NvAPI_GetErrorMessage_t NvAPI_GetErrorMessage__;

static bool nvapi_err_check(char *msg, int status) {
	if (status != 0) {
		if (OS::get_singleton()->is_stdout_verbose()) {
			NvAPI_ShortString err_desc = { 0 };
			NvAPI_GetErrorMessage__(status, err_desc);
			print_verbose(vformat("%s: %s(code %d)", msg, err_desc, status));
		}
		return false;
	}
	return true;
}

// On windows we have to disable threaded optimization when using NVIDIA graphics cards
// to avoid stuttering, see https://github.com/microsoft/vscode-cpptools/issues/6592
// also see https://github.com/Ryujinx/Ryujinx/blob/master/Ryujinx.Common/GraphicsDriver/NVThreadedOptimization.cs
void ContextGL_Windows::_nvapi_disable_threaded_optimization() {
	HMODULE nvapi = 0;
#ifdef _WIN64
	nvapi = LoadLibraryA("nvapi64.dll");
#else
	nvapi = LoadLibraryA("nvapi.dll");
#endif

	if (nvapi == NULL) {
		print_verbose("Error loading NVAPI library");
		return;
	}

	void *(__cdecl * NvAPI_QueryInterface)(unsigned int interface_id) = 0;

	NvAPI_QueryInterface = (void *(__cdecl *)(unsigned int))GetProcAddress(nvapi, "nvapi_QueryInterface");

	if (NvAPI_QueryInterface == NULL) {
		print_verbose("Error getting NVAPI NvAPI_QueryInterface");
		return;
	}

	// Setup NVAPI function pointers
	NvAPI_Initialize_t NvAPI_Initialize = (NvAPI_Initialize_t)NvAPI_QueryInterface(0x0150E828);
	NvAPI_GetErrorMessage__ = (NvAPI_GetErrorMessage_t)NvAPI_QueryInterface(0x6C2D048C);
	NvAPI_DRS_CreateSession_t NvAPI_DRS_CreateSession = (NvAPI_DRS_CreateSession_t)NvAPI_QueryInterface(0x0694D52E);
	NvAPI_DRS_DestroySession_t NvAPI_DRS_DestroySession = (NvAPI_DRS_DestroySession_t)NvAPI_QueryInterface(0xDAD9CFF8);
	NvAPI_DRS_FindApplicationByName_t NvAPI_DRS_FindApplicationByName = (NvAPI_DRS_FindApplicationByName_t)NvAPI_QueryInterface(0xEEE566B2);
	NvAPI_Unload_t NvAPI_Unload = (NvAPI_Unload_t)NvAPI_QueryInterface(0xD22BDD7E);
	NvAPI_DRS_LoadSettings_t NvAPI_DRS_LoadSettings = (NvAPI_DRS_LoadSettings_t)NvAPI_QueryInterface(0x375DBD6B);
	NvAPI_DRS_CreateProfile_t NvAPI_DRS_CreateProfile = (NvAPI_DRS_CreateProfile_t)NvAPI_QueryInterface(0xCC176068);
	NvAPI_DRS_CreateApplication_t NvAPI_DRS_CreateApplication = (NvAPI_DRS_CreateApplication_t)NvAPI_QueryInterface(0x4347A9DE);
	NvAPI_DRS_SaveSettings_t NvAPI_DRS_SaveSettings = (NvAPI_DRS_SaveSettings_t)NvAPI_QueryInterface(0xFCBC7E14);
	NvAPI_DRS_SetSetting_t NvAPI_DRS_SetSetting = (NvAPI_DRS_SetSetting_t)NvAPI_QueryInterface(0x577DD202);
	NvAPI_DRS_FindProfileByName_t NvAPI_DRS_FindProfileByName = (NvAPI_DRS_FindProfileByName_t)NvAPI_QueryInterface(0x7E4A9A0B);

	if (!nvapi_err_check("NVAPI: Init failed", NvAPI_Initialize())) {
		return;
	}

	print_verbose("NVAPI: Init OK!");

	NvDRSSessionHandle session_handle;

	if (!nvapi_err_check("NVAPI: Error creating DRS session", NvAPI_DRS_CreateSession(&session_handle))) {
		NvAPI_Unload();
		return;
	}

	if (!nvapi_err_check("NVAPI: Error loading DRS settings", NvAPI_DRS_LoadSettings(session_handle))) {
		NvAPI_DRS_DestroySession(session_handle);
		NvAPI_Unload();
		return;
	}

	String app_executable_name = OS::get_singleton()->get_executable_path().get_file();
	String app_friendly_name = GLOBAL_GET("application/config/name");
	String app_profile_name = app_friendly_name + " Nvidia Profile";

	// We need a name anyways, so let's use the engine name if an application name is not available
	// (this is used mostly by the Project Manager)
	if (app_friendly_name.empty()) {
		app_friendly_name = VERSION_NAME;
	}

	NvDRSProfileHandle profile_handle;
	NvAPI_UnicodeString nvapi_profile_name;
	memcpy(nvapi_profile_name, app_profile_name.c_str(), sizeof(CharType) * (app_profile_name.size() + 1));

	int status = NvAPI_DRS_FindProfileByName(session_handle, nvapi_profile_name, &profile_handle);

	if (status != 0) {
		print_verbose("NVAPI: Profile not found, creating....");

		NVDRS_PROFILE profile_info;
		profile_info.version = NVDRS_PROFILE_VER;
		profile_info.isPredefined = 0;
		memcpy(profile_info.profileName, app_profile_name.c_str(), sizeof(CharType) * (app_profile_name.size() + 1));

		if (!nvapi_err_check("NVAPI: Error creating profile", NvAPI_DRS_CreateProfile(session_handle, &profile_info, &profile_handle))) {
			NvAPI_DRS_DestroySession(session_handle);
			NvAPI_Unload();
			return;
		}

		NVDRS_APPLICATION_V4 app;
		app.version = NVDRS_APPLICATION_VER_V4;
		app.isPredefined = 0;
		app.isMetro = 1;
		app.isCommandLine = 1;
		memcpy(app.appName, app_executable_name.c_str(), sizeof(CharType) * (app_executable_name.size() + 1));
		memcpy(app.userFriendlyName, app_friendly_name.c_str(), sizeof(CharType) * (app_friendly_name.size() + 1));
		memcpy(app.launcher, L"", 1);
		memcpy(app.fileInFolder, L"", 1);

		if (!nvapi_err_check("NVAPI: Error creating application", NvAPI_DRS_CreateApplication(session_handle, profile_handle, &app))) {
			NvAPI_DRS_DestroySession(session_handle);
			NvAPI_Unload();
			return;
		}
	}

	NVDRS_SETTING setting;
	setting.version = NVDRS_SETTING_VER;
	setting.settingId = OGL_THREAD_CONTROL_ID;
	setting.settingType = NVDRS_DWORD_TYPE;
	setting.settingLocation = NVDRS_CURRENT_PROFILE_LOCATION;
	setting.isCurrentPredefined = 0;
	setting.isPredefinedValid = 0;
	int thread_control_val = OGL_THREAD_CONTROL_DISABLE;
	if (!GLOBAL_GET("rendering/threads/nvidia_disable_threaded_optimization")) {
		thread_control_val = OGL_THREAD_CONTROL_ENABLE;
	}
	setting.u32CurrentValue = thread_control_val;
	setting.u32PredefinedValue = thread_control_val;

	if (!nvapi_err_check("NVAPI: Error calling NvAPI_DRS_SetSetting", NvAPI_DRS_SetSetting(session_handle, profile_handle, &setting))) {
		NvAPI_DRS_DestroySession(session_handle);
		NvAPI_Unload();
		return;
	}

	if (!nvapi_err_check("NVAPI: Error saving settings", NvAPI_DRS_SaveSettings(session_handle))) {
		NvAPI_DRS_DestroySession(session_handle);
		NvAPI_Unload();
		return;
	}
	if (thread_control_val == OGL_THREAD_CONTROL_DISABLE) {
		print_verbose("NVAPI: Disabled OpenGL threaded optimization succesfully");
	} else {
		print_verbose("NVAPI: Enabled OpenGL threaded optimization succesfully");
	}
	NvAPI_DRS_DestroySession(session_handle);
}

ContextGL_Windows::ContextGL_Windows(HWND hwnd, bool p_opengl_3_context) {
	opengl_3_context = p_opengl_3_context;
	hWnd = hwnd;
	hRC_offscreen = NULL;
	use_vsync = false;
	vsync_via_compositor = false;
}

ContextGL_Windows::~ContextGL_Windows() {
}

#endif
