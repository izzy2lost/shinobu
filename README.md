# Godot (Desktop windows on ANGLE)

(The original README is available in `README-original.md`)

This is a hacked together modification of the Godot Engine 3.2 for use in Project Heartbeat, this version contains a replacement for the GLES context used in the Windows desktop version of the engine. This replacement makes use of frankensteined bits from the UWP version to make it work with ANGLE.

ANGLE is the "Almost Native Graphics Layer Engine", in this case it's used to translate the original GL calls to DirectX 11 and Vulkan, the reasons why one might want to do this are the following:

- OpenGL on some optimus cards has framepacing issues, aparently running OpenGL programs through ANGLE makes these issues go away (a native Vulkan port would fix it too but 4.0 is pretty far away).
- Certain console allows only Direct3D 11 and 12 to be used. So this will serve as my testbed for porting to that platform.
- The existing UWP port uses an old version of ANGLE (I think it also runs on the GLES2 code?) which has issues rendering viewports, I hope this port doesn't present this problems when running on UWP, but I haven't tested it on UWP, it will be interesting to see if the issues have gone away.

# Building

Only MSVC is supported for building this version due to not having enough time to test it myself, but feel free to modify the `detect.py` of the windows platform to try and make it work with MinGW.

This has been tested against the current angle master, this would be commit `be774187b1cefe94f4717cbd747ebdea0b349478`.

Do all of this inside the MSVC's native tools command prompt for your platform.

First of all, set your ANGLE_PATH variable to the path where you store the ANGLE code.

`set ANGLE_PATH=C:\Users\EIREXE\angle`

We don't take care of actually building ANGLE, you must do this yourself.

Afterwards, you can build godot normally.

After you are done building it, copy the `libEGL.dll` and `libGLESv2.dll` files to the bin folder.

# Running & Configuration

The original `--video-driver` command line argument is retained.

A new additional argument is added, ``--angle-backend`` which supports setting the backend to ``vulkan``

This version of godot defaults to DirectX 11, however you can cange the default in the `platform/windows/context_gl_windows.cpp` file.

# Known issues

- I haven't figured out how to make ANGLE link statically if it's even possible, so help would be appreciated in that front.
- ANGLE doesn't fully implement GLES 3.2 in their Vulkan backend so please use GLES2 if you plan to use vulkan.

# Debugging

I found that the release version of ANGLE doesn't produce errors as descriptive as the debug version, if you want to have those errors you can simply drop in the debug-built ANGLE dlls.