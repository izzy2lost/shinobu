"""Functions used to generate source files during build time

All such functions are invoked in a subprocess on Windows to prevent build flakiness.

"""

import os
from io import StringIO

# See also `editor/icons/editor_icons_builders.py`.
def make_imgui_fonts_action(target, source, env):
    dst = target[0].srcnode().abspath

    g = open(dst, "w", encoding="utf-8")

    g.write("/* THIS FILE IS GENERATED DO NOT EDIT */\n")
    g.write("#ifndef _IMGUI_FONTS_H\n")
    g.write("#define _IMGUI_FONTS_H\n")

    g.write("#ifdef DEBUG_ENABLED\n")
    

    # Saving uncompressed, since FreeType will reference from memory pointer.
    for i in range(len(source)):
        with open(str(source[i]), "rb") as f:
            buf = f.read()

        name = os.path.splitext(os.path.basename(str(source[i])))[0]

        g.write("static const int _font_" + name + "_size = " + str(len(buf)) + ";\n")
        g.write("static const unsigned char _font_" + name + "[] = {\n")
        for j in range(len(buf)):
            g.write("\t" + str(buf[j]) + ",\n")

        g.write("};\n")

    g.write("#endif // DEBUG_ENABLED\n")
    g.write("#endif // _IMGUI_FONTS_H")

    g.close()

