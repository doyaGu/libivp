# IVPEmbedShaders.cmake -- write shader sources into a C header
#
# Script mode, run at build time:
#   cmake -DSHADER_DIR=<dir> -DNAMES=<a,b,...> -DOUTPUT=<header> -P IVPEmbedShaders.cmake
#
# Each <dir>/<name>.glsl becomes
#   static const char ivp_gles_<name>[] = "...";
# with <name> made a C identifier (line3d.vert -> ivp_gles_line3d_vert).

string(REPLACE "," ";" names "${NAMES}")
set(content "/* Generated from src/render/shaders/es3 by cmake/IVPEmbedShaders.cmake. */\n")
foreach (name IN LISTS names)
    file(READ "${SHADER_DIR}/${name}.glsl" text)
    string(REPLACE "\\" "\\\\" text "${text}")
    string(REPLACE "\"" "\\\"" text "${text}")
    string(REPLACE "\n" "\\n\"\n    \"" text "${text}")
    string(MAKE_C_IDENTIFIER "ivp_gles_${name}" ident)
    string(APPEND content "\nstatic const char ${ident}[] =\n    \"${text}\";\n")
endforeach ()
file(WRITE "${OUTPUT}" "${content}")
