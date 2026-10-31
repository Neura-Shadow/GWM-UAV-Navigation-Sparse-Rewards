"""Offscreen EGL/GL renderer probe (no window, no simulator): prints GL_RENDERER per EGL platform.

Preflight for diagnostic gate N on a native NVIDIA host; also used to verify WSL D3D12 selection.
"""
import ctypes, ctypes.util, json, sys

egl = ctypes.CDLL(ctypes.util.find_library("EGL") or "libEGL.so.1")
gl = ctypes.CDLL(ctypes.util.find_library("OpenGL") or "libOpenGL.so.0")
EGL_PLATFORM_SURFACELESS_MESA, EGL_PLATFORM_DEVICE_EXT = 0x31DD, 0x313F
EGL_OPENGL_API, EGL_NONE, EGL_RENDERER_STRING = 0x30A2, 0x3038, 0x1F01
egl.eglGetProcAddress.restype = ctypes.c_void_p
egl.eglGetProcAddress.argtypes = [ctypes.c_char_p]
egl.eglGetDisplay.restype = ctypes.c_void_p
egl.eglCreateContext.restype = ctypes.c_void_p
egl.eglCreateContext.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
egl.eglInitialize.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
egl.eglChooseConfig.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
egl.eglMakeCurrent.argtypes = [ctypes.c_void_p] * 4
egl.eglQueryString.restype = ctypes.c_char_p
egl.eglQueryString.argtypes = [ctypes.c_void_p, ctypes.c_int]
gl.glGetString.restype = ctypes.c_char_p
get_platform = ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p)(
    egl.eglGetProcAddress(b"eglGetPlatformDisplayEXT"))
query_devices = egl.eglGetProcAddress(b"eglQueryDevicesEXT")


def renderer(display):
    if not display or not egl.eglInitialize(display, None, None):
        return "eglInitialize failed"
    egl.eglBindAPI(EGL_OPENGL_API)
    config, count = ctypes.c_void_p(), ctypes.c_int()
    attrs = (ctypes.c_int * 1)(EGL_NONE)
    egl.eglChooseConfig(display, attrs, ctypes.byref(config), 1, ctypes.byref(count))
    context = egl.eglCreateContext(display, config if count.value else None, None, None)
    if not context or not egl.eglMakeCurrent(display, None, None, context):
        return "context failed"
    return (gl.glGetString(EGL_RENDERER_STRING) or b"?").decode()


result = {"surfaceless": renderer(get_platform(EGL_PLATFORM_SURFACELESS_MESA, None, None))}
if query_devices:
    qd = ctypes.CFUNCTYPE(ctypes.c_uint, ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p)(query_devices)
    devices, n = (ctypes.c_void_p * 8)(), ctypes.c_int()
    qd(8, devices, ctypes.byref(n))
    for i in range(n.value):
        result[f"device{i}"] = renderer(get_platform(EGL_PLATFORM_DEVICE_EXT, devices[i], None))
print(json.dumps(result))
