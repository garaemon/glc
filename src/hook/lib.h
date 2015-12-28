/**
 * \file hook/lib.h
 * \brief wrapper library
 * \author Pyry Haulos <pyry.haulos@gmail.com>
 * \date 2007-2008
 * For conditions of distribution and use, see copyright notice in glc.h
 */

/**
 * \defgroup hook wrapper library
 *  \{
 */

#ifndef _LIB_H
#define _LIB_H

#include <X11/X.h>
#include <X11/Xlib.h>
#include <X11/extensions/xf86vmode.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <alsa/asoundlib.h>
#include <packetstream.h>
#include <pthread.h>

#include <glc/common/glc.h>

#define LIB_CAPTURING    0x1

typedef struct {
	void *(*dlopen)(const char *filename, int flag);
	void *(*dlsym)(void *, const char *);
	void *(*dlvsym)(void *, const char *, const char *);
	void *(*__libc_dlsym)(void *, const char *);
	int initialized;
	int running;
	pthread_mutex_t init_lock;
	glc_flags_t flags;
} glc_lib_t;

#define INIT_GLC \
	if (!lib.initialized) \
		init_glc();

/**
 * \addtogroup main
 *  \{
 */
__PRIVATE extern glc_lib_t lib;
__PRIVATE void init_glc();
__PRIVATE int start_glc();
__PRIVATE void get_real_dlsym();
__PRIVATE void *wrapped_func(const char *symbol);
__PRIVATE int open_stream();
__PRIVATE int close_stream();
__PRIVATE int reload_stream();
int start_capture();
int stop_capture();
__PRIVATE void increment_capture();
/**  \} */

/**
 * \addtogroup alsa
 *  \{
 */
__PRIVATE int alsa_init(glc_t *glc);
__PRIVATE int alsa_start(ps_buffer_t *buffer);
__PRIVATE int alsa_close();
__PRIVATE int alsa_capture_start_all();
__PRIVATE int alsa_capture_stop_all();
__PRIVATE int alsa_unhook_so(const char *soname);
/**  \} */

/**
 * \addtogroup opengl
 *  \{
 */
__PRIVATE int opengl_init(glc_t *glc);
__PRIVATE int opengl_start(ps_buffer_t *buffer);
__PRIVATE int opengl_capture_start();
__PRIVATE int opengl_capture_stop();
__PRIVATE int opengl_refresh_color_correction();
__PRIVATE int opengl_close();
__PRIVATE int opengl_push_message(glc_message_header_t *hdr, void *message, size_t message_size);
/**  \} */

/**
 * \addtogroup x11
 *  \{
 */
__PRIVATE int x11_init(glc_t *glc);
__PRIVATE int x11_close();
/**  \} */

/**
 * \defgroup hooks Hooked functions
 *  \{
 */
__PRIVATE void *__main_dlopen(const char *filename, int flag);
__PRIVATE void *__main_dlsym(void *handle, const char *symbol);
__PRIVATE void *__main_dlvsym(void *handle, const char *symbol, const char *version);
__PRIVATE void *__main___libc_dlsym(void *handle, const char *symbol);

typedef void (*GLXextFuncPtr)(void);
__PRIVATE GLXextFuncPtr __opengl_glXGetProcAddressARB(const GLubyte *proc_name);
__PRIVATE void __opengl_glXSwapBuffers(Display *dpy, GLXDrawable drawable);
__PRIVATE void __opengl_glFinish(void);
__PRIVATE void __opengl_glXSwapBuffers(Display *dpy, GLXDrawable drawable);
__PRIVATE GLXWindow __opengl_glXCreateWindow(Display *dpy, GLXFBConfig config, Window win, const int *attrib_list);

__PRIVATE int __x11_XNextEvent(Display *display, XEvent *event_return);
__PRIVATE int __x11_XPeekEvent(Display *display, XEvent *event_return);
__PRIVATE int __x11_XWindowEvent(Display *display, Window w, long event_mask, XEvent *event_return);
__PRIVATE Bool __x11_XCheckWindowEvent(Display *display, Window w, long event_mask, XEvent *event_return);
__PRIVATE int __x11_XMaskEvent(Display *display, long event_mask, XEvent *event_return);
__PRIVATE Bool __x11_XCheckMaskEvent(Display *display, long event_mask, XEvent *event_return);
__PRIVATE Bool __x11_XCheckTypedEvent(Display *display, int event_type, XEvent *event_return);
__PRIVATE Bool __x11_XCheckTypedWindowEvent(Display *display, Window w, int event_type, XEvent *event_return);
__PRIVATE int __x11_XIfEvent(Display *display, XEvent *event_return, Bool ( *predicate)(), XPointer arg);
__PRIVATE Bool __x11_XCheckIfEvent(Display *display, XEvent *event_return, Bool ( *predicate)(), XPointer arg);
__PRIVATE int __x11_XPeekIfEvent(Display *display, XEvent *event_return, Bool ( *predicate)(), XPointer arg);
__PRIVATE Bool __x11_XF86VidModeSetGamma(Display *display, int screen, XF86VidModeGamma *Gamma);

__PRIVATE int __alsa_snd_pcm_open(snd_pcm_t **pcmp, const char *name, snd_pcm_stream_t stream, int mode);
__PRIVATE int __alsa_snd_pcm_open_lconf(snd_pcm_t **pcmp, const char *name, snd_pcm_stream_t stream, int mode, snd_config_t *lconf);
__PRIVATE int __alsa_snd_pcm_close(snd_pcm_t *pcm);
__PRIVATE int __alsa_snd_pcm_hw_params(snd_pcm_t *pcm, snd_pcm_hw_params_t *params);
__PRIVATE snd_pcm_sframes_t __alsa_snd_pcm_writei(snd_pcm_t *pcm, const void *buffer, snd_pcm_uframes_t size);
__PRIVATE snd_pcm_sframes_t __alsa_snd_pcm_writen(snd_pcm_t *pcm, void **bufs, snd_pcm_uframes_t size);
__PRIVATE snd_pcm_sframes_t __alsa_snd_pcm_mmap_writei(snd_pcm_t *pcm, const void *buffer, snd_pcm_uframes_t size);
__PRIVATE snd_pcm_sframes_t __alsa_snd_pcm_mmap_writen(snd_pcm_t *pcm, void **bufs, snd_pcm_uframes_t size);
__PRIVATE int __alsa_snd_pcm_mmap_begin(snd_pcm_t *pcm, const snd_pcm_channel_area_t **areas, snd_pcm_uframes_t *offset, snd_pcm_uframes_t *frames);
__PRIVATE snd_pcm_sframes_t __alsa_snd_pcm_mmap_commit(snd_pcm_t *pcm, snd_pcm_uframes_t offset, snd_pcm_uframes_t frames);
/**  \} */

#endif

/**  \} */
