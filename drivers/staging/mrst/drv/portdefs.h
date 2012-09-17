/**
 * Definitions to assist in creating code that builds under both kernel
 * 3.0 and 3.4.
 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0))
#define MEMBER_HANDLE handle
#define MEMBER_PITCH pitch
#define drm_mode_fb_cmd2 drm_mode_fb_cmd
#else
#define MEMBER_HANDLE handles[0]
#define MEMBER_PITCH pitches[0]
#endif
