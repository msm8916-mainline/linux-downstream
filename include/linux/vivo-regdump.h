#ifndef VIVO_REGDUMP_H
#define VIVO_REGDUMP_H

struct vivo_regdump_handler {
	char *name;
	void (*callback)(void);     
};

int vivo_audio_regdump_register_callback(struct vivo_regdump_handler *handler);
void vivo_audio_regdump_deregister_callback(char *callback_name);
void vivo_audio_regdump_do_callback(void);

#endif