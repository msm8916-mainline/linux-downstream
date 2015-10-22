#ifndef CADIZ_H
#define CADIZ_H

#define MAX_CADIZ_REGS 100

typedef struct cadiz_regs {
    int len;
    int regs[MAX_CADIZ_REGS][2];
}cadiz_settings;

#define CADIZ_IOCTL_SET_REGISTERS _IOW('s', 1, cadiz_settings)
#define CADIZ_IOCTL_GET_REGISTERS _IOWR('s', 2, cadiz_settings)

#endif
