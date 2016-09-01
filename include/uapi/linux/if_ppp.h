#include <linux/ppp-ioctl.h>

/*[BUGFIX]-Add-BEGIN by zhiqiang.hu,1/13/2016,task1416591, port from FR807274 and 622650*/
/*Failed to Connect PPTP with Encryption*/
#define        PPP_MTU         1500    /* Default MTU (size of Info field) */
#define        PPP_MAXMRU     65000   /* Largest MRU we allow */
/*[BUGFIX]-Add-END by zhiqiang.hu*/
