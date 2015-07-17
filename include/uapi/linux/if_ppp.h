/*
| ============================================================================ |
|      Modifications on Features list / Changes Request / Problems Report      |
| **************************************************************************** |
|    date   |        author        |         Key          |      comment       |
| **********|**********************|**********************|******************* |
| 12/02/2014|      wen.ye          |      FR-793371       |Failed to Connect PPTP with Encryption |
| **********|**********************|**********************|******************* |
================================================================================
 */
#include <linux/ppp-ioctl.h>

/*[BUGFIX]-Add-BEGIN by TCTNB.wen.ye,12/02/2014,FR793371, port from FR807274 and 622650*/
/*Failed to Connect PPTP with Encryption*/
#define        PPP_MTU         1500    /* Default MTU (size of Info field) */
#define        PPP_MAXMRU     65000   /* Largest MRU we allow */
/*[BUGFIX]-Add-END by TCTNB.wen.ye*/
