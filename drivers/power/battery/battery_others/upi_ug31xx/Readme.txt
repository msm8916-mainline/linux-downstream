Last Update : 2014-06-12, Version 1.00

This document is maintained by Allen Teng <allen.kuoliang.teng@gmail.com> 
as part of the UPI uG31xx gas gauge project.

-------------------------------------------------------------------------------

Steps of porting upi_ug31xx gauge driver

1. Put the "upi_ug31xx" folder at driver/power/battery/

2. Add following lines in Kconfig at driver/power/battery/

    config COVER_GAUGE_UG31XX_Z380KL
      tristate "UPI uG31xx gauge driver"
      depends on I2C
      help
        Say Y here to enable support for UPI uG31xx gauge IC.

3. Add following line in Makefile at driver/power/battery/

    obj-$(CONFIG_COVER_GAUGE_UG31XX_Z380KL) += upi_ug31xx/

4. Check the UPI_BATTERY with menuconfig

