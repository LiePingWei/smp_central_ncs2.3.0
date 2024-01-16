.. _central_smp:

Bluetooth: Central UART with SMP cental based on ncs2.3.0
#######################
1. 52840 central OTA 52840 peripheral，or 52840 central OTA  5340 peripheral
2. if device smp length is less than 2400, please change #define UPLOAD_CHUNK		2400；
3. replase dfu_smp.c in "v2.3.0\nrf\subsys\bluetooth\services\dfu_smp.c"  from root folder
4. replase zcbor_decode.c  in "v2.3.0\modules\lib\zcbor\src"   from root foler

测试：
Please follow below steps to test it.

1.	cd: ..\testing images 840central dfu 5340 periph folder.
2.	Program 5340dk as peripheral using program5340dk_periph.bat,please double click it.
3.	Program 840dk as central using program_52840dk_central.bat, please double click it and it will automatically program 5340 app_update.bin to customer_storage, please see pm_static.yml.
4.	Press button2 and start to upload app_update.bin to 5340 through SMP service，it will take about 10 to 20 seconds
5.	Press button1 to list images, more important is to get image2 hash value in slot1 through SMP list command.
6.	Press button4 to confirm the images as below: the pending value in slot1 will be true.
7.	Press button3 from 840DK to reset 5340 peripheral, and it will automatically reboot.wait about 10 to 20 seconds and see the log.
8.  restart device and see the log,the image will revert back to previous one.



