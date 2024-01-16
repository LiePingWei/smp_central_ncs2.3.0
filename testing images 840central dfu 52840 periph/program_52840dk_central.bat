
nrfjprog --recover

nrfjprog --program  ../build/zephyr/merged.hex --verify

arm-none-eabi-objcopy --input-target=binary --output-target=ihex --change-address 0x80000 --gap-fill=0xff  app_update.bin app_update.hex

nrfjprog --program app_update.hex  --verify


for /f %%i in ('python getsize.py') do set "output=%%i"

::rem 输出结果
echo Python returns:%output%

::set  MAC_3MSB=D40108

nrfjprog --family NRF52 --memwr 0xF0000 --val %output:~0,8%

nrfjprog --reset


del app_update.hex

pause