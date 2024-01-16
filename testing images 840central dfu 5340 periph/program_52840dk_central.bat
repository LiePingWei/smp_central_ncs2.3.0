
nrfjprog --recover --snr 683999535

nrfjprog --program  ../build/zephyr/merged.hex --verify --snr 683999535

arm-none-eabi-objcopy --input-target=binary --output-target=ihex --change-address 0x80000 --gap-fill=0xff  app_update5340.bin app_update5340.hex

nrfjprog --program app_update5340.hex  --verify --snr 683999535


for /f %%i in ('python getsize.py') do set "output=%%i"

::rem 输出结果
echo Python returns:%output%

::set  MAC_3MSB=D40108

nrfjprog --family NRF52 --memwr 0xF0000 --val %output:~0,8% --snr 683999535
nrfjprog --reset --snr 683999535

pause