nrfjprog -f NRF53 --coprocessor CP_NETWORK --recover --snr 1050006222

nrfjprog --recover --snr 1050006222

nrfjprog --coprocessor CP_NETWORK --eraseall --snr 1050006222
::nrfjprog --program build/hci_rpmsg/zephyr/merged_CPUNET.hex --sectorerase --verify
nrfjprog -f NRF53 --coprocessor CP_NETWORK --program merged_CPUNET.hex --verify --snr 1050006222

nrfjprog --eraseall --snr 1050006222

nrfjprog --program merged.hex --verify --snr 1050006222

 
nrfjprog --debugreset --snr 1050006222

pause