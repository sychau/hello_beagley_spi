Modified from Olof Astrand's hello_beagley repo
https://github.com/Ebiroll/hello_beagley/tree/main

The blog(not really related)
https://medium.com/@olof-astrand/hello-world-on-the-c7x-dsp-coprocessor-with-the-beagley-ai-c18d646c8655

# What this repo can do
- Boot from remote(Linux)
- Redirect debug trace to remote linux file (doesn't work everywhere in the code)
- Receive RPMessage(ping) from remote(Linux)
- Listen for RPMessage packets and send data to LCD with MCSPI driver
- Gracefully shutdown from remote(Linux)

# Prerequisite
Need at least 10Gb disk space to install all dependencies, but you can also use the pre-compiled firmware and the bash scripts if you want to skip: hello_spi.release.out.

# Install dependencies
1. [PROCESSOR-SDK-RTOS-J722S — Processor SDK RTOS for J722S](https://www.ti.com/tool/PROCESSOR-SDK-J722S)\

2. [SYSCONFIG](https://www.ti.com/tool/download/SYSCONFIG)

3. [TI CLANG compiler toolchain](https://dr-download.ti.com/software-development/ide-configuration-compiler-or-debugger/MD-ayxs93eZNN/3.2.2.LTS/ti_cgt_armllvm_3.2.2.LTS_linux-x64_installer.bin)

4. [C7000 code generation tools - compiler](https://www.ti.com/tool/C7000-CGT)

Install all above tools to ~/ti


# Fix
1. Fix the any tools(sysconfig, comopiler etc.) version that doesn't match in the imports.mak in the MCU+SDK directory.

# Clone this repo
```bash
user@debian:~/ti/ti-processor-sdk-rtos-j722s-evm-10_01_00_04/mcu_plus_sdk_j722s_10_01_00_22/examples
```
This repo have to be cloned to this specific directory. Otherwise it can't detect the TI tools we installed because the makefile is hardcoded. Probably could be fixed later.

# Build
```bash
cd j722s-evm/mcu-r5fss0-0_freertos/ti-arm-clang
make scrub
make
```
hello_spi.release.out is the R5F firmware.

# Use
Assumed you have copied the firmware to your board. Also copied the bash scripts for running the firmware.

98% of the time remoteproc2 is R5F MCU domain(79000000.r5f), but it is not guaranteed because it depends on which was booted first. Just try remoteproc0/1/3/4 if it doesn't work.

Upload the firmware and start remoteproc
```bash
sudo ./uploadFirmwareAndStart.sh 2 hello_spi.release.out
```

Check debug trace remoteproc2
```bash
sudo ./observeProc.sh 2
```

Shutdown remoteproc2
```bash
sudo ./shutdownProc.sh 2
```

If any issue, check dmesg.
# How to generate configuration code using Sysconfig GUI
```bash
user@debian:~/ti/ti-processor-sdk-rtos-j722s-evm-10_01_00_04/mcu_plus_sdk_j722s_10_01_00_22$ gmake -s -C examples/hello_beagley_spi/j722s-evm/mcu-r5fss0-0_freertos/ti-arm-clang/ syscfg-gui
```
In case you want to change the configuration, e.g. use new driver, modify the SPI setting.

The syscofig GUI has to be opened in this specific location. This will read the example.syscfg and create a template for you to modify in the GUI, after modifying the configuration, save the output to the ti-arm-clang/generated.(the generated example.syscfg can be deleted, just a duplication) You can save the your j722s-evm/mcu-r5fss0-0_freertos/example.syscfg if you want.

You should not modify any ti_* files manually because it is error prone. You should generate them using the GUI.



# Note
1. It should be fine keeping the /overlays/k3-am67a-beagley-ai-spidev0.dtbo from you assignment 2 on. The registers will be overwritten by the R5F firmware when it boots anyway.

2. I used freeRTOS just as a starter code. Currently no special freeRTOS features is used in this repo so not necessary to know hwo it works


# Document
[J722S MCU+ SDK](https://software-dl.ti.com/jacinto7/esd/processor-sdk-rtos-j722s/10_01_00_04/exports/docs/mcu_plus_sdk_j722s_10_01_00_22/docs/api_guide_j722s/index.html)