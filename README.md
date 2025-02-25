Modified from Olof Astrand's hello_beagley repo
https://github.com/Ebiroll/hello_beagley/tree/main

https://medium.com/@olof-astrand/hello-world-on-the-c7x-dsp-coprocessor-with-the-beagley-ai-c18d646c8655

Please refer to Project Setup and Build Guide written by Saran Sujan

Files of generated/ti_* are generated using the TI Sysconfig software, it needs to be opened in the SDK directory this command

```bash
user@debian:~/ti/ti-processor-sdk-rtos-j722s-evm-10_01_00_04/mcu_plus_sdk_j722s_10_01_00_22$ gmake -s -C examples/hello_beagley_spi/j722s-evm/mcu-r5fss0-0_freertos/ti-arm-clang/ syscfg-gui
```
It will craete a template based on j722s-evm/example.syscfg, use the GUI to configure the firmware, output to the generated directory.

Current this can use echo start, stop to boot and shutdown the remote MCU R5F core. We also can use MCU_SPI0 channel, I tested it with SPI loopback test (connect MCU_SPI0_D0 And MCU_SPI0_D1 with a wire).

It should be fine on keeping the /overlays/k3-am67a-beagley-ai-spidev0.dtbo on. The registers will be overwritten by the R5F firmware when it boots anyway.