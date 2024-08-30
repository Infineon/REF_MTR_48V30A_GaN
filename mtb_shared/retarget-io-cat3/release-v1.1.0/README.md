# Retarget IO (CAT3)

### Overview

A utility library to retarget the standard input/output (STDIO) messages to a UART port on XMC™ 1000 and XMC™ 4000 devices. With this library, you can directly print messages on a UART terminal using `printf()`. The USIC peripheral channel, TX pin and RX pin have to be configured by the application before calling the `cy_retarget_io_init()` function.

**NOTE:** The standard library is not standard in how it treats an I/O stream. Some implement a data buffer by default. The buffer is not flushed until it is full. In that case it may appear that your I/O is not working. You should be aware of how the library buffers data, and you should identify a buffering strategy and buffer size for a specified stream. If you supply a buffer, it must exist until the stream is closed. The following line of code disables the buffer for the standard library that accompanies the GCC compiler:

    setvbuf( stdin, NULL, _IONBF, 0 );

**NOTE:** If the application is built using newlib-nano, by default, floating point format strings (%f) are not supported. To enable this support, you must add `-u _printf_float` to the linker command line.

### RTOS Integration
To avoid concurrent access to the UART peripheral in a RTOS environment, the ARM and IAR libraries use mutexes to control access to stdio streams. For Newlib (GCC_ARM), the mutex must be implemented in _write() and can be enabled by adding `DEFINES+=CY_RTOS_AWARE` to the Makefile. For all libraries, the program must start the RTOS kernel before calling any stdio functions.

### Quick Start
1. Check CYBSP_DEBUG_UART, CYBSP_DEBUG_UART_RX and CYBSP_DEBUG_UART_TX are enabled and configured in the BSP design.modus
2. Add `#include "cybsp.h"`
3. Add `#include "cy_retarget_io.h"`
4. Call `cybsp_init();`
5. Call `cy_retarget_io_init(CYBSP_DEBUG_UART_HW);`
6. Start printing using `printf()`

### Enabling Conversion of '\\n' into "\r\n"
If you want to use only '\\n' instead of "\r\n" for printing a new line using printf(), define the macro `CY_RETARGET_IO_CONVERT_LF_TO_CRLF` using the *DEFINES* variable in the application Makefile. The library will then append '\\r' before '\\n' character on the output direction (STDOUT). No conversion occurs if "\r\n" is already present.

### Floating Point Support
By default, floating point support is enabled in printf. If floating point values will not be used in printed strings, this functionality can be disabled to reduece flash consumption. To disable floating support, add the following to the application makefile: `DEFINES += CY_RETARGET_IO_NO_FLOAT`.

### More information

* [API Reference Guide](https://infineon.github.io/retarget-io-cat3/html/index.html)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Infineon GitHub](https://github.com/infineon)
* [ModusToolbox™](https://www.cypress.com/products/modustoolbox-software-environment)

---
© Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation, 2021-2023.
