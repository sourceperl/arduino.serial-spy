# arduino.serial-spy
A tool for spy serial line (RS232/485) from an Arduino Mega 2560 (in development)

Bitlash shell, spy function :

- **spy_dump(MaxLine2Display, lifo mode [default off])**:

    Display byte data on console i hex, dec, ascii format.

- **spy_stat**

    Display min/max time step on serial line

- **spy_baud**

    Display baudrates : measure baudrate with normalized baudrate, UART baudrate (can be change with spy_setbaud command).

- **spy_setbaud(UART baudrate [default is 9600])**

    Use to set UART baudrate.
