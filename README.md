# Breadcrumb

A modular system to route traffic between  a GSM node and a Wifi node, using an Xbee interlink. Comprehensive documentation visable at: https://www.overleaf.com/read/zqypdsthbbch

Built as final project for the EE 475 Embedded Systems Capstone at the Univeristy of Washington, under the instruction of James Peckol. 


## ESP8266 Build Instructions:

The project is based on the excellent esp-open-sdk and esp-open-rtos projects, and requires both to be present in order to be built. Simply clone and setup the esp-open-sdk toolchain, and then clone the esp-open-rtos project into the ESP8266 directory. The program can now be built by running make, and programmed into an ESP8266 using esptool. 
