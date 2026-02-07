\## ZASS MANTIS (Battery Monitor(DISPLAY) and Logger)



STM32 based display and control unit designed for ZASS Lithium Battery Packs.



The system reads battery telemetry data via FDCAN/RS485 and visualizes it on a DWIN HMI screen using UART communication.





\## Features

* Displays Cell Voltages, Total Pack Voltage, SOC, and Temperature.
* \*\*2x FDCAN\*\* and \*\*1x RS485\*\* ports for communication.
* \*\*UART\*\* implementation for DWIN display driver.
* Detailed fault monitoring (Over-Voltage, Under-Voltage, Over-Temp, Over-Curr).
* Detailed log saving into SD Card included all data that comes from pack.





\## Communication Flow

* &nbsp; \*\*BMS -> MANTIS (FDCAN/RS485):\*\* Receiving raw battery data.
* &nbsp; \*\*MANTIS -> DISPLAY (UART/FDCAN):\*\* Parsing data and updating screen variables.



---

* #### Status:  **Hardware Completed** 
