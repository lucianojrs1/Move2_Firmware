# LTE and CAN wiring notes

## Pins reserved by the LILYGO T-A7670G SA R2 modem

- GPIO26: modem UART TX
- GPIO27: modem UART RX
- GPIO4: modem PWRKEY
- GPIO12: peripheral/modem power enable, must be driven HIGH by firmware
- GPIO5: modem reset
- GPIO25: modem DTR
- GPIO33: modem RING

Do not reconnect the CAN transceiver to GPIO26/GPIO27. Those pins now belong
to the modem UART.

## Default CAN relocation used by this firmware

- ESP32 GPIO23 -> CAN transceiver TXD/CTX
- ESP32 GPIO36 -> CAN transceiver RXD/CRX

GPIO36 is input-only, which is fine for CAN RX because the transceiver drives
that signal.

GPIO23 was previously reserved in the firmware as GPS PPS, but the current GPS
logic does not use PPS. The firmware now leaves PPS disabled so GPIO23 can be
used as CAN TX.

Keep the TF card pins free for the card: GPIO15 MOSI, GPIO14 SCLK, GPIO13 CS
and GPIO2 MISO.

## CAN transceiver wiring

- ESP32 GPIO23 to transceiver TXD/CTX
- ESP32 GPIO36 to transceiver RXD/CRX
- ESP32 GND to transceiver GND and vehicle/device GND
- CANH to bus CANH
- CANL to bus CANL
- VCC according to your transceiver module
