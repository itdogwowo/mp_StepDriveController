# mp_StepDriveController
A MicroPython controller for stepper motor driver ICs commonly used in 3D printers.

## Wiring (ESP32-C3 -> TMC2226 module)
- GPIO8  -> DIR
- GPIO9  -> STEP
- GPIO21 -> EN / ENN
- GPIO20 -> UART TX
- GPIO10 -> UART RX

UART single-wire note (recommended): TMC2226 uses a single-wire, bidirectional UART on PDN_UART. A common wiring is:
- ESP RX directly to PDN_UART
- ESP TX to PDN_UART through ~1k series resistor
- Ensure the PDN_UART line is idle high (add a pull-up to VCC_IO if needed)

## Quick start
- Treat `app/` as the MCU filesystem root.
- Copy the contents of `app/` (including `main.py` and `tmc2226.py`) to the board filesystem root.
- Reset the board and open the serial REPL to see prints.
