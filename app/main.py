import time

from tmc2226 import StepDriveController


GPIO_DIR = 8
GPIO_STEP = 9
GPIO_EN = 21

GPIO_UART_TX = 20
GPIO_UART_RX = 10

UART_ID = 1
TMC_ADDR = 0

TEST_FREQ_HZ = 5000
TEST_UP_STEPS = 10000

def _hex32(v):
    if v is None:
        return None
    return "0x%08x" % (v & 0xFFFFFFFF)


def uart_init(ctrl: StepDriveController):
    gstat = ctrl.tmc.read_gstat()
    ifcnt0 = ctrl.tmc.read_ifcnt()
    ctrl.tmc.set_current(ihold=6, irun=16, iholddelay=8)
    ifcnt1 = ctrl.tmc.read_ifcnt()
    print("GSTAT:", gstat)
    print("IFCNT:", ifcnt0, "->", ifcnt1)
    print("IHOLD_IRUN(shadow):", _hex32(ctrl.tmc.ihold_irun_shadow()))


def move_steps(ctrl: StepDriveController, steps: int, forward: bool, freq_hz: int):
    if steps <= 0:
        return
    if freq_hz <= 0:
        return

    ctrl.io.set_dir(forward)
    duration_ms = (int(steps) * 1000) // int(freq_hz)
    if duration_ms < 1:
        duration_ms = 1

    ctrl.io.run(freq_hz=freq_hz)
    time.sleep_ms(duration_ms)
    ctrl.io.stop()


def motor_test_up_down_center(ctrl: StepDriveController, up_steps: int, freq_hz: int):
    if ctrl.io.en is not None:
        ctrl.io.en.value(0)

    print("Move up:", up_steps)
    move_steps(ctrl, up_steps, True, freq_hz)
    time.sleep_ms(300)

    print("Move down:", up_steps)
    move_steps(ctrl, up_steps, False, freq_hz)
    time.sleep_ms(300)

    mid = up_steps // 2
    print("Move to center:", mid)
    move_steps(ctrl, mid, True, freq_hz)
    time.sleep_ms(300)


def run():
    ctrl = StepDriveController(
        step_pin=GPIO_STEP,
        dir_pin=GPIO_DIR,
        en_pin=GPIO_EN,
        uart_id=UART_ID,
        uart_tx_pin=GPIO_UART_TX,
        uart_rx_pin=GPIO_UART_RX,
        tmc_addr=TMC_ADDR,
        en_active_low=True,
    )

    uart_init(ctrl)
    motor_test_up_down_center(ctrl, up_steps=TEST_UP_STEPS, freq_hz=TEST_FREQ_HZ)


if __name__ == "__main__":
    run()
