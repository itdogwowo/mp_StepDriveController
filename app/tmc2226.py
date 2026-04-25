try:
    from machine import Pin, UART, PWM
except ImportError:
    Pin = None
    UART = None
    PWM = None


def _u32_to_be(value: int) -> bytes:
    value &= 0xFFFFFFFF
    return bytes([(value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF])


def _be_to_u32(b: bytes) -> int:
    return ((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]) & 0xFFFFFFFF


def _crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        current = byte & 0xFF
        for _ in range(8):
            if ((crc >> 7) ^ (current & 0x01)) & 0x01:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            current >>= 1
    return crc & 0xFF


def _hex_bytes(b) -> str:
    if b is None:
        return "None"
    return " ".join(["%02x" % x for x in b])


class TMCUART:
    def __init__(
        self,
        uart_id: int,
        tx_pin: int,
        rx_pin: int,
        addr: int = 0,
        baudrate: int = 9600,
        timeout_ms: int = 20,
        rxbuf: int = 256,
    ):
        if UART is None or Pin is None:
            raise RuntimeError("machine.UART not available")
        self.addr = addr & 0x03
        try:
            self.uart = UART(
                uart_id,
                baudrate=baudrate,
                bits=8,
                parity=None,
                stop=1,
                tx=Pin(tx_pin),
                rx=Pin(rx_pin),
                timeout=timeout_ms,
                rxbuf=rxbuf,
            )
        except TypeError:
            self.uart = UART(
                uart_id,
                baudrate=baudrate,
                bits=8,
                parity=None,
                stop=1,
                tx=Pin(tx_pin),
                rx=Pin(rx_pin),
                timeout=timeout_ms,
            )
        self._shadow = {}

    def _flush(self) -> None:
        if self.uart.any():
            self.uart.read()

    def write_reg(self, reg: int, value: int) -> bytes:
        reg = (reg & 0x7F) | 0x80
        payload = _u32_to_be(value)
        frame_wo_crc = bytes([0x05, self.addr, reg]) + payload
        frame = frame_wo_crc + bytes([_crc8(frame_wo_crc)])
        self._flush()
        self.uart.write(frame)
        self._shadow[reg & 0x7F] = value & 0xFFFFFFFF
        return frame

    def _collect_bytes(self, read_timeout_ms: int):
        import time

        deadline = time.ticks_add(time.ticks_ms(), read_timeout_ms)
        buf = b""
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            n = self.uart.any()
            if n:
                chunk = self.uart.read(n)
                if chunk:
                    buf += chunk
            time.sleep_ms(1)
        return buf

    def _find_last_frame(self, buf, want_reg: int):
        if buf is None or len(buf) < 8:
            return None

        want_reg &= 0x7F
        last = None
        for i in range(0, len(buf) - 7):
            resp = buf[i : i + 8]
            if resp[0] != 0x05:
                continue
            if not (resp[1] == 0xFF or (resp[1] & 0x03) == self.addr):
                continue
            if (resp[2] & 0x7F) != want_reg:
                continue
            if _crc8(resp[:-1]) != resp[-1]:
                continue
            last = resp
        return last

    def read_reg(self, reg: int, read_timeout_ms: int = 30):
        import time

        reg = reg & 0x7F
        req_wo_crc = bytes([0x05, self.addr, reg])
        req = req_wo_crc + bytes([_crc8(req_wo_crc)])
        self._flush()
        self.uart.write(req)
        time.sleep_ms(2)

        buf = self._collect_bytes(read_timeout_ms)
        if not buf:
            return None

        last = self._find_last_frame(buf, reg)
        if last is None:
            return None

        return _be_to_u32(last[3:7])

    def debug_read_reg(self, reg: int, read_timeout_ms: int = 30):
        import time

        reg = reg & 0x7F
        req_wo_crc = bytes([0x05, self.addr, reg])
        req = req_wo_crc + bytes([_crc8(req_wo_crc)])
        self._flush()
        self.uart.write(req)
        time.sleep_ms(2)

        buf = self._collect_bytes(read_timeout_ms)
        last = self._find_last_frame(buf, reg) if buf else None
        value = None
        if last:
            value = _be_to_u32(last[3:7])
        return {
            "req": req,
            "resp": last,
            "value": value,
            "req_hex": _hex_bytes(req),
            "resp_hex": _hex_bytes(last),
            "rx_len": 0 if not buf else len(buf),
            "rx_hex": _hex_bytes(buf[:64] if buf else None),
        }

    def shadow(self, reg: int):
        return self._shadow.get(reg & 0x7F)

    def read_gstat(self):
        return self.read_reg(0x01)

    def clear_gstat(self) -> bytes:
        return self.write_reg(0x01, 0x07)

    def read_ifcnt(self):
        v = self.read_reg(0x02)
        if v is None:
            return None
        return v & 0xFF

    def set_current(self, ihold: int, irun: int, iholddelay: int = 8) -> bytes:
        ihold = int(ihold) & 0x1F
        irun = int(irun) & 0x1F
        iholddelay = int(iholddelay) & 0x0F
        value = ihold | (irun << 8) | (iholddelay << 16)
        return self.write_reg(0x10, value)

    def ihold_irun_shadow(self):
        return self.shadow(0x10)


class StepDir:
    def __init__(self, step_pin: int, dir_pin: int, en_pin=None, en_active_low: bool = True):
        if Pin is None:
            raise RuntimeError("machine.Pin not available")
        self.step = Pin(step_pin, Pin.OUT, value=0)
        self.dir = Pin(dir_pin, Pin.OUT, value=0)
        self.en = Pin(en_pin, Pin.OUT, value=1 if en_active_low else 0) if en_pin is not None else None
        self.en_active_low = bool(en_active_low)
        self._pwm = None

    def enable(self, on: bool = True) -> None:
        if self.en is None:
            return
        if self.en_active_low:
            self.en.value(0 if on else 1)
        else:
            self.en.value(1 if on else 0)

    def set_dir(self, forward: bool) -> None:
        self.dir.value(1 if forward else 0)

    def step_pulses(self, count: int, high_us: int = 2, low_us: int = 2) -> None:
        import time

        if count <= 0:
            return
        if high_us < 1:
            high_us = 1
        if low_us < 1:
            low_us = 1
        for _ in range(count):
            self.step.value(1)
            time.sleep_us(high_us)
            self.step.value(0)
            time.sleep_us(low_us)

    def run(self, freq_hz: int, duty_u16: int = 32768) -> None:
        if PWM is None:
            raise RuntimeError("machine.PWM not available")
        if freq_hz <= 0:
            self.stop()
            return
        if self._pwm is None:
            self._pwm = PWM(self.step)
        self._pwm.freq(int(freq_hz))
        duty_u16 = int(duty_u16) & 0xFFFF
        if hasattr(self._pwm, "duty_u16"):
            self._pwm.duty_u16(duty_u16)
        else:
            self._pwm.duty((duty_u16 * 1023) >> 16)

    def stop(self) -> None:
        if self._pwm is None:
            return
        self._pwm.deinit()
        self._pwm = None
        self.step.value(0)


class StepDriveController:
    def __init__(
        self,
        step_pin: int,
        dir_pin: int,
        en_pin: int,
        uart_id: int,
        uart_tx_pin: int,
        uart_rx_pin: int,
        tmc_addr: int = 0,
        en_active_low: bool = True,
    ):
        self.io = StepDir(step_pin=step_pin, dir_pin=dir_pin, en_pin=en_pin, en_active_low=en_active_low)
        self.tmc = TMCUART(uart_id=uart_id, tx_pin=uart_tx_pin, rx_pin=uart_rx_pin, addr=tmc_addr)
