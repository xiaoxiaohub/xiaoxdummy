import errno
import fcntl
import os
import select
import termios
import time


BAUD_RATE_MAP = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}

if hasattr(termios, 'B230400'):
    BAUD_RATE_MAP[230400] = termios.B230400
if hasattr(termios, 'B460800'):
    BAUD_RATE_MAP[460800] = termios.B460800
if hasattr(termios, 'B921600'):
    BAUD_RATE_MAP[921600] = termios.B921600


class SerialTransport:
    def __init__(self, serial_port, baud_rate, logger):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.logger = logger
        self.serial_fd = -1

    @property
    def is_open(self):
        return self.serial_fd >= 0

    def open(self):
        flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
        try:
            self.serial_fd = os.open(self.serial_port, flags)
        except OSError as exc:
            self.logger.error(f'Cannot open {self.serial_port}: {exc}')
            self.serial_fd = -1
            return False

        current_flags = fcntl.fcntl(self.serial_fd, fcntl.F_GETFL)
        fcntl.fcntl(self.serial_fd, fcntl.F_SETFL, current_flags & ~os.O_NONBLOCK)

        try:
            attrs = termios.tcgetattr(self.serial_fd)
        except termios.error as exc:
            self.logger.error(f'tcgetattr failed: {exc}')
            self.close()
            return False

        speed = BAUD_RATE_MAP.get(self.baud_rate, termios.B115200)
        attrs[4] = speed
        attrs[5] = speed

        attrs[0] &= ~(
            termios.IXON | termios.IXOFF | termios.IXANY |
            termios.IGNBRK | termios.BRKINT | termios.PARMRK |
            termios.ISTRIP | termios.INLCR | termios.IGNCR | termios.ICRNL
        )
        attrs[1] &= ~termios.OPOST
        attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
        if hasattr(termios, 'CRTSCTS'):
            attrs[2] &= ~termios.CRTSCTS
        if hasattr(termios, 'HUPCL'):
            attrs[2] &= ~termios.HUPCL
        attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 2

        try:
            termios.tcsetattr(self.serial_fd, termios.TCSANOW, attrs)
            termios.tcflush(self.serial_fd, termios.TCIOFLUSH)
        except termios.error as exc:
            self.logger.error(f'tcsetattr failed: {exc}')
            self.close()
            return False

        return True

    def close(self):
        if self.serial_fd >= 0:
            os.close(self.serial_fd)
            self.serial_fd = -1

    def flush_input(self):
        if self.serial_fd >= 0:
            termios.tcflush(self.serial_fd, termios.TCIFLUSH)

    def send(self, message):
        if self.serial_fd < 0:
            return False

        data = message.encode('ascii')
        total_written = 0
        while total_written < len(data):
            try:
                written = os.write(self.serial_fd, data[total_written:])
            except OSError as exc:
                if exc.errno == errno.EINTR:
                    continue
                self.logger.warning(f'Serial write failed: {exc}')
                return False

            if written <= 0:
                return False
            total_written += written

        return True

    def read_line(self, timeout_ms):
        if self.serial_fd < 0:
            return ''

        deadline = time.monotonic() + (timeout_ms / 1000.0)
        line = bytearray()

        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            readable, _, _ = select.select([self.serial_fd], [], [], remaining)
            if not readable:
                break

            try:
                chunk = os.read(self.serial_fd, 1)
            except OSError as exc:
                if exc.errno == errno.EINTR:
                    continue
                self.logger.warning(f'Serial read failed: {exc}')
                return ''

            if not chunk:
                break

            char = chunk[0]
            if char == ord('\n'):
                break
            if char != ord('\r'):
                line.append(char)

        return line.decode('ascii', errors='ignore').strip()

    def send_and_read_reply(self, message, retries, timeout_ms, retry_sleep_ms):
        for attempt in range(retries):
            self.flush_input()
            if not self.send(message):
                return None

            reply = self.read_line(timeout_ms)
            if reply:
                return reply

            if attempt + 1 < retries:
                time.sleep(retry_sleep_ms / 1000.0)

        return None
