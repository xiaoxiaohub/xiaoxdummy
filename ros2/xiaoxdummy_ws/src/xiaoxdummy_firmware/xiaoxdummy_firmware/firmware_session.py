from dataclasses import dataclass
import time

from .serial_transport import SerialTransport


NUM_JOINTS = 6
DEFAULT_COMMAND_MODE = 2
DEFAULT_COMMAND_SPEED = 180.0
DEFAULT_BAUD_RATE = 115200
DEFAULT_TIMEOUT_MS = 200
DEFAULT_RETRIES = 25
DEFAULT_RETRY_SLEEP_MS = 200
DEFAULT_FEEDBACK_RETRIES = 5
DEFAULT_FEEDBACK_RETRY_SLEEP_MS = 50
DEFAULT_RECOVERY_RETRIES = 5


@dataclass(frozen=True)
class FirmwareBridgeConfig:
    serial_port: str
    baud_rate: int = DEFAULT_BAUD_RATE
    command_mode: int = DEFAULT_COMMAND_MODE
    command_speed: float = DEFAULT_COMMAND_SPEED
    reply_timeout_ms: int = DEFAULT_TIMEOUT_MS
    init_retries: int = DEFAULT_RETRIES
    retry_sleep_ms: int = DEFAULT_RETRY_SLEEP_MS
    feedback_retries: int = DEFAULT_FEEDBACK_RETRIES
    feedback_retry_sleep_ms: int = DEFAULT_FEEDBACK_RETRY_SLEEP_MS
    recovery_retries: int = DEFAULT_RECOVERY_RETRIES


class FirmwareSession:
    def __init__(self, config, logger):
        self.config = config
        self.logger = logger
        self.transport = SerialTransport(config.serial_port, config.baud_rate, logger)
        self.motors_enabled = False
        self.joint_positions = [0.0] * NUM_JOINTS
        self.joint_velocities = [0.0] * NUM_JOINTS
        self.last_feedback_time = None

    def cached_feedback(self):
        return list(self.joint_positions), list(self.joint_velocities)

    def start_session(self):
        if self.transport.is_open:
            return True, 'serial session already open'

        if not self.transport.open():
            return False, f'cannot open {self.config.serial_port}'

        if not self.initialize_serial_session(reset_feedback=True):
            self.transport.close()
            return False, 'failed to initialize firmware session'

        self.motors_enabled = False
        return True, 'serial session initialized'

    def stop_session(self):
        if self.transport.is_open:
            self.stop_and_disable_motors(retries=3)
        self.motors_enabled = False
        self.transport.close()
        return True, 'serial session closed'

    def set_active(self, enabled):
        if not self.transport.is_open:
            ok, message = self.start_session()
            if not ok:
                return ok, message

        if enabled:
            command_ok = self.send_control_command('!START\n', retries=self.config.init_retries)
        else:
            command_ok = self.stop_and_disable_motors(retries=self.config.init_retries)
        if not command_ok:
            recovered = self.recover_connection(
                'motor state change failed',
                retries=self.config.recovery_retries,
            )
            if enabled:
                command_ok = self.send_control_command('!START\n', retries=self.config.recovery_retries)
            else:
                command_ok = self.stop_and_disable_motors(retries=self.config.recovery_retries)
            if not recovered or not command_ok:
                return False, 'failed to enable motors' if enabled else 'failed to disable motors'

        self.motors_enabled = enabled
        positions = self.request_feedback_positions()
        if positions is not None:
            self.update_feedback_cache(positions, reset_velocity=True)
        return True, 'motors enabled' if enabled else 'motors disabled'

    def write_positions(self, positions):
        if len(positions) != NUM_JOINTS:
            return False, f'expected {NUM_JOINTS} joint commands, got {len(positions)}'
        if not self.transport.is_open:
            return False, 'serial port is not open'

        command = '>{}\n'.format(
            ','.join(f'{float(position):.2f}' for position in positions) +
            f',{self.config.command_speed:.2f}'
        )

        if not self.transport.send(command):
            recovered = self.recover_connection(
                'joint command write failure',
                retries=self.config.recovery_retries,
            )
            if not recovered or not self.transport.send(command):
                return False, 'failed to send joint command'

        return True, 'joint command sent'

    def refresh_positions(self):
        if not self.transport.is_open:
            return False, 'serial port is not open'

        positions = self.request_feedback_positions()
        if positions is None:
            self.logger.warning(
                f'Joint feedback timed out on {self.config.serial_port}, attempting bounded recovery'
            )
            recovered = self.recover_connection(
                'joint feedback timeout',
                retries=self.config.recovery_retries,
            )
            if recovered:
                positions = self.request_feedback_positions()

        if positions is None:
            return False, 'failed to read joint positions'

        self.update_feedback_cache(positions)
        return True, 'joint positions updated'

    def request_feedback_positions(self):
        return self.request_joint_positions(
            retries=self.config.feedback_retries,
            timeout_ms=self.config.reply_timeout_ms,
            retry_sleep_ms=self.config.feedback_retry_sleep_ms,
        )

    def stop_and_disable_motors(self, retries):
        stop_ok = self.send_control_command('!STOP\n', retries=retries)
        disable_ok = self.send_control_command('!DISABLE\n', retries=retries)
        return stop_ok and disable_ok

    def recover_connection(self, reason, retries=None):
        restore_motor_state = self.motors_enabled
        retry_count = self.config.recovery_retries if retries is None else retries
        self.logger.warning(
            f'Serial issue detected ({reason}), attempting recovery on {self.config.serial_port}'
        )

        self.transport.close()
        if not self.transport.open():
            return False

        if not self.initialize_serial_session(reset_feedback=True, retries=retry_count):
            self.transport.close()
            return False

        if restore_motor_state and not self.send_control_command('!START\n', retries=retry_count):
            self.transport.close()
            return False

        self.motors_enabled = restore_motor_state
        self.logger.info(f'Serial recovery succeeded on {self.config.serial_port}')
        return True

    def initialize_serial_session(self, reset_feedback, retries=None):
        retry_count = self.config.init_retries if retries is None else retries
        self.transport.flush_input()
        time.sleep(0.25)
        self.transport.flush_input()

        reply = self.transport.send_and_read_reply(
            f'#CMDMODE {self.config.command_mode}\n',
            retries=retry_count,
            timeout_ms=self.config.reply_timeout_ms,
            retry_sleep_ms=self.config.retry_sleep_ms,
        )
        if reply is None:
            self.logger.error(f'No response to CMDMODE on {self.config.serial_port}')
            return False

        positions = self.request_joint_positions(
            retries=retry_count,
            timeout_ms=self.config.reply_timeout_ms,
            retry_sleep_ms=self.config.retry_sleep_ms,
        )
        if positions is None:
            self.logger.error(
                f'Failed to read initial joint positions from {self.config.serial_port}'
            )
            return False

        self.update_feedback_cache(positions, reset_velocity=reset_feedback)
        self.logger.info(f'CMDMODE response: {reply}')
        return True

    def send_control_command(self, command, retries):
        reply = self.transport.send_and_read_reply(
            command,
            retries=retries,
            timeout_ms=self.config.reply_timeout_ms,
            retry_sleep_ms=50,
        )
        if reply is None:
            self.logger.warning(
                f'No response to control command {command.strip()} on {self.config.serial_port}'
            )
            return False
        command_name = command.strip().lstrip('!')
        self.logger.info(f'{command_name} response: {reply}')
        return True

    def update_feedback_cache(self, positions, reset_velocity=False):
        now = time.monotonic()
        if self.last_feedback_time is None or reset_velocity:
            velocities = [0.0] * NUM_JOINTS
        else:
            dt = now - self.last_feedback_time
            if dt <= 0.0:
                velocities = [0.0] * NUM_JOINTS
            else:
                velocities = [
                    (new - old) / dt for new, old in zip(positions, self.joint_positions)
                ]

        self.joint_positions = list(positions)
        self.joint_velocities = velocities
        self.last_feedback_time = now

    def request_joint_positions(self, retries=1, timeout_ms=None, retry_sleep_ms=None):
        timeout_ms = self.config.reply_timeout_ms if timeout_ms is None else timeout_ms
        retry_sleep_ms = self.config.feedback_retry_sleep_ms if retry_sleep_ms is None else retry_sleep_ms

        for attempt in range(retries):
            self.transport.flush_input()
            if not self.transport.send('#GETJPOS\n'):
                return None

            for _ in range(3):
                reply = self.transport.read_line(timeout_ms)
                if not reply:
                    break

                ok_pos = reply.find('ok')
                if ok_pos > 0:
                    reply = reply[ok_pos:]

                if not reply.startswith('ok'):
                    continue

                parts = reply.split()
                if len(parts) < NUM_JOINTS + 1:
                    continue

                try:
                    return [float(value) for value in parts[1:NUM_JOINTS + 1]]
                except ValueError:
                    continue

            if attempt + 1 < retries:
                time.sleep(retry_sleep_ms / 1000.0)

        return None
