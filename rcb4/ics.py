import sys
import threading
import time

from colorama import Fore
from colorama import Style
import readchar
import serial
import serial.tools.list_ports
import yaml


class KeyListener(threading.Thread):
    def __init__(self):
        super().__init__()
        self.key = None
        self.lock = threading.Lock()
        self.running = True  # Add a running flag

    def run(self):
        """Continuously read keys and store the latest key in self.key while running is True."""
        while self.running:
            try:
                key = readchar.readkey()
                if key == 'q':
                    with self.lock:
                        self.key = key
                    break
            except KeyboardInterrupt:
                self.stop()
                break
            with self.lock:
                self.key = key

    def get_key(self):
        """Return the latest key and reset it."""
        with self.lock:
            key = self.key
            self.key = None
        return key

    def stop(self):
        """Stop the listener thread."""
        self.running = False


def load_and_process_yaml(yaml_path):
    with open(yaml_path) as f:
        joint_data = yaml.safe_load(f)
    servo_candidates = []
    servo_id_to_rotation = {}

    for _joint_name, properties in joint_data.get("joint_name_to_servo_id", {}).items():
        candidate_servo_id = properties["id"] // 2
        rotation = False
        if properties.get("type") == "continuous":
            rotation = True
        servo_id_to_rotation[candidate_servo_id] = rotation
        servo_candidates.append(candidate_servo_id)
    servo_candidates = sorted(set(servo_candidates))
    return servo_candidates, servo_id_to_rotation


def format_baud(baud):
    """Convert baud rate to a more readable format."""
    if baud >= 1_000_000:
        return f"{baud / 1_000_000:.2f}M"
    elif baud >= 1_000:
        return f"{baud / 1_000:.0f}k"
    else:
        return str(baud)


class ICSServoController:
    def __init__(self, baudrate=1250000, yaml_path=None):
        if baudrate not in [1250000, 625000, 115200]:
            print(f";; baud={baudrate} is wrong.")
            print(";; baud should be one of 1250000, 625000, 115200")
            print("Use default baudrate 1250000")
            baudrate = 1250000
        if yaml_path is not None:
            self.servo_candidates, self.servo_id_to_rotation = load_and_process_yaml(
                yaml_path
            )
        else:
            self.servo_id_to_rotation = None
            self.servo_candidates = list(range(18))
        self.servo_id_index = 0
        self.servo_id = 0
        self.selected_index = 0
        self.baudrate = baudrate
        self.timeout = 0.1
        self.ics = None
        self.servo_eeprom_params64 = [
            ("fix-header", [1, 2]),
            ("stretch-gain", [3, 4]),
            ("speed", [5, 6]),
            ("punch", [7, 8]),
            ("dead-band", [9, 10]),
            ("dumping", [11, 12]),
            ("safe-timer", [13, 14]),
            ("mode-flag-b7slave-b4rotation-b3pwm-b1free-b0reverse", [15, 16]),
            ("pulse-max-limit", [17, 18, 19, 20]),
            ("pulse-min-limit", [21, 22, 23, 24]),
            ("fix-dont-change-25", [25, 26]),
            ("ics-baud-rate-10-115200-00-1250000", [27, 28]),
            ("temperature-limit", [29, 30]),
            ("current-limit", [31, 32]),
            ("fix-dont-change-33", [33, 34]),
            ("fix-dont-change-35", [35, 36]),
            ("fix-dont-change-37", [37, 38]),
            ("fix-dont-change-39", [39, 40]),
            ("fix-dont-change-41", [41, 42]),
            ("fix-dont-change-43", [43, 44]),
            ("fix-dont-change-45", [45, 46]),
            ("fix-dont-change-47", [47, 48]),
            ("fix-dont-change-49", [49, 50]),
            ("response", [51, 52]),
            ("user-offset", [53, 54]),
            ("fix-dont-change-55", [55, 56]),
            ("servo-id", [57, 58]),
            ("stretch-1", [59, 60]),
            ("stretch-2", [61, 62]),
            ("stretch-3", [63, 64]),
        ]

    def open_connection(self):
        ports = serial.tools.list_ports.comports()
        if len(ports) == 0:
            print(f"{Fore.RED}No USB Found.{Style.RESET_ALL}")
            print(f"{Fore.RED}May Dual USB Adapter is wrong.{Style.RESET_ALL}")
        for p in ports:
            if p.vid == 0x165C and p.pid == 0x08:
                for baudrate in [115200, 625000, 1250000]:
                    try:
                        self.ics = serial.Serial(
                            f"/dev/{p.name}",
                            baudrate,
                            timeout=self.timeout,
                            parity=serial.PARITY_EVEN,
                        )
                        if baudrate != self.baudrate:
                            self.baud(self.baudrate)
                        return True
                    except IndexError:
                        continue
        return False

    def read_baud(self):
        _, result = self.read_param()
        return result["baud"]

    def baud(self, baud=None, servo_id=None):
        if baud is None:
            return self.read_baud()
        if baud not in [1250000, 625000, 115200]:
            print(f";; baud={baud} is wrong.")
            print(";; baud should be one of 1250000, 625000, 115200")
            return None

        if servo_id is None:
            servo_id = self.get_servo_id()

        ics_param64, _ = self.read_param()
        if baud == 1250000:
            ics_param64[27] = 0
        elif baud == 625000:
            ics_param64[27] = 1
        elif baud == 115200:
            ics_param64[27] = 10

        # Send updated parameters to the device
        self.set_param(ics_param64, servo_id)
        # Re-open the connection with the updated baud rate
        self.open_connection()
        return self.read_baud()

    def get_servo_id(self):
        self.ics.write(bytes([0xFF, 0x00, 0x00, 0x00]))
        time.sleep(0.1)
        ret = self.ics.read(5)
        servo_id = ret[4] & 0x1F
        return servo_id

    def set_servo_id(self, servo_id):
        self.ics.write(bytes([0xE0 | (0x1F & servo_id), 0x01, 0x01, 0x01]))
        time.sleep(0.1)
        ret = self.ics.read(5)
        return 0x1F & ret[4]

    def reset_servo_position(self):
        self.set_angle(7500)
        print(f"{Fore.YELLOW}Servo reset to zero position.{Fore.RESET}")

    def toggle_rotation_mode(self):
        rotation_mode = self.read_rotation()
        self.set_rotation(not rotation_mode)
        rotation_mode = self.read_rotation()
        mode_text = "Enabled" if rotation_mode else "Disabled"
        print(f"{Fore.CYAN}Rotation mode set to {mode_text}{Fore.RESET}")

    def set_free_mode(self):
        free_mode = self.read_free()
        self.set_free(not free_mode)
        time.sleep(0.1)
        free_mode = self.read_free()
        mode_text = "Enabled" if free_mode else "Disabled"
        print(f"{Fore.MAGENTA}Free mode set to {mode_text}{Fore.RESET}")

    def increase_angle(self):
        angle = self.read_angle()
        angle = min(10000, angle + 500)
        self.set_angle(angle)
        print(f"{Fore.BLUE}Angle increased to {angle}{Fore.RESET}")

    def decrease_angle(self):
        angle = self.read_angle()
        angle = max(0, angle - 500)
        self.set_angle(angle)
        print(f"{Fore.RED}Angle decreased to {angle}{Fore.RESET}")

    def parse_param64_key_value(self, v):
        alist = {}
        for param in self.servo_eeprom_params64:
            param_name, indices = param[0], param[1]
            alist[param_name] = self._4bit2num(indices, v)

        baud_value = alist.get("ics-baud-rate-10-115200-00-1250000", 0)
        baud_rate = {10: 115200, 1: 625000, 0: 1250000}.get(baud_value, None)
        mode_flag_value = alist.get(
            "mode-flag-b7slave-b4rotation-b3pwm-b1free-b0reverse", 0
        )
        alist.update(self.ics_flag_dict(mode_flag_value))
        alist.update({"servo-id": alist.get("servo-id", 0), "baud": baud_rate})
        return alist

    def _4bit2num(self, lst, v):
        sum_val = 0
        for i in lst:
            sum_val = (sum_val << 4) + (v[i - 1] & 0x0F)
        return sum_val

    def ics_flag_dict(self, v):
        return {
            "slave": (v & 0xF0) >> 4 & 0x8 == 0x8,
            "rotation": (v & 0xF0) >> 4 & 0x1 == 0x1,
            "free": v & 0xF & 0x2 == 0x2,
            "reverse": v & 0xF & 0x1 == 0x1,
            "serial": v & 0xF & 0x8 == 0x8,
        }

    def set_flag(self, flag_name, value, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        ics_param64, _ = self.read_param(servo_id=servo_id)

        # Calculate byte and bit for manipulation
        byte_idx = 14 if flag_name in ["slave", "rotation"] else 15
        bit_position = 3 if flag_name in ["slave", "serial"] else 0
        mask = 1 << bit_position

        # Set or clear the bit based on the `value` argument
        if value:
            ics_param64[byte_idx] |= mask  # Set the bit
        else:
            ics_param64[byte_idx] &= ~mask
            # Clear the bit
        # Set updated parameters to the servo
        self.set_param(ics_param64, servo_id=servo_id)

    def set_slave(self, slave=None, servo_id=None):
        return self.set_flag("slave", slave, servo_id=servo_id)

    def set_rotation(self, rotation=None, servo_id=None):
        return self.set_flag("rotation", rotation, servo_id=servo_id)

    def set_serial(self, serial=None, servo_id=None):
        return self.set_flag("serial", serial, servo_id=servo_id)

    def set_reverse(self, reverse=None, servo_id=None):
        return self.set_flag("reverse", reverse, servo_id=servo_id)

    def set_free(self, free, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()

        ics_param64, _ = self.read_param()
        if free is None or free == 0:
            ics_param64[15] = ics_param64[15] & 0xD
        else:
            ics_param64[15] = ics_param64[15] | 0x2
        self.set_param(ics_param64, servo_id)
        if servo_id != self.read_param()[1].get("servo-id"):
            return self.read_free(servo_id)

    def read_free(self, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        _, result = self.read_param()
        return result.get("free", None)

    def read_rotation(self, servo_id=None):
        _, result = self.read_param(servo_id=servo_id)
        return result["rotation"]

    def set_param(self, ics_param64, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        self.ics.write(bytes([0xC0 | servo_id, 0x00] + ics_param64))
        time.sleep(0.5)
        ret = self.ics.read(68)
        ret_ics_param64 = ret[4:]
        self.parse_param64_key_value(ret_ics_param64)

    def close_connection(self):
        if self.ics and self.ics.is_open:
            self.ics.close()
        self.ics = None

    def display_status(self):
        options = [
            "Current Servo ID",
            "Angle",
            "Baud Rate",
            "Rotation Mode",
            "Slave Mode",
            "Reverse Mode",
            "Serial Mode",
            "Free",
        ]
        selectable_options = ["Current Servo ID", "Angle"]

        key_listener = KeyListener()
        key_listener.daemon = True
        key_listener.start()
        try:
            use_previous_result = False
            previous_servo_id = None
            while key_listener.running:
                if not self.ics or not self.ics.is_open:
                    # Clear the previous output at the start of each loop
                    sys.stdout.write("\033[H\033[J")
                    sys.stdout.flush()
                    print(f"{Fore.RED}Connection is not open.{Style.RESET_ALL}")
                    print(f"{Fore.RED}Please check the following:")
                    print("1. Use Dual USB Adapter (https://kondo-robot.com/product/02116)")
                    print("2. Set the device to ICS mode.")
                    print(f"3. Connect only one servo correctly.{Style.RESET_ALL}")

                    print(
                        f"{Fore.RED}To establish the connection, please execute the following commands in Linux:{Style.RESET_ALL}"
                    )
                    print()
                    print(f"{Fore.RED}  $ sudo su{Style.RESET_ALL}")
                    print(f"{Fore.RED}  modprobe ftdi-sio{Style.RESET_ALL}")
                    print(
                        f"{Fore.RED}  echo 165C 0008 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id{Style.RESET_ALL}"
                    )
                    print(f"{Fore.RED}  exit{Style.RESET_ALL}")
                    try:
                        ret = self.open_connection()
                    except Exception:
                        continue
                    if ret is False:
                        time.sleep(1.0)
                        continue
                # Print servo status
                try:
                    servo_id = self.get_servo_id()
                    if servo_id != previous_servo_id:
                        use_previous_result = False
                    previous_servo_id = servo_id
                    if use_previous_result is False:
                        _, result = self.read_param()
                        print('======================================')
                        sys.stdout.write("\033[H\033[J")
                        sys.stdout.flush()
                        print("--- Servo Status ---")
                        for i, option in enumerate(options):
                            if i == self.selected_index:
                                print(
                                    f">> {option}: {self.get_status(option, result, selected=True)}"
                                )
                            else:
                                print(f"   {option}: {self.get_status(option, result, selected=False)}")

                        print("----------------------\n")
                        print(
                            "Use ↑↓ to navigate, ←→ to adjust Current Servo ID or Servo Angles"
                        )
                        print(f"Press 'Enter' when Current Servo ID is selected to save the currently highlighted ID in {Fore.GREEN}green{Style.RESET_ALL}.")
                        print("Press 'z' to reset servo position")
                        print(
                            "Press 'r' to toggle rotation mode (enables continuous wheel-like rotation)"
                        )
                        print("Press 'f' to set free mode\n")
                        print("'q' to quit.")

                    key = key_listener.get_key()
                    use_previous_result = False

                    # Perform actions based on key
                    if key == "z":
                        self.reset_servo_position()
                    elif key == "r":
                        self.toggle_rotation_mode()
                    elif key == "f":
                        self.set_free_mode()
                    elif key == "q":
                        print("Exiting...")
                        break
                    elif key == readchar.key.UP:
                        self.selected_index = (self.selected_index - 1) % len(
                            selectable_options
                        )
                    elif key == readchar.key.DOWN:
                        self.selected_index = (self.selected_index + 1) % len(
                            selectable_options
                        )
                    elif (
                        key == readchar.key.ENTER
                        and selectable_options[self.selected_index] == "Current Servo ID"
                    ):
                        self.set_servo_id(self.servo_id)
                        time.sleep(0.3)
                        if self.servo_id_to_rotation is not None:
                            self.set_rotation(self.servo_id_to_rotation[self.servo_id])
                            time.sleep(0.3)
                    elif (
                        key == readchar.key.LEFT
                        and selectable_options[self.selected_index] == "Current Servo ID"
                    ):
                        if self.servo_id_index == 0:
                            self.servo_id_index = len(self.servo_candidates) - 1
                        else:
                            self.servo_id_index = max(0, self.servo_id_index - 1)
                        self.servo_id = self.servo_candidates[self.servo_id_index]
                    elif (
                        key == readchar.key.RIGHT
                        and selectable_options[self.selected_index] == "Current Servo ID"
                    ):
                        if self.servo_id_index == len(self.servo_candidates) - 1:
                            self.servo_id_index = 0
                        else:
                            self.servo_id_index = min(
                                len(self.servo_candidates) - 1, self.servo_id_index + 1
                            )
                        self.servo_id = self.servo_candidates[self.servo_id_index]
                    elif (
                        key == readchar.key.LEFT
                        and selectable_options[self.selected_index] == "Angle"
                    ):
                        self.decrease_angle()
                    elif (
                        key == readchar.key.RIGHT
                        and selectable_options[self.selected_index] == "Angle"
                    ):
                        self.increase_angle()
                    else:
                        use_previous_result = True
                except Exception as e:
                    print(f'[ERROR] {e}')
                    use_previous_result = False
                    self.close_connection()
                    continue
                # Flush the output to ensure it displays correctly
                sys.stdout.flush()

                # Wait for a short period before updating again
                # time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nDisplay stopped by user.")
        key_listener.stop()

    def get_status(self, option, param=None, selected=False):
        """Return the status based on the selected option."""
        if option == "Current Servo ID":
            if param is not None:
                current_servo_id = param["servo-id"]
            else:
                current_servo_id = self.get_servo_id()

            s = f'{current_servo_id}'
            if selected:
                str = f"{Style.RESET_ALL}["
                for i, servo_id in enumerate(self.servo_candidates):
                    if i == self.servo_id_index:
                        str += f"{Fore.GREEN}{servo_id}{Style.RESET_ALL} "
                    else:
                        str += f"{servo_id} "
                str += "]"
                s += f' -> Next Servo ID: {str}'
            return s
        elif option == "Angle":
            return f"{self.read_angle()}"
        elif option == "Baud Rate":
            if param is not None:
                baudrate = param["baud"]
            else:
                baudrate = self.read_baud()
            return f"{format_baud(baudrate)}bps"
        elif option == "Rotation Mode":
            if param is not None:
                rotation = param["rotation"]
            else:
                rotation = self.read_rotation()
            return f"{Fore.GREEN if rotation else Fore.RED}{'Enabled' if rotation else 'Disabled'}{Style.RESET_ALL}"
        elif option == "Slave Mode":
            if param is not None:
                slave = param["slave"]
            else:
                slave = self.read_slave()
            return f"{Fore.GREEN if slave else Fore.RED}{'Enabled' if slave else 'Disabled'}{Style.RESET_ALL}"
        elif option == "Reverse Mode":
            if param is not None:
                reverse = param["reverse"]
            else:
                reverse = self.read_reverse()
            return f"{Fore.GREEN if reverse else Fore.RED}{'Enabled' if reverse else 'Disabled'}{Style.RESET_ALL}"
        elif option == "Serial Mode":
            if param is not None:
                serial = param["serial"]
            else:
                serial = self.read_serial()
            return f"{Fore.GREEN if serial else Fore.RED}{'Enabled' if serial else 'Disabled'}{Style.RESET_ALL}"
        elif option == "Free":
            if param is not None:
                free = param["free"]
            else:
                free = self.read_free()
            return f"{Fore.GREEN if free else Fore.RED}{'Enabled' if free else 'Disabled'}{Style.RESET_ALL}"

    def read_angle(self, sid=None):
        if sid is None:
            sid = self.get_servo_id()
        self.ics.write(bytes([0xA0 | (sid & 0x1F), 5]))
        time.sleep(0.1)
        v = self.ics.read(6)
        angle = ((v[4] & 0x7F) << 7) | (v[5] & 0x7F)
        return angle

    def set_angle(self, v=7500, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        self.ics.write(bytes([0x80 | (servo_id & 0x1F), (v >> 7) & 0x7F, v & 0x7F]))
        time.sleep(0.1)
        v = self.ics.read(6)
        angle = ((v[4] & 0x7F) << 7) | (v[5] & 0x7F)
        return angle

    def read_param(self, servo_id=None):
        if servo_id is None:
            servo_id = self.get_servo_id()
        self.ics.write(bytes([0xA0 | servo_id, 0x00]))
        time.sleep(0.1)
        ret = self.ics.read(68)
        ics_param64 = ret[4:]
        result = self.parse_param64_key_value(list(ics_param64))
        return list(ics_param64), result

    def _4bit2num(self, lst, v):
        sum_val = 0
        for i in lst:
            sum_val = (sum_val << 4) + (v[i - 1] & 0x0F)
        return sum_val


if __name__ == '__main__':
    servo_controller = ICSServoController()
    servo_controller.open_connection()