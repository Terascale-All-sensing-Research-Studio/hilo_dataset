import sys
import time
import serial.tools.list_ports


class Turntable:
    def __init__(self, port=None, verbose=False):
        """
        Initializes the turntable controller.
        """

        self.verbose = verbose

        if port is not None:
            self.arduino_port = port

        else:
            # Finds the USB port for the Arduino controller.
            self.arduino_port = None
            all_ports = serial.tools.list_ports.comports()
            for port, desc, hwid in all_ports:
                # print("{}: {}: {}".format(port, desc, hwid))
                if "Arduino" in desc:
                    self.arduino_port = port
                    break

            if self.arduino_port is None:
                raise FileNotFoundError("No Arduino found.")

        print("Arduino port: ", self.arduino_port)

        # Configures the serial connection.
        self.arduino = serial.Serial(
            port=self.arduino_port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

        # Waits for the Arduino controller to reset.
        time.sleep(2)

    def rotate(self, angle_deg, wait=False):
        """
        Rotates the turntable to the given degree of angle.
        :param angle_deg: the degree of angle
        :param wait: True if blocking the process to wait for the rotation finishes, False otherwise
        """

        self.arduino.write(str(int(angle_deg)).encode('ascii'))
        time.sleep(0.5)
        response_angle = self.arduino.readline().decode('ascii')

        if self.verbose:
            print("Steps: ", response_angle)

        if wait:
            time.sleep(0.5)
            response_done = self.arduino.readline().decode('ascii')

            if self.verbose:
                print(response_done)


if __name__ == "__main__":
    turntable = Turntable(verbose=True)

    if len(sys.argv) == 1:
        print("Rotates 8 X 45 deg...")
        for i in range(8):
            turntable.rotate(45, True)  # Rotates 45 deg.

    elif len(sys.argv) == 2:
        angle = int(sys.argv[1])
        print("Rotates: %s deg..." % angle)
        turntable.rotate(angle, True)

    else:
        raise ValueError("Invalid rotating angle!")
