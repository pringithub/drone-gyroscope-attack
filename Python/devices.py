from __future__ import division
import serial, visa, struct, copy, time


class arduino_com:
    serial_port = None
    baudrate = 9600

    def __init__(self, port_num, baudrate):
        self.serial_port = '/dev/tty.usbmodem' + str(port_num)
        self.baudrate = baudrate
        self.serial_stream = None

    # Connect to Arduino
    def connect(self):
        self.serial_stream = serial.Serial(self.serial_port, baudrate=self.baudrate)
        self.serial_stream.flushInput()  # flush input buffer
        self.serial_stream.flushOutput()  # flush output buffer
        print("Connecting to Arduino on port: " + self.serial_port + " ...")
        while not self.serial_stream.isOpen():  # and not outputStream.isOpen()):
            continue
        print("Connected to Arduino!")
        time.sleep(2)

    # Disconnect from Arduino
    def disconnect(self):
        self.serial_stream.close()
        print
        "Disconnected from Arduino"

    # Start experiment code on Arduino
    def start_exp(self):
        self.serial_stream.write('s')  # write any character to serial stream to start exp

    # Acquire X, Y, Z, MEMS Device Data
    def acquire_mems_data(self, num_data_points):
        # check if serial port opened
        if self.serial_stream == None:
            print
            "ERROR: arduino serial stream not open!"
            return None

        # acquire data
        x_data = []
        y_data = []
        z_data = []

        for i in range(num_data_points):
            # Get X Data
            inputByte = self.serial_stream.read(size=1)
            while (inputByte != 'x'):
                if (inputByte == 'e'):
                    print
                    inputByte
                    print
                    self.serial_stream.read(size=1)
                    raw_input(r"Error trying to read mems device data. Press anykey to restart experiment...")
                    self.serial_stream.write('s')
                inputByte = self.serial_stream.read(size=1)
            byte_obj = self.serial_stream.read(size=2)
            x_data.append(struct.unpack('<h', str(byte_obj[0:2]))[0])

            # Get Y Data
            inputByte = self.serial_stream.read(size=1)
            while (inputByte != 'y'):
                inputByte = self.serial_stream.read(size=1)
            byte_obj = self.serial_stream.read(size=2)
            y_data.append(struct.unpack('<h', str(byte_obj[0:2]))[0])

            # Get Z Data
            inputByte = self.serial_stream.read(size=1)
            while (inputByte != 'z'):
                inputByte = self.serial_stream.read(size=1)
            byte_obj = self.serial_stream.read(size=2)
            z_data.append(struct.unpack('<h', str(byte_obj[0:2]))[0])

        return [x_data, y_data, z_data]


class function_generator:
    resource_str = u'USB0::2391::1031::MY44050469::INSTR'
    instrument = None
    amplitude = None

    def __init__(self, amplitude):
        self.resource_str = self.GetFirstResourceString()
        self.instrument = None
        self.amplitude = amplitude

    def GetFirstResourceString(self):
        rm = visa.ResourceManager()
        availableResources = rm.list_resources()

        print("Resource string: " + availableResources[0])
        return availableResources[0]

    def connect(self):
        print
        "Connecting to Function Generator..."
        rm = visa.ResourceManager()
        self.instrument = rm.open_resource(self.resource_str)
        print
        "CONNECTED! --> " + str(self.instrument)

    def disconnect(self):
        self.instrument.close()
        print
        "Disconnected from Function Generator"

    def initialize(self):
        self.instrument.write('SOURce1:FREQuency:MODE CW')
        self.instrument.write('SOURce1:VOLTage:LEVel:IMMediate:AMPLitude ' + str(self.amplitude))
        self.turn_off()

    def turn_on(self):
        self.instrument.write('OUTPut1:STATe ON')
        time.sleep(1)

    def turn_off(self):
        self.instrument.write('OUTPut1:STATe OFF')
        time.sleep(1)

    def set_frequency(self, freq):
        self.instrument.write('SOURce1:FREQuency:CW ' + str(freq) + 'Hz')
        time.sleep(0.1)


class oscilloscope:
    resource_str = u'USB0::0x0699::0x0507::C010253::INSTR'
    instrument = None

    def __init__(self):
        self.resource_str = u'USB0::0x0699::0x0507::C010253::INSTR'
        self.instrument = None

    def connect(self):
        print
        "Connecting to Oscilloscope..."
        rm = visa.ResourceManager()
        self.instrument = rm.open_resource(self.resource_str)
        print
        "CONNECTED! --> " + str(self.instrument)

    def disconnect(self):
        self.instrument.close()
        print
        "Disconnected from Oscilloscope"

    def send_noninteractive_command(self, command):
        self.instrument.write(command)
        time.sleep(0.1)
        self.check_error_code()

    def send_interactive_command(self, command):
        self.instrument.write(command)
        time.sleep(0.1)
        return self.instrument.read()

    def get_status(self):
        print
        str(self.send_interactive_command('*IDN?'))

    def check_error_code(self):
        error_code = int(self.send_interactive_command('*ESR?'))
        if error_code != 0:
            print
            self.send_interactive_command(':allev?;')
            print

    def linspace(self, start, stop, n):
        if n == 1:
            yield stop
            return
        h = (stop - start) / (n - 1)
        for i in range(n):
            yield start + h * i

    def get_waveform(self, channel):
        self.send_noninteractive_command('DATA:SOURCE ' + str(channel))
        self.send_noninteractive_command('DATA:ENCDG RIBINARY')
        self.send_noninteractive_command('WFMOutpre:BYT_Nr 2')
        num_values = self.send_interactive_command('HORizontal:RECOrdlength?')
        self.send_noninteractive_command('DATA:START 1')
        self.send_noninteractive_command('DATA:STOP ' + str(num_values))
        data = self.instrument.query_binary_values('CURV?', datatype='h', is_big_endian=True)
        self.check_error_code()

        self.send_noninteractive_command('HEADER OFF')
        # Get Y axis ratios and offsets
        yMultiplier = float(self.send_interactive_command(':wfmo:ymu?'))
        yOffset = float(self.send_interactive_command(':wfmo:yof?'))
        yZero = float(self.send_interactive_command(':wfmo:yzero?'))
        values = copy.deepcopy(data)
        for i in range(len(values)):
            values[i] = ((float(values[i]) - yOffset) * yMultiplier) + yZero

        # Get X axis ratios and offsets
        numPoints = float(self.send_interactive_command(':wfmo:nr_pt?'))
        xIncrements = float(self.send_interactive_command(':wfmo:xinc?'))
        xZero = float(self.send_interactive_command(':wfmo:xzero?'))
        time_points = list(self.linspace(xZero, xZero + (xIncrements * numPoints), int(numPoints)))
        return [time_points, values]

    def get_single_aquisition_and_stop(self):
        self.send_noninteractive_command('ACQUIRE:STOPAFTER SEQUENCE')
        self.send_noninteractive_command('ACQUIRE:STATE RUN')

    # self.send_interactive_command('*OPC? ')
