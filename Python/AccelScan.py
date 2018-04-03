import os
import datetime
import shutil
import serial
import struct
import time
import subprocess
import sys
import socket
from enum import Enum

import instrumentDriver

# Experiment configuration settings
WAIT_FOR_ACCOUSTIC_WAVE_TO_PROPOGATE = 0.1
START_FREQ = 26000  # in Hz
END_FREQ = 28000  # in Hz
FREQ_INTERVAL = 10  # in Hz
FUNCTION_GENERATOR_AMPLITUDE = 8  # in Vpp
NUM_ACCELERATION_SAMPLES_TO_COLLECT = 128

# Equipment configuration settings
# SERIAL_PORT 						 = '/dev/cu.usbmodem14131'
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 38400
WAIT_FOR_ARDUINO_TO_REBOOT = 2
WAIT_FOR_ARDUINO_TO_RESPOND = 0.01
USE_FUNCTION_GENERATOR = True
USE_MATLAB_PLOTTER = False

# System configuration constants
X_PREFIX = "X-Data: "
Y_PREFIX = "Y-Data: "
Z_PREFIX = "Z-Data: "
MEASUREMENTS_FILENAME_BASE = "RawData/measurements"
MEASUREMENTS_FILENAME_EXT = ".csv"
MATLAB_STRING = 'matlab -nosplash -r rt_stats_plotter'


class MeasuringDevice(Enum):
    Arduino = 1


class Experiment:
    def __init__(self, start_freq, end_freq, freq_interval, sound_amplitude, measuring_device=MeasuringDevice.Arduino,
                 experiment_name=''):
        self.measurements_file_name = MEASUREMENTS_FILENAME_BASE + MEASUREMENTS_FILENAME_EXT
        self.start_freq = start_freq
        self.end_freq = end_freq
        self.freq_interval = freq_interval
        self.sound_amplitude = sound_amplitude
        self.experiment_name = experiment_name
        self.function_generator = None
        self.measurements_file = None
        self.serial_com = None
        self.measuring_device = measuring_device

        measurement_dir_name = os.path.dirname(self.measurements_file_name)
        if not os.path.isdir(measurement_dir_name):
            os.makedirs(measurement_dir_name)

        if USE_FUNCTION_GENERATOR:
            self.function_generator = instrumentDriver.instrument("/dev/usbtmc0", 'usb')

    # def UnpackBinaryData(self, line, prefix_string):
    #     # print( "Receiving data with prefix: " + prefix_string )
    #     num_bytes = int(line[len(prefix_string):])
    #     # print( "Receiving %d bytes" % num_bytes )
    #     binary_data = self.serial_com.read(num_bytes)
    #     # print( "--->Received %d bytes" % len( binary_data ) )
    #     num_measurements = len(binary_data) / 2  # assume 16-bit measurements
    #     unpacking_string = 'h' * num_measurements

        # return struct.unpack(unpacking_string, binary_data)
    #
    # def FlushUDPSocket(self):
    #     MAX_UDP_MESSAGE_SIZE = 8192
    #     try:
    #         while True:
    #             self.socket.recvfrom(MAX_UDP_MESSAGE_SIZE)
    #         # print "flushing UDP packets..."
    #     except socket.error:
    #         pass  # no pending UDP messages, can now return

    def GetMeasurementsFromArduino(self):
        DUMMY_CHARACTER = 's'
        self.serial_com.write(DUMMY_CHARACTER)  # trigger measurement
        time.sleep(WAIT_FOR_ARDUINO_TO_RESPOND)
        measurements = {X_PREFIX: [], Y_PREFIX: [], Z_PREFIX: []}

        while len(measurements[X_PREFIX]) != NUM_ACCELERATION_SAMPLES_TO_COLLECT:
            bytes_available = self.serial_com.inWaiting()
            # print( "bytes_available = %d" % bytes_available )
            if bytes_available == 0:
                self.serial_com.write(DUMMY_CHARACTER)  # trigger measurement
                time.sleep(WAIT_FOR_ARDUINO_TO_RESPOND)
                continue

            line = self.serial_com.readline()
            line = line.strip()
            x, y, z = struct.unpack('fff', line)
            measurements[X_PREFIX].append(x)
            measurements[Y_PREFIX].append(y)
            measurements[Z_PREFIX].append(z)

            # # Get X-Axis Data
            # if line.startswith(X_PREFIX):
            #     measurements[X_PREFIX] = self.UnpackBinaryData(line, X_PREFIX)
            #     self.serial_com.readline()  # flush EOL
            #     received_x = True
            # # Get Y-Axis Data
            # elif line.startswith(Y_PREFIX):
            #     measurements[Y_PREFIX] = self.UnpackBinaryData(line, Y_PREFIX)
            #     self.serial_com.readline()  # flush EOL
            #     received_y = True
            # # Get Z-Axis Data
            # elif line.startswith(Z_PREFIX):
            #     measurements[Z_PREFIX] = self.UnpackBinaryData(line, Z_PREFIX)
            #     self.serial_com.readline()  # flush EOL
            #     received_z = True
            # else:
            #     time.sleep(WAIT_FOR_ARDUINO_TO_RESPOND)

        return measurements

    def LogMeasurementsToFile(self, current_frequency, measurements):
        # launch MATLAB if not first logged data
        if (current_frequency == (self.start_freq + self.freq_interval)) and USE_MATLAB_PLOTTER:
            proc = subprocess.Popen(MATLAB_STRING, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE)
        # log data to file
        for (x, y, z) in zip(measurements[X_PREFIX], measurements[Y_PREFIX], measurements[Z_PREFIX]):
            self.measurements_file.write("{},{},{},{}\n".format(current_frequency, x, y, z))
        # self.measurements_file.write("%d " % current_frequency)
        # self.measurements_file.write(' '.join(map(str, measurements[X_PREFIX])) + '\n')
        # self.measurements_file.write("%d " % current_frequency)
        # self.measurements_file.write(' '.join(map(str, measurements[Y_PREFIX])) + '\n')
        # self.measurements_file.write("%d " % current_frequency)
        # self.measurements_file.write(' '.join(map(str, measurements[Z_PREFIX])) + '\n')

        self.measurements_file.flush()

    def Run(self):
        self.measurements_file = open(self.measurements_file_name, "w")
        self.SetupMeasuringDevice()
        scan_frequencies = range(self.start_freq, self.end_freq + self.freq_interval, self.freq_interval)

        try:
            for current_frequency in scan_frequencies:
                self.SetFrequencyAndWait(current_frequency)
                measurements = self.GetMeasurements()
                self.LogMeasurementsToFile(current_frequency, measurements)
                # self.measurements_file.flush()
        except KeyboardInterrupt:
            print("User interrupted - quitting!")
        finally:
            self.measurements_file.close()
            self.BackupMeasurementsFile()

            if self.measuring_device == MeasuringDevice.Arduino:
                self.serial_com.close()

            self.TeardownFunctionGeneratorIfUsed()

    def SetupMeasuringDevice(self):
        if self.measuring_device == MeasuringDevice.Arduino:
            self.serial_com = serial.Serial(SERIAL_PORT, BAUD_RATE)
            self.serial_com.reset_input_buffer()  # flush input buffer
            self.serial_com.reset_output_buffer()  # flush output buffer
            self.SetupFunctionGeneratorIfUsed()
            time.sleep(WAIT_FOR_ARDUINO_TO_REBOOT)
        else:
            raise Exception('Unknown measuring device')

    def GetMeasurements(self):
        if self.measuring_device == MeasuringDevice.Arduino:
            return self.GetMeasurementsFromArduino()
        else:
            raise Exception('Unknown measuring device')

    def BackupMeasurementsFile(self):
        ts = time.time()
        timestamp = datetime.datetime.fromtimestamp(ts).strftime('_%Y-%m-%d_%H-%M-%S')
        backupmeasurements_file_name = MEASUREMENTS_FILENAME_BASE + timestamp + self.experiment_name + MEASUREMENTS_FILENAME_EXT
        shutil.copy(self.measurements_file_name, backupmeasurements_file_name)
        print("Backup up measurements as " + backupmeasurements_file_name)

    def SetFrequencyAndWait(self, current_frequency):
        if USE_FUNCTION_GENERATOR:
            print("Setting frequency to %d" % current_frequency)
            self.function_generator.set_frequency(current_frequency, amplitude=self.sound_amplitude)
            time.sleep(WAIT_FOR_ACCOUSTIC_WAVE_TO_PROPOGATE)
        else:
            print("Skipping frequency %d" % current_frequency)

    def SetupFunctionGeneratorIfUsed(self):
        if USE_FUNCTION_GENERATOR:
            self.function_generator.change_output_state(True)

    def TeardownFunctionGeneratorIfUsed(self):
        if USE_FUNCTION_GENERATOR:
            self.function_generator.change_output_state(False)


def main():
    experiment_name = ''
    if len(sys.argv) >= 2:
        experiment_name = sys.argv[1]

    experiment = Experiment(START_FREQ,
                            END_FREQ,
                            FREQ_INTERVAL,
                            FUNCTION_GENERATOR_AMPLITUDE,
                            MeasuringDevice.Arduino,
                            experiment_name)
    experiment.Run()


if __name__ == '__main__':
    main()
