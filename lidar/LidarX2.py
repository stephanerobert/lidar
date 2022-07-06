"""
This code was taken from https://youyeetoo.com/blog/ydlidar-x2-lidar-laser-radar-ld0011-65
I'm not responsible for the code quality and only made the code more PEP8 compliant
"""
from math import atan, pi
from threading import Thread
from time import sleep

from serial import Serial


class LidarMeasure:
    def __init__(self, angle, distance):
        self.angle = angle
        self.distance = distance

    def __repr__(self):
        return "" + str(self.angle) + ": " + str(self.distance) + "mm"


class LidarX2:
    def __init__(self, port):
        self.port = port
        self.baudrate = 115200
        self.connected = False
        self.measuring_thread = None
        self.stop = False
        self.measures = []
        self.serial = None

    def open(self):
        try:
            if not self.connected:
                # Open serial
                self.serial = Serial(self.port, self.baudrate)
                timeout = 4000  # ms
                while not self.serial.isOpen() and timeout > 0:
                    timeout -= 10  # ms
                    sleep(0.01)  # 10ms
                if self.serial.isOpen():
                    self.connected = True
                    self.serial.flushInput()
                else:
                    return False
                # Start measure thread
                self.stop = False
                self.measuring_thread = Thread(target=LidarX2._measure_thread, args=(self,))
                self.measuring_thread.setDaemon(True)
                self.measuring_thread.start()
                return True
        except Exception as e:
            print(e)
        return False

    def close(self):
        self.stop = True
        if self.measuring_thread:
            self.measuring_thread.join()
        if self.connected:
            self.serial.close()
            self.connected = False

    def get_measures(self):
        return list(self.measures)

    def _measure_thread(self):
        start_angle = 0
        while not self.stop:
            measures = self._get_readings()
            if len(measures) == 0:
                continue
            # Get Start an End angles
            end_angle = measures[len(measures) - 1].angle
            # Clear measures in the angle range
            i = 0
            while i < len(self.measures):
                m = self.measures[i]
                if end_angle > start_angle:
                    in_range = start_angle <= m.angle and m.angle <= end_angle
                else:
                    in_range = (start_angle <= m.angle and m.angle <= 360) or (0 <= m.angle and m.angle <= end_angle)
                if in_range:
                    self.measures.pop(i)
                    i -= 1
                i += 1
            # Add measures
            for measure in measures:
                self._insert_measure(self.measures, measure)
            start_angle = end_angle

    def _insert_measure(self, values, measure, low=0, high=None):
        if low < 0:
            raise ValueError('lo must be non-negative')
        if high is None:
            high = len(values)
        while low < high:
            mid = (low + high) // 2
            if measure.angle < values[mid].angle:
                high = mid
            else:
                low = mid + 1
        values.insert(low, measure)

    def _read_byte(self):
        # serial.read can return byte or str depending on python version...
        return self._to_int(self.serial.read(1))

    def _to_int(self, value):
        if isinstance(value, str):
            return int(value.encode('hex'), 16)
        if isinstance(value, int):
            return value
        return int.from_bytes(value, byteorder='big')

    def _get_readings(self):
        result = []
        # Check and flush serial
        if not self.connected:
            return result
        # Wait for data start bytes
        found = False
        checksum = 0x55AA
        while not found and not self.stop:
            while self.serial.read(1) != b"\xaa":
                pass
            if self.serial.read(1) == b"\x55":
                found = True
        if self.stop:
            return []
        # Check packet type
        ct = self._read_byte()
        if ct != 0:
            return result
        # Get sample count in packet
        ls = self._read_byte()
        sample_count = ls  # int(ls.encode('hex'), 16)
        if sample_count == 0:
            return result
        # Get start angle
        fsa_l = self._read_byte()
        fsa_m = self._read_byte()
        fsa = fsa_l + fsa_m * 256
        checksum ^= fsa
        start_angle = (fsa >> 1) / 64
        # Get end angle
        lsa_l = self._read_byte()
        lsa_m = self._read_byte()
        lsa = lsa_l + lsa_m * 256
        end_angle = (lsa >> 1) / 64
        # Compute angle diff
        a_diff = float(end_angle - start_angle)
        if (a_diff < 0):
            a_diff = a_diff + 360
        # Get checksum
        cs_l = self._read_byte()
        cs_m = self._read_byte()
        cs = cs_l + cs_m * 256
        # Read and parse data
        raw_data = self.serial.read(sample_count * 2)
        data = []
        for i in range(0, sample_count * 2):
            data.append(self._to_int(raw_data[i]))
        for i in range(0, sample_count * 2, 2):
            # Get distance
            si_l = data[i]
            si_m = data[i + 1]
            checksum ^= (si_l + si_m * 256)
            distance = float(si_l + si_m * 256) / 4
            # Get angle and correct value from distance
            angle = start_angle + (a_diff / float(sample_count)) * i / 2
            angle_correction = 0
            if distance > 0:
                angle_correction = (atan(21.8 * ((155.3 - distance) / (155.3 * distance))) * (180 / pi))
            angle = angle + angle_correction
            if angle > 360:
                angle = angle - 360
            # Append to result
            result.append(LidarMeasure(angle, distance))
        checksum ^= (ct + ls * 256)
        checksum ^= lsa
        # Validate checksum
        if checksum == cs:
            return result
        return []
