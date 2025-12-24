import threading
import time
import numpy as np


from data_collection.submodules.ati_sensor_socket import NetFTSensor


class ForceWrenchRecorder:
    def __init__(self, sensor: NetFTSensor, sensor_frequency: int):

        self.sensor = sensor
        self.sensor_frequency = sensor_frequency
        self.running = False
        self.data_thread = None

        self.signal_averaged = np.zeros(6)
        self.complete_signal = []

        self.wrench_timestamp = None  # will store timestamp of last packet

    def start_filling(self):

        if not self.running:

            self.running = True  # Thread Active
            self.data_thread = threading.Thread(target=self._store_data)
            self.data_thread.start()

            print("DATA THREAD START RUNNING")
        else:
            print("DATA THREAD IS ALREADY RUNNING")

    def _store_data(self):

        while self.running:

            try:
                time.sleep(
                    1 / self.sensor_frequency
                )  # To match data publishing rate of ATI

                _wrench_packet = self.sensor.get_most_recent_data()

                # print(
                #     "Force-Torque Data:",
                #     np.array2string(
                #         _wrench_packet.force_torque_data,
                #         precision=3,
                #         separator=", ",
                #         suppress_small=True,
                #     ),
                # )

                if _wrench_packet != "No data received yet.":

                    try:
                        _wrench = _wrench_packet.get_force_torque_array()
                        self.wrench_timestamp = _wrench_packet.get_ft_timestamp()
                        self.complete_signal.append(_wrench)

                    except Exception as e:
                        print("Exception error occured!")

            except:

                print("Exception error occured in store_data !")

    def stop_filling(self):

        if self.running:
            print("Stopping DATA THREAD")
            self.running = False

            if self.data_thread:
                self.data_thread.join()

        self.signal_averaged = np.mean(self.complete_signal, axis=0)

        return self.complete_signal, self.signal_averaged, self.wrench_timestamp
