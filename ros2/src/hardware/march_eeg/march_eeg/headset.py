import subprocess
from time import sleep

import mne
import numpy as np

from march_eeg.eeg_utility import convert_to_list_of_floats


class Headset:
    amountOfChannels: int
    sfreq: int

    def get_data(self, number_of_samples):
        raise NotImplementedError("This should be an abstract method")

    def preprocessing(self, data):
        data = data[:, 0:self.amountOfChannels]
        data = data.transpose()

        my_iir_params = mne.filter.construct_iir_filter(dict(order=2, ftype='butter'), [8, 30.0], None, self.sfreq,
                                                        'bandpass', return_copy=False, verbose=False)
        data = mne.filter.filter_data(data=data, sfreq=self.sfreq, l_freq=8.0, h_freq=30.0, method="iir",
                                      iir_params=my_iir_params, verbose=False)

        return data

    def get_processed_data(self, number_of_samples):
        return self.preprocessing(self.get_data(number_of_samples))


def headset_factory(name: str, file_loc: str) -> Headset:
    if name == Nautilus.NAME:
        return Nautilus()
    elif name == Unicorn.NAME:
        return Unicorn(file_loc)
    elif name == MockUnicorn.NAME:
        return MockUnicorn(file_loc)
    else:
        raise TypeError(f"The Headset: {name} is not an headset type. Choose from: [{Nautilus.NAME}, {Unicorn.NAME}, "
                        f"{MockUnicorn.NAME}].")


class Nautilus(Headset):
    NAME = 'NAUTILUS'

    def __init__(self):
        self.amountOfChannels = 16
        self.sfreq = 500

    def get_data(self, number_of_samples):
        pass


class Unicorn(Headset):
    NAME = 'UNICORN'
    BIN_SINGLE_LINE_BYTE_SIZE = 68

    def __init__(self, data_file_loc: str):
        self.amountOfChannels = 8
        self.sfreq = 250
        self.data_file_loc = data_file_loc  # /home/george/repos/eeg/realtime/Unicorn Linux C API/x64/data.bin'

    def get_data(self, number_of_samples: int):
        sample_list = []
        line_byte_size = self.BIN_SINGLE_LINE_BYTE_SIZE * number_of_samples
        bin_proces = subprocess.Popen(['tail', '-c', str(line_byte_size), '-f', self.data_file_loc]
                                      , stdout=subprocess.PIPE, close_fds=True)
        convert_proces = subprocess.Popen(['unicorn_converter', '-'], stdin=bin_proces.stdout,
                                          stdout=subprocess.PIPE, close_fds=True, universal_newlines=True)
        for i, stdout_line in enumerate(iter(convert_proces.stdout.readline, "")):
            if len(sample_list) < number_of_samples:
                sample_list.append(convert_to_list_of_floats(stdout_line))
            else:
                bin_proces.terminate()
                convert_proces.terminate()
                break
        return np.array(sample_list)


class MockUnicorn(Unicorn):
    NAME = 'MOCK_Unicorn'

    def __init__(self, data_file_loc: str):
        super().__init__(data_file_loc)
        self.start_line = 0

    def get_data(self, number_of_samples: int):
        sample_list = []
        # TODO: Built in a check for that it doesn't take binary files.
        convert_proces = subprocess.Popen(['cat', self.data_file_loc],
                                          stdout=subprocess.PIPE, close_fds=True, universal_newlines=True)
        for i, stdout_line in enumerate(iter(convert_proces.stdout.readline, "")):
            if i >= self.start_line:
                if len(sample_list) < number_of_samples:
                    sample_list.append(convert_to_list_of_floats(stdout_line))
                else:
                    convert_proces.terminate()
                    break

        self.start_line += number_of_samples
        sleep(3)
        return np.array(sample_list)
