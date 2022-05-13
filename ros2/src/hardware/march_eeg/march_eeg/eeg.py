import os
import threading



import numpy as np
from ament_index_python import get_package_share_directory

from march_eeg import headset
from joblib import load

from march_eeg.eeg_utility import nmf, feature_extraction, ncfs, lda

SAMPLE_SIZE = 729


class Eeg:

    def __init__(self,
                 headset_name: str,
                 data_file_name: str,
                 logger,
                 weights_file_name: str = 'manual_scaled_weights.csv',
                 cref_file_name: str = 'Cref.csv',
                 features_file_name: str = 'feature_select_coefs.csv',
                 ):
        file_loc = os.path.join(
            get_package_share_directory("march_eeg"),
            "resource",
            data_file_name
        )
        self.headset = headset.headset_factory(headset_name, file_loc)
        self.manual_scaled_weights = np.loadtxt(get_config_file_loc(weights_file_name), delimiter=',')
        self.cref = np.loadtxt(get_config_file_loc(cref_file_name), delimiter=',')
        self.feature_select_coefs = np.loadtxt(get_config_file_loc(features_file_name), delimiter=',')
        self.lda_model = load(get_config_file_loc('LDAmodel.pkl'))
        self.walking_thought = False
        self.event_toggle = threading.Event()
        thread = threading.Thread(name='eeg_data_update_thread',
                                  target=lambda: self._loop(),
                                  )
        thread.daemon = True
        thread.start()
        self.count = 0
        self._logger = logger

    def start(self):
        self.event_toggle.set()

    def stop(self):
        self.event_toggle.clear()
        self.walking_thought = False

    def _loop(self) -> None:
        while self.event_toggle.wait():
            try:
            # self.count += 1
            # if self.count > 10000000:
            #     print(f"count reached switching: {self.count} -> {self.walking_thought}")
            #     self.count = 0
            #     self.walking_thought = not self.walking_thought
                data = self.headset.get_data(SAMPLE_SIZE)
                data = self.headset.preprocessing(data)
                nmf_data = nmf(data, self.manual_scaled_weights)
                feature_extraction_data = feature_extraction(nmf_data, self.cref)
                ncfs_data = ncfs(feature_extraction_data, self.feature_select_coefs)
                self.walking_thought = bool(lda(ncfs_data, self.lda_model))
                logger_msg = f"Walking thought: {self.walking_thought}"
                if type(self.headset) == headset.MockUnicorn:
                    logger_msg += f" | lines ({self.headset.start_line - SAMPLE_SIZE} - {self.headset.start_line})"
                self._logger.info(logger_msg)
            except (ValueError, IndexError):
                self._logger.info("Value error")



def get_config_file_loc(file: str):
    return os.path.join(
        get_package_share_directory("march_eeg"),
        "config",
        file)
