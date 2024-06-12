###############################################################################
# Imports
###############################################################################

import time
from threading import Lock, Thread
from copy import deepcopy

from .detected_points import Detected_Points
from .ti import TI

###############################################################################
# Class
###############################################################################

class iwr6843_interface(object):
    def __init__(
        self,
        cli_port: str,
        data_port: str,
        ti: TI
    ):
        self.ti = ti
        
        self.detected_points = Detected_Points()
        self.stream = self.detected_points.data_stream_iterator(
            cli_port,
            data_port,
            self.ti
        )

        self.xyz_lock = Lock()
        self._xyz_data = []

        self.get_data_thread = None
        
        self._stop = False
        
    def start(self):
        self._stop = False
        self.get_data_thread = Thread(target=self.get_data)
        self.get_data_thread.start()
        
    def stop(self):
        self._stop = True
        
        if self.get_data_thread is not None:
            self.get_data_thread.join()
            self.get_data_thread = None

    @property
    def xyz_data(self):
        with self.xyz_lock:
            return deepcopy(self._xyz_data)

    def update(self):
        data = next(self.stream)

        with self.xyz_lock:
            self._xyz_data = data

    def get_data(self):
        while not self._stop:
            try:
                self.update()
                time.sleep(self.ti.ms_per_frame/1000)   
            except Exception as exception:
                print(exception)
                return
