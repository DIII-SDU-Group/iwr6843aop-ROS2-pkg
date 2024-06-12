###############################################################################
# Imports
###############################################################################

import time

from .constants import MAGIC_WORD
from .ti import TI

###############################################################################
# Class
###############################################################################

class Detected_Points:
    def data_stream_iterator(
        self,
        cli_loc: str,
        data_loc: str,
        ti: TI
    ):
        interval = ti.ms_per_frame/1000
        
        data = b''
        warn = 0
        
        while 1:
            time.sleep(interval)
            byte_buffer = ti._read_buffer()
            
            if(len(byte_buffer)==0):
                warn+=1
                
            else:
                warn=0
                
            if(warn >= 100): # after 10 empty frames
                print("10 empty frames, closing radar connection")
                break
        
            data+=byte_buffer
        
            try:
                idx1 = data.index(MAGIC_WORD)   
                idx2 = data.index(MAGIC_WORD,idx1+1)

            except:
                continue

            datatmp = data[idx1:idx2]
            data = data[idx2:]
            points = ti._process_detected_points(byte_buffer)
            ret = points[:,:3]

            yield ret

        print("Closing radar connection")
        
        ti.close()
