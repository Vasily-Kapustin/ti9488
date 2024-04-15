import numpy as np
import os
import time

buf = np.memmap('/dev/fb0', dtype='uint16',mode='w+', shape=(480,320))
buf[:] = 0xffff
time.sleep(1)
buf[:] = 0Xffc0
