import time
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

def get_header(frame_id='map'):
    seconds = int(time.time())
    nanoseconds = int((time.time() - int(time.time())) * 1000000000.0)

    header = Header()
    header.frame_id = frame_id
    header.stamp = Time(sec=seconds, nanosec=nanoseconds)

    return header