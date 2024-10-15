def TimeMsg(msg):
    return msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
