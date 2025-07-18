import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pipescorder/ackermann_pipe/install/ackermann_pipe'
