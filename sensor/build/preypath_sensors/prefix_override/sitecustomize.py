import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jonathan/git/PreyPath_RS1/sensor/install/preypath_sensors'
