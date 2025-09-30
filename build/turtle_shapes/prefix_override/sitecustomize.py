import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/youssef/ros2_shapes_project/install/turtle_shapes'
