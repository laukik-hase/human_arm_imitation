import time
import pypot.dynamixel

offset = [0, 0, 0, 0]
ID_LIST = [1, 2, 3, 4]
ID_SIZE = len(ID_LIST)

DXL_DICT_1      = dict(zip(ID_LIST, [1]*ID_SIZE))
DXL_DICT_0      = dict(zip(ID_LIST, [0]*ID_SIZE))
DXL_DICT_PID    = dict(zip(ID_LIST, [[1,0,0]]*ID_SIZE))

if ( len(sys.argv) == 2 ):
    file_name = sys.argv[1]
else:
    print("Usage: python csv_angle_publisher.py <csv_file_name>")
    exit()

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])

with open(file_name, mode='r') as openfileobject:
    for line in openfileobject:
        str_state = line.split(',')
        state = [float(i) for i in str_state[1:]]
        print(state)
        
        time.sleep(0.05)