import can
import time
import os

bus = None
def checkAndActivateCANInterface():
    global bus
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate='1000000')
    except:
        #failed first attempt, bringup the interface manually and try again
        os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000 restart-ms 100")
        time.sleep(0.1)
        try:
            bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate='1000000')
        except:
            print('Failed to open CAN interface')
            return False
    message = bus.recv() # wait till messsage arrives on bus
    if(message.arbitration_id == 321 or message.arbitration_id == 322):
        print('CAN - Message received!')
    bus.shutdown()
    return True

if __name__ == '__main__':
    print('SUCCESS') if (checkAndActivateCANInterface()) else print('FAIL')
        