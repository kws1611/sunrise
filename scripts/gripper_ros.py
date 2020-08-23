#!/usr/bin/python
import rospy
import sys, time
import re, time
import pexpect
from std_msgs.msg import Bool

def scanble(hci="hci0", timeout=1):
    conn = pexpect.spawn("sudo hciconfig %s reset" % hci)
    time.sleep(0.2)

    conn = pexpect.spawn("sudo timeout %d hcitool lescan" % timeout)
    time.sleep(0.2)

    conn.expect("LE Scan \.+", timeout=timeout)
    output = b""
    adr_pat = "(?P<addr>([0-9A-F]{2}:){5}[0-9A-F]{2}) (?P<name>.*)"
    while True:
        try:
            res = conn.expect(adr_pat)
            output += conn.after
        except pexpect.EOF:
            break

    lines = re.split('\r?\n', output.strip().decode("utf-8"))
    lines = list(set(lines))
    lines = [line for line in lines if re.match(adr_pat, line)]
    lines = [re.match(adr_pat, line).groupdict() for line in lines]
    lines = [line for line in lines if re.match('.*', line['name'])]
    print(lines)

    return lines

class BLEDevice:
    def __init__(self, addr=None):
        self.services = {}
        self.characteristics = {}
        if addr is not None:
            self.connect(addr)
            self.getcharacteristics()
        self.status_pub = rospy.Publisher('/gripper_status', Bool, queue_size=10)
        self.status = False
        self.mission_switch = False
        rospy.Subscriber('/gripper_switch', Bool,self.gripper_switchCb)

    def gripper_switchCb(self, msg):
        self.mission_switch = msg

    def publishing(self):
        status = Bool()
        status.data = self.status
        self.status_pub.publish(status)

    def connect(self, addr):
        print("connecting...")
        # Run gatttool interactively.
        self.gatt = pexpect.spawn("gatttool -b " + addr + " -I")
        self.gatt.expect('\[LE\]>', timeout=10)
        self.gatt.sendline('connect')
        self.gatt.expect('Connection successful.*\[LE\]>', timeout=5)
        print("Successfully connected!")

    def getservices(self):
        pass

    def getcharacteristics(self):
        self.gatt.sendline('characteristics')
        time.sleep(0.2)
        ch_pat='handle: (\S+), char properties: (\S+), char value handle: (\S+), uuid: (\S+)'
        #self.gatt.expect('\[LE\]>')
        while True:
            try:
                self.gatt.expect(ch_pat, timeout=2)
                ch_tuple = self.gatt.match.groups()
                uuid = ch_tuple[3][4:8]
                self.characteristics[uuid]=ch_tuple
                #print(ch_tuple)
            except pexpect.TIMEOUT:
                break

    def gethandle(self, uuid):
        ch = self.characteristics[uuid]
        return int(ch[0],16)

    def getvaluehandle(self, uuid):
        ch = self.characteristics[uuid]
        return int(ch[2],16)

    def writecmd(self, handle, value):
        cmd = "char-write-cmd 0x%04x %s" % (handle, value)
        #cmd = "char-write-cmd 0x%02x %s" % (handle, value.encode('hex'))
        self.gatt.sendline(cmd)

    def writereq(self, handle, value):
        req = "char-write-req 0x%04x %s" % (handle, value)
        #cmd = "char-write-cmd 0x%02x %s" % (handle, value.encode('hex'))
        self.gatt.sendline(req)

    def readreq(self, value):
        req = "char-read-uuid %s" % (value)
        self.gatt.sendline(req)
        while True:
            try:            
                num = self.gatt.expect('handle: .*? \r', timeout=1)
            except pexpect.TIMEOUT:
                break
            if num == 0:
                hxstr = self.gatt.after.split()[3:]
                #print("Received: ", hxstr[2:])
                return hxstr[2]
        return None


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('gripper', anonymous=True, disable_signals=True)
    gripper = BLEDevice('24:62:AB:B2:26:5A') ##connect
    state = 0
    cnt = 0
    cnt2 = 0
    openstate = False
    try:


        while not rospy.is_shutdown():
            vh=gripper.getvaluehandle(b'0001')
            if not gripper.status:
                if gripper.mission_switch:
                    gripper.writereq(vh, "01\r\n")
                    print("Send open ready signal to arduino")
                    while True:
                        receive = gripper.readreq('2a19')
                        if receive == b'80':
                            openstate = True
                            print("Open state :",openstate)
                            gripper.status = True    ######### gripper finished
                            break
                        print("Open state :",openstate)
                        #time.sleep(1)
                        cnt2+=1
                        if cnt2==60:
                            break
                            gripper.status = True   ######### gripper finished
                        gripper.publishing()
                else:
                    gripper.writereq(vh, "00\r\n")
                    gripper.publishing()
            gripper.publishing()
            #data = gripper.notify()
            #if data is not None:
            #    print("Received: ", data)
            time.sleep(0.1)
            cnt+=1

    except rospy.ROSInterruptException:
        pass
