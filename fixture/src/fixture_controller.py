#! /usr/bin/python

from threading import Lock
import rospy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIORequest, SetIO, SetIOResponse

class FixtureController:
    def __init__(self):
        rospy.init_node("fixture_controller")
        self.joint_state_pub = rospy.Publisher(name="/joint_states", data_class=JointState, queue_size=1,latch=True)
        if rospy.get_param(param_name="/execution_type", default="") == "real":
            rospy.wait_for_service(service="/ur_hardware_interface/set_io", timeout=rospy.Duration(10.0))
            self.set_IO_service = rospy.ServiceProxy(name="/ur_hardware_interface/set_io", service_class=SetIO, persistent=True)
        else:
            self.set_IO_service = None
        self.fixture_moving_mutex = Lock()
        self.fixture_status = "INIT"
        self.set_fixture_cb(SetBoolRequest(data=False))
        self.set_fixture_service = rospy.Service(name="/close_fixture", service_class=SetBool, handler=self.set_fixture_cb)
    
    def close_fixture(self, invert_to_open, pub_frequency_hz=10, duration_s=3):
        rate = rospy.Rate(hz=pub_frequency_hz)
        if self.set_IO_service is not None:
            request_pin_0 = SetIORequest(fun=1, pin=0)
            request_pin_1 = SetIORequest(fun=1, pin=1)
            request_order = []
            if invert_to_open:
                request_pin_0.state = 0
                request_pin_1.state = 1
                request_order = [request_pin_0, request_pin_1]    
            else:
                request_pin_0.state = 1
                request_pin_1.state = 0
                request_order = [request_pin_1, request_pin_0]
            for request in request_order:
                try:
                    if not self.set_IO_service.call(request).success:
                        return False
                except:
                    return False
                rospy.sleep(0.5)
        joint_state_msg = JointState()
        joint_state_msg.name = ["fixture_static_dynamic"]
        for i in range(duration_s * pub_frequency_hz + 1):
            i = float(i)
            if invert_to_open:
                joint_state_msg.position = [0.186 * (1 - i / (duration_s * pub_frequency_hz))]
            else:
                joint_state_msg.position = [0.186 * (i / (duration_s * pub_frequency_hz))]
            joint_state_msg.header.stamp = rospy.Time.now()
            self.joint_state_pub.publish(joint_state_msg)
            rate.sleep()
        return True


    def set_fixture_cb(self, request):
        if self.fixture_moving_mutex.locked():
            return SetBoolResponse(success=False, message="Another fixture process is running")
        with self.fixture_moving_mutex:
            if self.fixture_status == "INIT" or (self.fixture_status == "CLOSED" and not request.data) or (self.fixture_status == "OPEN" and request.data):
                if self.close_fixture(not request.data):
                    if request.data:
                        self.fixture_status = "CLOSED"
                    else:
                        self.fixture_status = "OPEN"
                    return SetBoolResponse(success=True, message="Changed position to {}".format(self.fixture_status))
                else:
                    self.set_fixture_service.shutdown()
                    return SetBoolResponse(success=False, message="Error occured during position setting. This service will be disabled...")
            else:
                return SetBoolResponse(success=True, message="Already in requested position")    

    def clean_up(self):
        self.set_IO_service.close()

if __name__ == "__main__":
    fc = FixtureController()
    rospy.spin()
    fc.clean_up()