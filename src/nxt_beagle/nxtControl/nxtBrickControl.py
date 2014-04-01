#!/usr/bin/env python

import rospy
import nxtBrick
import nxt.locator
from nxt_beagle.msg import nxtPower
from nxt_beagle.srv import nxtTicks, nxtTicksResponse, nxtUltrasonic, nxtUltrasonicResponse, nxtAddUltrasonic, nxtAddUltrasonicResponse

brick = nxtBrick.BrickController()
_pow_topic = "cmd_pow"
_ticks_srv = "get_ticks"
_ulso_srv = "get_ultrasonic"
_addulso_srv = "add_ultrasonic"

def processMotorPowerMsg(power_msg):
    rospy.logdebug("nxtPower Message: left: %s right: %s", power_msg.leftMotor, power_msg.rightMotor)
    brick.setMotors(power_msg.leftMotor, power_msg.rightMotor)
    
def handleGetTicksRqt(ticks_rqt):
    response = nxtTicksResponse()
    response.leftTicks = brick.getMotorTicks(brick.id_left).block_tacho_count
    response.rightTicks = brick.getMotorTicks(brick.id_right).block_tacho_count
    brick.resetMotors()
    return response

def handleGetUltrasonicRqt(ulso_rqt):
    return nxtUltrasonicResponse(brick.getUltrasonicData(ulso_rqt.sensorID))
    
def handleAddUltrasonicRqt(addulso_rqt):
    response = nxtAddUltrasonicResponse()
    response.sensorID = brick.addUltrasonicSensor(addulso_rqt.port)
    return response

def initNodeCommunication():
    rospy.loginfo("Subscribing to topic \"%s\"...", _pow_topic)
    rospy.Subscriber(_pow_topic, nxtPower, processMotorPowerMsg)
    
    rospy.loginfo("Offering service \"%s\"...", _ticks_srv)
    rospy.Service(_ticks_srv, nxtTicks, handleGetTicksRqt)
    
    rospy.loginfo("Offering service \"%s\"...", _ulso_srv)
    rospy.Service(_ulso_srv, nxtUltrasonic, handleGetUltrasonicRqt)
    
    rospy.loginfo("Offering service \"%s\"...", _addulso_srv)
    rospy.Service(_addulso_srv, nxtAddUltrasonic, handleAddUltrasonicRqt)   
    
def startControl():
    rospy.init_node('nxtBrickControl', anonymous=True)
    
    rospy.loginfo("Connecting to Brick...",)
    try:
        brick.connectToBrick()
    except nxt.locator.BrickNotFoundError as e:
        rospy.logerr("No Brick found. Check USB or Bluetooth. %s", e.strerror)
        return
    
    initNodeCommunication()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    startControl()