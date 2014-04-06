#!/usr/bin/env python

import rospy
import thread
import time
import nxtBrick
import nxt.locator
from nxt_beagle.msg import nxtPower, SamplingInterval, UltraSensor
from nxt_beagle.srv import nxtTicks, nxtTicksResponse, nxtUltrasonic, nxtUltrasonicResponse, nxtAddUltrasonic, nxtAddUltrasonicResponse

brick = nxtBrick.BrickController()
_pow_topic = "cmd_pow"
_ticks_srv = "get_ticks"
_ulso_srv = "get_ultrasonic"
_addulso_srv = "add_ultrasonic"
_measure_ultra_topic = "measure_ultrasensor"
_set_sampling_topic = "set_sampling_interval"
_sampling_interval = 0.0
_threads = []
_sampling_lock = thread.allocate_lock()

def processSamplingIntervalMsg(sampling_msg):
    with _sampling_lock:
        _sampling_interval = sampling_msg.sec

def getSleepTime(delta):
    result = 0.0
    with _sampling_lock:
        result = _sampling_interval - delta
    if(result < 0.0):
        result = 0.0
    return result

def measureUltraSonicThread(*sensor_id):
    #create publisher for each ultrasonic sensor
    sensor_publisher = rospy.Publisher(_measure_ultra_topic + str(sensor_id[0]), UltraSensor)
    msg = UltraSensor()
    rospy.loginfo("Added Ultrasonic Sensor with ID: %d", sensor_id[0])
    
    #as long as ros runs thread has to run
    while not rospy.is_shutdown():
        with _sampling_lock:
            sample = _sampling_interval > 0
        sleep_time = 0.2
        if sample:
            begin = rospy.get_rostime()
            msg.distance = brick.getUltrasonicData(sensor_id[0])
            sensor_publisher.publish(msg)
            end = rospy.get_rostime()
            diff = end - begin
            sleep_time = getSleepTime(diff.secs)
        
        time.sleep(sleep_time)

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
    
    #create ultrasonic publisher thread
    _threads.append(thread.start_new_thread(measureUltraSonicThread,(response.sensorID,)))
    return response

def initNodeCommunication():
    rospy.loginfo("Subscribing to topic \"%s\"...", _pow_topic)
    rospy.Subscriber(_pow_topic, nxtPower, processMotorPowerMsg)
    
    rospy.loginfo("Subscribing to topic \"%s\"...", _set_sampling_topic)
    rospy.Subscriber(_set_sampling_topic, SamplingInterval, processSamplingIntervalMsg)
    
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
    for t in _threads:
        t.join()

if __name__ == '__main__':
    startControl()