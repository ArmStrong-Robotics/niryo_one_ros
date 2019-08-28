#!/usr/bin/env python

import rospy

from flask import Flask, render_template, jsonify, request
from niryo_one_python_api.niryo_one_api import *
#import time
#import os
import threading
from sensor_msgs.msg import Joy

# ROS node class definition
class OrthopusWebApp(threading.Thread): 
    def __init__(self): 
      threading.Thread.__init__(self)
      # The node is started in a separate thread to avoid conflicts with Flask.
      # The parameter *disable_signals* must be set if node is not initialized
      # in the main thread.
      rospy.init_node('orthopus_webapp', disable_signals=True)

      self.niryo_api = NiryoOne()

      self._lock = threading.Lock()
      self.pub = rospy.Publisher("sim_joy", Joy, queue_size=1)  
      
      self.rate = rospy.Rate(10) # 10hz
      
      self.speed_factor = 0.0;
      self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      self.buttons = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      self.enabled = False;

      self.X_AXIS = 4
      self.Y_AXIS = 0
      self.Z_AXIS = 1
      self.X_ROT_AXIS = 7
      self.Y_ROT_AXIS = 6
      self.Z_ROT_AXIS = 3
      
    def run(self):
      while not rospy.is_shutdown():
        with self._lock:
          msg = Joy()
          msg.axes = self.axes
          msg.buttons = self.buttons
          self.pub.publish(msg)
        self.rate.sleep()
                        
    def setX(self, value):
      with self._lock:
        self.axes[self.X_AXIS] = value * self.speed_factor;
                   
    def setY(self, value):
      with self._lock:
        self.axes[self.Y_AXIS] = value * self.speed_factor;   
                        
    def setZ(self, value):
      with self._lock:
        self.axes[self.Z_AXIS] = value * self.speed_factor;
                                
    def setRotX(self, value):
      with self._lock:
        self.axes[self.X_ROT_AXIS] = value * self.speed_factor;
            
    def setRotY(self, value):
      with self._lock:
        self.axes[self.Y_ROT_AXIS] = value * self.speed_factor;
           
    def setRotZ(self, value):
      with self._lock:
        self.axes[self.Z_ROT_AXIS] = value * self.speed_factor;
        
    def updateSpeedFactor(self, speed_factor):
      with self._lock:
        self.speed_factor = speed_factor;
                
    def setEnabled(self, enabled):
      with self._lock:
        response = self.niryo_api.call_service("/niryo_one/joystick_interface/enable", SetInt, [int(enabled)])
        if(response.status == 200):
            self.enabled = enabled
           
    def getEnabled(self):
      with self._lock:
        return self.enabled
      
    def closeGripper(self):
      self.niryo_api.close_gripper(12, 300)
            
    def openGripper(self):
      self.niryo_api.open_gripper(12, 300)

          
app = Flask(__name__)
orthopus_webapp = OrthopusWebApp()
orthopus_webapp.start()

@app.route("/")
def index():
    return render_template("index.html")
    
@app.route("/gripper_open")
def gripper_open():
    message="Info: Open gripper"
    print message
    orthopus_webapp.openGripper()
    data = {"message":message}
    return jsonify(data)
  
@app.route("/gripper_close")
def gripper_close():
    message="Info: Close gripper"
    print message
    orthopus_webapp.closeGripper()
    data = {"message":message}
    return jsonify(data)
  
@app.route("/cartesian_enable")
def cartesian_enable():
    state = True if request.args.get('state') == 'true' else False
    message="Info: Activate cartesian control : " + str(state)
    print message
    orthopus_webapp.setEnabled(state);
    new_state = orthopus_webapp.getEnabled();
    if new_state != state:
      message = message + " -> ERROR"
    data = {"message":message, "new_state":new_state}
    return jsonify(data)
  
@app.route("/speed_factor")
def speed_factor():
    speed_factor = request.args.get('speed_factor', type = float)
    message="Info: Speed factor update to " + str(speed_factor)
    print message
    orthopus_webapp.updateSpeedFactor(speed_factor)
    data = {"message":message, "speed_factor":speed_factor}
    return jsonify(data)
    
@app.route("/set_value")
def set_value():
    name = request.args.get('name', type = str)
    value = float(request.args.get('value', type = str))
    message="Info: Set "+name+" to "+str(value)
    print message
    if(name=="linear_x"):
      orthopus_webapp.setX(value)
    elif(name=="linear_y"):
      orthopus_webapp.setY(value)
    elif(name=="linear_z"):
      orthopus_webapp.setZ(value)
    elif(name=="angular_x"):
      orthopus_webapp.setRotX(value)
    elif(name=="angular_y"):
      orthopus_webapp.setRotY(value)
    elif(name=="angular_z"):
      orthopus_webapp.setRotZ(value)
    data = {"message":message}
    return jsonify(data) 
    
@app.route("/calibrate.json")
def calibrate():
    message="Info: Calibration"
    print message
    try:
        orthopus_webapp.niryo_api.calibrate_auto()
        data = {"message":message}
        print "[ OK ] finished !"
        return jsonify(data)

    except NiryoOneException as e:
        print "[FAIL]"
        print e
        data = {"message":message, "error":str(e)}
        return jsonify(data)

@app.route("/activate_learning_mode.json")
def activate_learning_mode():
    message="Info: Activate learning mode"
    print message
    try:
        orthopus_webapp.niryo_api.activate_learning_mode(True)
        data = {"message":message}
        print "[ OK ] finished !"
        return jsonify(data)

    except NiryoOneException as e:
        print "[FAIL]"
        print e
        data = {"message":message, "error":str(e)}
        return jsonify(data)

@app.route("/deactivate_learning_mode.json")
def deactivate_learning_mode():
    message="Info: Deactivate learning mode"
    print message
    try:
        orthopus_webapp.niryo_api.activate_learning_mode(False)
        data = {"message":message}
        print "[ OK ] finished !"
        return jsonify(data)

    except NiryoOneException as e:
        print "[FAIL]"
        print e
        data = {"message":message, "error":str(e)}
        return jsonify(data)

@app.route("/goto_zero.json")
def goto_zero():
    message="Info: Goto zero position"
    print message
    try:
        orthopus_webapp.niryo_api.activate_learning_mode(False)
        orthopus_webapp.niryo_api.move_joints([0,0,0,0,0,0])
        data = {"message":message}
        print "[ OK ] finished !"
        return jsonify(data)

    except NiryoOneException as e:
        print "[FAIL]"
        print e
        data = {"message":message, "error":str(e)}
        return jsonify(data)

@app.route("/goto_rest_position.json")
def goto_rest_position():
    message="Info: Goto rest position"
    print message
    try:
        orthopus_webapp.niryo_api.activate_learning_mode(False)
        orthopus_webapp.niryo_api.move_joints([0,0.64,-1.397,0,0,0])
        orthopus_webapp.niryo_api.activate_learning_mode(True)
        data = {"message":message}
        print "[ OK ] finished !"
        return jsonify(data)

    except NiryoOneException as e:
        print "[FAIL]"
        print e
        data = {"message":message, "error":str(e)}
        return jsonify(data)

@app.route('/move', methods=['GET', 'POST'])
def move():
    joint_1 = request.args.get('j1', type = float)
    joint_2 = request.args.get('j2', type = float)
    joint_3 = request.args.get('j3', type = float)
    joint_4 = request.args.get('j4', type = float)
    joint_5 = request.args.get('j5', type = float)
    joint_6 = request.args.get('j6', type = float)
    message="Info: Move"
    print message
    try:
        orthopus_webapp.niryo_api.move_joints([joint_1,joint_2,joint_3,joint_4,joint_5,joint_6])
        pos = orthopus_webapp.niryo_api.joints
        data = {"message":message, "pos":pos}
        print "[ OK ] finished !"
        return jsonify(data)

    except NiryoOneException as e:
        print "[FAIL]"
        print e
        data = {"message":message, "error":str(e)}
        return jsonify(data)
      
if __name__ == '__main__':
    print "RUN FLASK"
    host_ip = rospy.get_param("~host_ip")
    app.run(
      host=host_ip, 
      port=8080,
      debug=True
      )

