import socket
import time
import threading
import artificial_potential_field
import olympe
import os
import math
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

class ParrotDrone(threading.Thread):
    def __init__(self, dist_to_target, run_duration, connected_drone, nose_position):
      self.drone = connected_drone    
      self.dist_to_target = dist_to_target
      self.run_duration = run_duration
      self.nose_position = nose_position
      self.sensor_readings = [2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0]
      self.avg_sensor_readings = [2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0]
      self.sensor_readings_last_received_timestamp = 0.0

      threading.Thread.__init__(self)
    
    def run(self):
      print("Starting " + self.name) 
      #initialize variables
      self.start_time = time.time()
    
      #initialize APF algo
      self.apf = artificial_potential_field.APF()
      
      
      #initialize and connect sensors
      UDP_IP = ""   # listen on all available interfaces
      UDP_PORT = 12345      # choose an available port number
      # 2390 is the sensor's local port
      # 192.168.0.152 is the sensor's address
      # create a UDP socket object
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      # bind the socket to a specific IP address and port number
      self.sock.bind((UDP_IP, UDP_PORT))
        
      sensors_thread = threading.Thread(target=self.TOFSensors)
      movedrone_thread = threading.Thread(target=self.apply_apf_and_move)
      sensors_thread.daemon = True
      sensors_thread.start()
      time.sleep(2.0)
      #takeoff and fly to average person head level
      assert self.drone(TakeOff()).wait().success()
      #self.drone(
      #      moveBy(0.0, 0.0, 0.3, 0.0)
      #      >> FlyingStateChanged(state="hovering", _timeout=5)
      #  ).wait().success()
      movedrone_thread.daemon = True
      movedrone_thread.start()
      time.sleep(2.0)
      
      while True:
        if (time.time() - self.start_time) > self.run_duration:
          assert self.drone(Landing()).wait().success()
          print("socket is now closing")
          self.sock.close()
          return
        

    def TOFSensors(self):
        try:
          while True:
            if (time.time() - self.start_time) > self.run_duration:
              return
            else:
              # receive data and the address of the sender
              #print("!DEBUG attempting to recieve message")
              # sock.setblocking(False)
              data, addr = self.sock.recvfrom(1024)  # buffer size is 1024 bytes
          
              # print received data and sender's address
              print(f"Received message: {data.decode()} from {addr}")
              
              raw_sensor_readings = data.decode('utf-8')
              self.sensor_readings = list(map(int, raw_sensor_readings.split(";")))
              self.sensor_readings[1] = 1
              self.sensor_readings_last_received_timestamp = time.time()
        except KeyboardInterrupt:
          print("Thread cancelled by keyboard interrupt.")
          return
        except Exception:
          return
          
        
    def apply_apf_and_move(self):
      while True:
       if (time.time() - self.start_time) > self.run_duration:
           return
       else:
         if(len(self.sensor_readings) > 0 and (time.time() - self.sensor_readings_last_received_timestamp) < 2): #COMMENTED OUT FOR PERSON TRACK TESTING
          #self.avg_sensor_readings = self.get_avg_sensor_readings()
          #fx_net, fy_net = apf.obs_avoid_APF(self.avg_sensor_readings, 100)
          print("the sensor readings are:" , self.sensor_readings)
          
          fx_net, fy_net = self.apf.obs_avoid_APF(self.sensor_readings, self.dist_to_target)
          print("the dist_to_target is:" , self.dist_to_target)
          self.move_drone(fx_net, fy_net) 
          #print(fx_net)
          #print(fy_net)
          #print(" at " + self.sensor_readings_last_received_timestamp)
          
    def get_avg_sensor_readings(self):
        avgs = [0,0,0,0,0,0,0]
        for i in range(7):
          avg = (self.sensor_readings[i] + self.avg_sensor_readings[i]) / 2
          avgs[i] = avg
        return avgs
    
    
    def move_drone(self, fx, fy):
          
        if(fx > 0.1):
          fx = 0.5
        elif(fx < -0.1):
          fx = -0.5
        else:
          fx = 0.0
         
        if(fy > 0.1):
          fy = 0.5
        elif(fy < -0.1):
          fy = -0.5
        else:
          fy = 0.0
        
        turning_factor = 0
        height_factor = 0
        # the amount you turn by is psi in the moveby command. it is in radians
        # the frame from the parrot drone is 720x1080
        # the nose_position[0] is the x value of the nose position, and [1] is the y value
        if self.nose_position[0] < 360:
            turning_factor = -math.pi/12
            print("!WARNING turning right with magnitude: ", turning_factor)
        elif self.nose_position[0] > 720:
            turning_factor = math.pi/12
            print("!WARNING turning left with magnitude: ", turning_factor)
        if self.nose_position[1] > 570:
            height_factor = 0.3
            print("!WARNING flying down with magnitude: ", height_factor)
        elif self.nose_position[1] < 150:
            height_factor = -0.3
            print("!WARNING flying up with magnitude: ", height_factor)
        
        if (fx<0):
          print("!WARNING flying backward with magnitude: ", fx)
        else:
          print("!WARNING flying forward with magnitude: ", fx)
        if (fy>0):
          print("!WARNING flying left with magnitude: ", fy)
        else:
          print("!WARNING flying right with magnitude: ", fy)
        
        try:
          self.drone(
              moveBy(fx, -fy, height_factor, turning_factor)
              >> FlyingStateChanged(state="hovering", _timeout=5)
          ).wait().success()
        except Exception:
          pass
        
      
        
      



