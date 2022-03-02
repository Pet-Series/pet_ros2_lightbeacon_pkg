#!/usr/bin/env python3'
# coding = utf-8
########################################################################################
##
## Maintainer: stefan.kull@gmail.com
##
## Input:  Subscribe to ROS2-topics
## Output: PWM-sequence to control Light Beacon (Servo PWM-connector/wire)
##
##     1) "RotatingFast"
##     2) "RotatingSlow"
##     3) "Flashing"
##     4) "Strobing"
##     5) "LEDoff"
##     6)  GOTO 1
##
## Behaviour: ## 1) Once: Read/Set all the parameters 
## 2) Repeatedly: Subscribe ROS2-topics for "/beacon mode" 
## 3) Repeatedly: ...toggle/cycles truth the control PWM-sequence of the beacon.
##
## Prerequisite: Linux/Ubuntu vs. Hardware
## Hardware/SBC   : Raspberry Pi 4(Ubuntu) with I2C
## Hardware/Light : Servo(3-wire) LightBeacon... Normaly found on model/RC-cars or trucks.
##  +--------+--------+--------+
##  | GND    | Brown  | GND    |
##  +--------+--------+--------+
##  | GPIO23 | Red    | VCC    |
##  +--------+--------+--------+
##  | GPIO24 | Yellow | PWM    |
##  +--------+--------+--------+
##
## Prerequisite: Linux/Ubuntu vs. Software
## $ sudo apt install python3-pip
## $ sudo apt-get install python3-rpi.gpio
## $ sudo pip3 install gpiozero
##
## Launch sequence:
## 1) ros2 run pet_ros2_lightbeacon_pkg pet_lightbeacon_node 
## 2) ...se examples of manually injecting topics to beacon node
##
## Test: Set beacon-mode from a terminal/commandline. Publish only topic onence "-1"
## $ ros2 topic pub /beacon_mode std_msgs/msg/String "data: RotatingFast" -1
## $ ros2 topic pub /beacon_mode std_msgs/msg/String "data: RotatingSlow" -1
## $ ros2 topic pub /beacon_mode std_msgs/msg/String "data: Flashing" -1
## $ ros2 topic pub /beacon_mode std_msgs/msg/String "data: Strobing" -1
## $ ros2 topic pub /beacon_mode std_msgs/msg/String "data: LEDoff" -1
## $ ros2 topic pub /beacon_mode std_msgs/msg/String "data: Reset" -1
##

#  Include the ROS2 stuff...
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

# Importe the Raspberry Pi I/O stuff
from gpiozero import Servo
from gpiozero import LED

#  Include Linux/Ubuntu stuff...
import sys
import signal
from time import *

class LightBeaconNode(Node): 
    def __init__(self):
        super().__init__("light_beacon_node")

        self.beacon_current_mode = "RotatingFast"
        self.beacon_new_mode = "LEDoff"

        # Set default pin for ligh_beacon +3.3V-power. Accessed via ROS Parameters...
        self.declare_parameter( 'gpio_pin_power', 23, ParameterDescriptor(description='GPIO-pin for ligh_beacon +3.3V-power [default <23>]') )
        self.GPIO_PIN_POWER = self.get_parameter('gpio_pin_power').value

        # Set default pin for ligh_beacon input/signal to toggle state. Accessed via ROS Parameters...
        self.declare_parameter( 'gpio_pin_signal', 24, ParameterDescriptor(description='GPIO-pin for input/signal to toggle state [default <24>]') )
        self.GPIO_PIN_SIGNAL = self.get_parameter('gpio_pin_signal').value

        # Set default pin for ligh_beacon input/signal to toggle state. Accessed via ROS Parameters...
        self.declare_parameter( 'beacon_topic', "beacon_mode", ParameterDescriptor(description='GPIO-pin for input/signal to toggle state [default <24>]') )
        self.BEACON_TOPIC = self.get_parameter('beacon_topic').get_parameter_value().string_value

        # .... something..else :-)
        self.get_logger().info("- self.BEACON_TOPIC: '" + self.BEACON_TOPIC + "'" )
        self.new_mode = self.create_subscription(String, self.BEACON_TOPIC, self.light_beacon_mode_callback, 10)   
        self.new_mode # prevent unused variable warning

        exit = False

        # Check we can open/contact the GPIO-pins for the Light Beacon
        try:
            #.... something
            self.led = LED( self.GPIO_PIN_POWER )               
            self.led.on()    # Bring the light ON!
            self.get_logger().info("light_beacon_node initiating")
            self.beacon_signal = Servo( self.GPIO_PIN_SIGNAL ) 
            self.beacon_signal.min()

            # self.beacon_current_mode = self.light_beacon_toogle()

            self.get_logger().info("light_beacon_node has started")
            self.get_logger().info("- beacon_power  GPIO-pin: " + str( self.GPIO_PIN_POWER ) )
            self.get_logger().info("- beacon_signal GPIO-pin: " + str( self.GPIO_PIN_SIGNAL ) )
            self.get_logger().info("- beacon_mode  ROS Topic: " + "beacon_mode" )
        except:
            # Note: a permission or operatings system error... ;-(
            self.get_logger().error("light_beacon_node canceled:"  + str(sys.exc_info()[1]) )
            self.exit = True

    # Callback when new value set for topic 
    def light_beacon_mode_callback(self, msg):
        self.beacon_new_mode = msg.data
        self.get_logger().info("-------light_beacon_mode_callback(self, msg):-------" )
        self.get_logger().info("| light_beacon_node update = " + self.beacon_new_mode )   
        self.get_logger().info("| Current:'" + str(self.beacon_current_mode) + "'" )
        self.get_logger().info("| New:    '" + str(self.beacon_new_mode)     + "'" )        
        
        self.light_beacon_toogle()

        self.beacon_current_mode = self.beacon_new_mode

    def light_beacon_toogle(self):

        if  ( self.beacon_new_mode == "RotatingFast"):
            new = 1
        elif self.beacon_new_mode ==  "RotatingSlow":
            new = 2
        elif self.beacon_new_mode ==  "Flashing":
            new = 3
        elif self.beacon_new_mode ==  "Strobing":
            new = 4
        elif self.beacon_new_mode ==  "LEDoff":
            new = 5
        elif self.beacon_new_mode ==  "Reset":
            new = 5
            self.beacon_current_mode =  "LEDoff"
            self.led.off()
            sleep(0.1)  
            self.led.on()
            sleep(0.2) 

        else:
            new = 0
            self.get_logger().warning("| Input ERROR in Light Beacon node |")

        if   (self.beacon_current_mode == "RotatingFast" ):
            current = 1
        elif (self.beacon_current_mode ==  "RotatingSlow"):
            current = 2
        elif (self.beacon_current_mode ==  "Flashing"    ):
            current = 3
        elif (self.beacon_current_mode ==  "Strobing"    ):
            current = 4
        elif (self.beacon_current_mode ==  "LEDoff"   ):
            current = 5
        else:
            current = 0
            self.get_logger().warning("| Input ERROR in Light Beacon node |")

        print("We have a vote Current=" + str(current) + " New=" + str(new) + " SUM=" + str(new - current))
        if ( new < current ):
            print("New < Current  (+5)")
            self.cycles( new - current +5 )
        else:
            print ("New => Current")
            self.cycles( new - current)            
        
    def cycles(self, i ):
        for x in range(0, i):
            self.cycle_once()
    
    def cycle_once(self):
        self.get_logger().warning("| CYCLES |")
        self.beacon_signal.min()
        sleep(0.1)
        self.beacon_signal.max()
        sleep(0.1)
        self.beacon_signal.min()

def main(args=None):
    rclpy.init(args=args)
    node = LightBeaconNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("light_beacon_node **** 💀 Ctrl-C detected...")
    
    finally:
        print("light_beacon_node **** 🪦 Ending... ")
        print( str(sys.exc_info()[1]) )
        
        # Time to clean up stuff!
        # - Destroy the node explicitly
        #   (optional - otherwise it will be done automatically
        #   when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
