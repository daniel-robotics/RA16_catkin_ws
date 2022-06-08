from template_rospy_pkg.base_node import BaseNode
import rospy    # Documentation: http://docs.ros.org/en/melodic/api/rospy/html/
import std_msgs.msg

import board
import adafruit_ina219

class CurrentSensorINA219Node(BaseNode):
    def __init__(self, name, param_namespace, package_path, clear_params_on_exit):
       
        super().__init__(name,
                         param_namespace,
                         package_path,
                         clear_params_on_exit)
        
        self.sensor = self.init_hardware()

        self.voltage = 0    # Voltage (V) at VIN-
        self.current = 0    # Current (A) across VIN- to VIN+
        self.power = 0      # Power (W)

        self.highest_voltage = 0
        self.highest_current = 0
        self.highest_power = 0
        
        self.logtimer = rospy.Timer(rospy.Duration(1), self.log_async)

        self.start_loop()

    def init_hardware(self):
        i2c = board.I2C()
        sensor = adafruit_ina219.INA219(i2c)
        return sensor

    #********************************************************************************************** 
    #   PUBLISHERS
    #   Publish a message to the topic defined in the 'topic' parameter
    #   Documentation: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    #********************************************************************************************** 
    def init_publishers(self):
        self.loginfo("Starting publisher on %s..." % (self.params['current_topic']))
        self.current_msg = std_msgs.msg.Float32()                           # The message to publish
        self.current_pub = rospy.Publisher(name=self.params['current_topic'], # name: topic to publish messages to
                                         data_class=type(self.current_msg), # data_class: type of message to publish
                                         queue_size=1,                      # queue_size: messages to queue before dropping old messages (decrease for lowest latency, increase if too many messages are dropped)
                                         latch=False,                       # latch: if True, the most recent message is always available to new subscribers, even if they subscribed after it was published
                                         subscriber_listener=None)          # subscriber_listener: An instance of a rospy.SubscribeListener class to receive callbacks when nodes subscribe to this topic
        self.loginfo("Starting publisher on %s..." % (self.params['voltage_topic']))
        self.voltage_msg = std_msgs.msg.Float32()                           # The message to publish
        self.voltage_pub = rospy.Publisher(name=self.params['voltage_topic'], # name: topic to publish messages to
                                         data_class=type(self.voltage_msg), # data_class: type of message to publish
                                         queue_size=1,                      # queue_size: messages to queue before dropping old messages (decrease for lowest latency, increase if too many messages are dropped)
                                         latch=False,                       # latch: if True, the most recent message is always available to new subscribers, even if they subscribed after it was published
                                         subscriber_listener=None)          # subscriber_listener: An instance of a rospy.SubscribeListener class to receive callbacks when nodes subscribe to this topic
        self.loginfo("Starting publisher on %s..." % (self.params['power_topic']))
        self.power_msg = std_msgs.msg.Float32()                             # The message to publish
        self.power_pub = rospy.Publisher(name=self.params['power_topic'],   # name: topic to publish messages to
                                         data_class=type(self.power_msg),   # data_class: type of message to publish
                                         queue_size=1,                      # queue_size: messages to queue before dropping old messages (decrease for lowest latency, increase if too many messages are dropped)
                                         latch=False,                       # latch: if True, the most recent message is always available to new subscribers, even if they subscribed after it was published
                                         subscriber_listener=None)          # subscriber_listener: An instance of a rospy.SubscribeListener class to receive callbacks when nodes subscribe to this topic
        

    #********************************************************************************************** 
    #   SUBSCRIBERS
    #   Function is called when a message is received on the topic defined by the 'topic' parameter
    #   Documentation: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    #********************************************************************************************** 
    def init_subscribers(self):
        pass


    #********************************************************************************************** 
    #   MAIN LOOP 
    #   Function is called at the frequency defined in the 'rate' parameter 
    #********************************************************************************************** 
    def loop(self):
        voltage, current, power = self.sample_and_publish()

        if voltage > self.highest_voltage:
            self.highest_voltage = voltage
        if current > self.highest_current:
            self.highest_current = current
        if power > self.highest_power:
            self.highest_power = power

    
    def sample_and_publish(self):
        # Sample
        self.voltage, self.current, self.power = self.return_sample() 
        # Package
        self.voltage_msg.data=self.voltage
        self.current_msg.data=self.current
        self.power_msg.data=self.power
        # Publish
        self.voltage_pub.publish(self.voltage_msg)
        self.current_pub.publish(self.current_msg)
        self.power_pub.publish(self.power_msg)
        return self.voltage, self.current, self.power

    def return_sample(self):
        voltage = self.sensor.bus_voltage
        current = self.sensor.current/1000.0 
        power   = self.sensor.power

        if voltage is None:
            self.logerr("IO error: IMU returned voltage=%s" % format(voltage))
            voltage = self.voltage
        if current is None:
            self.logerr("IO error: IMU returned current=%s" % format(current))
            current = self.current
        if power is None:
            self.logerr("IO error: IMU returned power=%s" % format(power))
            power = self.power

        return voltage, current, power


    #********************************************************************************************** 
    #   EVENT HANDLERS
    #     on_reload(): Runs when self.params has been changed
    #     on_close():  Runs when ROS is shutting down or if CTRL-C is pressed in the terminal
    #********************************************************************************************** 
    def on_reload(self):
        pass
    
    def on_close(self):
        pass

    def has_nonetype(self, tup):
        return any(map(lambda e: e is None, tup))
        
    def log_async(self, timer_event):
        self.loginfo("Voltage(V): %.2f\tCurrent|Peak(A): %.3f|%.3f\tPower|Peak(W): %.2f|%.2f)" % (self.voltage, self.current, self.highest_current, self.power, self.highest_power))
    