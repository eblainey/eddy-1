#include "eddy_constants.h"

#include <IBusBM.h>
#include <Adafruit_DotStar.h>
#include <Servo.h>
#include <SPI.h>

#define IBUS_READ_ANALOG(ibus, chnl)  (map((float)(ibus).readChannel((chnl)), 1000.0, 2000.0, -1.0, 1.0))
#define IBUS_READ_DIGITAL(ibus, chnl)  (map((int)(ibus).readChannel((chnl)), 1000.0, 2000.0, -1.0, 1.0))

IBusBM IBus;
Servo motors[NUM_MOTORS];
Adafruit_DotStar strip(NUM_LEDS, DOTSTAR_BRG);

// ROS Includes

#include <ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Wrench.h>

ros::NodeHandle nh;

char joy_frame_id[] = "joystick";
sensor_msgs::Joy joy_msg;
ros::Publisher joy_pub("/eddy/joystick", &joy_msg);

char battery_frame_id[] = "cpu_battery";
sensor_msgs::BatteryState battery_msg;
ros::Publisher battery_pub("/eddy/battery/cpu", &battery_msg);

// Setup Buzzer + LED subscribers

void buzzer_callback(const std_msgs::Bool& msg) {
	digitalWrite(BUZZER_PIN, msg.data ? HIGH : LOW);
}
ros::Subscriber<std_msgs::Bool> buzzer_sub("/eddy/status/buzzer", &buzzer_callback);

void rosbag_led_callback(const std_msgs::ColorRGBA& color) {
	strip.setPixelColor(ROSBAG, color.r, color.b, color.g);
	strip.show();
}
ros::Subscriber<std_msgs::ColorRGBA> rosbag_led_sub("/eddy/status/led/rosbag", &rosbag_led_callback);

void controller_led_callback(const std_msgs::ColorRGBA& color) {
	strip.setPixelColor(CONTROLLER, color.r, color.b, color.g);
	strip.show();
}
ros::Subscriber<std_msgs::ColorRGBA> controller_led_sub("/eddy/status/led/controller", &controller_led_callback);

void sonar_led_callback(const std_msgs::ColorRGBA& color) {
	strip.setPixelColor(SONAR, color.r, color.b, color.g);
	strip.show();
}
ros::Subscriber<std_msgs::ColorRGBA> sonar_led_sub("/eddy/status/led/sonar", &sonar_led_callback);

void gps_led_callback(const std_msgs::ColorRGBA& color) {
	strip.setPixelColor(GPS, color.r, color.b, color.g);
	strip.show();
}
ros::Subscriber<std_msgs::ColorRGBA> gps_led_sub("/eddy/status/led/gps", &gps_led_callback);

void camera_led_callback(const std_msgs::ColorRGBA& color) {
	strip.setPixelColor(CAMERA, color.r, color.b, color.g);
	strip.show();
}
ros::Subscriber<std_msgs::ColorRGBA> camera_led_sub("/eddy/status/led/camera", &camera_led_callback);

// Setup motor thrust callback

void cmd_thrust_callback(const geometry_msgs::Wrench& wrench) {
	
	double fx = wrench.force.x;
	double tauz = wrench.force.z;
	
	// retrict torque to maximum capable by the robot (using reverse thrust)
	float max_tauz = -2 * EDDY_MAX_REVERSE_THRUST * EDDY_THRUSTER_SEPARATION;
	tauz = min(tauz, max_tauz);
	tauz = max(tauz, -max_tauz);
	
	// prioritize rotation
	double left_thrust = -tauz / (2 * EDDY_THRUSTER_SEPARATION);
	double right_thrust = tauz / (2 * EDDY_THRUSTER_SEPARATION);
	
	// scale back linear force to prevent oversaturating thrusters 
	double max_fx = 0;
	if (tauz >= 0) {
		if (fx >= 0) {
			max_fx = (EDDY_MAX_FORWARD_THRUST - left_thrust) * 2;
			fx = min(max_fx, fx);
		} else {
			max_fx = (EDDY_MAX_REVERSE_THRUST - right_thrust) * 2;
			fx = max(max_fx, fx);
		}
	} else {
		if (fx >= 0) {
			max_fx = (EDDY_MAX_FORWARD_THRUST - right_thrust) * 2;
			fx = min(max_fx, fx);
		} else {
			max_fx = (EDDY_MAX_FORWARD_THRUST - left_thrust) * 2;
			fx = max(max_fx, fx);
		}
	}
	
	left_thrust += fx/2.0;
	right_thrust += fx/2.0;
	
	left_thrust = constrain(left_thrust, EDDY_MAX_REVERSE_THRUST, EDDY_MAX_FORWARD_THRUST);
	right_thrust = constrain(right_thrust, EDDY_MAX_REVERSE_THRUST, EDDY_MAX_FORWARD_THRUST);
	
	int left_pwm = t200_thrust_pwm(left_thrust);
	int right_pwm = t200_thrust_pwm(right_thrust);
	
	motors[MOTOR_LEFT].writeMicroseconds(left_pwm);
	motors[MOTOR_RIGHT].writeMicroseconds(right_pwm);
}
ros::Subscriber<geometry_msgs::Wrench> cmd_thrust_sub("/eddy/cmd_thrust", &cmd_thrust_callback);

void setup() {
	
	// Initialize Peripherals
	
	IBus.begin(Serial1, IBUSBM_NOTIMER);
	while (IBus.cnt_rec == 0) {
		IBus.loop();
		delay(5);
	}
	
	strip.begin();
	strip.show();
	
	pinMode(BUZZER_PIN, OUTPUT);
	
	motors[MOTOR_LEFT].attach(LEFT_ESC_PIN);
	motors[MOTOR_RIGHT].attach(RIGHT_ESC_PIN);
	motors[MOTOR_LEFT].writeMicroseconds(PWM_STOP);
	motors[MOTOR_RIGHT].writeMicroseconds(PWM_STOP);
	
	// Initialize Messages
	
	joy_msg.header.seq = 0;
	joy_msg.header.frame_id = joy_frame_id;
	joy_msg.axes = (float*) malloc(6 * sizeof(float));
	joy_msg.buttons = (long int*) malloc(4 * sizeof(long int));
	
	battery_msg.header.seq = 0;
	battery_msg.header.frame_id = battery_frame_id;
	battery_msg.current = 0xFFFFFFFF;
	battery_msg.charge = 0xFFFFFFFF;
	battery_msg.capacity = 0xFFFFFFFF;
	battery_msg.design_capacity = 5.3;
	battery_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
	battery_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
	battery_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
	battery_msg.present = true;
	battery_msg.cell_voltage = (float*) malloc(4 * sizeof(float));
    
	// Initialize ROS connections

	nh.initNode();
	nh.getHardware()->setBaud(115200);
}

#define HERTZ(hz) (1000 / (hz))

unsigned long lastControllerUpdate = 0;
unsigned long lastBatteryUpdate = 0;

void loop() {
	
	unsigned long currentMillis = millis();
	
	IBus.loop(); // possiblly move into throttled code
	
	// Throttle the polling of signals from the controller
	if (currentMillis - lastControllerUpdate >= HERTZ(100)) {
		
		for (int i = 0; i < 6; i++) {
			joy_msg.axes[i] = IBUS_READ_ANALOG(IBus, i);
		}
		for (int i = 6; i < 10; i++) {
			joy_msg.buttons[i-6] = IBUS_READ_DIGITAL(IBus, i);
		}
		joy_msg.header.seq += 1;
		
		joy_pub.publish(&joy_msg);
		
		lastControllerUpdate = currentMillis;
	}
	
	// Throttle the polling of signals from the battery monitor
	if (currentMillis - lastBatteryUpdate >= HERTZ(1)) {
		
		int input = analogRead(VOLT_PIN);
		double voltage = ((input * 3.3) / 1024) / 0.2;
		
		battery_msg.voltage = voltage;
		battery_msg.percentage = map(voltage, 12.0, 16.8, 0.0, 1.0);
		
		if (battery_msg.percentage == 1.0) {
			battery_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
		} else {
			battery_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
		}
		
		if (battery_msg.voltage < 12.8) {
			battery_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
		} else {
			battery_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
		}
		
		battery_msg.header.seq += 1;
		
		battery_pub.publish(&battery_msg);
		
		lastBatteryUpdate = currentMillis;
	}
	
	nh.spinOnce();
	delay(1);
}
