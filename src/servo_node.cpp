// Good information on servo control with the Raspberry Pi here:
//     https://projects.drogon.net/raspberry-pi/wiringpi/functions
//     https://raspberrypi.stackexchange.com/questions/4906/control-hardware-pwm-frequency

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <wiringPi.h>

class ServoController {
	public:
		ServoController(ros::NodeHandle nh) : m_nh(nh), m_time_last_write(ros::Time::now()) {
			
			if(wiringPiSetupGpio() == -1) {
				ROS_ERROR("Error setting up GPIO.\n");
			}
			else {
				// Get parameters
				ros::NodeHandle pr_nh("~");

				// Servo hardware specifications
				pr_nh.param<int>("min_us", m_min_us, 1000);
				pr_nh.param<int>("max_us", m_max_us, 2000);
				pr_nh.param<int>("max_deg", m_max_deg, 180);
				
				// Derived servo params
				m_centerpoint_us = (m_min_us + m_max_us)/2;
				m_us_per_deg = (m_max_us-m_min_us)/m_max_deg;

				// Panning speed specification
				pr_nh.param<double>("sec_per_deg", m_sec_per_deg, 0.025);
				if(m_sec_per_deg <= 0.02) {
					ROS_WARN("Specified seconds per degree is faster than PWM write frequency. Setting to default...");
					m_sec_per_deg = 0.025;
				}
				
				
				pinMode(18, PWM_OUTPUT); // Configure GPIO18 for PWM control of servo
				pwmSetMode(PWM_MODE_MS); // Mark:space mode, standard PWM

				// PWM Frequency (Hz) = 19.2e6 Hz / pwmClock / pwmRange
				//     Analog servos expect a frequency of 50 Hz
				// PWM range is the # of steps between 0% and 100% duty cycle.
				//     With a period of 20 ms (50 Hz), a range of 4000 gives us a resolution
				//     of 5 us. For a standard analog servo, whose range is 1000 to 2000 us for
				//     0 to 180 degrees, this gives us an angular resolution of 0.9 degrees.
				//     The max range for the Pi's PWM hardware is 4096, so this is about the 
				//     best resolution we can get.
				pwmSetClock(96);
				pwmSetRange(4000);

				// Move the servo to the centerpoint
				m_target = m_centerpoint_us;
				m_current = m_target;
				// With a pwm clock of 192 and range of 2000, pwmWrite input is one-to-one with microseconds/10.
				//    But since we doubled the resolution, we have to multiply our microseconds/10 by 2.
				
				pwmWrite(18, 2*m_current/10);

				// Setup servo angle subscriber
				m_servo_angle_sub = m_nh.subscribe("servo_angle", 1, &ServoController::servoAngleCallback, this);
			}

		}

		void servoAngleCallback(const std_msgs::Float32::ConstPtr & angle) {
			m_target = (uint16_t) (m_centerpoint_us + (angle -> data)*m_us_per_deg);
		}

		void update() {
			// If we're not at our target position, incrementally proceed to the position
			//    in accordance with the specified timestep
			if(((ros::Time::now() - m_time_last_write).toSec() >  m_sec_per_deg)
				&& (m_current != m_target)
			) {
				if(m_current < m_target) {
					m_current += m_us_per_deg;
					if(m_current > m_target) {
						m_current = m_target;
					}
				}
				else if(m_current > m_target) {
					m_current -= m_us_per_deg;
					if(m_current < m_target) {
						m_current = m_target;
					}
				}
				ROS_INFO("Writing %u micros to servo.\n", m_current);
				pwmWrite(18, 2*m_current/10);
				m_time_last_write = ros::Time::now();
			}
		}


	private:
		ros::NodeHandle m_nh;
		ros::Subscriber m_servo_angle_sub;

		uint16_t m_target, m_current; // Target command for servo and the current command
		
		int m_min_us, m_max_us, m_max_deg;
		uint16_t m_centerpoint_us;
		double m_us_per_deg;
		double m_sec_per_deg;

		ros::Time m_time_last_write;
};

int main(int argc, char** argv) {
	// Initialize ROS node
	ros::init(argc, argv, "servo_node");
	ros::NodeHandle nh;

	ServoController servo_controller(nh);
	ROS_INFO("Servo controller initialized.\n");

	while(ros::ok()) {
		ros::spinOnce();
		servo_controller.update();
	}

	return 0;
}
