#ifndef KINECT_DOA_H_INCLUDED
#define KINECT_DOA_H_INCLUDED

#include <ros/ros.h>
extern "C" {
	#include "minidsp.h" // For GCCPHAT
}

class KinectDOA {
	public:
		KinectDOA(ros::NodeHandle nh);
		~KinectDOA();

		double findAngle();

		std::vector<double*> m_xcor_data;
		unsigned int m_numsamples_xcor;

		// Variables for white noise filter.
		std::vector<uint64_t> m_sumd0;
		std::vector<uint64_t> m_sumd1;

	private:
		bool isNoise();

		ros::NodeHandle m_nh;

		// Variables to control angle estimation
		double m_mic_positions[4];
		double m_sound_speed;
		double m_sample_freq;
		unsigned int m_max_lag;
		double m_white_noise_ratio;
		double m_last_angle_estimate;
};

#endif // KINECT_DOA_H_INCLUDED
