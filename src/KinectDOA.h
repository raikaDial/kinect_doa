#ifndef KINECT_DOA_H_INCLUDED
#define KINECT_DOA_H_INCLUDED

#include <ros/ros.h>
#include "minidsp.h" // For GCCPHAT

class KinectDOA {
	public:
		KinectDOA(ros::NodeHandle nh);
		~KinectDOA();

		unsigned int m_numsamples_xcor;

	private:
		bool isNoise(); 
		double findAngle();

		ros::NodeHandle m_nh;

		// Variables to control angle estimation
		double m_mic_positions[4];
		double m_sound_speed;
		double m_sample_freq;
		unsigned int m_max_lag;
		std::vector<double*> m_xcor_data;
};

#endif // KINECT_DOA_H_INCLUDED