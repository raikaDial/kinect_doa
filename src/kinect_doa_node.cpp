#include <ros/ros.h>
#include <boost/thread/thread.hpp>

// DSP
#include "KinectDOA.h"
#include <deque>
#include <vector>

// Libfreenect libraries
#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect_audio.h>
#include "ofxKinectExtras.h"

KinectDOANode* kinect_doa_node;

// Callback to process microphone data.
void audioInCallback(freenect_device* dev, int num_samples,
	int32_t* mic1, int32_t* mic2, int32_t* mic3, int32_t* mic4,
	int16_t* cancelled, void *unknown
) {

	std::vector<int32_t*> sound_packet{mic1, mic2, mic3, mic4}; // Easier to iterate
	kinect_doa_node -> processAudioPacket(sound_packet);
}

class KinectDOANode {
	public:
		KinectDOANode(ros::NodeHandle nh) : m_nh(nh), m_kinect_doa(nh), m_xcor_counter(0), m_sound_data_ready(false) 
		{
			// Retrieve Parameters
			ros::NodeHandle pr_nh("~");
			pr_nh.param<bool>("using_kinect_1473", m_using_kinect_1473, false);

			// Initialize freenect context
			if (freenect_init(&m_f_ctx, NULL) < 0) {
				ROS_ERROR("freenect_init() failed\n");
			}

			// Need to upload special firmware for Kinect #1473
			// For some reason, if we try to upload this when the camera and motor subdevices are selected, the upload will
			//     not work and the program can't open the audio. So the solution is to open just the audio device, upload,
			//     the firmware, then close it.
			if(m_using_kinect_1473) {
				ROS_INFO("Uploading firmware to Kinect 1473\n");

				freenect_set_fw_address_nui(m_f_ctx, ofxKinectExtras::getFWData1473(), ofxKinectExtras::getFWSize1473());
				freenect_set_fw_address_k4w(m_f_ctx, ofxKinectExtras::getFWDatak4w(), ofxKinectExtras::getFWSizek4w());
				
				bool status = freenectConnect((freenect_device_flags)(FREENECT_DEVICE_AUDIO));
				
				if(status) {
					ROS_INFO("Firmware upload successful.\n");
					freenect_close_device(m_f_dev);
					freenect_shutdown(m_f_ctx);
				}
			}

			// Now connect to all the subdevices we care about
			bool status = freenectConnect((freenect_device_flags)(FREENECT_DEVICE_AUDIO));

			// Initialize sound buffers
			for(int i=0; i<4; ++i) {
				m_sound_buffers.push_back(std::deque<int32_t>());
			}

			// Start thread to get Kinect data
			m_freenect_thread = new boost::thread(boost::bind(&KinectDOANode::freenectThreadFunc, this));

		}

		~KinectDOANode() {
			m_freenect_thread -> join();
			delete m_freenect_thread;
		}

		bool freenectConnect(freenect_device_flags devices) {
			freenect_select_subdevices(m_f_ctx, devices);
			int num_devices = freenect_num_devices(m_f_ctx);
			if (num_devices < 1) {
				ROS_ERROR("Found no Kinect devices.\n");
				freenect_shutdown(m_f_ctx);
				return false;
			}

			int user_device_number = 0;
			if (freenect_open_device(m_f_ctx, &m_f_dev, user_device_number) < 0) {
				ROS_ERROR("Could not open device\n");
				freenect_shutdown(m_f_ctx);
				return false;
			}
			return true;
		}

		void processAudioPacket(const std::vector<int32_t*> & sound_packet) {
			// Store microphone data
			for(size_t i=0; i<m_sound_buffers.size(); ++i) {
				for(size_t j=0; j<num_samples; ++j) {
					m_sound_buffers[i].push_back(sound_packet[i][j]);
				}
				// Rolling buffer
				while(m_sound_buffers[i].size() > m_kinect_doa.m_numsamples_xcor) {
					m_sound_buffers[i].pop_front();
				}
			}
			m_xcor_counter += num_samples;

			// Trigger angle estimation if we've collected enough samples. Overlap
			//     cross-correlations by 50%.
			if((m_xcor_counter >= m_kinect_doa.m_numsamples_xcor/2) 
				&& (m_sound_buffers[0].size() == m_kinect_doa.m_numsamples_xcor)
			) {
				m_xcor_counter = 0;

				// Copy data
				for(int i=0; i<4; ++i) {
					for(size_t j=0; j<m_kinect_doa.m_numsamples_xcor; ++j) {
						m_kinect_doa.m_xcor_data[i][j] = m_sound_buffers[i][j];
					}
				}

				// Signal main thread that data is ready.
				boost::unique_lock<boost::mutex> lock(m_sound_data_ready_mutex);
				m_sound_data_ready = true;
				m_sound_data_ready_cond.notify_all();
			}	
		}

		void freenectThreadFunc() {
			freenect_set_audio_in_callback(m_f_dev, audioInCallback);

			//freenect_set_audio_in_callback(m_f_dev, boost::bind(&KinectDOANode::audioInCallback, this));
			freenect_start_audio(m_f_dev);

			while((freenect_process_events(m_f_ctx) >= 0) && ros::ok());

			// Close everything down
			freenect_stop_audio(m_f_dev);
			freenect_close_device(m_f_dev);
			freenect_shutdown(m_f_ctx);
		}

		void update() {
			// Wait for new data to be available
			boost::unique_lock<boost::mutex> lock(m_sound_data_ready_mutex);
			while(!m_sound_data_ready) {
				m_sound_data_ready_cond.wait(lock);
			}

			// Calculate DOA
			double angle = m_kinect_doa.findAngle();
			ROS_INFO("Angle to event: %lf\n", angle);

			m_sound_data_ready = false;
		}

	private:
		ros::NodeHandle m_nh;

		// Put data collection in its own thread and notify main loop when data is ready.
		boost::thread* m_freenect_thread;
		boost::condition_variable m_sound_data_ready_cond;
		boost::mutex m_sound_data_ready_mutex;
		bool m_sound_data_ready;

		KinectDOA m_kinect_doa;
		std::vector<std::deque<int32_t>> m_sound_buffers;
		int m_xcor_counter; // Triggers xcor if we've received enough samples;

		bool m_using_kinect_1473; // Have to upload special firmware for this Kinect version

		// libfreenect variables
		freenect_context* m_f_ctx;
		freenect_device* m_f_dev;
		
};


int main(int argc, char** argv) {
	// Initialize ROS node
	ros::init(argc, argv, "kinect_doa_node");
	ros::NodeHandle nh;

	kinect_doa_node = new KinectDOANode(nh);

	while(ros::ok()) {
		kinect_doa_node -> update();
		ros::spinOnce();
	}

	delete kinect_doa_node;

}
