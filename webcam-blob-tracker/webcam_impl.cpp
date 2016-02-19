#include "webcam_impl.h"

webcam_impl::webcam_impl()
{
	this->_streaming = false;
	this->_converted = false;
	this->image_width = 0;
	this->image_height = 0;
	this->image_channels = 0;

	if (!cam.open(0))
	{
		std::cout << "Failed to open camera" << std::endl;
		return;
	}
	else
	{
		//cv::Mat frame;
		cam >> this->image_frame;
		if (this->image_frame.empty())
			std::cout << "Captured Empty Frame" << std::endl;
		else
		{
			this->image_height = this->image_frame.rows;
			this->image_width = this->image_frame.cols;
			this->image_channels = this->image_frame.step / this->image_frame.cols;
		}
	}
	//this->image_data = new uint8_t[this->image_width * this->image_height * this->image_channels];
	t1 = boost::thread(boost::bind(&webcam_impl::background_worker, this));
}
webcam_impl::~webcam_impl()
{
	t1.interrupt();
	t1.join();
	cv::destroyAllWindows();
}

RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > webcam_impl::getCurrentImage()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > I(new edu::rpi::cats::sensors::camera_interface::Image());


	I->width = this->image_width;
	I->height = this->image_height;

	cv::Mat image_frame_hsv;
	

	cv::cvtColor(this->image_frame, image_frame_hsv, CV_BGR2HSV);
	memcpy(this->image_frame.data, image_frame_hsv.data, this->image_height * this->image_width * 3);
	
		
	if (this->image_width != 0 && this->image_height != 0)
		I->data = RobotRaconteur::AttachRRArrayCopy<uint8_t>(this->image_frame.data, I->width * I->height * 3);

	return I;
}
RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > webcam_impl::getImageHeader()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > I(new edu::rpi::cats::sensors::camera_interface::ImageHeader());
	I->width = this->image_width;
	I->height = this->image_height;
	I->step = 1;
	I->channels = this->image_channels;

	return I;
}

void webcam_impl::StartStreaming()
{



	this->_streaming = true;
}
void webcam_impl::StopStreaming()
{


	this->_streaming = false;
}

RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > webcam_impl::get_ImageStream()
{
	return this->image_pipe;
}
void webcam_impl::set_ImageStream(RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > value)
{
	this->image_pipe = value;
}

void webcam_impl::background_worker()
{
	
	std::cout << "Starting up background thread" << std::endl;
	while (true)
	{
		try
		{
			if (this->_streaming)
			{
				cam >> this->image_frame;
				if (this->image_frame.empty())
				{
					std::cout << "Dropped Frame" << std::endl;
					continue;
				}
				else
				{
					cv::Mat image_frame_hsv;
					cv::Mat image_frame_hsv_thresh;
					cv::Mat cc_labels, cc_stats, cc_centroids;
					cv::cvtColor(this->image_frame, image_frame_hsv, CV_BGR2HSV);
					//cv::inRange(image_frame_hsv, cv::Scalar(185, 70, 64), cv::Scalar(255, 90, 192), image_frame_hsv_thresh);
					cv::inRange(image_frame_hsv, cv::Scalar(100, 110, 128), cv::Scalar(115, 200, 255), image_frame_hsv_thresh);
					cv::erode(image_frame_hsv_thresh, image_frame_hsv_thresh, cv::Mat());
					cv::dilate(image_frame_hsv_thresh, image_frame_hsv_thresh, cv::Mat());
					
					int n_cc = cv::connectedComponentsWithStats(image_frame_hsv_thresh, cc_labels, cc_stats, cc_centroids);

					std::cout << "Found " << n_cc << " connected components." << std::endl;
					

					if (n_cc > 1)
					{
						int max_a1 = 0, max_a2 = 0;
						int max_i1 = 0, max_i2 = 0;
						for (int i = 1; i < n_cc; i++)
						{
							if (cc_stats.at<int>(i, cv::CC_STAT_AREA) > max_a1)
							{
								max_a2 = max_a1;
								max_i2 = max_i1;
								max_a1 = cc_stats.at<int>(i, cv::CC_STAT_AREA);
								max_i1 = i;
							}
							else if (cc_stats.at<int>(i, cv::CC_STAT_AREA) > max_a2)
							{
								max_a2 = cc_stats.at<int>(i, cv::CC_STAT_AREA);
								max_i2 = i;
							}
						}
						cv::rectangle(image_frame_hsv_thresh, cv::Rect(cc_stats.at<int>(max_i1, cv::CC_STAT_LEFT), cc_stats.at<int>(max_i1, cv::CC_STAT_TOP), cc_stats.at<int>(max_i1, cv::CC_STAT_WIDTH), cc_stats.at<int>(max_i1, cv::CC_STAT_HEIGHT)), cv::Scalar(255));
						cv::rectangle(image_frame_hsv_thresh, cv::Rect(cc_stats.at<int>(max_i2, cv::CC_STAT_LEFT), cc_stats.at<int>(max_i2, cv::CC_STAT_TOP), cc_stats.at<int>(max_i2, cv::CC_STAT_WIDTH), cc_stats.at<int>(max_i2, cv::CC_STAT_HEIGHT)), cv::Scalar(255));
					}

					
					imshow("hsv", this->image_frame);
					imshow("threshold", image_frame_hsv_thresh);
					cv::waitKey(30);
				}

			
			}
			boost::this_thread::interruption_point();
		}
		catch (...) {
			break;
		}
	}
	std::cout << "Exiting background thread" << std::endl;
}