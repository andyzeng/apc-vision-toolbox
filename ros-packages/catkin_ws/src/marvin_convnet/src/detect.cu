// ---------------------------------------------------------
// Copyright (c) 2016, Andy Zeng
// 
// This file is part of the APC Vision Toolbox and is available 
// under the terms of the Simplified BSD License provided in 
// LICENSE. Please retain this notice and LICENSE if you use 
// this file (or any portion of it) in your project.
// ---------------------------------------------------------

#include "depth_utils.h"
#include "ros/ros.h"
#include "marvin_convnet/DetectObjects.h"
#include "marvin_convnet/object_mask.h"
//#include "realsense_camera/StreamSensor.h"
#include <opencv2/opencv.hpp>

// Marvin
#define DATATYPE 0
#include "marvin.hpp"

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>




//std::string shelf_net_arch_filename = "/home/joinet/text-pick-n-place-baseline/ros-packages/catkin_ws/src/marvin_convnet/models/competition/net.json";
std::string tote_net_arch_filename = "/home/joinet/text-pick-n-place-baseline/ros-packages/catkin_ws/src/marvin_convnet/models/competition/net.json";
//std::string shelf_net_weights_filename = "/home/joinet/text-pick-n-place-baseline/ros-packages/catkin_ws/src/marvin_convnet/models/competition/weights_shelf.marvin";
std::string tote_net_weights_filename = "/home/joinet/text-pick-n-place-baseline/ros-packages/catkin_ws/src/marvin_convnet/models/competition/weights_tote.marvin";


// Service modes and names
std::string service_name;

// Directory to read/write all RGB-D files and response maps
std::string read_directory;

// Global buffers for sensor data retrieval
int frame_width = 640;
int frame_height = 480;
uint8_t * color_buffer = new uint8_t[frame_width * frame_height * 3];
uint8_t * HHA_buffer = new uint8_t[frame_width * frame_height * 3];

// Load Marvin FCN network architectures
//marvin::Net shelf_net(shelf_net_arch_filename);
marvin::Net tote_net(tote_net_arch_filename);

// Marvin responses
StorageT* color_data_CPU = NULL;
StorageT* HHA_data_CPU = NULL;
StorageT* prob_CPU_StorageT = NULL;
ComputeT* prob_CPU_ComputeT = NULL;

ros::ServiceClient client_sensor;

const int num_apc_objects = 39;

std::string shelf_bin_ids = "ABCDEFGHIJKL";


// For realtime prediction
class ImageConverter
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh_p;
  ros::Publisher pub = nh_p.advertise<sensor_msgs::Image>("/mask_prediction", 1);

  ros::NodeHandle nh_ps;
  ros::Publisher pub_s = nh_ps.advertise<marvin_convnet::object_mask>("/mask_prediction_with_class", 1);

  marvin_convnet::object_mask object_mask;


  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
    

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat color_frame;
    
    try
    {
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      color_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

      color_buffer = color_frame.data;
       
      for (int r = 0; r < frame_height; ++r)
    	  for (int c = 0; c < frame_width; ++c) {
      color_data_CPU[0 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(color_buffer[0 + 3 * (c + frame_width * r)]) - ComputeT(102.9801f)); // B
      color_data_CPU[1 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(color_buffer[1 + 3 * (c + frame_width * r)]) - ComputeT(115.9465f)); // G
      color_data_CPU[2 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(color_buffer[2 + 3 * (c + frame_width * r)]) - ComputeT(122.7717f)); // R
      } 



	  // Run forward pass through marvin FCN
	  ROS_INFO("Forward Marvin to get segmentation results.");
	  marvin::Response * rDataRGB;
	  marvin::Response * rDataHHA;
	  marvin::Response * rProb;
          int bin_id = -1; //force to use tote model
	  if (bin_id == -1) {
	    rDataRGB = tote_net.getResponse("data_RGB");
	    rProb = tote_net.getResponse("prob");
	  } else {
	    //rDataRGB = shelf_net.getResponse("data_RGB");
	    //rProb = shelf_net.getResponse("prob");
	  }
	  cudaMemcpy(rDataRGB->dataGPU, color_data_CPU, rDataRGB->numBytes(), cudaMemcpyHostToDevice);
	  if (bin_id == -1)
	    tote_net.forward();
	  //else
	    //shelf_net.forward();
	  cudaMemcpy(prob_CPU_StorageT, rProb->dataGPU, rProb->numBytes(), cudaMemcpyDeviceToHost);
	  for (int i = 0; i < frame_height * frame_width * (num_apc_objects + 1); ++i)
	    prob_CPU_ComputeT[i] = CPUStorage2ComputeT(prob_CPU_StorageT[i]);

  // Get full object list
  std::vector<std::string> all_object_names = {"background", "barkely_hide_bones", "cherokee_easy_tee_shirt", "clorox_utility_brush", "cloud_b_plush_bear", "command_hooks", "cool_shot_glue_sticks", "crayola_24_ct", "creativity_chenille_stems", "dasani_water_bottle",
                                               "dove_beauty_bar", "dr_browns_bottle_brush", "easter_turtle_sippy_cup", "elmers_washable_no_run_school_glue", "expo_dry_erase_board_eraser", "fiskars_scissors_red", "fitness_gear_3lb_dumbbell", "folgers_classic_roast_coffee", "hanes_tube_socks", "i_am_a_bunny_book",
                                               "jane_eyre_dvd", "kleenex_paper_towels", "kleenex_tissue_box", "kyjen_squeakin_eggs_plush_puppies", "laugh_out_loud_joke_book", "oral_b_toothbrush_green", "oral_b_toothbrush_red", "peva_shower_curtain_liner", "platinum_pets_dog_bowl", "rawlings_baseball",
                                               "rolodex_jumbo_pencil_cup", "safety_first_outlet_plugs", "scotch_bubble_mailer", "scotch_duct_tape", "soft_white_lightbulb", "staples_index_cards", "ticonderoga_12_pencils", "up_glucose_bottle", "womens_knit_gloves", "woods_extension_cord"};

 std::vector<std::string> selected_object_names = all_object_names;  // do all the objects segmetation
  

  // Loop through each object in selected list
  for (int selected_idx = 0; selected_idx < selected_object_names.size(); selected_idx++) {
    std::string curr_object_name = selected_object_names[selected_idx];
    int curr_object_idx = std::distance(all_object_names.begin(), find(all_object_names.begin(), all_object_names.end(), curr_object_name));
    std::vector<ComputeT> predMap_object(prob_CPU_ComputeT + curr_object_idx * frame_height * frame_width, prob_CPU_ComputeT + (curr_object_idx + 1) * frame_height * frame_width);

    // Create a folder to save results
    //std::ifstream file(read_directory + "/masks");
    //if (file.fail())
      //system(std::string("mkdir -p " + read_directory + "/masks").c_str());

    // Write segmentation response maps to 16-bit grayscale png image
    //std::string result_filename = read_directory + "/masks/frame-" + frame_prefix.str() + "." + all_object_names[curr_object_idx] + ".mask.png";
    cv::Mat result_mat(frame_height, frame_width, CV_16UC1);
    for (size_t y = 0; y < frame_height; y++)
      for (size_t x = 0; x < frame_width; x++) {
        unsigned short depth_short = (unsigned short)(predMap_object[y * frame_width + x] * 65535);
        result_mat.at<unsigned short>(y, x) = depth_short;
      }

    cv_bridge::CvImage cv_image;
    cv::Mat result_mat_final(480, 640, CV_16UC1);
    result_mat_final = result_mat;

    cv_image.image = result_mat_final;
    cv_image.encoding = "mono16";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    
    
    std::stringstream ss;
    ss << "/mask_prediction/" << selected_idx; //asign specific topic to prediction
    std::string s_object = ss.str();
    
    object_mask.probmask = ros_image;
    object_mask.object = selected_object_names[selected_idx];
    pub_s.publish(object_mask);


    pub.publish(ros_image);
  }


  }


};

















int main(int argc, char **argv) {

  // Setup ROS
  ros::init(argc, argv, "marvin_convnet", ros::init_options::AnonymousName);
  //ros::NodeHandle n;
  //ros::NodeHandle priv_nh("~");

  // Get service parameters
  //priv_nh.param("service_name", service_name, std::string("marvin_convnet"));
  //priv_nh.param("read_directory", read_directory, std::string(""));

  // Assert parameters and create folder to save segmentation masks
  //assert(!read_directory.empty());
  //system(std::string("mkdir -p " + read_directory).c_str());

  // Start service
  //ros::ServiceServer service_detect = n.advertiseService(service_name, srv_detect);

  // Connect to Realsense camera
  //ROS_INFO("Reading data from directory: %s", read_directory.c_str());

  // Setup Marvin
  ROS_INFO("Loading Marvin.");
  //shelf_net.Malloc(marvin::Testing);
  tote_net.Malloc(marvin::Testing);
  //shelf_net.loadWeights(shelf_net_weights_filename);
  tote_net.loadWeights(tote_net_weights_filename);
  color_data_CPU = new StorageT[frame_width * frame_height * 3];
  HHA_data_CPU = new StorageT[frame_width * frame_height * 3];
  prob_CPU_StorageT = new StorageT[frame_width * frame_height * (num_apc_objects + 1)];
  prob_CPU_ComputeT = new ComputeT[frame_height * frame_width * (num_apc_objects + 1)];


  ImageConverter ic;
  ROS_INFO("Ready.");
  ros::spin();

  return 0;
}

