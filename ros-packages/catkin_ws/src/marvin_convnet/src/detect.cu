#include "depth_utils.h"
#include "ros/ros.h"
#include "marvin_convnet/DetectObjects.h"
#include "realsense_camera/StreamSensor.h"
#include <opencv2/opencv.hpp>

// Marvin
#define DATATYPE 0
#include "marvin.hpp"

std::string shelf_net_arch_filename = "/home/andyz/apc/toolbox/ros-packages/catkin_ws/src/marvin_convnet/models/competition/net.json";
std::string tote_net_arch_filename = "/home/andyz/apc/toolbox/ros-packages/catkin_ws/src/marvin_convnet/models/competition/net.json";
std::string shelf_net_weights_filename = "/home/andyz/apc/toolbox/ros-packages/catkin_ws/src/marvin_convnet/models/competition/weights_shelf.marvin";
std::string tote_net_weights_filename = "/home/andyz/apc/toolbox/ros-packages/catkin_ws/src/marvin_convnet/models/competition/weights_tote.marvin";

// Service modes and names
std::string service_name;
std::string camera_service_name;

// Directory to read/write all RGB-D files and response maps
std::string read_directory;
std::string net_directory;

// Global buffers for sensor data retrieval
int frame_width = 640;
int frame_height = 480;
uint8_t * color_buffer = new uint8_t[frame_width * frame_height * 3];
uint8_t * HHA_buffer = new uint8_t[frame_width * frame_height * 3];

// Load Marvin FCN network architectures
marvin::Net shelf_net(shelf_net_arch_filename);
marvin::Net tote_net(tote_net_arch_filename);

// Marvin responses
StorageT* color_data_CPU = NULL;
StorageT* HHA_data_CPU = NULL;
StorageT* prob_CPU_StorageT = NULL;
ComputeT* prob_CPU_ComputeT = NULL;

ros::ServiceClient client_sensor;

const int num_apc_objects = 39;

std::string shelf_bin_ids = "ABCDEFGHIJKL";

// Service call
bool srv_detect(marvin_convnet::DetectObjects::Request  &req,
                marvin_convnet::DetectObjects::Response &res) {
  ROS_INFO("Recieved service request.");

  int bin_id = req.BinId;
  int frame_id = req.FrameId;
  res.FrameId = frame_id;

  // Get frame filenames
  std::ostringstream frame_prefix;
  frame_prefix << std::setw(6) << std::setfill('0') << frame_id;
  std::string color_frame_filename = "/frame-" + frame_prefix.str() + ".color.png";
  std::string depth_frame_filename = "/frame-" + frame_prefix.str() + ".depth.png";
  std::string raw_depth_frame_filename = "/raw/frame-" + frame_prefix.str() + ".depth.png";
  std::string HHA_frame_filename = "/HHA/frame-" + frame_prefix.str() + ".HHA.png";

  // Read color frame from disk
  cv::Mat color_frame = cv::imread(read_directory + color_frame_filename, CV_LOAD_IMAGE_COLOR);
  color_buffer = color_frame.data;
  cv::Mat HHA_frame = cv::imread(read_directory + HHA_frame_filename.c_str(), CV_LOAD_IMAGE_COLOR);
  HHA_buffer = HHA_frame.data;

  // Color: BGR format, mean subtracted
  for (int r = 0; r < frame_height; ++r)
    for (int c = 0; c < frame_width; ++c) {
      color_data_CPU[0 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(color_buffer[0 + 3 * (c + frame_width * r)]) - ComputeT(102.9801f)); // B
      color_data_CPU[1 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(color_buffer[1 + 3 * (c + frame_width * r)]) - ComputeT(115.9465f)); // G
      color_data_CPU[2 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(color_buffer[2 + 3 * (c + frame_width * r)]) - ComputeT(122.7717f)); // R
      HHA_data_CPU[0 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(HHA_buffer[0 + 3 * (c + frame_width * r)]) - ComputeT(102.9801f)); // B
      HHA_data_CPU[1 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(HHA_buffer[1 + 3 * (c + frame_width * r)]) - ComputeT(115.9465f)); // G
      HHA_data_CPU[2 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(HHA_buffer[2 + 3 * (c + frame_width * r)]) - ComputeT(122.7717f)); // R
      // color_data_CPU[0 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(color_buffer[0 * frame_height * frame_width + r * frame_width + c]) - ComputeT(102.9801f)); // B
      // color_data_CPU[1 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(color_buffer[1 * frame_height * frame_width + r * frame_width + c]) - ComputeT(115.9465f)); // G
      // color_data_CPU[2 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(color_buffer[2 * frame_height * frame_width + r * frame_width + c]) - ComputeT(122.7717f)); // R
      // ROS_INFO("%f",CPUStorage2ComputeT(color_data_CPU[0 * frame_height * frame_width + r * frame_width + c]));
      // ROS_INFO("%f",CPUStorage2ComputeT(color_data_CPU[1 * frame_height * frame_width + r * frame_width + c]));
      // ROS_INFO("%f",CPUStorage2ComputeT(color_data_CPU[2 * frame_height * frame_width + r * frame_width + c]));
    } 

  // for (int r = 0; r < frame_height; ++r)
  //   for (int c = 0; c < frame_width; ++c) {
  //     rgbCPUStorageT[ 0 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT( rgb_uint8[ 0 + 3 * (c + frame_width * r) ] ) - ComputeT(102.9801f)); // B
  //     rgbCPUStorageT[ 1 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT( rgb_uint8[ 1 + 3 * (c + frame_width * r) ] ) - ComputeT(115.9465f)); // G
  //     rgbCPUStorageT[ 2 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT( rgb_uint8[ 2 + 3 * (c + frame_width * r) ] ) - ComputeT(122.7717f)); // R
  //   }

  // Run forward pass through marvin FCN
  ROS_INFO("Forward Marvin to get segmentation results.");
  marvin::Response * rDataRGB;
  marvin::Response * rDataHHA;
  marvin::Response * rProb;
  if (bin_id == -1) {
    rDataRGB = tote_net.getResponse("data_RGB");
    rDataHHA = tote_net.getResponse("data_HHA");
    rProb = tote_net.getResponse("prob");
  } else {
    rDataRGB = shelf_net.getResponse("data_RGB");
    rDataHHA = shelf_net.getResponse("data_HHA");
    rProb = shelf_net.getResponse("prob");
  }
  cudaMemcpy(rDataRGB->dataGPU, color_data_CPU, rDataRGB->numBytes(), cudaMemcpyHostToDevice);
  cudaMemcpy(rDataHHA->dataGPU, HHA_data_CPU, rDataHHA->numBytes(), cudaMemcpyHostToDevice);
  if (bin_id == -1)
    tote_net.forward();
  else
    shelf_net.forward();
  cudaMemcpy(prob_CPU_StorageT, rProb->dataGPU, rProb->numBytes(), cudaMemcpyDeviceToHost);
  for (int i = 0; i < frame_height * frame_width * (num_apc_objects + 1); ++i)
    prob_CPU_ComputeT[i] = CPUStorage2ComputeT(prob_CPU_StorageT[i]);

  // Get full object list
  std::vector<std::string> all_object_names = {"background", "barkely_hide_bones", "cherokee_easy_tee_shirt", "clorox_utility_brush", "cloud_b_plush_bear", "command_hooks", "cool_shot_glue_sticks", "crayola_24_ct", "creativity_chenille_stems", "dasani_water_bottle",
                                               "dove_beauty_bar", "dr_browns_bottle_brush", "easter_turtle_sippy_cup", "elmers_washable_no_run_school_glue", "expo_dry_erase_board_eraser", "fiskars_scissors_red", "fitness_gear_3lb_dumbbell", "folgers_classic_roast_coffee", "hanes_tube_socks", "i_am_a_bunny_book",
                                               "jane_eyre_dvd", "kleenex_paper_towels", "kleenex_tissue_box", "kyjen_squeakin_eggs_plush_puppies", "laugh_out_loud_joke_book", "oral_b_toothbrush_green", "oral_b_toothbrush_red", "peva_shower_curtain_liner", "platinum_pets_dog_bowl", "rawlings_baseball",
                                               "rolodex_jumbo_pencil_cup", "safety_first_outlet_plugs", "scotch_bubble_mailer", "scotch_duct_tape", "soft_white_lightbulb", "staples_index_cards", "ticonderoga_12_pencils", "up_glucose_bottle", "womens_knit_gloves", "woods_extension_cord"};
  std::vector<std::string> selected_object_names = req.ObjectNames;

  // Remove duplicates in selected object list
  std::sort(selected_object_names.begin(), selected_object_names.end());
  selected_object_names.erase(std::unique(selected_object_names.begin(), selected_object_names.end()), selected_object_names.end());

  // Loop through each object in selected list
  for (int selected_idx = 0; selected_idx < selected_object_names.size(); selected_idx++) {
    std::string curr_object_name = selected_object_names[selected_idx];
    int curr_object_idx = std::distance(all_object_names.begin(), find(all_object_names.begin(), all_object_names.end(), curr_object_name));
    std::vector<ComputeT> predMap_object(prob_CPU_ComputeT + curr_object_idx * frame_height * frame_width, prob_CPU_ComputeT + (curr_object_idx + 1) * frame_height * frame_width);

    // Create a folder to save results
    std::ifstream file(read_directory + "/masks");
    if (file.fail())
      system(std::string("mkdir -p " + read_directory + "/masks").c_str());

    // Write segmentation response maps to 16-bit grayscale png image
    std::string result_filename = read_directory + "/masks/frame-" + frame_prefix.str() + "." + all_object_names[curr_object_idx] + ".mask.png";
    cv::Mat result_mat(frame_height, frame_width, CV_16UC1);
    for (size_t y = 0; y < frame_height; y++)
      for (size_t x = 0; x < frame_width; x++) {
        unsigned short depth_short = (unsigned short)(predMap_object[y * frame_width + x] * 65535);
        result_mat.at<unsigned short>(y, x) = depth_short;
      }
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    cv::imwrite(result_filename, result_mat, compression_params);
  }

  return true;
}

int main(int argc, char **argv) {

  // Setup ROS
  ros::init(argc, argv, "marvin_convnet", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  // Get service parameters
  priv_nh.param("service_name", service_name, std::string("marvin_convnet"));
  priv_nh.param("camera_service_name", camera_service_name, std::string("/realsense_camera"));
  priv_nh.param("read_directory", read_directory, std::string(""));
  priv_nh.param("net_directory", net_directory, std::string(""));

  // Assert parameters and create folder to save segmentation masks
  assert(!read_directory.empty());
  system(std::string("mkdir -p " + read_directory).c_str());

  // Start service
  ros::ServiceServer service_detect = n.advertiseService(service_name, srv_detect);

  // Connect to Realsense camera
  ROS_INFO("Reading data from directory: %s", read_directory.c_str());

  // Setup Marvin
  ROS_INFO("Loading Marvin.");
  shelf_net.Malloc(marvin::Testing);
  tote_net.Malloc(marvin::Testing);
  shelf_net.loadWeights(shelf_net_weights_filename);
  tote_net.loadWeights(tote_net_weights_filename);
  color_data_CPU = new StorageT[frame_width * frame_height * 3];
  HHA_data_CPU = new StorageT[frame_width * frame_height * 3];
  prob_CPU_StorageT = new StorageT[frame_width * frame_height * (num_apc_objects + 1)];
  prob_CPU_ComputeT = new ComputeT[frame_height * frame_width * (num_apc_objects + 1)];

  ROS_INFO("Ready.");
  ros::spin();

  return 0;
}

