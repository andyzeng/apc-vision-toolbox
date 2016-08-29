#include "depth_utils.h"
#include "ros/ros.h"
#include "marvin_convnet/DetectObjects.h"
#include "realsense_camera/StreamSensor.h"
#include <opencv2/opencv.hpp>

// Marvin
#define DATATYPE 0
#include "marvin.hpp"

std::string shelf_net_arch_filename = "/home/andyz/apc/toolbox/ros-packages/catkin_ws/src/marvin_convnet/net/shelf_tote_testing.json";
std::string tote_net_arch_filename = "/home/andyz/apc/toolbox/ros-packages/catkin_ws/src/marvin_convnet/net/shelf_tote_testing.json";
std::string shelf_net_weights_filename = "/home/andyz/apc/toolbox/ros-packages/catkin_ws/src/marvin_convnet/net/shelf_net.marvin";
std::string tote_net_weights_filename = "/home/andyz/apc/toolbox/ros-packages/catkin_ws/src/marvin_convnet/net/tote_net.marvin";

// Service modes and names
const int READ_SENSOR_NO_FORWARD_PASS = 0;
const int READ_SENSOR_FORWARD_PASS = 1;
const int READ_FILE_FORWARD_PASS = 2;
int service_mode;
std::string service_name;
std::string camera_service_name;

// Directory to read/write all RGB-D files and response maps
std::string read_directory;
std::string write_directory;
std::string net_directory;

// Global buffers for sensor data retrieval
int frame_width = 640;
int frame_height = 480;
bool cloud_buffer_loaded = false;
float * cloud_buffer_pts = new float[frame_width * frame_height * 3];
uint8_t * cloud_buffer_rgb = new uint8_t[frame_width * frame_height * 3];
float * buffer_depth = new float[frame_width * frame_height];
float * buffer_raw_depth = new float[frame_width * frame_height];
float * color_cam_intrinsics = new float[9];
float * depth_cam_intrinsics = new float[9];
float * depth2color_extrinsics = new float[12];

// Load Marvin FCN network architectures
marvin::Net shelf_net(shelf_net_arch_filename);
marvin::Net tote_net(tote_net_arch_filename);

// Marvin responses
StorageT* color_data_CPU = NULL;
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

  // Retrieve RGB-D data from camera service and save to disk
  if (service_mode == READ_SENSOR_FORWARD_PASS || service_mode == READ_SENSOR_NO_FORWARD_PASS) {
    realsense_camera::StreamSensor srv_sensor;
    if (!client_sensor.call(srv_sensor)) {
      std::cout << "Failed to call service " + camera_service_name << std::endl;
      return true;
    }

    // Load point cloud and depth buffers from camera service
    std::copy(srv_sensor.response.cloudXYZ.begin(), srv_sensor.response.cloudXYZ.end(), cloud_buffer_pts);
    std::copy(srv_sensor.response.cloudRGB.begin(), srv_sensor.response.cloudRGB.end(), cloud_buffer_rgb);
    std::copy(srv_sensor.response.rawDepth.begin(), srv_sensor.response.rawDepth.end(), buffer_raw_depth);
    for (int i = 0; i < frame_width * frame_height * 3; i += 3)
      buffer_depth[i / 3] = (float)(srv_sensor.response.cloudXYZ[i + 2]);
    // cloud_buffer_loaded = true;

    // Load camera information from camera service
    for (int i = 0; i < 9; ++i) {
      color_cam_intrinsics[i] = srv_sensor.response.colorCamIntrinsics[i];
      depth_cam_intrinsics[i] = srv_sensor.response.depthCamIntrinsics[i];
    }
    for (int i = 0; i < 12; ++i)
      depth2color_extrinsics[i] = srv_sensor.response.depth2colorExtrinsics[i];

    // Save Bin ID
    std::string cam_info_file = write_directory + "/cam.info.txt";
    FILE *fp = fopen(cam_info_file.c_str(), "w");
    if (bin_id == -1)
      fprintf(fp, "# Environment: tote\n# Bin ID: N/A\n");
    else
      fprintf(fp, "# Environment: shelf\n# Bin ID: %s\n", shelf_bin_ids.substr(bin_id, 1).c_str());

    // Save object list
    fprintf(fp, "# Objects: [");
    for (int i = 0; i < (req.ObjectNames.size() - 1); ++i)
      fprintf(fp, "\"%s\",", req.ObjectNames[i].c_str());
    if (req.ObjectNames.size() > 0)
      fprintf(fp, "\"%s\"",req.ObjectNames[req.ObjectNames.size() - 1].c_str());
    fprintf(fp, "]\n\n");

    // Save camera intrinsics of color sensor
    fprintf(fp, "# Color camera intrinsic matrix\n");
    for (int i = 0; i < 3; ++i)
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", (float)(color_cam_intrinsics[i * 3 + 0]), (float)(color_cam_intrinsics[i * 3 + 1]), (float)(color_cam_intrinsics[i * 3 + 2]));

    // Save camera intrinsics of depth sensor
    fprintf(fp, "\n# Depth camera intrinsic matrix\n");
    for (int i = 0; i < 3; ++i)
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", (float)(depth_cam_intrinsics[i * 3 + 0]), (float)(depth_cam_intrinsics[i * 3 + 1]), (float)(depth_cam_intrinsics[i * 3 + 2]));

    // Save camera-to-camera extrinsics from depth sensor to color sensor
    fprintf(fp, "\n# Depth-to-color camera extrinsic matrix\n");
    for (int i = 0; i < 3; ++i)
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", (float)(depth2color_extrinsics[i * 4 + 0]), (float)(depth2color_extrinsics[i * 4 + 1]), (float)(depth2color_extrinsics[i * 4 + 2]), (float)(depth2color_extrinsics[i * 4 + 3]));
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", 0.0f, 0.0f, 0.0f, 1.0f);
    fclose(fp);

    // Save color frame
    cv::Mat color_frame = cv::Mat(frame_width * frame_height * 3, 1, CV_8U, cloud_buffer_rgb).clone();
    color_frame = color_frame.reshape(3, frame_height);
    cv::cvtColor(color_frame, color_frame, CV_RGB2BGR);
    cv::imwrite(write_directory + color_frame_filename, color_frame);
    cloud_buffer_rgb = color_frame.data;

    // Save depth frame (aligned and un-aligned raw)
    WriteDepth(write_directory + depth_frame_filename, buffer_depth, frame_height, frame_width); 
    WriteDepth(write_directory + raw_depth_frame_filename, buffer_raw_depth, frame_height, frame_width);
  } else {

    // Read color frame
    cv::Mat color_frame = cv::imread(read_directory + color_frame_filename, CV_LOAD_IMAGE_COLOR);
    cloud_buffer_rgb = color_frame.data;
  }

  if (service_mode == READ_SENSOR_FORWARD_PASS || service_mode == READ_FILE_FORWARD_PASS) {

    // Color: BGR format, mean subtracted
    for (int r = 0; r < frame_height; ++r)
      for (int c = 0; c < frame_width; ++c) {
        color_data_CPU[0 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(cloud_buffer_rgb[0 + 3 * (c + frame_width * r)]) - ComputeT(102.9801f)); // B
        color_data_CPU[1 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(cloud_buffer_rgb[1 + 3 * (c + frame_width * r)]) - ComputeT(115.9465f)); // G
        color_data_CPU[2 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(cloud_buffer_rgb[2 + 3 * (c + frame_width * r)]) - ComputeT(122.7717f)); // R
        // color_data_CPU[0 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(cloud_buffer_rgb[0 * frame_height * frame_width + r * frame_width + c]) - ComputeT(102.9801f)); // B
        // color_data_CPU[1 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(cloud_buffer_rgb[1 * frame_height * frame_width + r * frame_width + c]) - ComputeT(115.9465f)); // G
        // color_data_CPU[2 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(cloud_buffer_rgb[2 * frame_height * frame_width + r * frame_width + c]) - ComputeT(122.7717f)); // R
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
    marvin::Response * rData;
    marvin::Response * rProb;
    if (bin_id == -1) {
      rData = tote_net.getResponse("data");
      rProb = tote_net.getResponse("prob");
    } else {
      rData = shelf_net.getResponse("data");
      rProb = shelf_net.getResponse("prob");
    }
    cudaMemcpy(rData->dataGPU, color_data_CPU, rData->numBytes(), cudaMemcpyHostToDevice);
    if (bin_id == -1)
      tote_net.forward();
    else
      shelf_net.forward();
    cudaMemcpy(prob_CPU_StorageT, rProb->dataGPU, rProb->numBytes(), cudaMemcpyDeviceToHost);
    for (int i = 0; i < frame_height * frame_width * (num_apc_objects + 1); ++i)
      prob_CPU_ComputeT[i] = CPUStorage2ComputeT(prob_CPU_StorageT[i]);

    // Get full object list
    std::vector<std::string> all_object_names = {"background", "barkely_hide_bones", "cherokee_easy_tee_shirt", "clorox_utility_brush", "cloud_b_plush_bear", "cool_shot_glue_sticks", "command_hooks", "crayola_24_ct", "creativity_chenille_stems", "dasani_water_bottle",
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

      // Write result to binary file
      std::string result_filename = write_directory + "/frame-" + frame_prefix.str() + "." + all_object_names[curr_object_idx] + ".mask.bin";
      std::ofstream out_file(result_filename, std::ios::binary | std::ios::out);
      for (int i = 0; i < frame_height * frame_width; i++)
        out_file.write((char*)&predMap_object[i], sizeof(float));
      out_file.close();
    }
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
  priv_nh.param("service_mode", service_mode, READ_SENSOR_NO_FORWARD_PASS);
  priv_nh.param("camera_service_name", camera_service_name, std::string("/realsense_camera"));
  priv_nh.param("read_directory", read_directory, std::string(""));
  priv_nh.param("write_directory", write_directory, std::string(""));
  priv_nh.param("net_directory", net_directory, std::string(""));

  // Assert parameters
  assert (!write_directory.empty());
  if (service_mode == READ_FILE_FORWARD_PASS)
    assert (!read_directory.empty());

  // Create a data folder to save RGB-D frames
  std::ifstream file(write_directory);
  if (file.fail())
    system(std::string("mkdir -p " + write_directory + "/raw").c_str());

  // Start service
  ros::ServiceServer service_detect = n.advertiseService(service_name, srv_detect);
  ROS_INFO("Writing data to directory: %s", write_directory.c_str());

  // Connect to Realsense camera
  if (service_mode == READ_SENSOR_FORWARD_PASS || service_mode == READ_SENSOR_NO_FORWARD_PASS) {
    ROS_INFO("Reading data from camera service: %s", camera_service_name.c_str());
    client_sensor = n.serviceClient<realsense_camera::StreamSensor>(camera_service_name);
  } else
    ROS_INFO("Reading data from directory: %s", read_directory.c_str());

  // Setup Marvin
  if (service_mode == READ_SENSOR_FORWARD_PASS || service_mode == READ_FILE_FORWARD_PASS) {
    ROS_INFO("Loading Marvin.");
    shelf_net.Malloc(marvin::Testing);
    tote_net.Malloc(marvin::Testing);
    shelf_net.loadWeights(shelf_net_weights_filename);
    tote_net.loadWeights(tote_net_weights_filename);
    color_data_CPU = new StorageT[frame_width * frame_height * 3];
    prob_CPU_StorageT = new StorageT[frame_width * frame_height * (num_apc_objects + 1)];
    prob_CPU_ComputeT = new ComputeT[frame_height * frame_width * (num_apc_objects + 1)];
  }

  ROS_INFO("Ready.");
  ros::spin();

  return 0;
}

