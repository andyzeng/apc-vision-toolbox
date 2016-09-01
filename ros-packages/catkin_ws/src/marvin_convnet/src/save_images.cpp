#include "depth_utils.h"
#include "ros/ros.h"
#include "marvin_convnet/DetectObjects.h"
#include "realsense_camera/StreamSensor.h"
#include <opencv2/opencv.hpp>

#include <fstream>

// Directory to write all RGB-D files and response maps
std::string write_directory;

// Global buffers for sensor data retrieval
int frame_width = 640;
int frame_height = 480;
float * cloud_buffer_pts = new float[frame_width * frame_height * 3];
uint8_t * cloud_buffer_rgb = new uint8_t[frame_width * frame_height * 3];
float * buffer_depth = new float[frame_width * frame_height];
float * buffer_raw_depth = new float[frame_width * frame_height];
float * color_cam_intrinsics = new float[9];
float * depth_cam_intrinsics = new float[9];
float * depth2color_extrinsics = new float[12];

std::string camera_service_name;
ros::ServiceClient client_sensor;

std::string shelf_bin_ids = "ABCDEFGHIJKL";

// Service call
bool srv_save(marvin_convnet::DetectObjects::Request  &req,
              marvin_convnet::DetectObjects::Response &res) {
  ROS_INFO("Recieved service request.");

  int bin_id = req.BinId;
  int frame_id = req.FrameId;
  res.FrameId = frame_id;

  // Create a data folder to save RGB-D frames
  std::ifstream file(write_directory + "/raw");
  if (file.fail())
    system(std::string("mkdir -p " + write_directory + "/raw").c_str());

  // Get frame filenames
  std::ostringstream frame_prefix;
  frame_prefix << std::setw(6) << std::setfill('0') << frame_id;
  std::string color_frame_filename = "/frame-" + frame_prefix.str() + ".color.png";
  std::string depth_frame_filename = "/frame-" + frame_prefix.str() + ".depth.png";
  std::string raw_depth_frame_filename = "/raw/frame-" + frame_prefix.str() + ".depth.png";

  // Retrieve RGB-D data from camera service and save to disk
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


  return true;
}

int main(int argc, char **argv) {

  // Setup ROS
  ros::init(argc, argv, "marvin_convnet", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  // Get service parameters
  priv_nh.param("camera_service_name", camera_service_name, std::string("/realsense_camera"));
  priv_nh.param("write_directory", write_directory, std::string(""));

  // Assert parameters
  assert (!write_directory.empty());

  // Start service
  ros::ServiceServer service_save = n.advertiseService("save_images", srv_save);
  ROS_INFO("Writing data to directory: %s", write_directory.c_str());

  // Connect to Realsense camera
  ROS_INFO("Reading data from camera service: %s", camera_service_name.c_str());
  client_sensor = n.serviceClient<realsense_camera::StreamSensor>(camera_service_name);

  ROS_INFO("Ready.");
  ros::spin();

  return 0;
}

