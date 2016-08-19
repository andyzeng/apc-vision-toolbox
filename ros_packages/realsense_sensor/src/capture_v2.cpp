#include <ros/ros.h>
#include "realsense_camera/StreamSensor.h"
#include "display.hpp"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <thread>  // for sleep

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

template<typename T1, typename T>
void SetArrayFromScalars(T1& a, T x0,T x1,T x2,T x3,T x4,T x5,T x6,T x7,T x8)
{a[0] = x0; a[1] = x1;  a[2] = x2;  a[3] = x3;  a[4] = x4;  a[5] = x5; 
 a[6] = x6; a[7] = x7;  a[8] = x8; }
template<typename T1, typename T>
void SetArrayFromScalars(T1& a, T x0,T x1,T x2,T x3,T x4,T x5,T x6,T x7,T x8,T x9,T x10,T x11)
{a[0] = x0; a[1] = x1;  a[2] = x2;  a[3] = x3;  a[4] = x4;   a[5] = x5; 
 a[6] = x6; a[7] = x7;  a[8] = x8;  a[9] = x9;  a[10] = x10; a[11] = x11;}

typedef pcl::PointXYZRGB PointXYZRGBT;

typedef PointXYZRGBT PointType;

class RosRealsense{
private:
  // Setup ROS node handles
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  
  // Setup ROS Image transports
  image_transport::ImageTransport image_transport;
  image_transport::CameraPublisher rgb_image_pub;
  
  // Setup ROS pointcloud publisher
  ros::Publisher pointcloud_pub;
  
  // Setup ROS Service
  ros::ServiceServer service_capture;
  
  sensor_msgs::CameraInfo rgb_camera_info;
  unsigned int header_sequence_id = 0;
  ros::Time header_time_stamp;
  
  // configs
  int frame_width = 640;
  int frame_height = 480;
  std::string topic_rgb_image = "rgb/image";
  std::string topic_pointcloud = "pointcloud";
  std::string rgb_frame_id = "realsense_rgb_optical_frame";
  std::string depth_frame_id = "realsense_depth_optical_frame";
  bool display;
  
  // configs from realsense
  float depth_scale;
  rs::intrinsics depth_K;
  rs::extrinsics depth2color_ext;
  rs::intrinsics color_K;
  
  // buffers
  bool cloud_buffer_loaded = false;
  float * cloud_buffer_pts;
  uint8_t * cloud_buffer_rgb;
  uint16_t * cloud_buffer_depth;
  float * cloud_buffer_depth_raw;
  float * depth_cam_intrinsics;
  float * color_cam_intrinsics;
  float * depth2color_extrinsics;
  uint8_t * rgb_cvmat_buffer;
  cv::Mat rgb_cvmat;
  // buffer for pointcloud
  pcl::PointCloud<PointType>::Ptr realsense_xyzrgb_cloud;
    
  // glfw
  GLFWwindow * win_3D;
  GLFWwindow * win_2D;
  
  // connecting to realsense
  rs::context ctx;
  rs::device* pdev;
  
  void publish_rgb_image_msg(cv::Mat& rgb_mat)
  {
    sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

    rgb_img->header.seq = header_sequence_id;
    rgb_img->header.stamp = header_time_stamp;
    rgb_img->header.frame_id = rgb_frame_id;

    rgb_img->width = rgb_mat.cols;
    rgb_img->height = rgb_mat.rows;

    rgb_img->encoding = sensor_msgs::image_encodings::RGB8;
    rgb_img->is_bigendian = 0;

    int step = sizeof(unsigned char) * 3 * rgb_img->width;
    int size = step * rgb_img->height;
    rgb_img->step = step;
    rgb_img->data.resize(size);
    memcpy(&(rgb_img->data[0]), rgb_mat.data, size);


    rgb_camera_info.header.frame_id = rgb_frame_id;
    rgb_camera_info.header.stamp = header_time_stamp;
    rgb_camera_info.header.seq = header_sequence_id;
    
    rgb_image_pub.publish(*rgb_img, rgb_camera_info);
    
  }
  
  void publish_pointcloud2_msg()
  {
    for (int dy = 0; dy < depth_K.height; dy++) {
      for (int dx = 0; dx < depth_K.width; dx++) {
        int i = (depth_K.width * dy + dx);
        realsense_xyzrgb_cloud->points[i].x = cloud_buffer_pts[i*3+0];
        realsense_xyzrgb_cloud->points[i].y = cloud_buffer_pts[i*3+1];
        realsense_xyzrgb_cloud->points[i].z = cloud_buffer_pts[i*3+2];
        
        int r = cloud_buffer_rgb[i*3+0];
        int g = cloud_buffer_rgb[i*3+1];
        int b = cloud_buffer_rgb[i*3+2];
        realsense_xyzrgb_cloud->points[i].rgba = (0 << 24) | (r << 16) | (g << 8) | b;
      }
    }
    
    pcl::PCLPointCloud2 pcl_xyzrgb_pc2;
    pcl::toPCLPointCloud2 (*realsense_xyzrgb_cloud, pcl_xyzrgb_pc2);

    sensor_msgs::PointCloud2 realsense_xyzrgb_cloud2;
    pcl_conversions::moveFromPCL(pcl_xyzrgb_pc2, realsense_xyzrgb_cloud2);

    realsense_xyzrgb_cloud2.header.seq = header_sequence_id;
    realsense_xyzrgb_cloud2.header.stamp = header_time_stamp;
    realsense_xyzrgb_cloud2.header.frame_id = rgb_frame_id;  //registered to rgb

    pointcloud_pub.publish (realsense_xyzrgb_cloud2);
  }
public:
  RosRealsense(): priv_nh("~"), image_transport(nh) {
    // prepare buffers
    cloud_buffer_pts = new float[frame_width * frame_height * 3];
    cloud_buffer_rgb = new uint8_t[frame_width * frame_height * 3];
    cloud_buffer_depth = new uint16_t[frame_width * frame_height];
    cloud_buffer_depth_raw = new float[frame_width * frame_height];
    color_cam_intrinsics = new float[9];
    depth_cam_intrinsics = new float[9];
    depth2color_extrinsics = new float[12];
    
    rgb_cvmat_buffer = new uint8_t[frame_width * frame_height * 3];
    rgb_cvmat = cv::Mat(frame_height, frame_width, CV_8UC3, rgb_cvmat_buffer);
    
    // Get parameters from rosparam
    priv_nh.param("display", display, false);
    
    // Setup connection to realsense
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    if (ctx.get_device_count() == 0) throw std::runtime_error("No Realsense device detected. Is it plugged in?");
    pdev = ctx.get_device(0);
    rs::device& dev = *pdev;

    dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev.enable_stream(rs::stream::color, rs::preset::best_quality);
    dev.start();

    if(display) initGL();
    
    // Determine depth value corresponding to one meter
    depth_scale = dev.get_depth_scale();

    // Retrieve camera parameters for mapping between depth and color
    depth_K        = dev.get_stream_intrinsics(rs::stream::depth);
    depth2color_ext = dev.get_extrinsics(rs::stream::depth, rs::stream::color);
    color_K        = dev.get_stream_intrinsics(rs::stream::color);
    
    SetArrayFromScalars(color_cam_intrinsics, color_K.fx,    0.0f,        color_K.ppx,
                                              0.0f,          color_K.fy,  color_K.ppy,
                                              0.0f,          0.0f,        1.0f);
    
    SetArrayFromScalars(rgb_camera_info.K, color_K.fx, 0.0f,        color_K.ppx,
                                           0.0f,       color_K.fy,  color_K.ppy,
                                           0.0f,       0.0f,        1.0f);

    SetArrayFromScalars(depth_cam_intrinsics, depth_K.fx,    0.0f,        depth_K.ppx,
                                              0.0f,          depth_K.fy,  depth_K.ppy,
                                              0.0f,          0.0f,        1.0f);
      
    SetArrayFromScalars(depth2color_extrinsics, depth2color_ext.rotation[0], depth2color_ext.rotation[3], depth2color_ext.rotation[6], depth2color_ext.translation[0],
                                                depth2color_ext.rotation[1], depth2color_ext.rotation[4], depth2color_ext.rotation[7], depth2color_ext.translation[1],
                                                depth2color_ext.rotation[2], depth2color_ext.rotation[5], depth2color_ext.rotation[8], depth2color_ext.translation[2]);
    
    // Setup ROS service
    service_capture = nh.advertiseService("realsense_camera", &RosRealsense::srv_capture, this);
    
    // Setup ROS publisher
    rgb_image_pub = image_transport.advertiseCamera(topic_rgb_image, 1);
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_pointcloud, 1);
    
    // prepare buffer for pcl
    realsense_xyzrgb_cloud.reset(new pcl::PointCloud<PointType>());
    realsense_xyzrgb_cloud->width = color_K.width;
    realsense_xyzrgb_cloud->height = color_K.height;
    realsense_xyzrgb_cloud->is_dense = false;
    realsense_xyzrgb_cloud->points.resize(color_K.width * color_K.height);
    
    ROS_INFO("Ready.");
    
  }
  
  void initGL(){
    glfwInit();
    // Open a GLFW window for point cloud streaming
    win_3D = glfwCreateWindow(1280, 960, "Processed 3D Pointcloud", nullptr, nullptr);
    glfwSetCursorPosCallback(win_3D, on_cursor_pos);
    glfwSetMouseButtonCallback(win_3D, on_mouse_button);

    // Open a GLFW window for color + depth streaming
    std::ostringstream ss; ss << "RGB-D frame capture (" << pdev->get_name() << ")";
    win_2D = glfwCreateWindow(pdev->is_stream_enabled(rs::stream::infrared2) ? 1920 : 1280, 960, ss.str().c_str(), 0, 0);
    glfwMakeContextCurrent(win_2D);
  }
  
  void updatePointCloud(const uint16_t * depth_frame, const uint8_t * color_frame, const uint16_t * depth_frame_raw){
    rs::device& dev = *pdev;
    if(display) {
      // Set up a perspective transform in a space that we can rotate by clicking and dragging the mouse
      glfwMakeContextCurrent(win_3D);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      gluPerspective(60, (float)1280 / 960, 0.01f, 20.0f);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
      glTranslatef(0, 0, +0.5f);
      glRotated(pitch, 1, 0, 0);
      glRotated(yaw, 0, 1, 0);
      glTranslatef(0, 0, -0.5f);

      // We will render our depth data as a set of points in 3D space
      glPointSize(2);
      glEnable(GL_DEPTH_TEST);
      glBegin(GL_POINTS);
    }

    // Create 3D point cloud
    
    for (int dy = 0; dy < color_K.height; dy++) {
      for (int dx = 0; dx < color_K.width; dx++) {

        // Retrieve the 16-bit depth value and map it into a depth in meters
        uint16_t depth_value = depth_frame[dy * color_K.width + dx];
        float depth_in_meters = (float)depth_value * depth_scale;
        //cloud_buffer_depth[dy * depth_K.width + dx] = (uint16_t) round(depth_in_meters * 1000);

        uint16_t raw_depth_value = depth_frame_raw[dy * depth_K.width + dx];
        float raw_depth_in_meters = (float)raw_depth_value * depth_scale;
        cloud_buffer_depth_raw[dy * depth_K.width + dx] = raw_depth_in_meters; // assuming depth is within 0-6 meter

        // Add RGB data to global frame buffer
        cloud_buffer_rgb[(dy * color_K.width + dx) * 3 + 0] = color_frame[(dy * color_K.width + dx) * 3 + 0];
        cloud_buffer_rgb[(dy * color_K.width + dx) * 3 + 1] = color_frame[(dy * color_K.width + dx) * 3 + 1];
        cloud_buffer_rgb[(dy * color_K.width + dx) * 3 + 2] = color_frame[(dy * color_K.width + dx) * 3 + 2];

        // Skip over pixels with a depth value of zero, which is used to indicate no data
        if (depth_value == 0) {
          // Add empty point to global frame buffer
          cloud_buffer_pts[(dy * depth_K.width + dx) * 3 + 0] = 0;
          cloud_buffer_pts[(dy * depth_K.width + dx) * 3 + 1] = 0;
          cloud_buffer_pts[(dy * depth_K.width + dx) * 3 + 2] = 0;
          continue;
        }

        // Map from pixel coordinates in the depth image to pixel coordinates in the color image
        rs::float2 depth_pixel = {(float)dx, (float)dy};
        rs::float3 depth_point = color_K.deproject(depth_pixel, depth_in_meters);


        // Add XYZ point to global point cloud buffer
        cloud_buffer_pts[(dy * depth_K.width + dx) * 3 + 0] = depth_point.x;
        cloud_buffer_pts[(dy * depth_K.width + dx) * 3 + 1] = depth_point.y;
        cloud_buffer_pts[(dy * depth_K.width + dx) * 3 + 2] = depth_point.z;


        

        if(display) {
          // Color vertices
          glColor3ubv(color_frame + (dy * color_K.width + dx) * 3);
          // Emit a vertex at the 3D location of this depth pixel
          glVertex3f(depth_point.x, depth_point.y, depth_point.z);
        }
      }
    }
    cloud_buffer_loaded = true;

    static int tmp = 0;    
    if(tmp == 0)  //hack
      publish_pointcloud2_msg();
    tmp = (tmp + 1) % 5;
      
    
    if(display) {
      glEnd();
      glfwSwapBuffers(win_3D);
      glfwMakeContextCurrent(win_2D);

      // Clear the framebuffer
      int w, h;
      glfwGetFramebufferSize(win_2D, &w, &h);
      glViewport(0, 0, w, h);
      glClear(GL_COLOR_BUFFER_BIT);

      // Draw the images
      glPushMatrix();
      glfwGetWindowSize(win_2D, &w, &h);
      glOrtho(0, w, h, 0, -1, +1);
      int s = w / (dev.is_stream_enabled(rs::stream::infrared2) ? 3 : 2);
      buffers[0].show(dev, rs::stream::color, 0, 0, s, h - h / 2);
      buffers[1].show(dev, rs::stream::color_aligned_to_depth, s, 0, s, h - h / 2);
      buffers[2].show(dev, rs::stream::depth_aligned_to_color, 0, h / 2, s, h - h / 2);
      buffers[3].show(dev, rs::stream::depth, s, h / 2, s, h - h / 2);
      if (dev.is_stream_enabled(rs::stream::infrared2)) {
        buffers[4].show(dev, rs::stream::infrared2_aligned_to_depth, 2 * s, 0, s, h - h / 2);
        buffers[5].show(dev, rs::stream::depth_aligned_to_infrared2, 2 * s, h / 2, s, h - h / 2);
      }
      glPopMatrix();
      glfwSwapBuffers(win_2D);
    }
  }
  
  void stopGL(){
    // Clear windows
    glfwDestroyWindow(win_2D);
    glfwDestroyWindow(win_3D);
    glfwTerminate();
  }
  
  void run_one_iter(){
    // Wait for new images
    glfwPollEvents();
    rs::device& dev = *pdev;
    dev.wait_for_frames();

    // Retrieve depth data, which was previously configured as a 640 x 480 image of 16-bit depth values
    const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth_aligned_to_color));
    const uint16_t * depth_frame_raw = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth));

    // Retrieve color data, which was previously configured as a 640 x 480 x 3 image of 8-bit color values
    const uint8_t * color_frame = reinterpret_cast<const uint8_t *>(dev.get_frame_data(rs::stream::color));

    header_sequence_id++;
    header_time_stamp = ros::Time::now();
    
    // publish the frame
    memcpy(rgb_cvmat_buffer, color_frame, frame_width * frame_height * 3);
    // need some mutex mechanism
    publish_rgb_image_msg(rgb_cvmat);
    updatePointCloud(depth_frame, color_frame, depth_frame_raw);
  }
  
  void shutdown_realsense(){
    pdev->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "Finish shutdown_realsense." << std::endl;
  }
  
  ~RosRealsense(){
    if(display) stopGL();
    shutdown_realsense();
  }
    
  // Service: get data from sensor
  bool srv_capture(realsense_camera::StreamSensor::Request  &req,
                   realsense_camera::StreamSensor::Response &res) {
    ROS_INFO("Recieved request for point cloud and RGB data from sensor.");

    // Make sure buffer is not empty
    if (cloud_buffer_loaded) {

      // Save point cloud buffers to ROS message
      res.cloudXYZ.resize(frame_width * frame_height * 3);
      std::copy(cloud_buffer_pts, cloud_buffer_pts + (frame_width * frame_height * 3), res.cloudXYZ.begin());
      res.cloudRGB.resize(frame_width * frame_height * 3);
      std::copy(cloud_buffer_rgb, cloud_buffer_rgb + (frame_width * frame_height * 3), res.cloudRGB.begin());
      
      // Save raw depth to ROS message
      res.rawDepth.resize(frame_width * frame_height);
      std::copy(cloud_buffer_depth_raw, cloud_buffer_depth_raw + (frame_width * frame_height), res.rawDepth.begin());

      // Save intrinsics of color camera
      res.colorCamIntrinsics.resize(9);
      std::copy(color_cam_intrinsics, color_cam_intrinsics + 9, res.colorCamIntrinsics.begin());
      
      // Save intrinsics of depth camera
      res.depthCamIntrinsics.resize(9);
      std::copy(depth_cam_intrinsics, depth_cam_intrinsics + 9, res.depthCamIntrinsics.begin());
      
      // Save extrinsics of depth camera
      res.depth2colorExtrinsics.resize(12);
      std::copy(depth2color_extrinsics, depth2color_extrinsics + 12, res.depth2colorExtrinsics.begin());
		
      publish_pointcloud2_msg();
    }
    
    
    return true;
  }
  bool ok(){
    return display? (!glfwWindowShouldClose(win_2D) && !glfwWindowShouldClose(win_3D)): true;
  }
};

#include <signal.h>


int flag = true; 
void mySigintHandler(int sig)
{
  std::cout << "reqesting shut down." << std::endl;
  flag = false;
  
}

int main(int argc, char **argv) try{

  // Setup ROS
  ros::init(argc, argv, "realsense", ros::init_options::AnonymousName);
  
  RosRealsense rr;
  signal(SIGINT, mySigintHandler);
  ros::Rate r(30);
  while(ros::ok() && rr.ok() && flag){
    rr.run_one_iter();
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();
  std::cout << "End of main." << std::endl;

  return 0;
}
catch(const rs::error & e)
{
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch(const std::exception & e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
