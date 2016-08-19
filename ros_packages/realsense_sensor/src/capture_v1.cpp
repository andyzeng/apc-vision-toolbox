#include "ros/ros.h"
#include "realsense_camera/StreamSensor.h"
#include "display.hpp"

// Global buffers for sensor data retrieval
int frame_width = 640;
int frame_height = 480;
bool cloud_buffer_loaded = false;
float * cloud_buffer_pts = new float[frame_width * frame_height * 3];
uint8_t * cloud_buffer_rgb = new uint8_t[frame_width * frame_height * 3];
float * buffer_raw_depth = new float[frame_width * frame_height];
float * color_cam_intrinsics = new float[9];
float * depth_cam_intrinsics = new float[9];
float * depth2color_extrinsics = new float[12];
bool display;

// Service: stream data from sensor
bool srv_stream(realsense_camera::StreamSensor::Request  &req,
                realsense_camera::StreamSensor::Response &res) {
  ROS_INFO("Recieved request for point cloud and RGB data from sensor.");

  // Make sure buffer is not empty
  if (cloud_buffer_loaded) {

    // Save point cloud buffers to ROS message
    for (int i = 0; i < frame_width * frame_height * 3; i++) {
      res.cloudXYZ.push_back(cloud_buffer_pts[i]);
      res.cloudRGB.push_back(cloud_buffer_rgb[i]);
    }

    // Save raw depth buffer to ROS message
    for (int i = 0; i < frame_width * frame_height; i++)
      res.rawDepth.push_back(buffer_raw_depth[i]);

    // Save intrinsics and extrinsics of depth and color cameras
    for (int i = 0; i < 9; i++) {
      res.colorCamIntrinsics.push_back(color_cam_intrinsics[i]);
      res.depthCamIntrinsics.push_back(depth_cam_intrinsics[i]);
      res.depth2colorExtrinsics.push_back(depth2color_extrinsics[i]);
    }
  }
  return true;
}

template<typename T1, typename T>
void SetArrayFromScalars(T1& a, T x0,T x1,T x2,T x3,T x4,T x5,T x6,T x7,T x8)
{a[0] = x0; a[1] = x1;  a[2] = x2;  a[3] = x3;  a[4] = x4;  a[5] = x5; 
 a[6] = x6; a[7] = x7;  a[8] = x8; }
template<typename T1, typename T>
void SetArrayFromScalars(T1& a, T x0,T x1,T x2,T x3,T x4,T x5,T x6,T x7,T x8,T x9,T x10,T x11)
{a[0] = x0; a[1] = x1;  a[2] = x2;  a[3] = x3;  a[4] = x4;   a[5] = x5; 
 a[6] = x6; a[7] = x7;  a[8] = x8;  a[9] = x9;  a[10] = x10; a[11] = x11;}

int main(int argc, char **argv) {

  // Setup ROS
  ros::init(argc, argv, "realsense_camera");
  ros::NodeHandle n;
  ros::ServiceServer service_stream = n.advertiseService("realsense_camera", srv_stream);
  ROS_INFO("Ready.");

  // Get parameters from rosparam
  ros::NodeHandle priv_nh("~");
  priv_nh.param("display", display, false);

  try {
    rs::log_to_console(rs::log_severity::warn);

    rs::context ctx;
    if (ctx.get_device_count() == 0) throw std::runtime_error("No Realsense device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev.enable_stream(rs::stream::color, rs::preset::best_quality);
    dev.start();

    GLFWwindow * win_3D;
    GLFWwindow * win_2D;
    if (display) {
      // Open a GLFW window for point cloud streaming
      glfwInit();
      win_3D = glfwCreateWindow(1280, 960, "Processed 3D Pointcloud", nullptr, nullptr);
      glfwSetCursorPosCallback(win_3D, on_cursor_pos);
      glfwSetMouseButtonCallback(win_3D, on_mouse_button);

      // Open a GLFW window for color + depth streaming
      std::ostringstream ss; ss << "RGB-D frame capture (" << dev.get_name() << ")";
      win_2D = glfwCreateWindow(dev.is_stream_enabled(rs::stream::infrared2) ? 1920 : 1280, 960, ss.str().c_str(), 0, 0);
      glfwMakeContextCurrent(win_2D);
    }

    // Determine depth value corresponding to one meter
    float depth_scale = dev.get_depth_scale();
    // const uint16_t one_meter = static_cast<uint16_t>(1.0f / depth_scale);

    // Retrieve camera parameters for mapping between depth and color
    rs::intrinsics color_K = dev.get_stream_intrinsics(rs::stream::color);
    rs::intrinsics depth_K = dev.get_stream_intrinsics(rs::stream::depth);
    rs::extrinsics depth2color_ext = dev.get_extrinsics(rs::stream::depth, rs::stream::color);
    SetArrayFromScalars(color_cam_intrinsics, color_K.fx,    0.0f,        color_K.ppx,
                                              0.0f,          color_K.fy,  color_K.ppy,
                                              0.0f,          0.0f,        1.0f);
    SetArrayFromScalars(depth_cam_intrinsics, depth_K.fx,    0.0f,        depth_K.ppx,
                                              0.0f,          depth_K.fy,  depth_K.ppy,
                                              0.0f,          0.0f,        1.0f);
    SetArrayFromScalars(depth2color_extrinsics, depth2color_ext.rotation[0], depth2color_ext.rotation[3], depth2color_ext.rotation[6], depth2color_ext.translation[0],
                                                depth2color_ext.rotation[1], depth2color_ext.rotation[4], depth2color_ext.rotation[7], depth2color_ext.translation[1],
                                                depth2color_ext.rotation[2], depth2color_ext.rotation[5], depth2color_ext.rotation[8], depth2color_ext.translation[2]);
    
    if (display) {
      while (ros::ok() && !glfwWindowShouldClose(win_2D) && !glfwWindowShouldClose(win_3D)) {

        // Wait for new images
        glfwPollEvents();
        dev.wait_for_frames();

        // Retrieve depth data, which was previously configured as a 640 x 480 image of 16-bit depth values
        const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth_aligned_to_color));
        const uint16_t * raw_depth_frame = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth));

        // Retrieve color data, which was previously configured as a 640 x 480 x 3 image of 8-bit color values
        const uint8_t * color_frame = reinterpret_cast<const uint8_t *>(dev.get_frame_data(rs::stream::color));

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

        // Create 3D point cloud
        for (int dy = 0; dy < depth_K.height; dy++) {
          for (int dx = 0; dx < depth_K.width; dx++) {

            // Retrieve the raw and aligned-to-color depth value and map it into a depth in meters
            uint16_t depth_value = depth_frame[dy * depth_K.width + dx];
            float depth_in_meters = depth_value * depth_scale;
            uint16_t raw_depth_value = raw_depth_frame[dy * depth_K.width + dx];
            buffer_raw_depth[dy * depth_K.width + dx] = raw_depth_value * depth_scale;

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
            rs::float3 depth_point = depth_K.deproject(depth_pixel, depth_in_meters);

            // Color vertices
            glColor3ubv(color_frame + (dy * color_K.width + dx) * 3);

            // Add XYZ point to global point cloud buffer
            cloud_buffer_pts[(dy * depth_K.width + dx) * 3 + 0] = depth_point.x;
            cloud_buffer_pts[(dy * depth_K.width + dx) * 3 + 1] = depth_point.y;
            cloud_buffer_pts[(dy * depth_K.width + dx) * 3 + 2] = depth_point.z;

            // Emit a vertex at the 3D location of this depth pixel
            glVertex3f(depth_point.x, depth_point.y, depth_point.z);
          }
        }
        cloud_buffer_loaded = true;
        glEnd();
        glfwSwapBuffers(win_3D);

        // ROS loop
        ros::spinOnce();

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

      // Clear windows
      glfwDestroyWindow(win_2D);
      glfwDestroyWindow(win_3D);
      glfwTerminate();
    } else {
      while (ros::ok()) {

        // Wait for new images
        dev.wait_for_frames();

        // Retrieve depth data, which was previously configured as a 640 x 480 image of 16-bit depth values
        const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth_aligned_to_color));
        const uint16_t * raw_depth_frame = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth));

        // Retrieve color data, which was previously configured as a 640 x 480 x 3 image of 8-bit color values
        const uint8_t * color_frame = reinterpret_cast<const uint8_t *>(dev.get_frame_data(rs::stream::color));

        // Create 3D point cloud
        for (int dy = 0; dy < depth_K.height; dy++) {
          for (int dx = 0; dx < depth_K.width; dx++) {

            // Retrieve the raw and aligned-to-color depth value and map it into a depth in meters
            uint16_t depth_value = depth_frame[dy * depth_K.width + dx];
            float depth_in_meters = depth_value * depth_scale;
            uint16_t raw_depth_value = raw_depth_frame[dy * depth_K.width + dx];
            buffer_raw_depth[dy * depth_K.width + dx] = raw_depth_value * depth_scale;

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
            rs::float3 depth_point = depth_K.deproject(depth_pixel, depth_in_meters);

            // Add XYZ point to global point cloud buffer
            cloud_buffer_pts[(dy * depth_K.width + dx) * 3 + 0] = depth_point.x;
            cloud_buffer_pts[(dy * depth_K.width + dx) * 3 + 1] = depth_point.y;
            cloud_buffer_pts[(dy * depth_K.width + dx) * 3 + 2] = depth_point.z;
          }
        }
        cloud_buffer_loaded = true;

        // ROS loop
        ros::spinOnce();
      }
    }

  } catch (const rs::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return 0;
}
