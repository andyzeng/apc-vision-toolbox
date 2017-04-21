#include "random_utils.h"
#include "depth_utils.h"
#include <librealsense/rs.hpp>
#include "display.hpp"

#include <iomanip>

int main(int argc, char * argv[]) try {

    // Check Realsense device
    rs::log_to_console(rs::log_severity::warn);
    rs::context rs_ctx;
    if (rs_ctx.get_device_count() == 0)
        throw std::runtime_error("No Realsense device detected.");
    rs::device & rs_dev = *rs_ctx.get_device(0);

    // Start Realsense device
    rs_dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    rs_dev.enable_stream(rs::stream::color, rs::preset::best_quality);
    try { rs_dev.enable_stream(rs::stream::infrared2, rs::preset::best_quality); } catch (...) {}
    rs_dev.start();

    // Open a GLFW window for rendering
    glfwInit();
    std::ostringstream win_name;
    win_name << "RGB-D Frame Capture (" << rs_dev.get_name() << ")";
    GLFWwindow * win = glfwCreateWindow(rs_dev.is_stream_enabled(rs::stream::infrared2) ? 1920 : 1280, 960, win_name.str().c_str(), 0, 0);
    glfwMakeContextCurrent(win);

    // Create a data folder (with a random hash name) to save frames
    std::string hash_name = GetRandomString(16);
    std::string data_path = "data/" + hash_name + "/";
    system(("mkdir -p " + data_path + "raw").c_str());

    // Determine depth value corresponding to one meter
    const uint16_t meter_scale = static_cast<uint16_t>(1.0f / rs_dev.get_depth_scale());

    // Save camera intrinsics of color sensor
    std::string cam_info_file = data_path + "cam.info.txt";
    FILE *fp = fopen(cam_info_file.c_str(), "w");
    rs::intrinsics color_K = rs_dev.get_stream_intrinsics(rs::stream::color);
    fprintf(fp, "# Color camera intrinsic matrix\n");
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", color_K.fx, 0.0f, color_K.ppx);
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", 0.0f, color_K.fy, color_K.ppy);
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", 0.0f, 0.0f, 1.0f);

    // Save camera intrinsics of depth sensor
    rs::intrinsics depth_K = rs_dev.get_stream_intrinsics(rs::stream::depth);
    fprintf(fp, "\n# Depth camera intrinsic matrix\n");
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", depth_K.fx, 0.0f, depth_K.ppx);
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", 0.0f, depth_K.fy, depth_K.ppy);
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", 0.0f, 0.0f, 1.0f);

    // Save caemra-to-camera extrinsics from depth sensor to color sensor
    rs::extrinsics depth2color_ext = rs_dev.get_extrinsics(rs::stream::depth, rs::stream::color);
    fprintf(fp, "\n# Depth-to-color camera extrinsic matrix\n");
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", depth2color_ext.rotation[0], depth2color_ext.rotation[3], depth2color_ext.rotation[6], depth2color_ext.translation[0]);
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", depth2color_ext.rotation[1], depth2color_ext.rotation[4], depth2color_ext.rotation[7], depth2color_ext.translation[1]);
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", depth2color_ext.rotation[2], depth2color_ext.rotation[5], depth2color_ext.rotation[8], depth2color_ext.translation[2]);
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", 0.0f, 0.0f, 0.0f, 1.0f);
    fclose(fp);

    // Load frame buffers
    int depth_frame_width = rs_dev.get_stream_width(rs::stream::depth_aligned_to_color);
    int depth_frame_height = rs_dev.get_stream_height(rs::stream::depth_aligned_to_color);
    float * depth_buffer_aligned = new float[depth_frame_height * depth_frame_width];
    float * depth_buffer_raw = new float[depth_frame_height * depth_frame_width];
    texture_buffer texture_buffers[6];

    // Stream RGB-D frames on the GLFW window
    int frame_idx = 0;
    int key_state = 0;
    while (!glfwWindowShouldClose(win)) {

        // Track spacebar key presses
        if (glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_PRESS && key_state == 0)
            key_state++;

        // Wait for new images
        glfwPollEvents();
        rs_dev.wait_for_frames();

        // Debug frame timestamps
        // std::cout << rs_dev.get_frame_timestamp(rs::stream::depth_aligned_to_color) << " " << rs_dev.get_frame_timestamp(rs::stream::color) << std::endl;

        // Retrieve depth data, which was previously configured as a 640 x 480 image of 16-bit depth values
        const uint16_t * depth_data_aligned = reinterpret_cast<const uint16_t *>(rs_dev.get_frame_data(rs::stream::depth_aligned_to_color));
        const uint16_t * depth_data_raw = reinterpret_cast<const uint16_t *>(rs_dev.get_frame_data(rs::stream::depth));

        // Retrieve color data, which was previously configured as a 640 x 480 x 3 image of 8-bit color values
        const uint8_t * color_data = reinterpret_cast<const uint8_t *>(rs_dev.get_frame_data(rs::stream::color));

        // If spacebar key is pressed, save the RGB-D frame
        if (glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_RELEASE && key_state >= 1) {
            key_state = 0;

            // Set frame filename
            std::ostringstream frame_name;
            frame_name << std::setw(6) << std::setfill('0') << frame_idx;
            frame_idx++;

            // Save aligned depth frame to disk
            for (size_t y = 0; y < depth_frame_height; y++)
                for (size_t x = 0; x < depth_frame_width; x++) {
                    depth_buffer_aligned[y * depth_frame_width + x] = ((float)depth_data_aligned[y * depth_frame_width + x]) / ((float)meter_scale);
                }
            std::string depth_file_aligned = data_path + "frame-" + frame_name.str() + ".depth.png";
            WriteDepth(depth_file_aligned, depth_buffer_aligned, depth_frame_height, depth_frame_width);

            // Save raw depth frame to disk
            for (size_t y = 0; y < depth_frame_height; y++)
                for (size_t x = 0; x < depth_frame_width; x++) {
                    depth_buffer_raw[y * depth_frame_width + x] = ((float)depth_data_raw[y * depth_frame_width + x]) / ((float)meter_scale);
                }
            std::string depth_file_raw = data_path + "raw/frame-" + frame_name.str() + ".depth.png";
            WriteDepth(depth_file_raw, depth_buffer_raw, depth_frame_height, depth_frame_width);

            // Save color frame to disk (RGB, 24-bit PNG)
            int color_frame_width = rs_dev.get_stream_width(rs::stream::color);
            int color_frame_height = rs_dev.get_stream_height(rs::stream::color);
            cv::Mat color_mat(color_frame_height, color_frame_width, CV_8UC3);
            for (int y = 0; y < color_frame_height; ++y)
                for (int x = 0; x < color_frame_width; ++x) {
                    cv::Vec3b& bgr_value = color_mat.at<cv::Vec3b>(y, x);
                    bgr_value[0] = color_data[y * color_frame_width * 3 + x * 3 + 2]; // Blue
                    bgr_value[1] = color_data[y * color_frame_width * 3 + x * 3 + 1]; // Green
                    bgr_value[2] = color_data[y * color_frame_width * 3 + x * 3 + 0]; // Red
                }
            std::vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9);
            std::string color_file = data_path + "frame-" + frame_name.str() + ".color.png";
            imwrite(color_file, color_mat, compression_params);
        }

        // Clear the GLFW framebuffer
        int win_width, win_height;
        glfwGetFramebufferSize(win, &win_width, &win_height);
        glViewport(0, 0, win_width, win_height);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw the RGB-D images on the GLFW window
        glPushMatrix();
        glfwGetWindowSize(win, &win_width, &win_height);
        glOrtho(0, win_width, win_height, 0, -1, +1);
        int s = win_width / (rs_dev.is_stream_enabled(rs::stream::infrared2) ? 3 : 2);
        texture_buffers[0].show(rs_dev, rs::stream::color, 0, 0, s, win_height - win_height / 2);
        texture_buffers[1].show(rs_dev, rs::stream::color_aligned_to_depth, s, 0, s, win_height - win_height / 2);
        texture_buffers[2].show(rs_dev, rs::stream::depth_aligned_to_color, 0, win_height / 2, s, win_height - win_height / 2);
        texture_buffers[3].show(rs_dev, rs::stream::depth, s, win_height / 2, s, win_height - win_height / 2);
        if (rs_dev.is_stream_enabled(rs::stream::infrared2)) {
            texture_buffers[4].show(rs_dev, rs::stream::infrared2_aligned_to_depth, 2 * s, 0, s, win_height - win_height / 2);
            texture_buffers[5].show(rs_dev, rs::stream::depth_aligned_to_infrared2, 2 * s, win_height / 2, s, win_height - win_height / 2);
        }
        glPopMatrix();
        glfwSwapBuffers(win);
    }

    glfwDestroyWindow(win);
    glfwTerminate();
    return EXIT_SUCCESS;
}
catch (const rs::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}