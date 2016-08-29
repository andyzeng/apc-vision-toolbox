#include <opencv2/opencv.hpp>

// Depth images are read and written as 16-bit PNG files, with 
// depth in decimillimeters (10^-4 meters) and circularly 
// bitshifted to the right by 3 bits. 
// 
// WriteDepth takes a row-major order float array of depth values 
// and saves it as a frame_height x frame_width depth image using 
// OpenCV. The filename is defined in depth_file.
//
// ---------------------------------------------------------
// Copyright (c) 2016, Andy Zeng
// 
// This file is part of the APC Vision Toolbox and is available 
// under the terms of the Simplified BSD License provided in 
// LICENSE. Please retain this notice and LICENSE if you use 
// this file (or any portion of it) in your project.
// ---------------------------------------------------------

void WriteDepth(const std::string &depth_file, float * depth_values, int frame_height, int frame_width) {
	cv::Mat depth_mat(frame_height, frame_width, CV_16UC1);
	for (size_t y = 0; y < frame_height; y++)
		for (size_t x = 0; x < frame_width; x++) {
			unsigned short depth_short = (unsigned short)(depth_values[y * frame_width + x] * 10000);
			depth_short = (depth_short >> 13 | depth_short << 3);
			depth_mat.at<unsigned short>(y, x) = depth_short;
		}
	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);
	cv::imwrite(depth_file, depth_mat, compression_params);
}