
#include <stdlib.h>
#include <stdint.h>
#include <iostream>
#include <thread>

#include "librealsense/rs.hpp"
#include "librealsense/rscore.hpp"

//#include "opencv2/opencv.hpp"

#include "CBgSubtractor.h"

//#define MOG2_DEPTH 1
//#define MOG2_COLOR 1

//#define KNN_DEPTH 1
//#define KNN_COLOR 1


void DepthCallback(rs::frame frame);

void ColorCallback(rs::frame frame);


#ifdef MOG2_DEPTH
    // 500, 16, true are default const values
    CBgSubtractor MOG2Depth(CBgSubtractor::SubtractorType::MOG2, "BgSub MOG2 (depth)");
#endif

#ifdef MOG2_COLOR
    // 500, 16, true are default const values
    CBgSubtractor MOG2Color(CBgSubtractor::SubtractorType::MOG2, "BgSub MOG2 (color)");
#endif

#ifdef KNN_DEPTH
    // 500, 400, true are default const values
    CBgSubtractor KNNDepth(CBgSubtractor::SubtractorType::KNN, "BgSub KNN (depth)");
#endif

#ifdef KNN_COLOR
    // 500, 400, true are default const values
    CBgSubtractor KNNColor(CBgSubtractor::SubtractorType::KNN, "BgSub KNN (color)");
#endif

int main(int argc, char *argv[]) try
{
    cv::namedWindow("Color", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Normalized depth", cv::WINDOW_AUTOSIZE);

    rs::context ctx;
    // Exit application if there are no devices connected. TODO: while true and check if devices count changes.
    if(ctx.get_device_count() == 0)
    {
        printf("There are no devices connected.\nExiting application...\n");
        return EXIT_FAILURE;
    }

    // For now get the first device, there might be more than 1 connected.
    rs::device *device = ctx.get_device(0);

    device->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);
    device->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 30);

    // Get device info before starting device.
    rs::intrinsics depth_intrinsics, color_intrinsics;
    rs::format depth_format, color_format;
    depth_intrinsics = device->get_stream_intrinsics(rs::stream::depth);
    depth_format = device->get_stream_format(rs::stream::depth);
    color_intrinsics = device->get_stream_intrinsics(rs::stream::color);
    color_format = device->get_stream_format(rs::stream::color);

    printf("\nUsing device 0, an %s\n", device->get_name());
    printf("\tSerial number: %s\n", device->get_serial());
    printf("\tFirmware version: %s\n", device->get_firmware_version());

    device->set_frame_callback(rs::stream::color, ColorCallback);
    device->set_frame_callback(rs::stream::depth, DepthCallback);

    device->start();

    while(true);

    device->stop();

    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("\t%s\n", e.what());
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void DepthCallback(rs::frame frame)
{
    uint16_t w = frame.get_width();
    uint16_t h = frame.get_height();

    cv::Mat depth(cv::Size(w, h), CV_16UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);

#ifdef MOG2_DEPTH
    MOG2Depth.Apply(depth);
#endif

#ifdef KNN_DEPTH
    KNNDepth.Apply(depth);
#endif

    //depth.convertTo(depth, CV_8U);
    cv::normalize(depth, depth, 100, 255, cv::NORM_MINMAX, CV_8UC1);

    //cv::bitwise_not(depth, depth);

    cv::imshow("Normalized depth", depth);
    cv::waitKey(1);
}

void ColorCallback(rs::frame frame)
{
    uint16_t w = frame.get_width();
    uint16_t h = frame.get_height();

    cv::Mat colorImage(cv::Size(w, h), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);

#ifdef MOG2_COLOR
    MOG2Color.Apply(colorImage);
#endif

#ifdef KNN_COLOR
    KNNColor.Apply(colorImage);
#endif

    cv::imshow("Color", colorImage);
    cv::waitKey(1);
}


