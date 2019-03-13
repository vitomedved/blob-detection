
#include <stdlib.h>
#include <stdint.h>
#include <iostream>
#include <thread>
#include <stdio.h>

#include <ctime>

#include "librealsense/rs.hpp"
#include "librealsense/rscore.hpp"

//#include "opencv2/opencv.hpp"

#include "CBgSubtractor.h"

//#define MOG2_DEPTH 1
#define MOG2_COLOR 1

//#define KNN_DEPTH 1
#define KNN_COLOR 1

#define PLAYBACK 1
//#define RECORDING 1




#ifdef PLAYBACK
#define PLAYBACK_PATH "../build-blob-detection-Desktop-Debug/04_run.avi"
#endif

void DepthCallback(rs::frame frame);

void ColorCallback(rs::frame frame);

bool isRecording = false;

#ifdef RECORDING
#define RECORDING_FPS 30
std::string timestamp = "05_run.avi";
cv::VideoWriter video(timestamp, CV_FOURCC('H','F','Y','U'), RECORDING_FPS, cv::Size(640, 480));
#endif

#ifdef MOG2_DEPTH
    // 500, 16, true are default const values
    CBgSubtractor MOG2Depth(CBgSubtractor::SubtractorType::MOG2, "BgSub MOG2 (depth)");
#endif

#ifdef MOG2_COLOR
    ///CBgSubtractor(CBgSubtractor::SubtractorType type, int history, int varThreshold, bool shadows, std::string windowName = "BgSubtractor");
    // 500, 16, true are default const values

    // Working fine
    //CBgSubtractor MOG2Color(CBgSubtractor::SubtractorType::MOG2, 1000, 10, false, "BgSub MOG2 (color)");

    CBgSubtractor MOG2Color(CBgSubtractor::SubtractorType::MOG2, 6000, 10, false, "BgSub MOG2 (color)");
#endif

#ifdef KNN_DEPTH
    // 500, 400, true are default const values
    CBgSubtractor KNNDepth(CBgSubtractor::SubtractorType::KNN, "BgSub KNN (depth)");
#endif

#ifdef KNN_COLOR
    ///CBgSubtractor(CBgSubtractor::SubtractorType type, int history, int varThreshold, bool shadows, std::string windowName = "BgSubtractor");
    // 500, 400, true are default const values

    // Working fine
    //CBgSubtractor KNNColor(CBgSubtractor::SubtractorType::KNN, 1000, 500, true, "BgSub KNN (color)");

    CBgSubtractor KNNColor(CBgSubtractor::SubtractorType::KNN, 6000, 500, true, "BgSub KNN (color)");

#endif

int main(int argc, char *argv[]) try
{
    cv::namedWindow("Color", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Normalized depth", cv::WINDOW_AUTOSIZE);

#ifdef PLAYBACK
    cv::VideoCapture capture(PLAYBACK_PATH);

    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 5000;
    params.maxArea = 100000;
    params.filterByCircularity = false;
    params.filterByColor = false;
    params.filterByConvexity = false;
    params.filterByInertia = false;

    cv::Ptr<cv::SimpleBlobDetector> blobDetector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Point2f> points;

    // Does not work.
    //capture.set(CV_CAP_PROP_FPS, 10);

    if(!capture.isOpened())
    {
        std::cout << "Video could not be opened." << std::endl;
        return -1;
    }

    while(true)
    {
        cv::Mat frame;
        capture >> frame;

        if(frame.empty())
        {
            capture.release();
            capture.open(PLAYBACK_PATH);
            continue;
        }

        cv::Mat img1;
        cv::Mat img2;
        cv::Mat merged;

#ifdef MOG2_COLOR
        img1 = MOG2Color.Apply(frame);
#endif

#ifdef KNN_COLOR
        img2 = KNNColor.Apply(frame);
#endif

#if defined(MOG2_COLOR) && defined(KNN_COLOR)
    // Merge 2 images together
    double alpha = 0.5;
    double beta = 1.0 - alpha;
    addWeighted( img1, alpha, img2, beta, 0.0, merged);

    // Threshold images to get white blobs
    cv::threshold(merged, merged, 10, 255, cv::THRESH_BINARY_INV);

    blobDetector->detect(merged, keypoints);
    cv::Mat imWithKeypoints;
    cv::drawKeypoints(merged, keypoints, imWithKeypoints, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    for(int i = 0; i < keypoints.size(); i++)
    {
        points.push_back(keypoints[i].pt);
    }

    for(int i = 1; i < points.size(); i++)
    {
        cv::line(imWithKeypoints, points[i-1], points[i], cv::Scalar(0, 0, 255), 5, cv::LINE_AA, 0);
    }

    cv::imshow("Linear blend", imWithKeypoints);
    cv::waitKey(1);
#endif

        cv::imshow("Color", frame);
        cv::waitKey(1);

    }
#endif
#ifdef RECORDING
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




    while(true)
    {
        //std::cout << cv::waitKey(0) << std::endl;
        if(cv::waitKey(0) == 114)
        {
            isRecording = !isRecording;
            std::cout << "UP button pressed. Starting/stopping to record." << std::endl;

            if(!isRecording)
            {
                video.release();
            }
        }
    }

    device->stop();

#endif
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

    cv::Mat img1;
    cv::Mat img2;
    cv::Mat merged;

#ifdef MOG2_COLOR
    img1 = MOG2Color.Apply(colorImage);
#endif

#ifdef KNN_COLOR
    img2 = KNNColor.Apply(colorImage);
#endif

#if defined(MOG2_COLOR) && defined(KNN_COLOR)
    cv::hconcat(img1, img2, merged);
    cv::imshow("Merged", merged);
    cv::waitKey(1);
#endif


#ifdef RECORDING
    if(isRecording)
    {
        video.write(colorImage);
    }
#endif


    cv::imshow("Color", colorImage);
    cv::waitKey(1);
}


