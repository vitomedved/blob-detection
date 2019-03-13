#ifndef CBGSUBTRACTOR_H
#define CBGSUBTRACTOR_H

#include "opencv2/opencv.hpp"

class CBgSubtractor
{
public:
    enum SubtractorType
    {
        MOG2,
        KNN
    };

    CBgSubtractor(CBgSubtractor::SubtractorType type, std::string windowName = "BgSubtractor");

    CBgSubtractor(CBgSubtractor::SubtractorType type, int history, int varThreshold, bool shadows, std::string windowName = "BgSubtractor");

    cv::Mat Apply(cv::Mat& frame);

private:
    cv::Ptr<cv::BackgroundSubtractor> m_bgSubtractor;
    std::string m_windowName;
};

#endif // CBGSUBTRACTOR_H
