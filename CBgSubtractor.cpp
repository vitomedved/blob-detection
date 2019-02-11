#include "CBgSubtractor.h"

CBgSubtractor::CBgSubtractor(CBgSubtractor::SubtractorType type, std::string windowName)
{
    m_windowName = windowName;

    switch(type)
    {
    case MOG2:
        m_bgSubtractor = cv::createBackgroundSubtractorMOG2();
        break;
    case KNN:
        m_bgSubtractor = cv::createBackgroundSubtractorKNN();
        break;
    }

    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
}

CBgSubtractor::CBgSubtractor(CBgSubtractor::SubtractorType type, int history, int varThreshold, bool shadows, std::string windowName)
{
    m_windowName = windowName;

    switch(type)
    {
    case MOG2:
        m_bgSubtractor = cv::createBackgroundSubtractorMOG2(history, varThreshold, shadows);
        break;
    case KNN:
        m_bgSubtractor = cv::createBackgroundSubtractorKNN(history, varThreshold, shadows);
        break;
    }
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
}

void CBgSubtractor::Apply(cv::Mat& frame)
{
    cv::Mat fgFrame;
    m_bgSubtractor->apply(frame, fgFrame);


    fgFrame.convertTo(fgFrame, CV_8U);

    assert(!(m_windowName.length() == 0));

    cv::imshow(m_windowName, fgFrame);
}



