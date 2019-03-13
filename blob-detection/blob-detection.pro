QT += core
QT -= gui

TARGET = blob-detection
CONFIG += console
QMAKE_CXXFLAGS += -std=c++11
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    CBgSubtractor.cpp

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../Desktop/Git/librealsense/build/release/ -lrealsense
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../Desktop/Git/librealsense/build/debug/ -lrealsense
else:unix:!macx: LIBS += -L$$PWD/../Desktop/Git/librealsense/build/ -lrealsense

#INCLUDEPATH += $$PWD/../Desktop/Git/librealsense
#DEPENDPATH += $$PWD/../Desktop/Git/librealsense


#INCLUDEPATH += $$PWD/../Desktop/Git/opencv
#DEPENDPATH += $$PWD/../Desktop/Git/opencv

#unix {
#    CONFIG += link_pkgconfig
#    PKGCONFIG += opencv
#}

INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/local/include/opencv2
INCLUDEPATH += /usr/local/include/opencv4

LIBS += -L/usr/local/lib \
-lopencv_core \
-lopencv_imgproc \
-lopencv_highgui \
-lopencv_ml \
-lopencv_video \
-lopencv_features2d \
-lopencv_calib3d \
-lopencv_objdetect \
-lopencv_flann \
-lopencv_imgcodecs \
-lopencv_videoio \

HEADERS += \
    CBgSubtractor.h

