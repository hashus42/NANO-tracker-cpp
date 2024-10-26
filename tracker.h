//
// Created by pardusumsu on 06.10.2024.
//

#ifndef NANO_TRACKER_CPP_TRACKER_H
#define NANO_TRACKER_CPP_TRACKER_H


#include "iostream"

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

class tracker {
public:
    tracker();
    ~tracker();
    void processFrame();
    static double calculateDistance(cv::Point2f p1, cv::Point2f p2);
    [[nodiscard]] cv::Rect fixRoi(cv::Rect roi) const;
    void similarityValue(cv::Mat first_rect, cv::Mat second_rect);
private:
    cv::Mat frame_show;
    cv::Mat frame_prc;
    cv::Mat prev_rect;
    cv::Mat current_rect;
    cv::Mat combined_rects;
    cv::Mat prev_curr_diff;
    cv::Mat black;
    cv::Mat all_combined;
    cv::Rect roi;
    cv::Ptr<cv::TrackerNano> tracker_NANO = cv::TrackerNano::create();
    cv::Point2f prevCenter;
    cv::Point2f currentCenter;
    cv::Mat hist1, hist2;
    cv::Mat base_rect;

    bool init = false;
    bool paused = false;
    bool roiSelected = false;
    bool success;
    bool targetLost = false;

    int ctr = 0;
    int lostCount = 0;

    double distance_moved;
    double similarity;

    std::vector<cv::Point2f> points;

    int histSize = 50;
    float range[2] = {0, 256};
    const float* histRange = {range};
    bool uniform = true;
    bool accumulate = false;

    cv::KalmanFilter kf = cv::KalmanFilter(4, 2, 0);
};


#endif //NANO_TRACKER_CPP_TRACKER_H
