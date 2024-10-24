//
// Created by pardusumsu on 06.10.2024.
//

#include "tracker.h"

tracker::tracker() {
    cv::namedWindow("NANO_tracker_cpp", cv::WINDOW_AUTOSIZE);
}

void tracker::processFrame() {
    double fps;
    cv::VideoCapture cap("../../tracker-tester-py/data/RIAT2018.mp4");

    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open camera\n";
        return;
    }

    while (true) {
        int64 time_start = cv::getTickCount();
        cap >> frame_prc;
        if (frame_prc.empty()) break;
//        cv::flip(frame_prc, frame_prc, 1);
//        cv::resize(frame_prc, frame_prc, cv::Size(640, 480), 0.5, 0.5);
        frame_prc.copyTo(frame_show);
        if (!roiSelected) {
            roi = cv::selectROI("NANO_tracker_cpp", frame_show, false, false);
            roiSelected = true;
        }
        if (roi.empty()) {
            std::cerr << "Roi is empty." << std::endl;
            break;
        } else if (!init) {
            tracker_NANO->init(frame_prc, roi);
            init = true;
        }
        try {
            // prev roi processes
            roi = fixRoi(roi);
            prevCenter = cv::Point2f(roi.x + roi.width / 2, roi.y + roi.height / 2);
            if (roi.x - (roi.width / 2) > 0 && roi.y - (roi.height / 2) > 0 && roi.x + (roi.width / 2) < frame_prc.cols && roi.y + (roi.height / 2) < frame_prc.rows) {
                prev_rect = frame_prc(cv::Rect (roi.x - (roi.width / 2), roi.y - (roi.height / 2), roi.width * 2, roi.height * 2));
            } else {
                prev_rect = frame_prc(cv::Rect (roi.x, roi.y, roi.width, roi.height));
            }
            success = tracker_NANO->update(frame_prc, roi);

            // after-update roi processes
            roi = fixRoi(roi);
            currentCenter = cv::Point2f(roi.x + roi.width / 2, roi.y + roi.height / 2);
            if (roi.x - (roi.width / 2) > 0 && roi.y - (roi.height / 2) > 0 && roi.x + (roi.width / 2) < frame_prc.cols && roi.y + (roi.height / 2) < frame_prc.rows) {
                current_rect = frame_prc(cv::Rect (roi.x - (roi.width / 2), roi.y - (roi.height / 2), roi.width * 2, roi.height * 2));
            } else {
                current_rect = frame_prc(cv::Rect (roi.x, roi.y, roi.width, roi.height));
            }

            // Draw track line
            points.push_back(currentCenter);
            for (int i = 0; i < points.size() - 1; ++i) {
                cv::line(frame_show, points[i], points[i + 1], cv::Scalar(0, 0, 255), 2);
            }

            // resize and find differences
            cv::resize(prev_rect, prev_rect, current_rect.size(), 0, 0, cv::INTER_LINEAR);
            cv::absdiff(prev_rect, current_rect, prev_curr_diff);

            // concat everything together
            cv::vconcat(prev_rect, current_rect, combined_rects);
            cv::vconcat(combined_rects, prev_curr_diff, combined_rects);
            if (combined_rects.rows < frame_show.rows) {
                black = cv::Mat(cv::Size(prev_rect.cols,
                                         frame_show.rows - prev_rect.rows - current_rect.rows - prev_curr_diff.rows),
                                CV_8UC3, cv::Scalar(0, 0, 0));
                cv::vconcat(combined_rects, black, combined_rects);
            }
            if (combined_rects.rows > frame_show.rows) {
                combined_rects = combined_rects(cv::Rect(0, 0, combined_rects.cols, frame_show.rows));
            }
            cv::hconcat(frame_show, combined_rects, frame_show);
        } catch (const cv::Exception& e) {
            std::cerr << "Error: (" << roi.x << ", " << roi.y << ", " << roi.width << ", " << roi.height << ")" << std::endl;
            std::cerr << "Exception: " << e.what() << std::endl;
        }

        if (success) {
            distance_moved = calculateDistance(prevCenter, currentCenter);
            cv::putText(frame_show, "FPS: " + std::to_string((int)fps), cv::Point(10, 30),
                        cv::FONT_HERSHEY_COMPLEX, 1.0, CV_RGB(255, 255, 0));
            cv::putText(frame_show, "Distance moved: " + std::to_string((int)distance_moved) + " pixels", cv::Point(10, 60),
                        cv::FONT_HERSHEY_COMPLEX, 1.0, CV_RGB(0, 0, 0));
            cv::rectangle(frame_show, roi, cv::Scalar(0, 255, 0), 2, 1); // green
            fps = cv::getTickFrequency() / (cv::getTickCount() - time_start);

            if ((int)distance_moved == 0) {
                lostCount++;
            }
            if (ctr < 60) {
                ctr++;
            } else {
                if (lostCount > 10) {
                    targetLost = true;
                    paused = true;
                }
                lostCount = 0;
                ctr = 0;
            }
            if (targetLost) {
                cv::putText(frame_show, "Aircraft lost", cv::Point(10, 90),
                            cv::FONT_HERSHEY_COMPLEX, 1.0, CV_RGB(255, 0, 0));
            }
            cv::imshow("NANO_tracker_cpp", frame_show);
        }

        int key = cv::waitKey(1) & 0xFF;
        if (key == 27) break;
        else if (key == 'q') {
            roiSelected = false;
            init = false;
            targetLost = false;
            points.clear();
        }
        else if (key == ' ') {
            paused = !paused;
        }
        while (paused) {
            key = cv::waitKey(1) & 0xFF;
            if (key == ' ') {
                paused = !paused;
                if (targetLost) {
                    roiSelected = false;
                    init = false;
                    targetLost = false;
                    points.clear();
                }
            }
            else if (key == 27) {
                paused = false;
                break;
            }
        }
    }
}

double tracker::calculateDistance(cv::Point2f p1, cv::Point2f p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

cv::Rect tracker::fixRoi(cv::Rect roi) const {
    if (roi.x < 0) {
        roi.x = 1;
    }
    if (roi.y < 0) {
        roi.y = 1;
    }
    if (roi.x + roi.width > frame_prc.cols) {
        roi.width -= ((roi.x + roi.width) - frame_prc.cols) + 1;
    }
    if (roi.y + roi.height > frame_prc.rows) {
        roi.height -= ((roi.y + roi.height) - frame_prc.rows) + 1;
    }
    if (roi.width > frame_prc.cols) {
        roi.width = frame_prc.cols;
    } else if (roi.width < 0) {
        roi.width = 1;
    }
    if (roi.height > frame_prc.rows) {
        roi.height = frame_prc.rows;
    } else if (roi.height < 0) {
        roi.height = 1;
    }
    return roi;
}

void tracker::similarityValue(cv::Mat first_rect, cv::Mat second_rect) {
    cv::cvtColor(first_rect, first_rect, cv::COLOR_BGR2HSV);
    cv::cvtColor(second_rect, second_rect, cv::COLOR_BGR2HSV);
    //
}

tracker::~tracker() = default;