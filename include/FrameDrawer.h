/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Eigen/Dense"
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>

#include "yolact.h"
#include "set"

namespace ORB_SLAM2
{

class Tracking;
class Viewer;
class MapPoint;
class MapObject;
class Map;
class KeyFrame;

class FrameDrawer
{
public:
    FrameDrawer(Map *pMap);

    // Update info from the last processed frame. Not just keyframes. Called by Tracker() after every frame
    void Update(Tracking *pTracker);
    void UpdateLater(KeyFrame *pKF,Tracking *pTracker);

    // Draw last processed frame.  called by separate Viewer thread.
    cv::Mat DrawFrame();
    cv::Mat DrawFrameLater();

protected:
    // by me  for object
    std::vector<cv::Rect> bbox_2ds;                // yolo detected 2D bbox_2d, which has 3D cuboid
    std::vector<Eigen::Matrix2Xi> box_corners_2ds; //2*8 corners on object.
    std::vector<Eigen::MatrixXi> edge_markers_2ds; // in order to plot 2d cuboids with 8 corners.
    std::vector<int> truth2d_trackid;              // truth 2d track id

    std::vector<int> potential_ground_fit_inds;
    int current_frame_id = -1;
    int saved_img_id = -1;

    // just for debug visualization
    std::vector<cv::KeyPoint> mvCurrentKeys_inlastframe;
    std::vector<cv::Point2f> mvfeaturesklt_lastframe;
    std::vector<cv::Point2f> mvfeaturesklt_thisframe;

    std::vector<cv::Scalar> box_colors; // fixed
    std::vector<int> point_Object_AssoID;
    bool whether_keyframe;
    cv::Mat cam_pose_cw; // camera pose, world to cam
    Eigen::Matrix3d Kalib;

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm;
    cv::Mat mIm_later;
    int N; // number of current frame features.
    std::vector<cv::KeyPoint> mvCurrentKeys;
    std::vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    std::vector<cv::KeyPoint> mvIniKeys;
    std::vector<int> mvIniMatches;
    int mState;

    Map *mpMap;

    std::mutex mMutex;

    //lin755 for Later
    std::vector<bool> mvbMapLater;
    std::set<MapPoint *> mvpMapPointsLater;
    std::vector<cv::KeyPoint> mvCurrentKeysLater;
    std::vector<MapObject *> pKFobjs;
    std::vector<yolact::Object> yolact_objs;
    int curPKFid;

    const char* class_names[81] = {"background",
                                        "person", "bicycle", "car", "motorcycle", "airplane", "bus",
                                        "train", "truck", "boat", "traffic light", "fire hydrant",
                                        "stop sign", "parking meter", "bench", "bird", "cat", "dog",
                                        "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
                                        "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
                                        "skis", "snowboard", "sports ball", "kite", "baseball bat",
                                        "baseball glove", "skateboard", "surfboard", "tennis racket",
                                        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
                                        "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
                                        "hot dog", "pizza", "donut", "cake", "chair", "couch",
                                        "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
                                        "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
                                        "toaster", "sink", "refrigerator", "book", "clock", "vase",
                                        "scissors", "teddy bear", "hair drier", "toothbrush"
    };
};

} // namespace ORB_SLAM2

#endif // FRAMEDRAWER_H
