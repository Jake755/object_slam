/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
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

#include "FrameDrawer.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

#include "Converter.h"
#include "MapObject.h"
#include "Parameters.h"
#include "detect_3d_cuboid/object_3d_util.h"

using namespace std;

namespace ORB_SLAM2 {

FrameDrawer::FrameDrawer(Map *pMap) : mpMap(pMap) {
  mState = Tracking::SYSTEM_NOT_READY;
  mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  mIm_later = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));

  box_colors.push_back(cv::Scalar(255, 255, 0));
  box_colors.push_back(cv::Scalar(255, 0, 255));
  box_colors.push_back(cv::Scalar(0, 255, 255));
  box_colors.push_back(cv::Scalar(145, 30, 180));
  box_colors.push_back(cv::Scalar(210, 245, 60));
  box_colors.push_back(cv::Scalar(128, 0, 0));

  whether_keyframe = false;
}

cv::Mat FrameDrawer::DrawFrame() {
  cv::Mat im;
  vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
  vector<int>
      vMatches; // Initialization: correspondeces with reference keypoints
  vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
  vector<bool> vbVO, vbMap;          // Tracked MapPoints in current frame
  int state;                         // Tracking state

  // for debug visualization
  vector<cv::KeyPoint> vCurrentKeys_inlastframe;
  vector<cv::Point2f> vfeaturesklt_lastframe;
  vector<cv::Point2f> vfeaturesklt_thisframe;

  // Copy variables within scoped mutex
  {
    unique_lock<mutex> lock(mMutex);
    state = mState;
    if (mState == Tracking::SYSTEM_NOT_READY)
      mState = Tracking::NO_IMAGES_YET;

    mIm.copyTo(im);

    if (mState == Tracking::NOT_INITIALIZED) {
      vCurrentKeys = mvCurrentKeys;
      vCurrentKeys_inlastframe = mvCurrentKeys_inlastframe;
      vfeaturesklt_lastframe = mvfeaturesklt_lastframe;
      vfeaturesklt_thisframe = mvfeaturesklt_thisframe;
      vIniKeys = mvIniKeys;
      vMatches = mvIniMatches;
    } else if (mState == Tracking::OK) {
      vCurrentKeys = mvCurrentKeys;
      vCurrentKeys_inlastframe = mvCurrentKeys_inlastframe;
      vfeaturesklt_lastframe = mvfeaturesklt_lastframe;
      vfeaturesklt_thisframe = mvfeaturesklt_thisframe;
      vbVO = mvbVO;
      vbMap = mvbMap;
    } else if (mState == Tracking::LOST) {
      vCurrentKeys = mvCurrentKeys;
      vCurrentKeys_inlastframe = mvCurrentKeys_inlastframe;
      vfeaturesklt_lastframe = mvfeaturesklt_lastframe;
      vfeaturesklt_thisframe = mvfeaturesklt_thisframe;
    }
  } // destroy scoped mutex -> release mutex

  if (im.channels() < 3) // this should be always true
    cvtColor(im, im, CV_GRAY2BGR);

  // Draw
  if (state == Tracking::NOT_INITIALIZED) // INITIALIZING
  {
    for (unsigned int i = 0; i < vMatches.size(); i++) {
      if (vMatches[i] >= 0) {
        cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt,
                 cv::Scalar(0, 255, 0));
      }
    }
  } else if (state == Tracking::OK) // TRACKING
  {
    mnTracked = 0;
    mnTrackedVO = 0;
    const float r = 5; // rectangle width
    for (int i = 0; i < N; i++) {
      if (vbVO[i] || vbMap[i]) // matched to map, VO point (rgbd/stereo)
      {
        cv::Point2f pt1, pt2;
        pt1.x = vCurrentKeys[i].pt.x - r;
        pt1.y = vCurrentKeys[i].pt.y - r;
        pt2.x = vCurrentKeys[i].pt.x + r;
        pt2.y = vCurrentKeys[i].pt.y + r;

        if (vbMap[i]) // This is a match to a MapPoint in the map    // green
        {
          cv::circle(im, vCurrentKeys[i].pt, 3, cv::Scalar(0, 255, 0), -1);
          mnTracked++;
        } else // This is match to a "visual odometry" MapPoint created in the
               // last frame    // blue
        {
          cv::circle(im, vCurrentKeys[i].pt, 3, cv::Scalar(255, 0, 0), -1);
          mnTrackedVO++;
        }

        if (associate_point_with_object &&
            (point_Object_AssoID.size() > 0)) // red object points
          if (point_Object_AssoID[i] > -1)
            cv::circle(im, vCurrentKeys[i].pt, 4,
                       box_colors[point_Object_AssoID[i] % box_colors.size()],
                       -1);

        if (vCurrentKeys_inlastframe.size() > 0 &&
            !(vCurrentKeys_inlastframe[i].pt.x == 0 &&
              vCurrentKeys_inlastframe[i].pt.y == 0))
          cv::line(im, vCurrentKeys[i].pt, vCurrentKeys_inlastframe[i].pt,
                   cv::Scalar(0, 0, 255), 2);
      } else // if not matched to map points. discarded points.
      {
        // cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,255),-1);
        // //magenda
      }
    }

    for (size_t i = 0; i < vfeaturesklt_thisframe.size(); i++) {
      if (!(vfeaturesklt_thisframe[i].x == 0 &&
            vfeaturesklt_thisframe[i].y == 0)) {
        cv::Scalar LineColor = cv::Scalar(0, 0, 255);
        LineColor = cv::Scalar(rand() % (int)(255 + 1), rand() % (int)(255 + 1),
                               rand() % (int)(255 + 1));
        cv::line(im, vfeaturesklt_thisframe[i], vfeaturesklt_lastframe[i],
                 LineColor, 2);
      }
    }
  }

  // draw ground pts
  for (size_t i = 0; i < potential_ground_fit_inds.size(); i++) {
    if (vbVO[i] || vbMap[i]) {
      cv::circle(im, vCurrentKeys[potential_ground_fit_inds[i]].pt, 2,
                 cv::Scalar(0, 0, 255), -1);
    }
  }
  cv::Mat imWithInfo;
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}

void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText) {
  stringstream s;
  if (nState == Tracking::NO_IMAGES_YET)
    s << " WAITING FOR IMAGES";
  else if (nState == Tracking::NOT_INITIALIZED)
    s << " TRYING TO INITIALIZE ";
  else if (nState == Tracking::OK) {
    if (!mbOnlyTracking)
      s << "SLAM MODE |  ";
    else
      s << "LOCALIZATION | ";
    int nKFs = mpMap->KeyFramesInMap();
    int nMPs = mpMap->MapPointsInMap();
    s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
    if (mnTrackedVO > 0)
      s << ", + VO matches: " << mnTrackedVO;
  } else if (nState == Tracking::LOST) {
    s << " TRACK LOST. TRYING TO RELOCALIZE ";
  } else if (nState == Tracking::SYSTEM_NOT_READY) {
    s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
  }

  int baseline = 0;
  cv::Size textSize =
      cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

  imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
  im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
  imText.rowRange(im.rows, imText.rows) =
      cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
  cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
}

cv::Mat FrameDrawer::DrawFrameLater() {
  cv::Mat imLater;
  vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
  int state;                         // Tracking state

  {
    unique_lock<mutex> lock(mMutex);
    state = mState;
    if (mState == Tracking::SYSTEM_NOT_READY)
      mState = Tracking::NO_IMAGES_YET;

    mIm_later.copyTo(imLater);
  }

  if (state == Tracking::OK) // TRACKING
  {
    vCurrentKeys = mvCurrentKeysLater;

    mnTracked = 0;
    mnTrackedVO = 0;
    const float r = 5; // rectangle width

    int normal_dots_limit = 0;
    int obj_dots_limit = 0;
    int normal_dots_max = 300;
    int obj_dots_max = 500;
    bool for_paper = false;
    for (int i = 0; i < vCurrentKeys.size(); i++) {
      if (mvbMapLater[i]) {
        if (normal_dots_limit < normal_dots_max) {
          //                    cv::circle(imLater, vCurrentKeys[i].pt, 2,
          //                    cv::Scalar(0, 255, 0), 2);
        }
        if (associate_point_with_object &&
            (point_Object_AssoID.size() > 0)) // red object points
          if (point_Object_AssoID[i] > -1) {
            if (obj_dots_limit < obj_dots_max) {
              cv::circle(imLater, vCurrentKeys[i].pt, 2,
                         box_colors[point_Object_AssoID[i] % box_colors.size()],
                         2);
              if (for_paper) {
                obj_dots_limit++;
              }
            }
          } else {
            if (for_paper) {
              normal_dots_limit++;
            }
          }
      }
    }
  }

  if (whether_detect_object) // draw object box
  { // better to write some warning that if it is keyframe or not, because I
    // only detect cuboid for keyframes.
    //        for (size_t i = 0; i < bbox_2ds.size(); i++)
    //        {
    //            cv::rectangle(imLater, bbox_2ds[i], box_colors[i %
    //            box_colors.size()], 2); // 2d bounding box.
    //        }

    // for normal
    for (size_t i = 0; i < pKFobjs.size(); i++) {
      cv::rectangle(imLater, pKFobjs[i]->bbox_2d,
                    box_colors[i % box_colors.size()], 2); // 2d bounding box.

      char text[256];
      sprintf(text, "%s %.1f%% id:%d", class_names[pKFobjs[i]->label],
              pKFobjs[i]->prob * 100, pKFobjs[i]->object_id_in_localKF);

      int baseLine = 0;
      cv::Size label_size =
          cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

      int x = pKFobjs[i]->bbox_2d.x;
      int y = pKFobjs[i]->bbox_2d.y - label_size.height - baseLine;
      if (y < 0)
        y = 0;
      if (x + label_size.width > imLater.cols)
        x = imLater.cols - label_size.width;

      //            cv::rectangle(imLater, cv::Rect(cv::Point(x, y),
      //            cv::Size(label_size.width, label_size.height + baseLine)),
      //                          cv::Scalar(255, 255, 255), -1);
      //
      //            cv::putText(imLater, text, cv::Point(x, y +
      //            label_size.height),
      //                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0,
      //                        0));

      // draw mask
      //            for (int k = 0; k < imLater.rows; k++)
      //            {
      //                const uchar* mp = pKFobjs[i]->mask.ptr(k);
      //                uchar* p = imLater.ptr(k);
      //                for (int l = 0; l < imLater.cols; l++)
      //                {
      //                    if (mp[l] == 255)
      //                    {
      //                        p[0] = cv::saturate_cast<uchar>(p[0] * 0.5 +
      //                        box_colors[i % box_colors.size()][0] * 0.5);
      //                        p[1] = cv::saturate_cast<uchar>(p[1] * 0.5 +
      //                        box_colors[i % box_colors.size()][1] * 0.5);
      //                        p[2] = cv::saturate_cast<uchar>(p[2] * 0.5 +
      //                        box_colors[i % box_colors.size()][2] * 0.5);
      //                    }
      //                    p += 3;
      //                }
      //            }
    }
    // for yolact
    //        for (size_t i = 0; i < yolact_objs.size(); i++)
    //        {
    //            cv::rectangle(imLater, yolact_objs[i].rect, box_colors[i %
    //            box_colors.size()], 2); // 2d bounding box.
    //
    //            yolact::Object obj=yolact_objs[i];
    //            char text[256];
    //            sprintf(text, "%s %.1f%%", class_names[obj.label], obj.prob *
    //            100);
    //
    //            int baseLine = 0;
    //            cv::Size label_size = cv::getTextSize(text,
    //            cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    //
    //            int x = obj.rect.x;
    //            int y = obj.rect.y - label_size.height - baseLine;
    //            if (y < 0)
    //                y = 0;
    //            if (x + label_size.width > imLater.cols)
    //                x = imLater.cols - label_size.width;
    //
    //            cv::rectangle(imLater, cv::Rect(cv::Point(x, y),
    //            cv::Size(label_size.width, label_size.height + baseLine)),
    //                          cv::Scalar(255, 255, 255), -1);
    //
    //            cv::putText(imLater, text, cv::Point(x, y +
    //            label_size.height),
    //                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0,
    //                        0));
    //        }
  }

  return imLater;
}

void FrameDrawer::UpdateLater(KeyFrame *pKF, Tracking *pTracker) {
  if (!enable_viewimage)
    return;
  point_Object_AssoID.clear();
  yolact_objs.clear();
  pKFobjs.clear();
  mvCurrentKeysLater.clear();
  mvpMapPointsLater.clear();
  pKFobjs = pKF->local_cuboids;
  if (associate_point_with_object)
    point_Object_AssoID = pKF->keypoint_associate_objectID;

  pKF->raw_color.copyTo(mIm_later); // 刷新关键帧
  mvCurrentKeysLater = pKF->mvKeys;
  mvpMapPointsLater = pKF->GetMapPoints();

  point_Object_AssoID = pKF->keypoint_associate_objectID;
  yolact_objs = pKF->yolact_objs;
  curPKFid = pKF->mnId;
  mvbMapLater = vector<bool>(mvCurrentKeysLater.size(), false);
  for (std::set<MapPoint *>::iterator iter = mvpMapPointsLater.begin();
       iter != mvpMapPointsLater.end(); iter++) {
    if ((*iter) != nullptr) {
      if (!(*iter)->isBad()) {
        if ((*iter)->GetIndexInKeyFrame(pKF) < mvbMapLater.size() &&
            (*iter)->GetIndexInKeyFrame(pKF) >= 0) {
          mvbMapLater[(*iter)->GetIndexInKeyFrame(pKF)] = true;
        }
      }
    }
  }
}

void FrameDrawer::Update(Tracking *pTracker) {
  if (!enable_viewimage)
    return;

  unique_lock<mutex> lock(mMutex);
  pTracker->mImGray.copyTo(mIm);
  mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
  N = mvCurrentKeys.size();
  mvbVO = vector<bool>(N, false);
  mvbMap = vector<bool>(N, false);
  mbOnlyTracking = pTracker->mbOnlyTracking;

  potential_ground_fit_inds.clear();
  current_frame_id = int(pTracker->mCurrentFrame.mnId);

  // for visualization
  mvCurrentKeys_inlastframe = pTracker->mCurrentFrame.mvpMapPoints_inlastframe;
  mvfeaturesklt_lastframe = pTracker->mCurrentFrame.featuresklt_lastframe;
  mvfeaturesklt_thisframe = pTracker->mCurrentFrame.featuresklt_thisframe;

  if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED) {
    mvIniKeys = pTracker->mInitialFrame.mvKeys; // deep copy... takes time
    mvIniMatches = pTracker->mvIniMatches;
  } else if (pTracker->mLastProcessedState == Tracking::OK) {
    for (int i = 0; i < N; i++) {
      MapPoint *pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
      if (pMP) {
        if (!pTracker->mCurrentFrame.mvbOutlier[i]) {
          if (pMP->Observations() > 0) // if it is already added to map points
            mvbMap[i] = true;
          else
            mvbVO[i] =
                true; // associated map points, but not yet add observation???
        }
      }
    }
  }
  mState = static_cast<int>(pTracker->mLastProcessedState);

  if (enable_ground_height_scale) {
    if (pTracker->mCurrentFrame.mpReferenceKF !=
        NULL) // mCurrentFrame.mpReferenceKF
      if ((pTracker->mCurrentFrame.mnId -
           pTracker->mCurrentFrame.mpReferenceKF->mnFrameId) <
          1) // if current frame is a keyframe
      {
        potential_ground_fit_inds =
            pTracker->mCurrentFrame.mpReferenceKF->ground_region_potential_pts;
      }
  }
}

} // namespace ORB_SLAM2
