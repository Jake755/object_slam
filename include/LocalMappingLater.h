/**
 * This file is part of object slam
 */

#ifndef SRC_LOCALMAPPINGLATER_H
#define SRC_LOCALMAPPINGLATER_H

#include "KeyFrameDatabase.h"
#include "MapPoint.h"

#include <mutex>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2 {
class Tracking;
class LoopClosing;
class Map;
class KeyFrame;

class LocalMappingLater {
public:
  LocalMappingLater(Map *pMap, const float bMonocular);

  void SetLoopCloser(LoopClosing *pLoopCloser);

  void SetTracker(Tracking *pTracker);

  // Main function
  void Run();
  bool RunMappingIteration();

  void bundle_adjustment_caller();
  void bundle_adjustment_caller_obj();

  void InsertKeyFrame(KeyFrame *pKF);

  // Thread Synch
  void RequestStop();
  void RequestReset();
  bool Stop();
  void Release();
  bool isStopped();
  bool stopRequested();
  bool AcceptKeyFrames(); // whether want to accept keyframes, whether idle
  void SetAcceptKeyFrames(bool flag);
  bool SetNotStop(bool flag);

  void InterruptBA();

  void RequestFinish();
  bool isFinished();

  int KeyframesInQueue() {
    std::unique_lock<std::mutex> lock(mMutexNewKFs);
    return mlNewKeyFrames.size();
  }

protected:
  bool CheckNewKeyFrames();
  void ProcessNewKeyFrame(); // update keyframe, compute Bow, insert points,
                             // frames if any.
  // triangulate with previous keyframes to create map points. only way for
  // mono. some points for stere/rgbd. new triangulated points added to map. be
  // put in recent buffer points, will be checked in recent frames.
  void CreateNewMapPoints();

  void
  MapPointCulling(); // strict map points check, remove recent bad mappoints.
  void SearchInNeighbors();

  void KeyFrameCulling();

  cv::Mat ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);

  cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

  bool mbMonocular;

  void ResetIfRequested();
  bool mbResetRequested;
  std::mutex mMutexReset;

  bool CheckFinish();
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  Map *mpMap;

  LoopClosing *mpLoopCloser;
  Tracking *mpTracker;

  std::list<KeyFrame *> mlNewKeyFrames;

  KeyFrame *mpCurrentKeyFrame;

  std::list<MapPoint *> mlpRecentAddedMapPoints;

  std::mutex mMutexNewKFs;

  bool mbAbortBA;

  bool mbStopped;
  bool mbStopRequested;
  bool mbNotStop;
  std::mutex mMutexStop;

  bool mbAcceptKeyFrames;
  std::mutex mMutexAccept;
};

} // namespace ORB_SLAM2

#endif // SRC_LOCALMAPPINGLATER_H
