/**
* This file is part of ORB-SLAM2.
* 地图 管理 关键帧 地图点
*  添加/删除  关键帧/地图点
*/

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{
// 初始 最大关键帧id = 0  最大变化id = 0
Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);// set 集合 插入关键帧
    if(pKF->mnId > mnMaxKFid)// 关键帧 id
        mnMaxKFid=pKF->mnId;// 最大关键帧 的id
}

void Map::AddMapPoint(MapPoint *pMP)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);// set 集合 插入 地图点
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    
    // wyw 添加
  //      auto  it = mspMapPoints.find(pMP);// 获取指针
  //      delete *it;//删除
    
    mspMapPoints.erase(pMP);// 删除了指针  内容清除？

    // TODO: This only erase the pointer.
    // Delete the MapPoint

}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    
        // wyw 添加
   //     auto  it = mspKeyFrames.find(pKF);// 获取指针
   //     delete *it;//删除
        
    mspKeyFrames.erase(pKF);// 删除了指针  内容清除？

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mnBigChangeIdx++;// id++
}

int Map::GetLastBigChangeIdx()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

// 返回所有关键帧
std::vector<KeyFrame*> Map::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

// 返回所有地图点
std::vector<MapPoint*> Map::GetAllMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

// 地图中地图点的数量
long unsigned int Map::MapPointsInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspMapPoints.size();
}
// 地图中 关键帧数量
long unsigned int Map::KeyFramesInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}
// 地图中 参考地图点
std::vector<MapPoint*> Map::GetReferenceMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}
// 最大 id的 关键帧 的 id
long unsigned int Map::GetMaxKFid()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(std::set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;// 删除地图点内容

    for(std::set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;// 删除 关键帧

    mspMapPoints.clear();//清楚全部
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
