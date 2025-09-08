# ORB-SLAM2 SSD语义SLAM系统类图设计

## 核心类关系图

```mermaid
classDiagram
    %% 系统管理类
    class System {
        -eSensor mSensor
        -ORBVocabulary* mpVocabulary
        -KeyFrameDatabase* mpKeyFrameDatabase
        -Map* mpMap
        -Tracking* mpTracker
        -LocalMapping* mpLocalMapper
        -LoopClosing* mpLoopCloser
        -Viewer* mpViewer
        -shared_ptr~PointCloudMapping~ mpPointCloudMapping
        +TrackStereo(imLeft, imRight, timestamp)
        +TrackRGBD(im, depthmap, timestamp)
        +TrackMonocular(im, timestamp)
        +ActivateLocalizationMode()
        +DeactivateLocalizationMode()
        +Shutdown()
    }
    
    %% 跟踪类
    class Tracking {
        -eTrackingState mState
        -Frame mCurrentFrame
        -cv::Mat mImGray
        -cv::Mat mImDepth
        -cv::Mat mImRGB
        -shared_ptr~PointCloudMapping~ mpPointCloudMapping
        +GrabImageStereo(imRectLeft, imRectRight, timestamp)
        +GrabImageRGBD(imRGB, imD, timestamp)
        +GrabImageMonocular(im, timestamp)
        +Track()
        +Relocalization()
    }
    
    %% 局部建图类
    class LocalMapping {
        -list~KeyFrame*~ mlNewKeyFrames
        -bool mbAbortBA
        -bool mbStopped
        -bool mbStopRequested
        +Run()
        +InsertKeyFrame(pKF)
        +ProcessNewKeyFrame()
        +CreateNewMapPoints()
        +MapPointCulling()
        +KeyFrameCulling()
    }
    
    %% 闭环检测类
    class LoopClosing {
        -queue~KeyFrame*~ mlpLoopKeyFrameQueue
        -bool mbResetRequested
        -bool mbFinishRequested
        +Run()
        +InsertKeyFrame(pKF)
        +DetectLoop()
        +ComputeSim3()
        +CorrectLoop()
    }
    
    %% 地图类
    class Map {
        -set~MapPoint*~ mspMapPoints
        -set~KeyFrame*~ mspKeyFrames
        -vector~KeyFrame*~ mvpKeyFrameOrigins
        -long unsigned int mnMaxKFid
        +AddKeyFrame(pKF)
        +AddMapPoint(pMP)
        +EraseMapPoint(pMP)
        +EraseKeyFrame(pKF)
        +GetAllKeyFrames()
        +GetAllMapPoints()
    }
    
    %% 关键帧类
    class KeyFrame {
        -long unsigned int mnId
        -double mTimeStamp
        -cv::Mat Tcw
        -cv::Mat Ow
        -vector~MapPoint*~ mvpMapPoints
        -DBoW2::BowVector mBowVec
        +SetPose(Tcw)
        +GetPose()
        +GetCameraCenter()
        +AddMapPoint(pMP, idx)
        +EraseMapPointMatch(idx)
    }
    
    %% 地图点类
    class MapPoint {
        -long unsigned int mnId
        -cv::Mat mWorldPos
        -map~KeyFrame*, size_t~ mObservations
        -int mnVisible
        -int mnFound
        +SetWorldPos(Pos)
        +GetWorldPos()
        +AddObservation(pKF, idx)
        +EraseObservation(pKF)
        +GetObservations()
    }
    
    %% 帧类
    class Frame {
        -long unsigned int mnId
        -double mTimeStamp
        -cv::Mat mTcw
        -vector~cv::KeyPoint~ mvKeys
        -cv::Mat mDescriptors
        -vector~MapPoint*~ mvpMapPoints
        +ExtractORB(im)
        +SetPose(Tcw)
        +GetPose()
        +isInFrustum(pMP, viewingCosLimit)
    }
    
    %% 语义点云建图类
    class PointCloudMapping {
        -PointCloud::Ptr globalMap
        -shared_ptr~thread~ viewerThread
        -vector~KeyFrame*~ keyframes
        -vector~cv::Mat~ colorImgs
        -vector~cv::Mat~ depthImgs
        -vector~cv::Mat~ RGBImgs
        -shared_ptr~Detector~ ncnn_detector_ptr
        -vector~Cluster~ clusters
        +insertKeyFrame(kf, color, depth, imgRGB)
        +generatePointCloud(kf, color, depth)
        +viewer()
        +sem_merge(cluster)
        +add_cube()
    }
    
    %% 语义检测类
    class Detector {
        -ncnn::Net* det_net_ptr
        -ncnn::Mat* net_in_ptr
        +Run(bgr_img, objects)
        +Show(bgr_img, objects)
    }
    
    %% 检测对象类
    class Object {
        +cv::Rect_~float~ rect
        +string object_name
        +int class_id
        +float prob
    }
    
    %% 语义聚类类
    class Cluster {
        +string object_name
        +int class_id
        +float prob
        +Eigen::Vector3f minPt
        +Eigen::Vector3f maxPt
        +Eigen::Vector3f centroid
        +operator==(string x)
    }
    
    %% 可视化类
    class Viewer {
        -bool mbFinishRequested
        -bool mbFinished
        -bool mbStopped
        -bool mbStopRequested
        +Run()
        +RequestFinish()
        +RequestStop()
    }
    
    %% 关系定义
    System --> Tracking : 管理
    System --> LocalMapping : 管理
    System --> LoopClosing : 管理
    System --> Map : 管理
    System --> Viewer : 管理
    System --> PointCloudMapping : 管理
    
    Tracking --> Frame : 处理当前帧
    Tracking --> KeyFrame : 创建关键帧
    Tracking --> PointCloudMapping : 传递关键帧
    
    LocalMapping --> KeyFrame : 处理关键帧
    LocalMapping --> MapPoint : 创建地图点
    LocalMapping --> Map : 更新地图
    
    LoopClosing --> KeyFrame : 检测闭环
    LoopClosing --> Map : 全局优化
    
    Map --> KeyFrame : 存储关键帧
    Map --> MapPoint : 存储地图点
    
    KeyFrame --> MapPoint : 观测关系
    Frame --> MapPoint : 匹配关系
    
    PointCloudMapping --> Detector : 语义检测
    PointCloudMapping --> Cluster : 语义聚类
    Detector --> Object : 检测结果
```

## 语义增强模块类图

```mermaid
classDiagram
    %% 核心语义检测器
    class Detector {
        -ncnn::Net* det_net_ptr
        -ncnn::Mat* net_in_ptr
        -vector~string~ class_names
        -float conf_threshold
        -float nms_threshold
        +Detector()
        +~Detector()
        +Run(bgr_img, objects) cv::Mat&, vector~Object~&
        +Show(bgr_img, objects) cv::Mat&, vector~Object~&
        -preprocess(img) cv::Mat
        -postprocess(out, objects) ncnn::Mat, vector~Object~&
    }
    
    %% 检测结果对象
    class Object {
        +cv::Rect_~float~ rect
        +string object_name
        +int class_id
        +float prob
        +Object()
        +Object(rect, name, id, confidence)
        +bool operator>(const Object& other)
    }
    
    %% 语义聚类对象
    class Cluster {
        +string object_name
        +int class_id
        +float prob
        +Eigen::Vector3f minPt
        +Eigen::Vector3f maxPt
        +Eigen::Vector3f centroid
        +pcl::PointIndices indices
        +int point_count
        +float volume
        +Cluster()
        +bool operator==(const string& x)
        +void updateBounds(point)
        +float calculateVolume()
    }
    
    %% 点云建图核心类
    class PointCloudMapping {
        -PointCloud::Ptr globalMap
        -shared_ptr~thread~ viewerThread
        -condition_variable keyFrameUpdated
        -mutex keyFrameUpdateMutex
        -mutex keyframeMutex
        -mutex shutDownMutex
        -bool shutDownFlag
        -double resolution
        -pcl::VoxelGrid~PointT~ voxel
        -shared_ptr~Detector~ ncnn_detector_ptr
        -vector~Cluster~ clusters
        -pcl::ExtractIndices~PointT~ extract_indices
        -pcl::StatisticalOutlierRemoval~PointT~ stat
        -shared_ptr~pcl::visualization::PCLVisualizer~ pcl_viewer_prt
        +PointCloudMapping(resolution)
        +insertKeyFrame(kf, color, depth, imgRGB)
        +shutdown()
        +viewer()
        +update()
        -generatePointCloud(kf, color, depth) PointCloud::Ptr
        -draw_rect_with_depth_threshold(bgr, depth, rect, scalar, indices)
        -sem_merge(cluster)
        -add_cube()
    }
    
    %% 语义数据库
    class SemanticDatabase {
        -vector~Cluster~ semantic_objects
        -map~int, vector~Cluster*~~ objects_by_class
        -mutex database_mutex
        -float merge_distance_threshold
        -float merge_size_threshold
        +addObject(cluster)
        +removeObject(id)
        +findNearbyObjects(position, radius) vector~Cluster*~
        +getObjectsByClass(class_id) vector~Cluster*~
        +mergeObjects(cluster1, cluster2)
        +updateObject(cluster)
        +saveToFile(filename)
        +loadFromFile(filename)
    }
    
    %% 动态检测器
    class DynamicDetector {
        -cv::Mat prev_image
        -vector~cv::Point2f~ prev_points
        -vector~cv::Point2f~ curr_points
        -vector~uchar~ status
        -vector~float~ errors
        -float flow_threshold
        -float geometry_threshold
        +DynamicDetector()
        +detectDynamicPoints(curr_img, curr_frame) vector~bool~
        +opticalFlowDetection(prev_img, curr_img) vector~cv::Point2f~
        +geometryConsistencyCheck(points, frame) vector~bool~
        +semanticPriorFilter(objects, points) vector~bool~
    }
    
    %% 关系定义
    PointCloudMapping --> Detector : 使用
    PointCloudMapping --> Cluster : 创建和管理
    PointCloudMapping --> SemanticDatabase : 存储语义信息
    Detector --> Object : 产生检测结果
    Cluster --> Object : 基于检测结果创建
    SemanticDatabase --> Cluster : 管理语义对象
    DynamicDetector --> Object : 使用语义先验
```

## 可视化模块类图

```mermaid
classDiagram
    %% 主视图器
    class Viewer {
        -bool mbFinishRequested
        -bool mbFinished
        -bool mbStopped
        -bool mbStopRequested
        -mutex mMutexFinish
        -mutex mMutexStop
        -MapDrawer* mpMapDrawer
        -FrameDrawer* mpFrameDrawer
        -System* mpSystem
        +Viewer(pSystem, pFrameDrawer, pMapDrawer, strSettingPath)
        +Run()
        +RequestFinish()
        +RequestStop()
        +isFinished()
        +isStopped()
    }
    
    %% 帧绘制器
    class FrameDrawer {
        -Frame mCurrentFrame
        -vector~cv::KeyPoint~ mvCurrentKeys
        -vector~bool~ mvbMap
        -vector~bool~ mvbVO
        -bool mbOnlyTracking
        -int mnTracked
        -vector~cv::KeyPoint~ mvIniKeys
        -vector~int~ mvIniMatches
        -int mState
        +FrameDrawer(pMap)
        +Update(pTracker)
        +DrawFrame()
        -DrawTextInfo(im, nState, im)
    }
    
    %% 地图绘制器
    class MapDrawer {
        -Map* mpMap
        -float mKeyFrameSize
        -float mKeyFrameLineWidth
        -float mGraphLineWidth
        -float mPointSize
        -float mCameraSize
        -float mCameraLineWidth
        +MapDrawer(pMap, strSettingPath)
        +DrawMapPoints()
        +DrawKeyFrames(bDrawKF, bDrawGraph)
        +DrawCurrentCamera(Twc)
        +SetCurrentCameraPose(Twc)
        +GetCurrentOpenGLCameraMatrix()
    }
    
    %% PCL点云可视化器封装
    class PCLViewer {
        -shared_ptr~pcl::visualization::PCLVisualizer~ viewer
        -PointCloud::Ptr cloud
        -vector~Cluster~ semantic_objects
        -bool update_flag
        -mutex viewer_mutex
        +PCLViewer()
        +updateCloud(cloud)
        +updateSemanticObjects(objects)
        +addBoundingBox(cluster)
        +removeBoundingBox(id)
        +spinOnce()
        +close()
        -setupViewer()
        -updateDisplay()
    }
    
    %% 关系定义
    Viewer --> FrameDrawer : 使用
    Viewer --> MapDrawer : 使用
    FrameDrawer --> Frame : 绘制当前帧
    FrameDrawer --> KeyFrame : 绘制关键帧
    MapDrawer --> Map : 绘制地图
    MapDrawer --> KeyFrame : 绘制关键帧
    MapDrawer --> MapPoint : 绘制地图点
    PCLViewer --> PointCloudMapping : 显示点云
    PCLViewer --> Cluster : 显示语义对象
```

## 数据结构类图

```mermaid
classDiagram
    %% 核心数据结构
    class ORBVocabulary {
        +ORBVocabulary()
        +loadFromTextFile(filename) bool
        +loadFromBinaryFile(filename) bool
        +transform(features) BowVector
        +score(v1, v2) double
    }
    
    class KeyFrameDatabase {
        -vector~list~KeyFrame*~~ mvInvertedFile
        -mutex mMutex
        +KeyFrameDatabase(voc)
        +add(pKF)
        +erase(pKF)
        +clear()
        +DetectLoopCandidates(pKF, minScore) vector~KeyFrame*~
        +DetectRelocalizationCandidates(F) vector~KeyFrame*~
    }
    
    class ORBextractor {
        -int nfeatures
        -float scaleFactor
        -int nlevels
        -int iniThFAST
        -int minThFAST
        +ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST)
        +operator()(image, mask, keypoints, descriptors)
        +GetScaleFactor() float
        +GetScaleFactors() vector~float~
        +GetInverseScaleFactors() vector~float~
        +GetScaleSigmaSquares() vector~float~
        +GetInverseScaleSigmaSquares() vector~float~
        +GetImagePyramid() vector~Mat~
    }
    
    class ORBmatcher {
        -float mfNNratio
        -bool mbCheckOrientation
        +ORBmatcher(nnratio, checkOri)
        +SearchByProjection(F, vpMapPoints, th) int
        +SearchByBoW(pKF, F, vpMapPointMatches) int
        +SearchForInitialization(F1, F2, vbPrevMatched, vnMatches12, windowSize) int
        +SearchForTriangulation(pKF1, pKF2, F12, vMatchedPairs, bOnlyStereo) int
    }
    
    class Optimizer {
        +static BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust)
        +static GlobalBundleAdjustemnt(pMap, nIterations, pbStopFlag, nLoopKF, bRobust)
        +static LocalBundleAdjustment(pKF, pbStopFlag, pMap)
        +static PoseOptimization(pFrame) int
        +static OptimizeEssentialGraph(pMap, pLoopKF, pCurKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, bFixScale)
        +static OptimizeSim3(pKF1, pKF2, vpMatches1, g2oS12, th2, bFixScale) int
    }
    
    %% 初始化器
    class Initializer {
        -vector~cv::KeyPoint~ mvKeys1
        -vector~cv::KeyPoint~ mvKeys2
        -vector~cv::Point2f~ mvbMatched12
        -int mK
        -float mSigma
        -float mSigma2
        +Initializer(ReferenceFrame, sigma, iterations)
        +Initialize(CurrentFrame, vMatches12, R21, t21, vP3D, vbTriangulated) bool
        -FindHomography(vbMatchesInliers, score, H21) bool
        -FindFundamental(vbInliers, score, F21) bool
        -CheckRT(R, t, mvKeys1, mvKeys2, mvMatches12, vbInliers, K, vP3D, th2, vbGood, parallax) bool
    }
    
    %% 关系定义
    KeyFrameDatabase --> ORBVocabulary : 使用词典
    Frame --> ORBextractor : 特征提取
    Tracking --> ORBmatcher : 特征匹配
    Tracking --> Initializer : 初始化
    LocalMapping --> Optimizer : 局部优化
    LoopClosing --> Optimizer : 全局优化
```

## 设计模式与架构特点

### 1. 单例模式
- `System`类作为系统入口，管理所有子模块

### 2. 观察者模式  
- 各线程通过条件变量和互斥锁协调工作

### 3. 工厂模式
- `ORBextractor`根据参数创建不同配置的特征提取器

### 4. 策略模式
- `Optimizer`提供多种优化策略

### 5. 模板模式
- PCL点云处理使用模板类支持不同点类型

这种模块化的类设计使得系统具有良好的可扩展性和维护性，各模块职责明确，便于独立开发和测试。