# 系统主要接口API文档

## 概述

本文档描述了ORB-SLAM2 SSD语义SLAM系统的主要API接口，包括系统管理、跟踪、建图、语义处理等核心功能的外部接口。这些API为上层应用提供了完整的语义SLAM功能访问。

## 系统管理API

### System类主要接口

#### 1. 系统初始化

```cpp
/**
 * @brief 系统构造函数
 * @param strVocFile ORB词典文件路径
 * @param strSettingsFile 配置文件路径  
 * @param sensor 传感器类型 (MONOCULAR/STEREO/RGBD)
 * @param bUseViewer 是否启用可视化界面
 */
System(const string &strVocFile, const string &strSettingsFile, 
       const eSensor sensor, const bool bUseViewer = true);

/**
 * @brief 系统关闭
 * @details 停止所有线程并清理资源
 */
void Shutdown();

/**
 * @brief 系统重置
 * @details 清空地图并重新初始化
 */
void Reset();
```

**使用示例**:
```cpp
// 初始化语义SLAM系统
ORB_SLAM2::System SLAM("Vocabulary/ORBvoc.txt", 
                       "config/TUM3.yaml", 
                       ORB_SLAM2::System::RGBD, 
                       true);

// 系统使用完毕后关闭
SLAM.Shutdown();
```

#### 2. 跟踪接口

```cpp
/**
 * @brief RGB-D跟踪
 * @param im RGB图像 (CV_8UC3 或 CV_8UC1)
 * @param depthmap 深度图 (CV_32F)
 * @param timestamp 时间戳
 * @return 相机位姿矩阵 (4x4), 跟踪失败时返回空矩阵
 */
cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

/**
 * @brief 双目跟踪
 * @param imLeft 左目图像
 * @param imRight 右目图像
 * @param timestamp 时间戳
 * @return 相机位姿矩阵
 */
cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

/**
 * @brief 单目跟踪
 * @param im 单目图像
 * @param timestamp 时间戳
 * @return 相机位姿矩阵
 */
cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);
```

**使用示例**:
```cpp
// RGB-D跟踪示例
cv::Mat rgb_image, depth_image;
double timestamp = getCurrentTimestamp();

// 执行跟踪
cv::Mat Tcw = SLAM.TrackRGBD(rgb_image, depth_image, timestamp);

if (!Tcw.empty()) {
    // 跟踪成功，Tcw为相机位姿
    cout << "Camera pose: " << Tcw << endl;
} else {
    // 跟踪失败
    cout << "Tracking lost!" << endl;
}
```

#### 3. 模式切换

```cpp
/**
 * @brief 激活定位模式
 * @details 停止建图，仅进行相机定位
 */
void ActivateLocalizationMode();

/**
 * @brief 取消定位模式
 * @details 恢复SLAM模式，继续建图
 */
void DeactivateLocalizationMode();

/**
 * @brief 检查地图是否发生重大变化
 * @return 是否有闭环或全局BA发生
 */
bool MapChanged();
```

#### 4. 状态查询

```cpp
/**
 * @brief 获取跟踪状态
 * @return 当前跟踪状态枚举值
 */
int GetTrackingState();

/**
 * @brief 获取当前跟踪的地图点
 * @return 地图点指针向量
 */
std::vector<MapPoint*> GetTrackedMapPoints();

/**
 * @brief 获取当前跟踪的特征点
 * @return 未失真特征点向量
 */
std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
```

#### 5. 数据保存

```cpp
/**
 * @brief 保存轨迹(TUM格式)
 * @param filename 输出文件名
 * @details 仅适用于双目和RGB-D，需先调用Shutdown()
 */
void SaveTrajectoryTUM(const string &filename);

/**
 * @brief 保存关键帧轨迹(TUM格式)
 * @param filename 输出文件名
 * @details 适用于所有传感器类型
 */
void SaveKeyFrameTrajectoryTUM(const string &filename);

/**
 * @brief 保存轨迹(KITTI格式)
 * @param filename 输出文件名
 */
void SaveTrajectoryKITTI(const string &filename);
```

## 语义检测API

### Detector类接口

#### 1. 检测器初始化

```cpp
/**
 * @brief 检测器构造函数
 * @details 加载NCNN模型并初始化网络
 */
Detector();

/**
 * @brief 检测器析构函数
 * @details 释放网络资源
 */
~Detector();
```

#### 2. 目标检测

```cpp
/**
 * @brief 执行目标检测
 * @param bgr_img 输入BGR图像
 * @param objects 输出检测结果
 */
void Run(const cv::Mat& bgr_img, std::vector<Object>& objects);

/**
 * @brief 显示检测结果
 * @param bgr_img 输入图像
 * @param objects 检测结果
 */
void Show(const cv::Mat& bgr_img, std::vector<Object>& objects);
```

**使用示例**:
```cpp
// 创建检测器
Detector detector;

// 执行检测
std::vector<Object> objects;
detector.Run(rgb_image, objects);

// 处理检测结果
for (const auto& obj : objects) {
    cout << "Object: " << obj.object_name 
         << ", Confidence: " << obj.prob
         << ", Box: [" << obj.rect.x << "," << obj.rect.y 
         << "," << obj.rect.width << "," << obj.rect.height << "]" << endl;
}
```

### Object结构体

```cpp
/**
 * @brief 检测对象结构体
 */
typedef struct Object {
    cv::Rect_<float> rect;      ///< 检测框坐标(归一化)
    std::string object_name;    ///< 物体类别名称
    int class_id;               ///< 类别ID
    float prob;                 ///< 置信度[0,1]
    
    Object();
    Object(cv::Rect_<float> r, std::string name, int id, float confidence);
    bool operator>(const Object& other) const;
} Object;
```

## 语义建图API

### PointCloudMapping类接口

#### 1. 点云建图初始化

```cpp
/**
 * @brief 点云建图构造函数
 * @param resolution 体素分辨率(米)
 */
PointCloudMapping(double resolution);
```

#### 2. 关键帧处理

```cpp
/**
 * @brief 插入关键帧进行语义点云建图
 * @param kf 关键帧指针
 * @param color 灰度图像
 * @param depth 深度图像
 * @param imgRGB RGB图像
 */
void insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth, cv::Mat& imgRGB);

/**
 * @brief 生成点云
 * @param kf 关键帧
 * @param color 彩色图像
 * @param depth 深度图像
 * @return 生成的点云指针
 */
PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
```

#### 3. 语义处理

```cpp
/**
 * @brief 语义融合
 * @param cluster 语义聚类
 */
void sem_merge(Cluster cluster);

/**
 * @brief 添加3D可视化框
 */
void add_cube();

/**
 * @brief 更新处理
 */
void update();
```

#### 4. 系统控制

```cpp
/**
 * @brief 启动可视化线程
 */
void viewer();

/**
 * @brief 关闭系统
 */
void shutdown();
```

**使用示例**:
```cpp
// 创建点云建图器
auto pointCloudMapper = std::make_shared<PointCloudMapping>(0.05);  // 5cm分辨率

// 在跟踪线程中插入关键帧
if (needNewKeyFrame) {
    KeyFrame* kf = new KeyFrame(currentFrame, mpMap, mpKeyFrameDB);
    pointCloudMapper->insertKeyFrame(kf, grayImage, depthImage, rgbImage);
}

// 系统结束时关闭
pointCloudMapper->shutdown();
```

### Cluster结构体

```cpp
/**
 * @brief 语义聚类结构体
 */
typedef struct Cluster {
    std::string object_name;     ///< 物体类别名称
    int class_id;                ///< 类别ID
    float prob;                  ///< 置信度
    Eigen::Vector3f minPt;       ///< 包围框最小点
    Eigen::Vector3f maxPt;       ///< 包围框最大点
    Eigen::Vector3f centroid;    ///< 中心点坐标
    
    // 扩展属性
    pcl::PointIndices indices;   ///< 点云索引
    int point_count;             ///< 点数量
    float volume;                ///< 体积
    int observation_count;       ///< 观测次数
    double last_updated;         ///< 最后更新时间
    
    bool operator==(const std::string& x);
    void updateBounds(const Eigen::Vector3f& point);
    float calculateVolume() const;
    Eigen::Vector3f getCenter() const;
} Cluster;
```

## 可视化API

### Viewer类接口

#### 1. 可视化控制

```cpp
/**
 * @brief 可视化器构造函数
 * @param pSystem 系统指针
 * @param pFrameDrawer 帧绘制器
 * @param pMapDrawer 地图绘制器
 * @param strSettingPath 配置文件路径
 */
Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, 
       const string &strSettingPath);

/**
 * @brief 主显示循环
 */
void Run();

/**
 * @brief 请求结束
 */
void RequestFinish();

/**
 * @brief 请求停止
 */
void RequestStop();

/**
 * @brief 检查是否结束
 */
bool isFinished();

/**
 * @brief 检查是否停止
 */
bool isStopped();
```

#### 2. 显示控制

```cpp
/**
 * @brief 设置图像尺寸
 * @param width 图像宽度
 * @param height 图像高度
 */
void SetImageSize(int width, int height);

/**
 * @brief 设置视点
 * @param x X坐标
 * @param y Y坐标
 * @param z Z坐标
 * @param f 焦距
 */
void SetViewpoint(float x, float y, float z, float f);
```

## 配置API

### 配置文件格式

系统使用YAML格式的配置文件，主要包含以下参数：

```yaml
# 相机参数
Camera.fx: 525.0
Camera.fy: 525.0
Camera.cx: 319.5
Camera.cy: 239.5

# 相机畸变参数
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0

# RGB-D相机参数
Camera.bf: 40.0        # 基线*焦距
Camera.fps: 30.0       # 帧率
ThDepth: 40.0          # 深度阈值

# ORB参数
ORBextractor.nFeatures: 1000     # 特征点数量
ORBextractor.scaleFactor: 1.2    # 尺度因子
ORBextractor.nLevels: 8          # 金字塔层数
ORBextractor.iniThFAST: 20       # FAST角点初始阈值
ORBextractor.minThFAST: 7        # FAST角点最小阈值

# 语义检测参数
Semantic.enable: true             # 是否启用语义检测
Semantic.modelPath: "models/"     # 模型路径
Semantic.confThreshold: 0.5       # 置信度阈值
Semantic.nmsThreshold: 0.4        # NMS阈值

# 点云参数
PointCloud.resolution: 0.05       # 体素分辨率
PointCloud.enable: true           # 是否启用点云建图

# 可视化参数
Viewer.KeyFrameSize: 0.05         # 关键帧显示尺寸
Viewer.KeyFrameLineWidth: 1       # 关键帧线宽
Viewer.GraphLineWidth: 0.9        # 图连线宽度
Viewer.PointSize: 2               # 点大小
Viewer.CameraSize: 0.08           # 相机尺寸
Viewer.CameraLineWidth: 3         # 相机线宽
Viewer.ViewpointX: 0              # 视点X
Viewer.ViewpointY: -0.7           # 视点Y
Viewer.ViewpointZ: -1.8           # 视点Z
Viewer.ViewpointF: 500            # 视点焦距
```

### 配置加载接口

```cpp
/**
 * @brief 配置参数结构体
 */
struct SystemConfig {
    // 相机参数
    float fx, fy, cx, cy;
    float k1, k2, p1, p2;
    float bf, fps;
    float depth_threshold;
    
    // ORB参数
    int orb_features;
    float orb_scale_factor;
    int orb_levels;
    int orb_fast_th_init;
    int orb_fast_th_min;
    
    // 语义参数
    bool semantic_enable;
    std::string model_path;
    float conf_threshold;
    float nms_threshold;
    
    // 点云参数
    float pointcloud_resolution;
    bool pointcloud_enable;
    
    // 可视化参数
    float viewer_keyframe_size;
    float viewer_point_size;
    float viewer_camera_size;
    
    /**
     * @brief 从YAML文件加载配置
     * @param filename 配置文件路径
     * @return 是否加载成功
     */
    bool loadFromFile(const std::string& filename);
    
    /**
     * @brief 保存配置到文件
     * @param filename 输出文件路径
     * @return 是否保存成功
     */
    bool saveToFile(const std::string& filename);
};
```

## 错误处理API

### 异常类定义

```cpp
/**
 * @brief SLAM系统异常基类
 */
class SLAMException : public std::exception {
public:
    explicit SLAMException(const std::string& message) : message_(message) {}
    virtual const char* what() const noexcept override {
        return message_.c_str();
    }
    
private:
    std::string message_;
};

/**
 * @brief 初始化异常
 */
class InitializationException : public SLAMException {
public:
    explicit InitializationException(const std::string& message) 
        : SLAMException("Initialization Error: " + message) {}
};

/**
 * @brief 跟踪异常
 */
class TrackingException : public SLAMException {
public:
    explicit TrackingException(const std::string& message) 
        : SLAMException("Tracking Error: " + message) {}
};

/**
 * @brief 语义处理异常
 */
class SemanticException : public SLAMException {
public:
    explicit SemanticException(const std::string& message) 
        : SLAMException("Semantic Error: " + message) {}
};
```

### 错误回调接口

```cpp
/**
 * @brief 错误回调函数类型
 */
typedef std::function<void(const std::string&, int)> ErrorCallback;

/**
 * @brief 设置错误回调
 * @param callback 错误回调函数
 */
void SetErrorCallback(ErrorCallback callback);

/**
 * @brief 日志级别枚举
 */
enum LogLevel {
    LOG_DEBUG = 0,
    LOG_INFO = 1,
    LOG_WARNING = 2,
    LOG_ERROR = 3
};

/**
 * @brief 设置日志级别
 * @param level 日志级别
 */
void SetLogLevel(LogLevel level);
```

## 性能监控API

### 性能统计接口

```cpp
/**
 * @brief 性能统计结构体
 */
struct PerformanceStats {
    double tracking_time;         ///< 跟踪耗时(ms)
    double mapping_time;          ///< 建图耗时(ms)
    double semantic_time;         ///< 语义处理耗时(ms)
    double total_time;            ///< 总耗时(ms)
    
    int tracked_features;         ///< 跟踪特征数
    int mapped_points;            ///< 地图点数
    int semantic_objects;         ///< 语义对象数
    
    float memory_usage;           ///< 内存使用量(MB)
    
    /**
     * @brief 重置统计信息
     */
    void reset();
    
    /**
     * @brief 输出统计信息
     */
    void print() const;
};

/**
 * @brief 获取性能统计
 * @return 性能统计结构体
 */
PerformanceStats GetPerformanceStats();

/**
 * @brief 重置性能统计
 */
void ResetPerformanceStats();
```

## 使用示例

### 完整使用示例

```cpp
#include "System.h"
#include "Detector.h"

int main() {
    try {
        // 1. 初始化系统
        ORB_SLAM2::System SLAM("Vocabulary/ORBvoc.txt", 
                               "config/TUM3.yaml", 
                               ORB_SLAM2::System::RGBD, 
                               true);
        
        // 2. 设置错误回调
        SLAM.SetErrorCallback([](const std::string& msg, int level) {
            std::cerr << "SLAM Error[" << level << "]: " << msg << std::endl;
        });
        
        // 3. 数据处理循环
        cv::VideoCapture cap(0);  // 相机输入
        cv::Mat rgb, depth;
        
        while (cap.read(rgb)) {
            // 获取深度图 (这里需要实际的深度数据)
            // depth = getDepthImage();
            
            double timestamp = cv::getTickCount() / cv::getTickFrequency();
            
            // 执行SLAM
            cv::Mat Tcw = SLAM.TrackRGBD(rgb, depth, timestamp);
            
            if (!Tcw.empty()) {
                // 获取性能统计
                auto stats = SLAM.GetPerformanceStats();
                
                std::cout << "Tracking time: " << stats.tracking_time << "ms" << std::endl;
                std::cout << "Semantic objects: " << stats.semantic_objects << std::endl;
            }
            
            // 检查用户输入
            char key = cv::waitKey(1);
            if (key == 27) break;  // ESC退出
        }
        
        // 4. 保存结果
        SLAM.Shutdown();
        SLAM.SaveTrajectoryTUM("trajectory.txt");
        
    } catch (const SLAMException& e) {
        std::cerr << "SLAM Exception: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
```

这套API为ORB-SLAM2 SSD语义SLAM系统提供了完整的外部访问接口，支持灵活的系统配置、实时语义SLAM处理、丰富的可视化显示和全面的错误处理机制。