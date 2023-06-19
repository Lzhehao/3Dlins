#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QtWidgetsApplication2.h"
#include "compute.h"
#include "Calibrate.h"
#include "CloudProcess.h"

#include <qbuttongroup.h>
#include <qstring.h>
#include <qapplication.h>
#include <qfiledialog.h>
#include <qmessagebox.h>
#include <qprocess.h>
#include <qmessagebox.h>
#include <Windows.h>
#include <QKeyEvent>
#include <highgui/highgui_c.h>

#include "opencv2/imgcodecs/legacy/constants_c.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui_c.h>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/features/normal_3d.h>//法向量特征估计相关类定义的头文件

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>


#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/gp3.h>

#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件

#include <vtkFileOutputWindow.h>
#include <vtkOutputWindow.h>
#include <vtkRenderWindow.h>

#include <iostream> 



#pragma execution_character_set("utf-8")
class QtWidgetsApplication2 : public QMainWindow
{
    Q_OBJECT

public:
	QtWidgetsApplication2(QWidget* parent = nullptr);
	~QtWidgetsApplication2();


private:
    Ui::QtWidgetsApplication2Class ui;

    static QString filePath;
    static int speed;
    static int steps;
    static int count;
    static int portNo;
    static std::string winname;

    //相机标定的图片路径，后面可以改成mat类型的
    std::vector<cv::Mat> Camera_Calib;
    //拼接基准的图片路径
    cv::Mat Base;
    //没有激光的标定板的图片路径，用于计算光平面
    std::vector<cv::Mat> Plane_Board_noLaser;
    //激光的标定板，用于计算光平面
    std::vector<cv::Mat> Plane_Laser;
    //步长计算的图片路径
    std::vector<cv::Mat> Step_Calculate;
    //重建图片
    std::vector<cv::Mat> Scan_Laser;
    //保存兴趣点
    vector<cv::Point3d> ROI;

    //激光三角法
    //相机
    CalibrateCamera* camera;
    //激光平面
    LaserPlane* ALaserPlane;
    //步长
    step* Astep;

    //标定标志
    bool isCameraCalib;
    bool isPlaneCalib;
    bool isStepCalib;
    bool isLaserImg;
    bool isBaseCalib;
    bool isCalibed;
    bool isNow;

    //PCL滤波
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_after_StatisticalRemoval;

    unsigned int red, green, blue, size;

private slots:

    //"修改参数"按键点击响应事件
    void on_pushButton_clicked();
    
    //"选择地址"按键点击响应事件
    void on_pushButton_2_clicked();

    //"不带激光图片"按键点击响应事件
    void on_pushButton_3_clicked();

    //"运行"按键点击响应事件
    void on_pushButton_4_clicked();

    //"端口确定"按键点击响应事件
    void on_pushButton_5_clicked();

    //"归零"按键点击响应事件
    void on_pushButton_6_clicked();

    //"预览"按键点击响应事件
    void on_pushButton_7_clicked();

    //"点云清清除"按键点击响应事件
    void on_pushButton_8_clicked();

    //"选择带激光"按键点击响应事件
    void on_pushButton_9_clicked();

    //"重建"按键点击响应事件
    void on_pushButton_10_clicked();

    //"开始标定"按键点击响应事件
    void on_pushButton_11_clicked();

    void on_pushButton_12_clicked();

    //"重建图片"按键点击响应事件
    void on_pushButton_13_clicked();

    //"查看图片"按键点击响应事件
    void on_pushButton_14_clicked();

    //"视角归位"按键点击响应事件
    void on_pushButton_15_clicked();

    //"曲面重建"按键点击响应事件
    void on_pushButton_16_clicked();


    //"移动距离"对应数据框数据改变响应时间，修改"修改参数"键可点击性
    void on_spinBox_valueChanged();

    //"通信端口"对应数据框数据改变响应时间，修改"端口确定"键可点击性
    void on_spinBox_4_valueChanged();

    //颜色滑动条red
    void on_spinBox_11_valueChanged();

    //颜色滑动条green
    void on_spinBox_12_valueChanged();

    //颜色滑动条blue
    void on_spinBox_13_valueChanged();
    
    //大小滑动条size
    void on_spinBox_14_valueChanged();

    //颜色控制函数
    void changeColor();
};
