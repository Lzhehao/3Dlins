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
#include <pcl/point_types.h> //PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�
#include <pcl/features/normal_3d.h>//������������������ඨ���ͷ�ļ�

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>


#include <pcl/filters/statistical_outlier_removal.h>   //ͳ���˲���ͷ�ļ�
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/gp3.h>

#include <pcl/kdtree/kdtree_flann.h>  //kd-tree����������ඨ���ͷ�ļ�
#include <pcl/surface/mls.h>        //��С���˷�ƽ�������ඨ��ͷ�ļ�

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

    //����궨��ͼƬ·����������Ըĳ�mat���͵�
    std::vector<cv::Mat> Camera_Calib;
    //ƴ�ӻ�׼��ͼƬ·��
    cv::Mat Base;
    //û�м���ı궨���ͼƬ·�������ڼ����ƽ��
    std::vector<cv::Mat> Plane_Board_noLaser;
    //����ı궨�壬���ڼ����ƽ��
    std::vector<cv::Mat> Plane_Laser;
    //���������ͼƬ·��
    std::vector<cv::Mat> Step_Calculate;
    //�ؽ�ͼƬ
    std::vector<cv::Mat> Scan_Laser;
    //������Ȥ��
    vector<cv::Point3d> ROI;

    //�������Ƿ�
    //���
    CalibrateCamera* camera;
    //����ƽ��
    LaserPlane* ALaserPlane;
    //����
    step* Astep;

    //�궨��־
    bool isCameraCalib;
    bool isPlaneCalib;
    bool isStepCalib;
    bool isLaserImg;
    bool isBaseCalib;
    bool isCalibed;
    bool isNow;

    //PCL�˲�
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_after_StatisticalRemoval;

    unsigned int red, green, blue, size;

private slots:

    //"�޸Ĳ���"���������Ӧ�¼�
    void on_pushButton_clicked();
    
    //"ѡ���ַ"���������Ӧ�¼�
    void on_pushButton_2_clicked();

    //"��������ͼƬ"���������Ӧ�¼�
    void on_pushButton_3_clicked();

    //"����"���������Ӧ�¼�
    void on_pushButton_4_clicked();

    //"�˿�ȷ��"���������Ӧ�¼�
    void on_pushButton_5_clicked();

    //"����"���������Ӧ�¼�
    void on_pushButton_6_clicked();

    //"Ԥ��"���������Ӧ�¼�
    void on_pushButton_7_clicked();

    //"���������"���������Ӧ�¼�
    void on_pushButton_8_clicked();

    //"ѡ�������"���������Ӧ�¼�
    void on_pushButton_9_clicked();

    //"�ؽ�"���������Ӧ�¼�
    void on_pushButton_10_clicked();

    //"��ʼ�궨"���������Ӧ�¼�
    void on_pushButton_11_clicked();

    void on_pushButton_12_clicked();

    //"�ؽ�ͼƬ"���������Ӧ�¼�
    void on_pushButton_13_clicked();

    //"�鿴ͼƬ"���������Ӧ�¼�
    void on_pushButton_14_clicked();

    //"�ӽǹ�λ"���������Ӧ�¼�
    void on_pushButton_15_clicked();

    //"�����ؽ�"���������Ӧ�¼�
    void on_pushButton_16_clicked();


    //"�ƶ�����"��Ӧ���ݿ����ݸı���Ӧʱ�䣬�޸�"�޸Ĳ���"���ɵ����
    void on_spinBox_valueChanged();

    //"ͨ�Ŷ˿�"��Ӧ���ݿ����ݸı���Ӧʱ�䣬�޸�"�˿�ȷ��"���ɵ����
    void on_spinBox_4_valueChanged();

    //��ɫ������red
    void on_spinBox_11_valueChanged();

    //��ɫ������green
    void on_spinBox_12_valueChanged();

    //��ɫ������blue
    void on_spinBox_13_valueChanged();
    
    //��С������size
    void on_spinBox_14_valueChanged();

    //��ɫ���ƺ���
    void changeColor();
};
