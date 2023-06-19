#include "QtWidgetsApplication2.h"

#pragma execution_character_set("utf-8")

QString QtWidgetsApplication2::filePath;
int QtWidgetsApplication2::speed = 100;
int QtWidgetsApplication2::steps = 0;
int QtWidgetsApplication2::count = 20;
int QtWidgetsApplication2::portNo = 0;
std::string QtWidgetsApplication2::winname = "预览";

QtWidgetsApplication2::QtWidgetsApplication2(QWidget* parent)
	: QMainWindow(parent)
{

	vtkNew<vtkFileOutputWindow> fileOutputWindow;
	fileOutputWindow->SetFileName("output.txt");
	vtkOutputWindow::SetInstance(fileOutputWindow);

	ui.setupUi(this);
	this->resize(1600, 1400);

	isNow = false;


	ui.spinBox_4->setKeyboardTracking(false);
	ui.pushButton_5->setEnabled(false);

	ui.spinBox->setKeyboardTracking(false);
	ui.pushButton->setEnabled(false);

	ui.spinBox_11->setKeyboardTracking(false);
	ui.spinBox_12->setKeyboardTracking(false);
	ui.spinBox_13->setKeyboardTracking(false);

	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	//viewer->addCoordinateSystem();
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	ui.qvtkWidget->update();
	//ui.radioButton_X_add->setChecked(true);

	connect(ui.spinBox_11, SIGNAL(valueChanged(int)), ui.horizontalSlider, SLOT(setValue(int)));
	connect(ui.horizontalSlider, SIGNAL(valueChanged(int)), ui.spinBox_11, SLOT(setValue(int)));

	connect(ui.spinBox_12, SIGNAL(valueChanged(int)), ui.horizontalSlider_2, SLOT(setValue(int)));
	connect(ui.horizontalSlider_2, SIGNAL(valueChanged(int)), ui.spinBox_12, SLOT(setValue(int)));

	connect(ui.spinBox_13, SIGNAL(valueChanged(int)), ui.horizontalSlider_3, SLOT(setValue(int)));
	connect(ui.horizontalSlider_3, SIGNAL(valueChanged(int)), ui.spinBox_13, SLOT(setValue(int)));

	connect(ui.spinBox_14, SIGNAL(valueChanged(int)), ui.horizontalSlider_4, SLOT(setValue(int)));
	connect(ui.horizontalSlider_4, SIGNAL(valueChanged(int)), ui.spinBox_14, SLOT(setValue(int)));

}

QtWidgetsApplication2::~QtWidgetsApplication2()
{}

//"修改参数"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_clicked() {

	QString a;

	/**信息提示区输出提示*/
	ui.textBrowser_6->append("修改成功：");

	a = QString::number(ui.spinBox->value());
	ui.textBrowser_6->append("移动距离修改为：" + a);
	a = QString::number(ui.spinBox_2->value());
	ui.textBrowser_6->append("移动速度修改为：" + a);
	a = QString::number(ui.spinBox_3->value());
	ui.textBrowser_6->append("图片拍摄数量修改为：" + a);

	/**"移动距离"、"移动速度"、"拍摄数量"参数保存至全局参数 "steps"、"speed"、"count"*/
	steps = ui.spinBox->value();
	speed = ui.spinBox_2->value();
	count = ui.spinBox_3->value();

}

//"选择地址"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_2_clicked() {
	filePath = QFileDialog::getExistingDirectory(this, "请选择文件保存路径...", "./");

	/**保存地址保存至全局变量"filePath"*/
	filePath += '/';

	ui.textBrowser_3->setText(filePath);

	ui.textBrowser_6->append("图片保存地址也修改为：" + filePath + "/");
}

//"选择无激光标定图片"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_3_clicked() {
	QMessageBox::StandardButton rb = QMessageBox::question(this, "标定", "选择不带激光图片用于光平面标定?", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);
	if (rb != QMessageBox::Yes)
		return;
	ui.textBrowser_6->append("选择无激光标定图片");
	QStringList fileName = QFileDialog::getOpenFileNames(
		this,
		QStringLiteral(""),
		"C:/Users/Lizh/Pictures",
		tr("images(*.png *jpeg *bmp *jpg);;All files(*.*)"));

	QList<QString>::Iterator it = fileName.begin();
	ui.textBrowser_6->append(QString::fromStdString("共选择" + std::to_string(fileName.length()) + "不带激光标定图片"));
	for (; it != fileName.end(); it++) {
		Plane_Board_noLaser.push_back(cv::imread((*it).toStdString(), CV_LOAD_IMAGE_GRAYSCALE));
	}
}

//"运行"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_4_clicked() {

	/**非法数据检查*/
	if (!filePath.isEmpty() && portNo != 0 && steps != 0) {

		if (isNow) {
			QMessageBox::StandardButton rb = QMessageBox::question(this, "运行", "预览正运行中，是否关闭？", QMessageBox::Yes | QMessageBox::No , QMessageBox::Yes);
			if (rb != QMessageBox::Yes)
				return;
			ui.textBrowser_6->append("---------------------预览已关闭---------------------");
			cv::destroyWindow(winname);
			isNow = false;
			ui.pushButton_7->setText("预览");
		}

		QString a, b, c, d;

		a = QString::number(portNo);
		b = QString::number(steps);
		c = QString::number(speed);
		d = QString::number(count);

		/**数据确认*/
		int ret = QMessageBox::information(this, "操作提示", "当前参数：\n  端口：" + a +
			"\n  移动距离：" + b + "\n  移动速度：" + c + "\n  拍摄数量：" + d,
			QMessageBox::Yes, QMessageBox::No);

		/**串口控制输入*/
		if (ret == QMessageBox::Yes) {
			std::string a = filePath.toStdString();

			/**输出提示信息*/
			ui.textBrowser_6->append("开始运行....\n");

			/**数组输入数据，a输入保存地址，拍摄数量，运行时间*/
			cam(com(steps, speed), a, portNo, count, ((steps / speed) * 1000 + 500), ui);

		}

		ui.textBrowser_6->append("可选择重建使用图片");

		QMessageBox::StandardButton rb = QMessageBox::question(this, "标定", "选择图片用于重建", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);
		if (rb != QMessageBox::Yes)
			return;

		QStringList fileName = QFileDialog::getOpenFileNames(
			this,
			QStringLiteral(""),
			filePath,
			tr("images(*.png *jpeg *bmp *jpg);;All files(*.*)"));
		
		QList<QString>::Iterator it = fileName.begin();
		ui.textBrowser_6->append(QString::fromStdString("共选择" + std::to_string(fileName.length()) + "张图片用于重建"));
		Scan_Laser.clear();
		for (; it != fileName.end(); it++) {
			Scan_Laser.push_back(cv::imread((*it).toStdString(), CV_LOAD_IMAGE_GRAYSCALE));
		}
	}
	else {

		/**未保存操作提醒*/
		if (steps != ui.spinBox->value() || speed != ui.spinBox_2->value()
			|| count != ui.spinBox_3->value() || portNo != ui.spinBox_4->value()) {
			if (steps != ui.spinBox->value())
				QMessageBox::information(this, "操作提示", " 移动距离 参数修改未保存！");
			else if (speed != ui.spinBox_2->value())
				QMessageBox::information(this, "操作提示", " 速度 参数修改未保存！");
			else if (count != ui.spinBox_3->value())
				QMessageBox::information(this, "操作提示", " 拍摄数量 参数修改未保存！");
			else if (portNo != ui.spinBox_4->value())
				QMessageBox::information(this, "操作提示", " 端口号 参数修改未保存！");
		}

		/**错误信息提示*/
		else if (portNo == 0)
			QMessageBox::critical(this, "错误消息提示", "连接端口号未选择，请选择！");
		else if (steps == 0)
			QMessageBox::critical(this, "错误消息提示", "移动距离参数不能为0，请重新设定！");
		else if (filePath.isEmpty())
			QMessageBox::information(this, "消息提示", "照片将保存在当前文件夹下！");
	}
}

//"端口确定"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_5_clicked() {

	QString a;
	a = QString::number(ui.spinBox_4->value());

	/**文字提示*/
	ui.textBrowser_6->append("修改成功：");
	ui.textBrowser_6->append("通信端口修改为：" + a);

	/**端口信息保存至"portNo"*/
	portNo = ui.spinBox_4->value();
}

//"异常归零"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_6_clicked() {
	zero();
}

//"预览"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_7_clicked() {
	/**预览函数*/
	//zero();
	if (!isNow) {
		ui.textBrowser_6->append("----------------------预览开启---------------------");

		isNow = true;
		cv::namedWindow(winname, cv::WINDOW_GUI_NORMAL);

		HWND hwnd = (HWND)cvGetWindowHandle(winname.c_str());
		HWND paraent = GetParent(hwnd);//得到nameWindow窗口的父句柄
		SetParent(hwnd, (HWND)ui.widget->winId());//设置ui控件的句柄是父句柄
		ShowWindow(paraent, SW_HIDE);//隐藏掉nameWindow窗口
		resizeWindow(winname, cv::Size(900, 700));

		ui.pushButton_7->setText("关闭预览");
		nowCa(ui);
	}
	else {
		//ui.widget->setFocus();
		ui.textBrowser_6->append("---------------------预览已关闭---------------------");
		cv::destroyWindow(winname);
		isNow = false;
		ui.pushButton_7->setText("预览");
	}
}

//"点云清除"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_8_clicked() {
	if (cloud != NULL) {
		cloud->points.clear();
		viewer->updatePointCloud(cloud, "cloud");
		viewer->resetCamera();
		ui.qvtkWidget->update();
		QMessageBox::information(this, "info", "点云数据已清空");
		ui.textBrowser->append("点云数据已清空");
		ui.textBrowser_6->append("点云数据已清空");
	}
	else {
		QMessageBox::information(this, "info", "不存在点云数据");
		ui.textBrowser->append("不存在点云数据");
		ui.textBrowser_6->append("不存在点云数据");
	}
}

//"选择带激光"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_9_clicked() {
	QMessageBox::StandardButton rb = QMessageBox::question(this, "标定", "选择带激光条图片用于光平面标定?", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);
	if (rb != QMessageBox::Yes)
		return;

	ui.textBrowser_6->append("选择带激光标定图片");
	QStringList fileName = QFileDialog::getOpenFileNames(
		this,
		QStringLiteral("选择带激光标定图片"),
		"C:/Users/Lizh/Pictures",
		tr("images(*.png *jpg *bmp *jpg);;All files(*.*)"));
	QList<QString>::Iterator it = fileName.begin();
	ui.textBrowser_6->append(QString::fromStdString("共选择" + std::to_string(fileName.length()) + "带激光标定图片"));

	for (; it != fileName.end(); it++) {
		Plane_Laser.push_back(cv::imread((*it).toStdString(), CV_LOAD_IMAGE_GRAYSCALE));
	}

}

//"重建"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_10_clicked() {
	if (isCalibed
		&& !Scan_Laser.empty())
	{
		ui.textBrowser_6->append("Start...");
		int k = 0;
		ProcessTool Tool;
		cv::Mat r = camera->GetBaseRvecMat();
		cv::Mat t = camera->GetBaseTvecMat();

		std::vector<cv::Point3f> Points3d_all;

		if (!cloud)
		{
			cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
			(*cloud).points.resize(Points3d_all.size());
		}
		else
			cloud->clear();

		if (!cloud_filtered)
		{
			cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
			(*cloud_filtered).points.resize(Points3d_all.size());
		}
		else
			cloud_filtered->clear();

		if (!cloud_after_StatisticalRemoval)
		{
			cloud_after_StatisticalRemoval.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
			(*cloud_after_StatisticalRemoval).points.resize(Points3d_all.size());
		}
		else
			cloud_after_StatisticalRemoval->clear();

		vector<vector<pcl::PointXYZRGB>> Points3ds;
		clock_t start, end;
		double averagetime = 0;
		for (vector<cv::Mat>::iterator it = Scan_Laser.begin(); it != Scan_Laser.end(); it++, k++)
		{
			start = clock();
			vector<cv::Point2d> Points;
			vector<pcl::PointXYZRGB> Pt;

			//两种不同的激光提取
			switch (ui.comboBox->currentIndex())
			{
			case 0:
				Points = Tool.AverageLine(*it, cv::Point2d(0, 0), cv::Point2d(it->cols, it->rows));
				break;
			case 1:
				Points = Tool.StegerLine(
					*it,

					//高斯内核长宽
					ui.spinBox_7->value(),
					ui.spinBox_5->value(),

					//高斯XY方向上的标准差
					ui.spinBox_8->value(),
					ui.spinBox_9->value(),

					//阈值
					ui.spinBox_6->value(),
					ui.doubleSpinBox->value()
				);
				break;
			}
			end = clock();
			//调试代码，计算处理时间
			double endtime = (double)(end - start) / CLOCKS_PER_SEC;
			averagetime += endtime * 1000;
			ui.textBrowser_6->append("average time = " + QString::number(endtime * 1000) + "ms");
			//调试代码

			for (int j = 0; j < Points.size(); j++)
			{
				//中心线提取函数返回的数组的坐标为(Points[2 * k + 0], Points[2 * k + 1])
				cv::Point3f Points3d = camera->getWorldPoints(cv::Point2f(Points[j].x, Points[j].y), r, t);
				Points3d.z = (ALaserPlane->Get_D() - ALaserPlane->Get_A() * Points3d.x - ALaserPlane->Get_B() * Points3d.y) / ALaserPlane->Get_C();

				Points3d.y += k * Astep->GetStep().at<double>(1, 0);
				Points3d.x += k * Astep->GetStep().at<double>(0, 0);
				Points3d.z += k * Astep->GetStep().at<double>(2, 0);

				cloud->points.push_back(pcl::PointXYZRGB(Points3d.x, Points3d.y, Points3d.z, ui.spinBox_11->value(), ui.spinBox_12->value(), ui.spinBox_13->value()));
			}
			//对每条线条进行处理
			//半径滤波
			pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
			outrem.setInputCloud(cloud);
			outrem.setRadiusSearch(0.5);
			outrem.setMinNeighborsInRadius(4);
			outrem.filter(*cloud_filtered);

			//简化线条，距离需要自行设定
			//CloudProcessTool CloudTool;
			//CloudTool.cutCloud(cloud_filtered, 1.5);

			for (int i = 0; i < cloud_filtered->points.size(); i++)
				Pt.push_back(cloud_filtered->points[i]);
			Points3ds.push_back(Pt);
			Pt.clear();

			cloud_filtered->points.clear();
			cloud->clear();
		}

		//调试代码，计算处理平均时间
		averagetime /= Points3ds.size();

		Scan_Laser.clear();
		int kk = 0;
		for (int i = 0; i < Points3ds.size(); i++) {
			for (int j = 0; j < Points3ds[i].size(); j++) {
				cloud->points.push_back(Points3ds[i][j]);
			}
			kk += Points3ds[i].size();
		}
		cloud->width = kk;
		cloud->height = 1;
		try {
			pcl::PCDWriter writer;
			writer.write<pcl::PointXYZRGB>("ppp.pcd", *cloud, false);
		}
		catch (exception e) {
			ui.textBrowser_6->append(e.what());
		}

		//展示到屏幕上
		viewer->addPointCloud(cloud, "cloud");
		viewer->updatePointCloud(cloud, "cloud");


		viewer->resetCamera();
		//单点大小设定为5
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui.spinBox_14->value(), "cloud");
		ui.qvtkWidget->update();


		//ui.extBrowser->append("================finish==================");
		cv::destroyWindow("GetLine");
		QMessageBox::information(NULL, "info", "Caculate finish!", QMessageBox::Ok);


	}
	else
		if (!isCalibed)
			QMessageBox::warning(NULL, "warning", "请先完成标定!", QMessageBox::Ok);
		else
			QMessageBox::warning(NULL, "warning", "请先选择重建使用图片!", QMessageBox::Ok);

}

//"开始标定"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_11_clicked() {
	cv::FileStorage fs("./new.xml", cv::FileStorage::READ);
	if (fs.isOpened() &&
		!Plane_Laser.empty() &&
		!Step_Calculate.empty()
		)
	{
		camera = new CalibrateCamera();
		ALaserPlane = new LaserPlane();
		ui.textBrowser_6->append("start....");
		////相机标定数据获取
		camera->calibrate();
		isCameraCalib = true;
		ui.textBrowser_6->append("CameraMatrix:");
		ui.textBrowser_6->append("fx = " + QString::number(camera->GetCameraMatrix().at<double>(0, 0)));
		ui.textBrowser_6->append("fy = " + QString::number(camera->GetCameraMatrix().at<double>(1, 1)));
		ui.textBrowser_6->append("cx = " + QString::number(camera->GetCameraMatrix().at<double>(0, 2)));
		ui.textBrowser_6->append("cy = " + QString::number(camera->GetCameraMatrix().at<double>(1, 2)));

		ui.textBrowser_6->append("Calibrate errors:");
		ui.textBrowser_6->append("Average E：" + QString::number(camera->GetAvg_Err()));

		//基准计算
		//camera->BaseCaculate(Base, caliboard);
		isBaseCalib = true;
		ui.textBrowser_6->append("base caculate finished");
		//光平面标定
		CalibrateBoard* PlaneBoard = new CalibrateBoard(Plane_Board_noLaser.size(), 8, 5, 7);
		//ui.textBrowser_6->append(QString::number(Plane_Board_noLaser.size()));
		//return;
		ALaserPlane->LoadBoard(Plane_Board_noLaser, camera, PlaneBoard);
		ui.textBrowser_6->append(QString::number(ALaserPlane->cout));
		ALaserPlane->LoadLaser(Plane_Laser, camera);

		ALaserPlane->CaculateLaserPlane();
		isPlaneCalib = true;

		ui.textBrowser_6->append("light plane caculate finished");
		ui.textBrowser_6->append("light plane参数A：" + QString::number(ALaserPlane->Get_A()));
		ui.textBrowser_6->append("light plane参数B：" + QString::number(ALaserPlane->Get_B()));
		ui.textBrowser_6->append("light plane参数C：" + QString::number(ALaserPlane->Get_C()));
		ui.textBrowser_6->append("light plane参数D：" + QString::number(ALaserPlane->Get_D()));

		//拼接间隔计算
		CalibrateBoard* TrackBoard = new CalibrateBoard(2, 8, 5, 7);
		Astep = new step(TrackBoard, camera, steps);
		Astep->LoadTrackImage(Step_Calculate);
		Astep->CaculateStep();
		ui.textBrowser_6->append("step caculate finished");
		ui.textBrowser_6->append("==============finish calibrate================");
		isCalibed = true;
		isStepCalib = true;

		ui.textBrowser_6->append(QString::number(Astep->GetStep().at<double>(0, 0)));
		ui.textBrowser_6->append(QString::number(Astep->GetStep().at<double>(1, 0)));
		ui.textBrowser_6->append(QString::number(Astep->GetStep().at<double>(2, 0)));
		//清空图片数组

		Plane_Laser.clear();
		Step_Calculate.clear();
		Scan_Laser.clear();
	}
	else {
		QMessageBox::warning(this, "warning", "请选择标定光平面所需图片！", QMessageBox::Ok);
	}

}

//"间隔标定图片"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_12_clicked() {

	QMessageBox::StandardButton rb = QMessageBox::question(this, "标定", "选择图片用于间隔标定", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);
	if (rb != QMessageBox::Yes)
		return;

	ui.textBrowser_6->append("选择间隔标定使用图片");
	QStringList fileName = QFileDialog::getOpenFileNames(
		this,
		QStringLiteral(""),
		filePath,
		tr("images(*.png *jpeg *bmp *jpg);;All files(*.*)"));

	QList<QString>::Iterator it = fileName.begin();
	ui.textBrowser_6->append(QString::fromStdString("共选择" + std::to_string(fileName.length()) + "张图片用于间隔标定"));
	Step_Calculate.clear();
	for (; it != fileName.end(); it++) {
		Step_Calculate.push_back(cv::imread((*it).toStdString(), CV_LOAD_IMAGE_GRAYSCALE));
	}
}

//"重建图片"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_13_clicked() {

	QMessageBox::StandardButton rb = QMessageBox::question(this, "标定", "选择图片用于重建", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);
	if (rb != QMessageBox::Yes)
		return;

	ui.textBrowser_6->append("选择重建使用图片");
	QStringList fileName = QFileDialog::getOpenFileNames(
		this,
		QStringLiteral(""),
		filePath,
		tr("images(*.png *jpeg *bmp *jpg);;All files(*.*)"));

	QList<QString>::Iterator it = fileName.begin();
	ui.textBrowser_6->append(QString::fromStdString("共选择" + std::to_string(fileName.length()) + "张图片用于重建"));
	Scan_Laser.clear();
	for (; it != fileName.end(); it++) {
		Scan_Laser.push_back(cv::imread((*it).toStdString(), CV_LOAD_IMAGE_GRAYSCALE));
	}
}

//"查看图片"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_14_clicked() {

	//打开资源管理器并高亮文件
	const QString explorer = "explorer";
	QStringList param;
	if (!QFileInfo(filePath).isDir()) {
		param << QLatin1String("/select,");
	}
	param << QDir::toNativeSeparators(filePath);
	QProcess::startDetached(explorer, param);
}

//"视角归位"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_15_clicked() {
	viewer->resetCamera();
	ui.qvtkWidget->update();
}

//"曲面重建"按键点击响应事件
void QtWidgetsApplication2::on_pushButton_16_clicked() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	//pcl::io::loadPCDFile("rabbit.pcd", cloud_blob);
	//pcl::fromPCLPointCloud2(cloud_blob, *cloud1);
	pcl::copyPointCloud(*cloud, *cloud1);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //法向量点云对象指针
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	tree->setInputCloud(cloud1);                        //用cloud构建tree对象
	n.setInputCloud(cloud1);                            //为法线估计对象设置输入点云
	n.setSearchMethod(tree);                          //设置搜索方法
	n.setKSearch(20);                                 //设置k搜索的k值为20
	n.compute(*normals);                              //估计法线存储结果到normals中
	//将点云和法线放到一起

	pcl::concatenateFields(*cloud1, *normals, *cloud_with_normals);//连接字段，cloud_with_normals存储有向点云

	//创建搜索树
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	//// 曲面重建
	////pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
	////mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计
	////mls.setInputCloud(cloud_with_normals);//设置参数
	////mls.setPolynomialFit(true);
	////mls.setSearchMethod(tree2);
	////mls.setSearchRadius(0.03);
	////pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_msl(new pcl::PointCloud<pcl::PointNormal>);
	////mls.process(*cloud_with_normals_msl);
	////cloud_with_normals = cloud_with_normals_msl;
	////std::cerr << "曲面重建   完成" << std::endl;

	//// 开始表面重建 ********************************************************************

	//创建Poisson对象，并设置参数
	//pcl::Poisson<pcl::PointNormal> pn;
	//pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
	//pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
	//pn.setDepth(8);
	////树的最大深度，求解2^d x 2^d x 2^d立方体元。
	//// 由于八叉树自适应采样密度，指定值仅为最大深度。
	//pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
	//pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 
	//// 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	//pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
	//pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
	//pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
	//pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
	////pn.setIndices();
	////设置搜索方法和输入点云
	//pn.setSearchMethod(tree2);
	//pn.setInputCloud(cloud_with_normals);
	////创建多变形网格，用于存储结果
	//pcl::PolygonMesh mesh;
	////执行重构
	//pn.performReconstruction(mesh);
	//保存网格图


	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh mesh;
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(50.0f);
	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);
	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(mesh);
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPolygonMesh(mesh, "mesh");
	//viewer->addCoordinateSystem(cloud->points[1].x, cloud->points[1].y, cloud->points[1].z,1.0);
	viewer->initCameraParameters();
	//viewer->addPolylineFromPolygonMesh(triangles);

	viewer->resetCamera();
	ui.qvtkWidget->update();
}

//"移动距离"对应数据框数据改变响应时间，修改"修改参数"键可点击性
void QtWidgetsApplication2::on_spinBox_valueChanged() {

	/**数值为0时不可点击*/
	if (ui.spinBox->value() != 0) {
		ui.pushButton->setEnabled(true);
	}
	else {
		ui.pushButton->setEnabled(false);
	}
}

//"通信端口"对应数据框数据改变响应时间，修改"端口确定"键可点击性
void QtWidgetsApplication2::on_spinBox_4_valueChanged() {

	/**数值为0时不可点击*/
	if (ui.spinBox_4->value() != 0) {
		ui.pushButton_5->setEnabled(true);
	}
	else {
		ui.pushButton_5->setEnabled(false);
	}
}

//"red"滑动条控制点云颜色0
void QtWidgetsApplication2::on_spinBox_11_valueChanged() {
	changeColor();
}

//"green"滑动条控制点云颜色
void QtWidgetsApplication2::on_spinBox_12_valueChanged() {
	changeColor();
}

//"blue"滑动条控制点云颜色
void QtWidgetsApplication2::on_spinBox_13_valueChanged() {
	changeColor();
}

//"size"滑动条控制点云点大小
void QtWidgetsApplication2::on_spinBox_14_valueChanged() {
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui.spinBox_14->value(), "cloud");
	ui.qvtkWidget->update();
}

//颜色改变函数
void QtWidgetsApplication2::changeColor() {
	if (cloud != NULL && viewer != NULL) {
		for (size_t t = 0; t < cloud->size(); t++) {
			cloud->points[t].r = ui.spinBox_11->value();
			cloud->points[t].g = ui.spinBox_12->value();
			cloud->points[t].b = ui.spinBox_13->value();
		}
		viewer->updatePointCloud(cloud, "cloud");
		ui.qvtkWidget->update();
	}
}