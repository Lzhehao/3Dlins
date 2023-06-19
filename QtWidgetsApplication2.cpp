#include "QtWidgetsApplication2.h"

#pragma execution_character_set("utf-8")

QString QtWidgetsApplication2::filePath;
int QtWidgetsApplication2::speed = 100;
int QtWidgetsApplication2::steps = 0;
int QtWidgetsApplication2::count = 20;
int QtWidgetsApplication2::portNo = 0;
std::string QtWidgetsApplication2::winname = "Ԥ��";

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

//"�޸Ĳ���"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_clicked() {

	QString a;

	/**��Ϣ��ʾ�������ʾ*/
	ui.textBrowser_6->append("�޸ĳɹ���");

	a = QString::number(ui.spinBox->value());
	ui.textBrowser_6->append("�ƶ������޸�Ϊ��" + a);
	a = QString::number(ui.spinBox_2->value());
	ui.textBrowser_6->append("�ƶ��ٶ��޸�Ϊ��" + a);
	a = QString::number(ui.spinBox_3->value());
	ui.textBrowser_6->append("ͼƬ���������޸�Ϊ��" + a);

	/**"�ƶ�����"��"�ƶ��ٶ�"��"��������"����������ȫ�ֲ��� "steps"��"speed"��"count"*/
	steps = ui.spinBox->value();
	speed = ui.spinBox_2->value();
	count = ui.spinBox_3->value();

}

//"ѡ���ַ"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_2_clicked() {
	filePath = QFileDialog::getExistingDirectory(this, "��ѡ���ļ�����·��...", "./");

	/**�����ַ������ȫ�ֱ���"filePath"*/
	filePath += '/';

	ui.textBrowser_3->setText(filePath);

	ui.textBrowser_6->append("ͼƬ�����ַҲ�޸�Ϊ��" + filePath + "/");
}

//"ѡ���޼���궨ͼƬ"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_3_clicked() {
	QMessageBox::StandardButton rb = QMessageBox::question(this, "�궨", "ѡ�񲻴�����ͼƬ���ڹ�ƽ��궨?", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);
	if (rb != QMessageBox::Yes)
		return;
	ui.textBrowser_6->append("ѡ���޼���궨ͼƬ");
	QStringList fileName = QFileDialog::getOpenFileNames(
		this,
		QStringLiteral(""),
		"C:/Users/Lizh/Pictures",
		tr("images(*.png *jpeg *bmp *jpg);;All files(*.*)"));

	QList<QString>::Iterator it = fileName.begin();
	ui.textBrowser_6->append(QString::fromStdString("��ѡ��" + std::to_string(fileName.length()) + "��������궨ͼƬ"));
	for (; it != fileName.end(); it++) {
		Plane_Board_noLaser.push_back(cv::imread((*it).toStdString(), CV_LOAD_IMAGE_GRAYSCALE));
	}
}

//"����"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_4_clicked() {

	/**�Ƿ����ݼ��*/
	if (!filePath.isEmpty() && portNo != 0 && steps != 0) {

		if (isNow) {
			QMessageBox::StandardButton rb = QMessageBox::question(this, "����", "Ԥ���������У��Ƿ�رգ�", QMessageBox::Yes | QMessageBox::No , QMessageBox::Yes);
			if (rb != QMessageBox::Yes)
				return;
			ui.textBrowser_6->append("---------------------Ԥ���ѹر�---------------------");
			cv::destroyWindow(winname);
			isNow = false;
			ui.pushButton_7->setText("Ԥ��");
		}

		QString a, b, c, d;

		a = QString::number(portNo);
		b = QString::number(steps);
		c = QString::number(speed);
		d = QString::number(count);

		/**����ȷ��*/
		int ret = QMessageBox::information(this, "������ʾ", "��ǰ������\n  �˿ڣ�" + a +
			"\n  �ƶ����룺" + b + "\n  �ƶ��ٶȣ�" + c + "\n  ����������" + d,
			QMessageBox::Yes, QMessageBox::No);

		/**���ڿ�������*/
		if (ret == QMessageBox::Yes) {
			std::string a = filePath.toStdString();

			/**�����ʾ��Ϣ*/
			ui.textBrowser_6->append("��ʼ����....\n");

			/**�����������ݣ�a���뱣���ַ����������������ʱ��*/
			cam(com(steps, speed), a, portNo, count, ((steps / speed) * 1000 + 500), ui);

		}

		ui.textBrowser_6->append("��ѡ���ؽ�ʹ��ͼƬ");

		QMessageBox::StandardButton rb = QMessageBox::question(this, "�궨", "ѡ��ͼƬ�����ؽ�", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);
		if (rb != QMessageBox::Yes)
			return;

		QStringList fileName = QFileDialog::getOpenFileNames(
			this,
			QStringLiteral(""),
			filePath,
			tr("images(*.png *jpeg *bmp *jpg);;All files(*.*)"));
		
		QList<QString>::Iterator it = fileName.begin();
		ui.textBrowser_6->append(QString::fromStdString("��ѡ��" + std::to_string(fileName.length()) + "��ͼƬ�����ؽ�"));
		Scan_Laser.clear();
		for (; it != fileName.end(); it++) {
			Scan_Laser.push_back(cv::imread((*it).toStdString(), CV_LOAD_IMAGE_GRAYSCALE));
		}
	}
	else {

		/**δ�����������*/
		if (steps != ui.spinBox->value() || speed != ui.spinBox_2->value()
			|| count != ui.spinBox_3->value() || portNo != ui.spinBox_4->value()) {
			if (steps != ui.spinBox->value())
				QMessageBox::information(this, "������ʾ", " �ƶ����� �����޸�δ���棡");
			else if (speed != ui.spinBox_2->value())
				QMessageBox::information(this, "������ʾ", " �ٶ� �����޸�δ���棡");
			else if (count != ui.spinBox_3->value())
				QMessageBox::information(this, "������ʾ", " �������� �����޸�δ���棡");
			else if (portNo != ui.spinBox_4->value())
				QMessageBox::information(this, "������ʾ", " �˿ں� �����޸�δ���棡");
		}

		/**������Ϣ��ʾ*/
		else if (portNo == 0)
			QMessageBox::critical(this, "������Ϣ��ʾ", "���Ӷ˿ں�δѡ����ѡ��");
		else if (steps == 0)
			QMessageBox::critical(this, "������Ϣ��ʾ", "�ƶ������������Ϊ0���������趨��");
		else if (filePath.isEmpty())
			QMessageBox::information(this, "��Ϣ��ʾ", "��Ƭ�������ڵ�ǰ�ļ����£�");
	}
}

//"�˿�ȷ��"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_5_clicked() {

	QString a;
	a = QString::number(ui.spinBox_4->value());

	/**������ʾ*/
	ui.textBrowser_6->append("�޸ĳɹ���");
	ui.textBrowser_6->append("ͨ�Ŷ˿��޸�Ϊ��" + a);

	/**�˿���Ϣ������"portNo"*/
	portNo = ui.spinBox_4->value();
}

//"�쳣����"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_6_clicked() {
	zero();
}

//"Ԥ��"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_7_clicked() {
	/**Ԥ������*/
	//zero();
	if (!isNow) {
		ui.textBrowser_6->append("----------------------Ԥ������---------------------");

		isNow = true;
		cv::namedWindow(winname, cv::WINDOW_GUI_NORMAL);

		HWND hwnd = (HWND)cvGetWindowHandle(winname.c_str());
		HWND paraent = GetParent(hwnd);//�õ�nameWindow���ڵĸ����
		SetParent(hwnd, (HWND)ui.widget->winId());//����ui�ؼ��ľ���Ǹ����
		ShowWindow(paraent, SW_HIDE);//���ص�nameWindow����
		resizeWindow(winname, cv::Size(900, 700));

		ui.pushButton_7->setText("�ر�Ԥ��");
		nowCa(ui);
	}
	else {
		//ui.widget->setFocus();
		ui.textBrowser_6->append("---------------------Ԥ���ѹر�---------------------");
		cv::destroyWindow(winname);
		isNow = false;
		ui.pushButton_7->setText("Ԥ��");
	}
}

//"�������"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_8_clicked() {
	if (cloud != NULL) {
		cloud->points.clear();
		viewer->updatePointCloud(cloud, "cloud");
		viewer->resetCamera();
		ui.qvtkWidget->update();
		QMessageBox::information(this, "info", "�������������");
		ui.textBrowser->append("�������������");
		ui.textBrowser_6->append("�������������");
	}
	else {
		QMessageBox::information(this, "info", "�����ڵ�������");
		ui.textBrowser->append("�����ڵ�������");
		ui.textBrowser_6->append("�����ڵ�������");
	}
}

//"ѡ�������"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_9_clicked() {
	QMessageBox::StandardButton rb = QMessageBox::question(this, "�궨", "ѡ���������ͼƬ���ڹ�ƽ��궨?", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);
	if (rb != QMessageBox::Yes)
		return;

	ui.textBrowser_6->append("ѡ�������궨ͼƬ");
	QStringList fileName = QFileDialog::getOpenFileNames(
		this,
		QStringLiteral("ѡ�������궨ͼƬ"),
		"C:/Users/Lizh/Pictures",
		tr("images(*.png *jpg *bmp *jpg);;All files(*.*)"));
	QList<QString>::Iterator it = fileName.begin();
	ui.textBrowser_6->append(QString::fromStdString("��ѡ��" + std::to_string(fileName.length()) + "������궨ͼƬ"));

	for (; it != fileName.end(); it++) {
		Plane_Laser.push_back(cv::imread((*it).toStdString(), CV_LOAD_IMAGE_GRAYSCALE));
	}

}

//"�ؽ�"���������Ӧ�¼�
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

			//���ֲ�ͬ�ļ�����ȡ
			switch (ui.comboBox->currentIndex())
			{
			case 0:
				Points = Tool.AverageLine(*it, cv::Point2d(0, 0), cv::Point2d(it->cols, it->rows));
				break;
			case 1:
				Points = Tool.StegerLine(
					*it,

					//��˹�ں˳���
					ui.spinBox_7->value(),
					ui.spinBox_5->value(),

					//��˹XY�����ϵı�׼��
					ui.spinBox_8->value(),
					ui.spinBox_9->value(),

					//��ֵ
					ui.spinBox_6->value(),
					ui.doubleSpinBox->value()
				);
				break;
			}
			end = clock();
			//���Դ��룬���㴦��ʱ��
			double endtime = (double)(end - start) / CLOCKS_PER_SEC;
			averagetime += endtime * 1000;
			ui.textBrowser_6->append("average time = " + QString::number(endtime * 1000) + "ms");
			//���Դ���

			for (int j = 0; j < Points.size(); j++)
			{
				//��������ȡ�������ص����������Ϊ(Points[2 * k + 0], Points[2 * k + 1])
				cv::Point3f Points3d = camera->getWorldPoints(cv::Point2f(Points[j].x, Points[j].y), r, t);
				Points3d.z = (ALaserPlane->Get_D() - ALaserPlane->Get_A() * Points3d.x - ALaserPlane->Get_B() * Points3d.y) / ALaserPlane->Get_C();

				Points3d.y += k * Astep->GetStep().at<double>(1, 0);
				Points3d.x += k * Astep->GetStep().at<double>(0, 0);
				Points3d.z += k * Astep->GetStep().at<double>(2, 0);

				cloud->points.push_back(pcl::PointXYZRGB(Points3d.x, Points3d.y, Points3d.z, ui.spinBox_11->value(), ui.spinBox_12->value(), ui.spinBox_13->value()));
			}
			//��ÿ���������д���
			//�뾶�˲�
			pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
			outrem.setInputCloud(cloud);
			outrem.setRadiusSearch(0.5);
			outrem.setMinNeighborsInRadius(4);
			outrem.filter(*cloud_filtered);

			//��������������Ҫ�����趨
			//CloudProcessTool CloudTool;
			//CloudTool.cutCloud(cloud_filtered, 1.5);

			for (int i = 0; i < cloud_filtered->points.size(); i++)
				Pt.push_back(cloud_filtered->points[i]);
			Points3ds.push_back(Pt);
			Pt.clear();

			cloud_filtered->points.clear();
			cloud->clear();
		}

		//���Դ��룬���㴦��ƽ��ʱ��
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

		//չʾ����Ļ��
		viewer->addPointCloud(cloud, "cloud");
		viewer->updatePointCloud(cloud, "cloud");


		viewer->resetCamera();
		//�����С�趨Ϊ5
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui.spinBox_14->value(), "cloud");
		ui.qvtkWidget->update();


		//ui.extBrowser->append("================finish==================");
		cv::destroyWindow("GetLine");
		QMessageBox::information(NULL, "info", "Caculate finish!", QMessageBox::Ok);


	}
	else
		if (!isCalibed)
			QMessageBox::warning(NULL, "warning", "������ɱ궨!", QMessageBox::Ok);
		else
			QMessageBox::warning(NULL, "warning", "����ѡ���ؽ�ʹ��ͼƬ!", QMessageBox::Ok);

}

//"��ʼ�궨"���������Ӧ�¼�
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
		////����궨���ݻ�ȡ
		camera->calibrate();
		isCameraCalib = true;
		ui.textBrowser_6->append("CameraMatrix:");
		ui.textBrowser_6->append("fx = " + QString::number(camera->GetCameraMatrix().at<double>(0, 0)));
		ui.textBrowser_6->append("fy = " + QString::number(camera->GetCameraMatrix().at<double>(1, 1)));
		ui.textBrowser_6->append("cx = " + QString::number(camera->GetCameraMatrix().at<double>(0, 2)));
		ui.textBrowser_6->append("cy = " + QString::number(camera->GetCameraMatrix().at<double>(1, 2)));

		ui.textBrowser_6->append("Calibrate errors:");
		ui.textBrowser_6->append("Average E��" + QString::number(camera->GetAvg_Err()));

		//��׼����
		//camera->BaseCaculate(Base, caliboard);
		isBaseCalib = true;
		ui.textBrowser_6->append("base caculate finished");
		//��ƽ��궨
		CalibrateBoard* PlaneBoard = new CalibrateBoard(Plane_Board_noLaser.size(), 8, 5, 7);
		//ui.textBrowser_6->append(QString::number(Plane_Board_noLaser.size()));
		//return;
		ALaserPlane->LoadBoard(Plane_Board_noLaser, camera, PlaneBoard);
		ui.textBrowser_6->append(QString::number(ALaserPlane->cout));
		ALaserPlane->LoadLaser(Plane_Laser, camera);

		ALaserPlane->CaculateLaserPlane();
		isPlaneCalib = true;

		ui.textBrowser_6->append("light plane caculate finished");
		ui.textBrowser_6->append("light plane����A��" + QString::number(ALaserPlane->Get_A()));
		ui.textBrowser_6->append("light plane����B��" + QString::number(ALaserPlane->Get_B()));
		ui.textBrowser_6->append("light plane����C��" + QString::number(ALaserPlane->Get_C()));
		ui.textBrowser_6->append("light plane����D��" + QString::number(ALaserPlane->Get_D()));

		//ƴ�Ӽ������
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
		//���ͼƬ����

		Plane_Laser.clear();
		Step_Calculate.clear();
		Scan_Laser.clear();
	}
	else {
		QMessageBox::warning(this, "warning", "��ѡ��궨��ƽ������ͼƬ��", QMessageBox::Ok);
	}

}

//"����궨ͼƬ"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_12_clicked() {

	QMessageBox::StandardButton rb = QMessageBox::question(this, "�궨", "ѡ��ͼƬ���ڼ���궨", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);
	if (rb != QMessageBox::Yes)
		return;

	ui.textBrowser_6->append("ѡ�����궨ʹ��ͼƬ");
	QStringList fileName = QFileDialog::getOpenFileNames(
		this,
		QStringLiteral(""),
		filePath,
		tr("images(*.png *jpeg *bmp *jpg);;All files(*.*)"));

	QList<QString>::Iterator it = fileName.begin();
	ui.textBrowser_6->append(QString::fromStdString("��ѡ��" + std::to_string(fileName.length()) + "��ͼƬ���ڼ���궨"));
	Step_Calculate.clear();
	for (; it != fileName.end(); it++) {
		Step_Calculate.push_back(cv::imread((*it).toStdString(), CV_LOAD_IMAGE_GRAYSCALE));
	}
}

//"�ؽ�ͼƬ"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_13_clicked() {

	QMessageBox::StandardButton rb = QMessageBox::question(this, "�궨", "ѡ��ͼƬ�����ؽ�", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);
	if (rb != QMessageBox::Yes)
		return;

	ui.textBrowser_6->append("ѡ���ؽ�ʹ��ͼƬ");
	QStringList fileName = QFileDialog::getOpenFileNames(
		this,
		QStringLiteral(""),
		filePath,
		tr("images(*.png *jpeg *bmp *jpg);;All files(*.*)"));

	QList<QString>::Iterator it = fileName.begin();
	ui.textBrowser_6->append(QString::fromStdString("��ѡ��" + std::to_string(fileName.length()) + "��ͼƬ�����ؽ�"));
	Scan_Laser.clear();
	for (; it != fileName.end(); it++) {
		Scan_Laser.push_back(cv::imread((*it).toStdString(), CV_LOAD_IMAGE_GRAYSCALE));
	}
}

//"�鿴ͼƬ"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_14_clicked() {

	//����Դ�������������ļ�
	const QString explorer = "explorer";
	QStringList param;
	if (!QFileInfo(filePath).isDir()) {
		param << QLatin1String("/select,");
	}
	param << QDir::toNativeSeparators(filePath);
	QProcess::startDetached(explorer, param);
}

//"�ӽǹ�λ"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_15_clicked() {
	viewer->resetCamera();
	ui.qvtkWidget->update();
}

//"�����ؽ�"���������Ӧ�¼�
void QtWidgetsApplication2::on_pushButton_16_clicked() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	//pcl::io::loadPCDFile("rabbit.pcd", cloud_blob);
	//pcl::fromPCLPointCloud2(cloud_blob, *cloud1);
	pcl::copyPointCloud(*cloud, *cloud1);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //���������ƶ���ָ��
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	tree->setInputCloud(cloud1);                        //��cloud����tree����
	n.setInputCloud(cloud1);                            //Ϊ���߹��ƶ��������������
	n.setSearchMethod(tree);                          //������������
	n.setKSearch(20);                                 //����k������kֵΪ20
	n.compute(*normals);                              //���Ʒ��ߴ洢�����normals��
	//�����ƺͷ��߷ŵ�һ��

	pcl::concatenateFields(*cloud1, *normals, *cloud_with_normals);//�����ֶΣ�cloud_with_normals�洢�������

	//����������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	//// �����ؽ�
	////pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
	////mls.setComputeNormals(true);  //��������С���˼�������Ҫ���з��߹���
	////mls.setInputCloud(cloud_with_normals);//���ò���
	////mls.setPolynomialFit(true);
	////mls.setSearchMethod(tree2);
	////mls.setSearchRadius(0.03);
	////pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_msl(new pcl::PointCloud<pcl::PointNormal>);
	////mls.process(*cloud_with_normals_msl);
	////cloud_with_normals = cloud_with_normals_msl;
	////std::cerr << "�����ؽ�   ���" << std::endl;

	//// ��ʼ�����ؽ� ********************************************************************

	//����Poisson���󣬲����ò���
	//pcl::Poisson<pcl::PointNormal> pn;
	//pn.setConfidence(false); //�Ƿ�ʹ�÷������Ĵ�С��Ϊ������Ϣ�����false�����з���������һ����
	//pn.setDegree(2); //���ò���degree[1,5],ֵԽ��Խ��ϸ����ʱԽ�á�
	//pn.setDepth(8);
	////���������ȣ����2^d x 2^d x 2^d������Ԫ��
	//// ���ڰ˲�������Ӧ�����ܶȣ�ָ��ֵ��Ϊ�����ȡ�
	//pn.setIsoDivide(8); //������ȡISO��ֵ����㷨�����
	//pn.setManifold(false); //�Ƿ���Ӷ���ε����ģ�����������ǻ�ʱ�� 
	//// �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
	//pn.setOutputPolygons(false); //�Ƿ������������񣨶��������ǻ��ƶ�������Ľ����
	//pn.setSamplesPerNode(3.0); //��������һ���˲�������е����������С��������������[1.0-5.0],������[15.-20.]ƽ��
	//pn.setScale(1.25); //���������ع���������ֱ���������߽�������ֱ���ı��ʡ�
	//pn.setSolverDivide(8); //����������Է������Gauss-Seidel�������������
	////pn.setIndices();
	////���������������������
	//pn.setSearchMethod(tree2);
	//pn.setInputCloud(cloud_with_normals);
	////����������������ڴ洢���
	//pcl::PolygonMesh mesh;
	////ִ���ع�
	//pn.performReconstruction(mesh);
	//��������ͼ


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

//"�ƶ�����"��Ӧ���ݿ����ݸı���Ӧʱ�䣬�޸�"�޸Ĳ���"���ɵ����
void QtWidgetsApplication2::on_spinBox_valueChanged() {

	/**��ֵΪ0ʱ���ɵ��*/
	if (ui.spinBox->value() != 0) {
		ui.pushButton->setEnabled(true);
	}
	else {
		ui.pushButton->setEnabled(false);
	}
}

//"ͨ�Ŷ˿�"��Ӧ���ݿ����ݸı���Ӧʱ�䣬�޸�"�˿�ȷ��"���ɵ����
void QtWidgetsApplication2::on_spinBox_4_valueChanged() {

	/**��ֵΪ0ʱ���ɵ��*/
	if (ui.spinBox_4->value() != 0) {
		ui.pushButton_5->setEnabled(true);
	}
	else {
		ui.pushButton_5->setEnabled(false);
	}
}

//"red"���������Ƶ�����ɫ0
void QtWidgetsApplication2::on_spinBox_11_valueChanged() {
	changeColor();
}

//"green"���������Ƶ�����ɫ
void QtWidgetsApplication2::on_spinBox_12_valueChanged() {
	changeColor();
}

//"blue"���������Ƶ�����ɫ
void QtWidgetsApplication2::on_spinBox_13_valueChanged() {
	changeColor();
}

//"size"���������Ƶ��Ƶ��С
void QtWidgetsApplication2::on_spinBox_14_valueChanged() {
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui.spinBox_14->value(), "cloud");
	ui.qvtkWidget->update();
}

//��ɫ�ı亯��
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