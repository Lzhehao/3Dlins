#include "ImageProcess.h"


std::vector<cv::Point2d> ProcessTool::AverageLine(cv::Mat img0, cv::Point2d leftup, cv::Point2f rightdown)
{
	//Ԥ����
	std::vector<cv::Point2d> result;
	cv::Mat  img;
	img = img0.clone();
	cv::Size smallSize;
	smallSize.height = rightdown.y;
	smallSize.width = rightdown.x;
	cv::Mat Img = cv::Mat(smallSize, CV_8UC3, cv::Scalar(0, 0, 0));// ������ͨ����ɫͼ��
	//GaussianBlur(img0, img0, Size(3, 3), 2, 2);
	cv::medianBlur(img0, img0, 5);
	//cvtColor(img0, img, CV_RGB2GRAY);
	//threshold(img0, img0, 250, 255, cv::THRESH_OTSU);
	for (size_t i = leftup.y; i < rightdown.y; i++)
	{
		//cout << "a" << endl;
		int sum = 0; int num = 0;
		for (size_t j = leftup.x; j < rightdown.x; j++)
		{
			if (img0.at<uchar>(i, j) == 255)
			{
				Img.at<uchar>(i, j * 3) = 255;
				Img.at<uchar>(i, j * 3 + 1) = 255;
				Img.at<uchar>(i, j * 3 + 2) = 255;
				sum += j;
				num++;
			}
		}
		if (num == 0)
			continue;
		result.push_back(cv::Point2d(1.0 * sum / (num * 1.0),i));
	}
	ShowLine(result, Img);
	return result;

}

std::vector<cv::Point2d> ProcessTool::StegerLine(cv::Mat img0, int col, int row, int sqrtx, int sqrty, int shreshold, float distance)
{
	if (img0.channels() == 3)
		cvtColor(img0, img0, CV_BGR2GRAY);
	cv::Mat img;
	img = img0.clone();
	//��˹�˲�
	img.convertTo(img, CV_32FC1);
	//����,ѡ���߿�
	GaussianBlur(img, img, cv::Size(col, row), sqrtx, sqrty);

	//��˹�������õ�һ��ɽ��ǿ�ȵĹ���ͼ
	//��˹������ʵ���Ͼ���һ����̬����
	// 
	//�����������ľ�����
	//����ʹ�õ�����ֵ�����ƫ�������ǵõ��󵼹�ʽ
	cv::Mat m1, m2;
	m1 = (cv::Mat_<float>(1, 2) << 1, -1);//xƫ��������һ��Mat�����һ��ƫ��=f(x+1,y)-f(x,y)
	m2 = (cv::Mat_<float>(2, 1) << 1, -1);//yƫ�����������һ��ƫ��=f(x,y+1)-f(x,y)
	cv::Mat dx, dy;
	//ʹ��m1���� ��ǰ����ĻҶ�ֵ-��x+1���ĻҶ�ֵ
	filter2D(img, dx, CV_32FC1, m1);//����
	filter2D(img, dy, CV_32FC1, m2);//����
	//����ƫ����
	cv::Mat m3, m4, m5;
	m3 = (cv::Mat_<float>(1, 3) << 1, -2, 1);//�����xƫ���ľ���=f(x+1,y)+f(x-1,y)-2f(x,y)
	m4 = (cv::Mat_<float>(3, 1) << 1, -2, 1);//����yƫ���ľ���
	m5 = (cv::Mat_<float>(2, 2) << 1, -1, -1, 1);//����xyƫ������=f(x+1,y+1)-f(x+1,y)-f(x,y+1)+f(x,y)

	//Hessian������һ���Գƾ��󣬶������;���Ҳ�ǶԳƾ���
	cv::Mat dxx, dyy, dxy;
	//��ö�Ӧ���ƫ����ֵ
	filter2D(img, dxx, CV_32FC1, m3);
	filter2D(img, dyy, CV_32FC1, m4);
	filter2D(img, dxy, CV_32FC1, m5);

	//hessian����
	double maxD = -1;
	int imgcol = img.cols;
	int imgrow = img.rows;

	std::vector<cv::Point2d> Pt;
	//std::vector<double> Pt;

	//
	for (int j = imgrow - 1; j >= 0; j--)
	{
		for (int i = 0; i < imgcol; i++)
		{
			//����һ����ֵ��ʱ��ʼ����
			if (img0.at<uchar>(cv::Point(i, j)) > shreshold)
			{
				cv::Mat hessian(2, 2, CV_32FC1);
				//�õ���Ӧ���hessian����
				//��ÿ���㣬�����hessian����
				hessian.at<float>(0, 0) = dxx.at<float>(j, i);
				hessian.at<float>(0, 1) = dxy.at<float>(j, i);
				hessian.at<float>(1, 0) = dxy.at<float>(j, i);
				hessian.at<float>(1, 1) = dyy.at<float>(j, i);

				cv::Mat eValue;
				cv::Mat eVectors;
				//hessian����2*2 �Գƾ��� ����������ֵ
				//��ȡ��ǰ�������ֵ����������
				// ������������ֵ����Ӧ��������
				// ����ֵһ��Ϊ��һ��Ϊ����
				//		eValue: [3.3130465;
				//				-37.06522]
				//	eVectors : [0.89511645, 0.44583231;
				//				-0.44583231, 0.89511645]
				eigen(hessian, eValue, eVectors);
				//std::cout << "eValue:" << eValue << std::endl << "eVectors:" << eVectors << std::endl;
				//nxΪ����ֵ����Ӧ��x��nyΪ����ֵ����Ӧ��y
				double nx, ny;
				double fmaxD = 0;
				//������ֵ����ֵ���ʱ��Ӧ����������
				//Hessian���������ֵ�����������ڸõ㸽��������������İ�͹�ԣ�����ֵԽ��͹��Խǿ��
				if (fabs(eValue.at<float>(0, 0)) >= fabs(eValue.at<float>(1, 0)))
				{
					nx = eVectors.at<float>(0, 0);
					ny = eVectors.at<float>(0, 1);
					fmaxD = eValue.at<float>(0, 0);
				}
				else
				{
					nx = eVectors.at<float>(1, 0);
					ny = eVectors.at<float>(1, 1);
					fmaxD = eValue.at<float>(1, 0);
				}
				// ʹ��̩��չ����ʾ��ǿ�ķֲ�����f(x+t*nx,y+t*ny)���Ա�ʾ��ǰ���긽���Ĺ�ǿ�ֲ�
				// f(x+t*nx,y+t*ny) hessian��������ֵ���Ķ�Ӧ����������(nx��ny)��ʾ�ݶ��������ķ��򣨹�ǿ�ı���죩��Ҫ���������ı�  
				// ����x��y��nx,ny��֪�������ǰ���긽���Ĺ�ǿ�ֲ��������Կ����ǹ���t��һ���ֲ�����
				// ����Զ�t�󵼣����Ϊ0�ĵط����ǹ�ǿ��ǿ�ĵط�����ֵ����Ҳ���Ǽ������ĵ㡣
				// ���t*nx��t*ny��С��0.5˵�������ֵ���λ�ڵ�ǰ�����ڣ�������ǿ�ֲ���������
				// ���t*nx��t*ny���ܴ󣬸�����ǿ�ֲ����������ã�����������㣬����ɨ����뼤�����ĸ����ĵ�
				double t = -
					(nx * dx.at<float>(j, i) + ny * dy.at<float>(j, i))
					/
					(nx * nx * dxx.at<float>(j, i) + 2 * nx * ny * dxy.at<float>(j, i) + ny * ny * dyy.at<float>(j, i));
				//t* nx��t* nx�����������ĵ���뵱ǰ���صľ���
				if (fabs(t * nx) <= distance && fabs(t * ny) <= distance)
				{
					Pt.push_back(cv::Point2d(i, j));
				}
			}
		}
	}
	cvtColor(img0, img0, CV_GRAY2BGR);
	ShowLine(Pt, img0);
	return Pt;
}

void ProcessTool::ShowLine(std::vector<cv::Point2d> Points, cv::Mat image)
{
	for (int k = 0; k < Points.size(); k++)
		image.at<cv::Vec3b>(Points[k].y, Points[k].x) = cv::Vec3b(0, 0, 255);
	cv::namedWindow("GetLine", cv::WINDOW_NORMAL);
	cv::imshow("GetLine", image);
	if (1)
	{
		std::ostringstream s;
		// �����������ļ����洢ͼƬ
		s << "image_"  << ".jpg";
		std::string imageName(s.str());
		//����OpenCV image.
		cv::imwrite(imageName, image);
	}
	cv::waitKey(1);

}
