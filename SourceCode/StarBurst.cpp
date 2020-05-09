#include "data.h"
#include "StarBurst.h"
#include "StarBurstHelper.h"
#include "ransac.h"

// atan2()���� 180/PI�� �ؾ� radian������ �����ټ��� ����.
extern double pupil_param[5];
StarBurst::StarBurst(Mat image, int CONTOUR_THRESHOLD) {
	// initializing...
	this->m_image = NULL;
	this->m_removed_image = NULL;
	this->m_image = image;
	this->CONTOUR_THRESHOLD = CONTOUR_THRESHOLD;
	// Clear Info Data Structure
	memset(&info, 0, sizeof(StarBurstInfo));
	edge_thresh = 6;
}

StarBurst::~StarBurst(void) {}

StarBurstInfo * StarBurst::Apply(void) {
	// create StarBurstHelper Class for PreProcessing...
	// Step 1: Fiding Corneal Reflex
	// Step 2: Remove Corneal Reflex by Interporation
	printf("Corneal Reflection Detect Start ---------\n");
	//if (helper == NULL) {
	helper = new StarBurstHelper(this->m_image);
	this->m_removed_image = helper->CornealReflexRemove(info);
	printf("Corneal Reflection Detect end ---------\n");
	printf("StarBurst Start ---------\n");
	ApplyRANSAC();
	return (&info);
}

void StarBurst::ApplyRANSAC(void) {
	// (1) ����������� �����ϱ�
	DetectCoutourPoints();
	printf("StarBurst end ---------\n");
	printf("Ransac Start ---------\n");
	// (2) RANSAC ����
	int max;
	int* inliers_index = pupil_fitting_inliers((UINT8*)m_removed_image.data, m_removed_image.size().width, m_removed_image.size().height, max, this->edge_point);

	printf("Ransac end ---------\n");
	info.PUPIL.width = (int)pupil_param[0];
	info.PUPIL.height = (int)pupil_param[1];
	info.PUPIL.center_of_x = (int)pupil_param[2];
	info.PUPIL.center_of_y = (int)pupil_param[3];
	info.PUPIL.theta = pupil_param[4];

	//printf("inliers_index %d\n", *inliers_index);
	//printf("���� Ÿ�� pupil ���� %d %d %d %d %d\n\n", (int)pupil_param[0], (int)pupil_param[1], (int)pupil_param[2], (int)pupil_param[3], (int)pupil_param[4]);
	//printf("ellipse width:%lf; height:%lf, pupil_x:%lf, pupil_y:%lf, theta:%lf inlier_index: %d; \n", pupil_param[0], pupil_param[1], pupil_param[2], pupil_param[3], pupil_param[4], inliers_index);
}

Mat StarBurst::getCornealRemovedImage(void) {
	return this->m_removed_image;
}

bool compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2)
{
	double i = fabs(contourArea(cv::Mat(contour1)));
	double j = fabs(contourArea(cv::Mat(contour2)));
	return (i > j);
}

void StarBurst::DetectCoutourPoints(void) {
	// �ʱ� ������ ã�� (contourpoint�� �ִ��� ���� �־ �����������ϰ� -> boundingrect 
	Mat threshold_image;
	vector<vector<Point>> Contours;
	//threshold(m_image, threshold_image, 12, 255, THRESH_BINARY_INV); // ������ �׻� ��Ĵٴ� �����Ͽ� �Ӱ谪�� �ִ��� ���߾� ������ ��Ƴ��� �ϰ�, 
	threshold(m_image, threshold_image, CONTOUR_THRESHOLD, 255, THRESH_BINARY_INV); // ������ �׻� ��Ĵٴ� �����Ͽ� �Ӱ谪�� �ִ��� ���߾� ������ ��Ƴ��� �ϰ�, 
	findContours(threshold_image, Contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	sort(Contours.begin(), Contours.end(), compareContourAreas);

	if (Contours.size() != 0) {
		rect1 = boundingRect(Contours[0]);
		rectangle(this->ex_image, rect1.tl(), rect1.br(), (0, 0, 255), 2);
	}
	// �������� ������ �߾�����
	int guess_crx = (rect1.tl().x + rect1.br().x) / 2; // rect.tl() �簢���� ������ �� rect.br() �簢���� ���ʾƷ�
	int guess_cry = (rect1.tl().y + rect1.br().y) / 2;
	// int guess_crx = this->m_removed_image.cols / 2;	// Best Guess Pupil center x
	// int guess_cry = this->m_removed_image.rows / 2 -20;	// Best Guess Pupil center y

	// Cleare All Edge Point
	int new_crx = 1, new_cry = 1; // x, y �Ѵ� ���� -1
	Point point; // ���� ���� ���Ͽ�
	vector<Point> vec_center_point;
	int k = 25; // ���� ���� -> k���ø��� ���� �� �����ϰ� �׵θ��� �� �����ϰ�
	//vec_center_point.push_back(Point(guess_crx, guess_cry));
	int stop = 1;
	// �Ķ���: �����ݻ��� / ������: �����߽� ������ / ��ȫ�� : �����߽� ���� �̵���
	// �����: 1���׵θ��� / �ʷϻ�: 2�� �׵θ��� 
	// ���� ������ ��ġ ���̰� 1�� �� (���� �߾����� �������� ��)
	while ((abs(guess_crx - new_crx) > 1) && (abs(guess_cry - new_cry) > 1)) {
		edge_point.clear();
		// new_crx �ʱ갪�� 1�̶�� ������ ó���� new_crx�� �ݿ��Ǵ°��� �������� �� do-while�� ���� ����.
		if (new_crx != 1 && new_cry != 1) {
			guess_crx = new_crx; guess_cry = new_cry;
			/*
			vec_center_point.push_back(Point(guess_crx, guess_cry));
			// �߽������� ������ �߽������ ��� �ݺ��Ǿ� ���ѷ����߻�����
			for (int i = 0; i < vec_center_point.size(); i++) {
				if (vec_center_point[i] == Point(guess_crx, guess_cry)) {
					stop = 0;
				}
			}
			*/
		}
		if (stop == 0)break;
		//printf("center [%3d , %3d]\n", guess_crx, guess_cry);

		// Step 1 : Finding Candicate
		// 0������ 25���� 360������ ���� - �������� K
		// i�� ���°� 1, 26, 51 ... �ε� 0�� 25�� ... �� ������ cos_array, sin_array������ 0���� ���ֱ� ����
		// 1�� ����� ã�� 25�� �������� 
		for (int i = 1; i < helper->angle_num; i = i + k) {
			shotRay(i, guess_crx, guess_cry, edge_thresh);
		}
		//printf("1�� �׵θ� ã�� ��\n");

		// draw - �����ݻ��� �׷�����
		if (draw_on) {
			circle(this->ex_image, Point(info.CORNEAL_REFLEX.center_of_x, info.CORNEAL_REFLEX.center_of_y), 2, BLUE, -1, 8);

			// draw - ���� ���� �߽� �׷�����
			circle(this->ex_image, Point(guess_crx, guess_cry), 2, RED, -1, 8);

			// draw - 1�� �׵θ� �׷�����. (�Ķ�)
			for (int i = 0; i < edge_point.size(); i++) {
				circle(this->ex_image, Point(edge_point[i].x, edge_point[i].y), 2, YELLOW, -1, 8);
			}
		}
		int degree = 0;

		double theta;
		// edge_point 20�������� ì��. 
		int last = edge_point.size();
		if (last > 20)
			last = 20;

		// 2���׵θ�ã�� & ��� edge_point�� ���������߽���(guess_crx, guess_cry)�� ����.
		for (int i = 0; i < last; i++) {
			int x = edge_point[i].x;
			int y = edge_point[i].y;

			//printf("���󵿰��߽��� guess(%d, %d)\n", guess_crx, guess_cry);
			//printf("%d��° edge_point�� (%d %d)\n", i+1, x, y);

			// �̺κ��� ������ �� ó���ǰ� �ִ��� Ȯ���� �ʿ���
			// ���� �����߽������� ������ �Ʒ��� ���
			if (guess_crx < x && guess_cry < y) {
				theta = atan2((double)y - guess_cry, (double)x - guess_crx);
				degree = (int)(2 * PI / theta) + 180;
			}

			// ���� �����߽������� ������ ���� ���
			else if (guess_crx > x && guess_cry < y) {
				theta = atan2((double)y - guess_cry, (double)guess_crx - x);
				degree = 360 - (int)(2 * PI / theta);
			}

			// ���� �����߽������� ���� ���� ���
			else if (guess_crx > x && guess_cry > y) {
				theta = atan2((double)y - guess_cry, (double)guess_crx - x);
				degree = (int)(2 * PI / theta);
			}

			// ���� �����߽������� ���� �Ʒ��� ���
			else if (guess_crx < x && guess_cry > y) {
				theta = atan2((double)guess_cry - y, (double)x - guess_crx);
				degree = 180 - (int)(2 * PI / theta);
			}

			// 2�� �׵θ� ã�� �� ���������� -50�� ~ 50�� ���� �������� �ٽ� �߰������� ������ ��� ã�� ��!!!
			for (int j = -50; j <= 50; j = j + k) {
				shotRay(degree + j, x, y, edge_thresh);
			}
		} // for�� - ��� edge_point ������

		// ��� ���� ������� ���ο� �߽��� ���ϱ� 
		point = calculateCoverage();
		new_crx = point.x;
		new_cry = point.y;

		// draw - ���� �߽��� �׷�����
		if (draw_on) {
			// draw - 2�� �׵θ� �׷�����. (�ʷ�) - �߰��� �����鸸
			for (int i = last; i < edge_point.size(); i++) {
				circle(this->ex_image, Point(edge_point[i].x, edge_point[i].y), 1, GREEN, -1, 8);
			}

			circle(ex_image, point, 2, MAGENTA, -1, 8);
			imshow("Corneal Reflection Removed Image View", this->m_removed_image);
			imshow("Pupil View", this->ex_image);
			waitKey(1);
		}
	} // while�� (guess�� - new���� ���̰� ������ stop)
	info.PUPIL.center_of_x = new_crx;
	info.PUPIL.center_of_y = new_cry;
}

// Feature Based Approach
// i = 1������ x���� ���� ��. , guess_x , guess_y, thresh
// 1���϶� ���ͼ� ������ 1�� �÷����鼭 pixelvalue ���غ��� ��.
void StarBurst::shotRay(int angleDegree, int x, int y, int thresh) {
	int angle = fitDegree(angleDegree);
	double r = 0.0;
	double xx = -1.0;			// x'
	double yy = -1.0;			// y'

	// init center pixel value
	int pixelvalue1 = m_removed_image.ptr<uchar>(y)[x];
	// int pixelvalue1 = *(m_removed_image.data + (y*(m_removed_image.size().width)) + x);
	while (1) {
		// ������ * cos(angle) - x�� �� or sin(angle) - y�� �� 
		xx = (r * helper->cos_array[angle]) + x;
		yy = (r * helper->sin_array[angle]) + y;

		// ������ ������Ű�鼭 ��� ã�Ƴ����µ� ������̿�����
		if (isBorder(xx, yy)) {
			printf("xx yy ���� �������\n");
			return;
		}
		else {
			int pixelvalue2 = m_removed_image.ptr<uchar>((int)yy)[(int)xx];
			//int pixelvalue2 = *(m_removed_image.data + ((int)yy*(m_removed_image.size().width)) + (int)xx);
			// ## �߿� print printf("%d�� ���� ���� ������(pix1): %d �񱳰�(pix2): %d\n", angle-1, pixelvalue1, pixelvalue2);

			// �ȼ������̰� �Ӱ谪���� ũ�� �����ϱ� �����ϰ� ����
			if (abs(pixelvalue2 - pixelvalue1) > thresh) {
				//printf("%d - %d = %d / %d\n",pixelvalue2 , pixelvalue1,pixelvalue2 -pixelvalue1 , thresh);
				// �׻� ���ʿ� ������ ��ġ�� �� �ֵ��� ��!!
				// 3.0�� ���� ������ ���� ���� �׵θ� �ȼ����� ���� �������� ������ؼ� �� �� 3pixel? ���� ���� edge�� �����ϴ°�
				xx = (r - 3.0) * helper->cos_array[angle] + (double)x;
				yy = (r - 3.0) * helper->sin_array[angle] + (double)y;

				CvDPoint point;
				point.x = xx; point.y = yy;
				edge_point.push_back(point);
				//printf("%d��° DETECT CONTOUR POINT = [%3d, %3d] \n", edge_point.size(), (int)xx, (int)yy);
				info.PUPIL.center_of_x = (int)point.x;
				info.PUPIL.center_of_y = (int)point.y;
				return;
			} // if�� pixelvalue2 - pixelvalue1
		}
		r = r + 1.0;
		//Sleep(10);
	} //  ��ü while��
}

// Ȥ�ó� �������� 360���� ������ 0~360 ������ ���߱�
int StarBurst::fitDegree(int degree) {
	if (degree >= 0 && degree < 360) {
		return degree;
	}
	else if (degree >= 360) {
		return degree % 360; //degree - 360;
	}
	else if (degree < 0) {
		return 360 + degree;
	}
}

// ��輱���� üũ
bool StarBurst::isBorder(int xx, int yy) {
	/*
	if((xx <= rect.x || xx >= rect.width +rect.x||
		yy <= rect.y || yy >= rect.height +rect.y))
	{
	*/
	if ((xx < 0 || xx >= m_removed_image.size().width ||
		yy < 0 || yy >= m_removed_image.size().height))
	{
		return true;
	}
	return false;
}

// ���� ��� ���ؼ� next �߽��� ��ȯ
Point StarBurst::calculateCoverage(void) {
	// size�� 0�̸� �׳� ������ �߽��� ��ȯ
	if (edge_point.size() == 0) {
		return Point(m_removed_image.size().width / 2, m_removed_image.size().height / 2);
	}
	int avg_x = 0;
	int avg_y = 0;
	int s = edge_point.size();
	for (int i = 0; i < s; i++) {
		avg_x = avg_x + edge_point[i].x;
		avg_y = avg_y + edge_point[i].y;
	}
	return Point(avg_x / s, avg_y / s);
}
