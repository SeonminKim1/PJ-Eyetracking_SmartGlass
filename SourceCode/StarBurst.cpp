#include "data.h"
#include "StarBurst.h"
#include "StarBurstHelper.h"
#include "ransac.h"

// atan2()에서 180/PI를 해야 radian단위로 맞춰줄수가 있음.
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
	// (1) 동공경계점들 추출하기
	DetectCoutourPoints();
	printf("StarBurst end ---------\n");
	printf("Ransac Start ---------\n");
	// (2) RANSAC 실행
	int max;
	int* inliers_index = pupil_fitting_inliers((UINT8*)m_removed_image.data, m_removed_image.size().width, m_removed_image.size().height, max, this->edge_point);

	printf("Ransac end ---------\n");
	info.PUPIL.width = (int)pupil_param[0];
	info.PUPIL.height = (int)pupil_param[1];
	info.PUPIL.center_of_x = (int)pupil_param[2];
	info.PUPIL.center_of_y = (int)pupil_param[3];
	info.PUPIL.theta = pupil_param[4];

	//printf("inliers_index %d\n", *inliers_index);
	//printf("최종 타원 pupil 정보 %d %d %d %d %d\n\n", (int)pupil_param[0], (int)pupil_param[1], (int)pupil_param[2], (int)pupil_param[3], (int)pupil_param[4]);
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
	// 초기 시작점 찾기 (contourpoint를 최대한 낮게 주어서 동공만남게하고 -> boundingrect 
	Mat threshold_image;
	vector<vector<Point>> Contours;
	//threshold(m_image, threshold_image, 12, 255, THRESH_BINARY_INV); // 동공은 항상 까맣다는 전제하에 임계값을 최대한 낮추어 동공만 살아남게 하고, 
	threshold(m_image, threshold_image, CONTOUR_THRESHOLD, 255, THRESH_BINARY_INV); // 동공은 항상 까맣다는 전제하에 임계값을 최대한 낮추어 동공만 살아남게 하고, 
	findContours(threshold_image, Contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	sort(Contours.begin(), Contours.end(), compareContourAreas);

	if (Contours.size() != 0) {
		rect1 = boundingRect(Contours[0]);
		rectangle(this->ex_image, rect1.tl(), rect1.br(), (0, 0, 255), 2);
	}
	// 시작점을 영상의 중앙으로
	int guess_crx = (rect1.tl().x + rect1.br().x) / 2; // rect.tl() 사각형의 오른쪽 위 rect.br() 사각형의 왼쪽아래
	int guess_cry = (rect1.tl().y + rect1.br().y) / 2;
	// int guess_crx = this->m_removed_image.cols / 2;	// Best Guess Pupil center x
	// int guess_cry = this->m_removed_image.rows / 2 -20;	// Best Guess Pupil center y

	// Cleare All Edge Point
	int new_crx = 1, new_cry = 1; // x, y 둘다 원래 -1
	Point point; // 다음 점을 위하여
	vector<Point> vec_center_point;
	int k = 25; // 각도 변수 -> k값올리면 직선 더 촘촘하게 테두리점 더 촘촘하게
	//vec_center_point.push_back(Point(guess_crx, guess_cry));
	int stop = 1;
	// 파란색: 각막반사점 / 빨간색: 동공중심 예상점 / 분홍색 : 동공중심 다음 이동점
	// 노란색: 1차테두리점 / 초록색: 2차 테두리점 
	// 다음 점과의 위치 차이가 1일 때 (거의 중앙으로 수렴했을 때)
	while ((abs(guess_crx - new_crx) > 1) && (abs(guess_cry - new_cry) > 1)) {
		edge_point.clear();
		// new_crx 초깃값을 1이라고 했으니 처음에 new_crx로 반영되는것을 막기위해 즉 do-while문 같은 개념.
		if (new_crx != 1 && new_cry != 1) {
			guess_crx = new_crx; guess_cry = new_cry;
			/*
			vec_center_point.push_back(Point(guess_crx, guess_cry));
			// 중심점들이 서로의 중심점들로 계속 반복되어 무한루프발생막음
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
		// 0도부터 25도씩 360도까지 돌림 - 각도변수 K
		// i에 들어가는게 1, 26, 51 ... 인데 0도 25도 ... 인 이유는 cos_array, sin_array값들이 0부터 되있기 때문
		// 1차 경계점 찾기 25도 간격으로 
		for (int i = 1; i < helper->angle_num; i = i + k) {
			shotRay(i, guess_crx, guess_cry, edge_thresh);
		}
		//printf("1차 테두리 찾기 끝\n");

		// draw - 각막반사점 그려보기
		if (draw_on) {
			circle(this->ex_image, Point(info.CORNEAL_REFLEX.center_of_x, info.CORNEAL_REFLEX.center_of_y), 2, BLUE, -1, 8);

			// draw - 추측 원의 중심 그려보기
			circle(this->ex_image, Point(guess_crx, guess_cry), 2, RED, -1, 8);

			// draw - 1차 테두리 그려보기. (파랑)
			for (int i = 0; i < edge_point.size(); i++) {
				circle(this->ex_image, Point(edge_point[i].x, edge_point[i].y), 2, YELLOW, -1, 8);
			}
		}
		int degree = 0;

		double theta;
		// edge_point 20개까지만 챙김. 
		int last = edge_point.size();
		if (last > 20)
			last = 20;

		// 2차테두리찾기 & 모든 edge_point를 추측동공중심점(guess_crx, guess_cry)와 비교함.
		for (int i = 0; i < last; i++) {
			int x = edge_point[i].x;
			int y = edge_point[i].y;

			//printf("예상동공중심점 guess(%d, %d)\n", guess_crx, guess_cry);
			//printf("%d번째 edge_point점 (%d %d)\n", i+1, x, y);

			// 이부분의 공식이 잘 처리되고 있는지 확인이 필요함
			// 추측 동공중심점보다 오른쪽 아래일 경우
			if (guess_crx < x && guess_cry < y) {
				theta = atan2((double)y - guess_cry, (double)x - guess_crx);
				degree = (int)(2 * PI / theta) + 180;
			}

			// 추측 동공중심점보다 오른쪽 위일 경우
			else if (guess_crx > x && guess_cry < y) {
				theta = atan2((double)y - guess_cry, (double)guess_crx - x);
				degree = 360 - (int)(2 * PI / theta);
			}

			// 추측 동공중심점보다 왼쪽 위일 경우
			else if (guess_crx > x && guess_cry > y) {
				theta = atan2((double)y - guess_cry, (double)guess_crx - x);
				degree = (int)(2 * PI / theta);
			}

			// 추측 동공중심점보다 왼쪽 아래일 경우
			else if (guess_crx < x && guess_cry > y) {
				theta = atan2((double)guess_cry - y, (double)x - guess_crx);
				degree = 180 - (int)(2 * PI / theta);
			}

			// 2차 테두리 찾기 각 엣지점에서 -50도 ~ 50도 사이 방향으로 다시 추가적으로 직선을 뻗어서 찾는 것!!!
			for (int j = -50; j <= 50; j = j + k) {
				shotRay(degree + j, x, y, edge_thresh);
			}
		} // for문 - 모든 edge_point 돌리는

		// 모든 엣지 평균으로 새로운 중심점 구하기 
		point = calculateCoverage();
		new_crx = point.x;
		new_cry = point.y;

		// draw - 다음 중심점 그려보기
		if (draw_on) {
			// draw - 2차 테두리 그려보기. (초록) - 추가된 엣지들만
			for (int i = last; i < edge_point.size(); i++) {
				circle(this->ex_image, Point(edge_point[i].x, edge_point[i].y), 1, GREEN, -1, 8);
			}

			circle(ex_image, point, 2, MAGENTA, -1, 8);
			imshow("Corneal Reflection Removed Image View", this->m_removed_image);
			imshow("Pupil View", this->ex_image);
			waitKey(1);
		}
	} // while문 (guess와 - new와의 차이가 없으면 stop)
	info.PUPIL.center_of_x = new_crx;
	info.PUPIL.center_of_y = new_cry;
}

// Feature Based Approach
// i = 1도부터 x도씩 더한 값. , guess_x , guess_y, thresh
// 1도일때 들어와서 반지름 1씩 늘려가면서 pixelvalue 비교해보는 것.
void StarBurst::shotRay(int angleDegree, int x, int y, int thresh) {
	int angle = fitDegree(angleDegree);
	double r = 0.0;
	double xx = -1.0;			// x'
	double yy = -1.0;			// y'

	// init center pixel value
	int pixelvalue1 = m_removed_image.ptr<uchar>(y)[x];
	// int pixelvalue1 = *(m_removed_image.data + (y*(m_removed_image.size().width)) + x);
	while (1) {
		// 반지름 * cos(angle) - x축 값 or sin(angle) - y축 값 
		xx = (r * helper->cos_array[angle]) + x;
		yy = (r * helper->sin_array[angle]) + y;

		// 반지름 증가시키면서 계속 찾아나가는데 경계점이였으면
		if (isBorder(xx, yy)) {
			printf("xx yy 점이 경계점임\n");
			return;
		}
		else {
			int pixelvalue2 = m_removed_image.ptr<uchar>((int)yy)[(int)xx];
			//int pixelvalue2 = *(m_removed_image.data + ((int)yy*(m_removed_image.size().width)) + (int)xx);
			// ## 중요 print printf("%d도 방향 값들 기존값(pix1): %d 비교값(pix2): %d\n", angle-1, pixelvalue1, pixelvalue2);

			// 픽셀값차이가 임계값보다 크면 엣지니까 저장하고 리턴
			if (abs(pixelvalue2 - pixelvalue1) > thresh) {
				//printf("%d - %d = %d / %d\n",pixelvalue2 , pixelvalue1,pixelvalue2 -pixelvalue1 , thresh);
				// 항상 안쪽에 값으로 위치할 수 있도록 함!!
				// 3.0을 빼는 이유는 기존 구한 테두리 픽셀보다 조금 안쪽으로 잡기위해서 약 한 3pixel? 같이 빼서 edge로 생각하는것
				xx = (r - 3.0) * helper->cos_array[angle] + (double)x;
				yy = (r - 3.0) * helper->sin_array[angle] + (double)y;

				CvDPoint point;
				point.x = xx; point.y = yy;
				edge_point.push_back(point);
				//printf("%d번째 DETECT CONTOUR POINT = [%3d, %3d] \n", edge_point.size(), (int)xx, (int)yy);
				info.PUPIL.center_of_x = (int)point.x;
				info.PUPIL.center_of_y = (int)point.y;
				return;
			} // if문 pixelvalue2 - pixelvalue1
		}
		r = r + 1.0;
		//Sleep(10);
	} //  전체 while문
}

// 혹시나 각도값이 360도를 넘으면 0~360 안으로 맞추기
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

// 경계선인지 체크
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

// 점들 평균 구해서 next 중심점 반환
Point StarBurst::calculateCoverage(void) {
	// size가 0이면 그냥 영상의 중심점 반환
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
