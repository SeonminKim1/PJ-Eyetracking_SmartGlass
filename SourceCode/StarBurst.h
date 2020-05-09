#pragma once
#include "data.h"
#include "StarBurstHelper.h"
//#define N 18	// Number of ray for detecting feature point

class StarBurst {
	// 생성자
public:
	StarBurst(Mat image, int CONTOUR_THRESHOLD);

	// 소멸자
public:
	~StarBurst(void);

public:
	StarBurstInfo * Apply(void);	// Unique Method for Apply StarBurst Algorithme
public:
	void DetectCoutourPoints(void);	// Step 1: Finding CoutourPoints
	void ApplyRANSAC(void);			// Step 2: Finding 

	StarBurstInfo  info, info2;			// return Structure
	int edge_thresh;				// 동공 경계 임계값

// 변수값들
public:
	vector <CvDPoint> edge_point;
	vector <int> edge_intensity_diff;
	StarBurstHelper * helper;

public:	// for Debug ... it is must private
	Mat m_image;			// (그레이 복사함)
	Mat m_removed_image;	// 각막 제거된 형태
	Mat ex_image;			// 연습용 - StarBurst코드에서 Draw하기 위한.
	//Rect rect;				// Region Of Interest
	int draw_on;			// Starburst에서 그림 그릴지 안그릴지
	Rect rect1; // 초기 시작점 찾기 위해서
	int CONTOUR_THRESHOLD;

public:
	Mat getCornealRemovedImage(void);
	void shotRay(int angle, int x, int y, int thresh);
	bool isBorder(int xx, int yy);
	Point calculateCoverage(void);
	int fitDegree(int degree);
};