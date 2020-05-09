#pragma once
#include "data.h"
#include "StarBurstHelper.h"
//#define N 18	// Number of ray for detecting feature point

class StarBurst {
	// ������
public:
	StarBurst(Mat image, int CONTOUR_THRESHOLD);

	// �Ҹ���
public:
	~StarBurst(void);

public:
	StarBurstInfo * Apply(void);	// Unique Method for Apply StarBurst Algorithme
public:
	void DetectCoutourPoints(void);	// Step 1: Finding CoutourPoints
	void ApplyRANSAC(void);			// Step 2: Finding 

	StarBurstInfo  info, info2;			// return Structure
	int edge_thresh;				// ���� ��� �Ӱ谪

// ��������
public:
	vector <CvDPoint> edge_point;
	vector <int> edge_intensity_diff;
	StarBurstHelper * helper;

public:	// for Debug ... it is must private
	Mat m_image;			// (�׷��� ������)
	Mat m_removed_image;	// ���� ���ŵ� ����
	Mat ex_image;			// ������ - StarBurst�ڵ忡�� Draw�ϱ� ����.
	//Rect rect;				// Region Of Interest
	int draw_on;			// Starburst���� �׸� �׸��� �ȱ׸���
	Rect rect1; // �ʱ� ������ ã�� ���ؼ�
	int CONTOUR_THRESHOLD;

public:
	Mat getCornealRemovedImage(void);
	void shotRay(int angle, int x, int y, int thresh);
	bool isBorder(int xx, int yy);
	Point calculateCoverage(void);
	int fitDegree(int degree);
};