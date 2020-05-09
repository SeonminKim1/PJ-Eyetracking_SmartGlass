

#pragma once
#include "data.h"

#define PI  3.141592653589		// PI value

class StarBurstHelper {
	// ������
public:
	StarBurstHelper(Mat image);	// Constructor for StarBurst Helper Class

// �Ҹ���
public:
	~StarBurstHelper(void);

	// ���� �����ݻ簡 ���ŵ� �̹���
public:
	Mat CornealReflexRemove(StarBurstInfo &info);	// Method for Corneal Reflex Remove : Unique *public* modifier
	//Rect rect;	// Region Of Interest

// ���� Ŀ�� ���� �Լ���
private:
	void CrDetect(void);				// Step 1 : Finding Corneal Reflex Point
	void CrRemove(StarBurstInfo info); 	// Step 2 : Removing Corneal Reflex by Interporation
	Mat cr_image;						// Gray Image for finding CR
	Mat cr_removed_image;				// Gray Image with CR removed
	Point m_pupil, m_startPoint;
	int crx;	// Corneal Center x
	int cry;	// Corneal Center y
	int crar;	// Corneal Radius

// ���� ����
public:
	int biggest_crar;	// ���� ū �����ݻ��� ������ (biggest radius of r in Image : it would be lower than (height/10) )

	float angle_delta;	// �İ���  - 1���� ���Ҷ����� ���ϴ� �� (it is a radian Value of one degree : 0.0174) 
	int angle_num;		// ���� � (number of angle : 360) 
	double *angle_array;	// 1��~360���� �ʱ�ȭ
	double *sin_array;		// cos ���� �־��� �� y�ప - cos�Լ� �׷����� 1���� -1����
	double *cos_array;		// sin ���� �־��� �� y�ప - sin�Լ� �׷����� 1���� -1����

	int getFitRadius(void);
};
