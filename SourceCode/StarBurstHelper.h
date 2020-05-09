

#pragma once
#include "data.h"

#define PI  3.141592653589		// PI value

class StarBurstHelper {
	// 생성자
public:
	StarBurstHelper(Mat image);	// Constructor for StarBurst Helper Class

// 소멸자
public:
	~StarBurstHelper(void);

	// 최종 각막반사가 제거된 이미지
public:
	Mat CornealReflexRemove(StarBurstInfo &info);	// Method for Corneal Reflex Remove : Unique *public* modifier
	//Rect rect;	// Region Of Interest

// 내부 커널 제거 함수들
private:
	void CrDetect(void);				// Step 1 : Finding Corneal Reflex Point
	void CrRemove(StarBurstInfo info); 	// Step 2 : Removing Corneal Reflex by Interporation
	Mat cr_image;						// Gray Image for finding CR
	Mat cr_removed_image;				// Gray Image with CR removed
	Point m_pupil, m_startPoint;
	int crx;	// Corneal Center x
	int cry;	// Corneal Center y
	int crar;	// Corneal Radius

// 변수 값들
public:
	int biggest_crar;	// 가장 큰 각막반사의 반지름 (biggest radius of r in Image : it would be lower than (height/10) )

	float angle_delta;	// Δ각도  - 1도씩 변할때마다 변하는 값 (it is a radian Value of one degree : 0.0174) 
	int angle_num;		// 각도 몇도 (number of angle : 360) 
	double *angle_array;	// 1도~360도값 초기화
	double *sin_array;		// cos 각도 넣었을 때 y축값 - cos함수 그려보면 1에서 -1사이
	double *cos_array;		// sin 각도 넣었을 때 y축값 - sin함수 그려보면 1에서 -1사이

	int getFitRadius(void);
};
