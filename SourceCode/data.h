#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>
#include <conio.h>
#include <Windows.h>
#include <time.h>
#include <sys/timeb.h>
#pragma warning(disable:4996)

using namespace cv;
using namespace std;


#ifndef _DATA_
#define _DATA_
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define CONTROLTIME 30
#define CONTROLTIME2 20
#define RED Scalar(0,0,255)
#define BLUE Scalar(255,0,0)
#define GREEN Scalar(0,255,0)
#define YELLOW Scalar(0,255,255)
#define MAGENTA Scalar(255,0,255)
#define SKYBLUE Scalar(255,255,0)
#define WHITE Scalar(255,255,255);
#define BLACK Scalar(0,0,0);


// data.h �� info�� �ִ� ������ �� cpp���� ������ ������ data.h�� �����Ű��
// �� ������ main���� ������ draw �ϱ� ���� ��.
struct StarBurstInfo {
	struct CORNEAL_INFO {
		int center_of_x;
		int center_of_y;
		int radius;
		int fit_radius;
	} CORNEAL_REFLEX;

	struct PUPIL_INFO {
		int width;
		int height;
		int center_of_x;
		int center_of_y;
		double theta;
		//int radius;
	} PUPIL;
};


// StarBurst���� edge_point (vector) �Ҷ� ���
struct CvDPoint {
	double x;
	double y;
};

#endif
