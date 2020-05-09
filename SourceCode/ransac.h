
#ifndef _RANSAC_H
#define _RANSAC_H


#include "data.h"
#include "svd.h"

extern double pupil_param[5];

//extern vector <CvDPoint> edge_point;
//------------ Ransac ellipse fitting -----------//
// Randomly select 5 indeics

void get_5_random_num(int max_num, int* rand_num);
bool solve_ellipse(double* conic_param, double* ellipse_param);
vector<CvDPoint> normalize_point_set(vector<CvDPoint> &point_set, double &dis_scale, CvDPoint &nor_center, int num);
vector<CvDPoint> normalize_edge_point(double &dis_scale, CvDPoint &nor_center, int ep_num, vector<CvDPoint> edge_point);
void denormalize_ellipse_param(double* par, double* normailized_par, double dis_scale, CvDPoint nor_center);
int* pupil_fitting_inliers(UINT8* pupil_image, int width, int height, int &return_max_inliers_num, vector<CvDPoint> edge_point);

#endif
