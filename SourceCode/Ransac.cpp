
#include "data.h"
#include "svd.h"
/*Ransanc�˰��� �ƿ�����
������ ������ �����Ϳ� �߿� ������ =��������(outlier)/���ǵ� ����(inliner)�� �ִ�
@outlier = �̻��� :������ �������� �����ϰ� ��� �ִ� ������

outlier�������ϱ����ؼ� �����ͼ����κ��� ������ ���� ã�ƾ��ϴµ�
�ڵ������� ������ ������ ����Ͽ���
�ʱ� �����͸� �ּҷ� ����ϰ� �ϰ��� �������� ������ Ȯ���� ���ư� �ݺ����� �۾��� ���� �ظ� ���� �ϴ� ���

*/
//------------ Ransac ellipse fitting -----------//
//���� �Ķ���� �迭 5�� 0���� �ʱ�ȭ
double pupil_param[5] = { 0, 0, 0, 0, 0 };
Point final_Point[5];

// Randomly select 5 indeics
// (1) ���õ����� rand_num �� �̾Ƽ� ���� �Ķ���͸� ���Ѵ� ----//
// max_num = ���� ������-1 , int* rand_num = ũ�� 5�� �迭
void get_5_random_num(int max_num, int* rand_num) {
	int rand_index = 0; //���� �ε����� = 0
	int r, i; // r�� ������ ����, i�� �ݺ�������
	bool is_new = 1;  // ������ �ߺ�üũ��
	//--------------------------------------
	//�ִ��� 4�̸� 

	if (max_num == 4) {
		for (i = 0; i < 5; i++) {
			//rand_num�迭 = [ 0 1 2 3 4 ]
			rand_num[i] = i;
			cout << rand_num[i] << endl;
		}
		return;
	}
	//----------------------------------------------
	//rand_index �ʱⰪ�� 0�̹Ƿ� while���� ����
	while (rand_index < 5) {
		is_new = 1;//������ =1 
		r = (int)((rand()*1.0 / RAND_MAX) * max_num); // RAND_MAX= 32767, �ִ� max_num-1�� ������ ����.

		// �̹� �� ������(������) �ߺ�üũ �� 5�� ��ä���� ���� �ִ���
		for (i = 0; i < rand_index; i++) {
			if (r == rand_num[i]) {
				//������ =0
				is_new = 0;
				break;
			}
		}

		// ���ο� �������϶� 
		if (is_new) {
			//new ���� 1�̸� rand_index +1
			rand_num[rand_index] = r;
			rand_index++;
		}
	}
}

// (2) �ּ��ڽ¹��� ���� Ÿ�� ������----------------------------------------//
/*���⼭ �ּ��ڽ¹��� ����ϴ� ������ ������� ����� ���ؼ� ���� Ÿ���� �־��� ������ ���߿��� 5���� �ݵ�� Ÿ���� �־���Ѵ�.
�ּ��ڽ¹��� �̿��Ѹ� ���� �������� ����� ���� ���ִ�
������ �����Ͱ� N>5�ϰ�� p = (A'' * A)^-1 * A'' * B �� ǥ���� ���� >>>> SVD�� Ȱ��
theta= Ÿ���� ȸ���� ����
���Թ����� conic equation: ax^2 + bxy + cy^2 + dx + ey + f = 0;
Ÿ�� ������ ax^2 + cy^2 + cx + cy + theta = 0;
*/
// solve_ellipse
// conic_param[6] is the parameters of a conic {a, b, c, d, e, f}; conic equation: ax^2 + bxy + cy^2 + dx + ey + f = 0;
// ellipse_param[5] is the parameters of an ellipse {ellipse_a, ellipse_b, cx, cy, theta}; a & b is the major or minor axis; 
// cx & cy is the ellipse center; theta is the ellipse orientation
bool solve_ellipse(double* conic_param, double* ellipse_param) {
	//���԰������ conic equation: ax^2 + bxy + cy^2 + dx + ey + f = 0;
	//������ ������� �ڸ��� Ÿ��/������/�ְ�� ��������� ���ϰ� �ȴ�.

	//���� �������� ���ؼ� Ÿ���� �Ķ���͸� ���ϴ� ����
	//ax^2 + cy^2 + cx + cy + theta = 0;
	double a = conic_param[0]; 	double b = conic_param[1];
	double c = conic_param[2]; 	double d = conic_param[3];
	double e = conic_param[4]; 	double f = conic_param[5];
	//get ellipse orientation

	double theta = atan2(b, a - c) / 2;

	//get scaled major/minor axes
	double ct = cos(theta);
	double st = sin(theta);
	double ap = a * ct*ct + b * ct*st + c * st*st;
	double cp = a * st*st - b * ct*st + c * ct*ct;

	//get translations
	double cx = (2 * c*d - b * e) / (b*b - 4 * a*c);
	double cy = (2 * a*e - b * d) / (b*b - 4 * a*c);

	//get scale factor
	double val = a * cx*cx + b * cx*cy + c * cy*cy;
	double scale_inv = val - f;

	ellipse_param[0] = sqrt(scale_inv / ap);
	ellipse_param[1] = sqrt(scale_inv / cp);
	ellipse_param[2] = cx;
	ellipse_param[3] = cy;
	ellipse_param[4] = theta;
	//Ÿ�� �Ķ���� ����

	//cout << "����: " << a << b << c << d << e << f << theta << endl;
	//cout << "Ÿ��: "<< ct << st << ap << cp << cx << cy << val << scale_inv << endl;
	return true;
}

//CvDPoint��ü�� x.y�·Ḧ ���Ѵ�.
//���� x,y����ü�� normalize_point ����ȭ�� ��Ų��.
// (3) �� ����ȭ ���� ------------------------//
vector<CvDPoint> normalize_point_set(vector<CvDPoint> &point_set, double &dis_scale, CvDPoint &nor_center, int num)
{
	double sumx = 0, sumy = 0;
	double sumdis = 0;
	//CvPoint *edge = point_set;
	int i;
	for (i = 0; i < num; i++) {
		//sumx,sumy�� point�� x,y ���� ���� ���Ѵ�.
		sumx += point_set[i].x;
		sumy += point_set[i].y;
		//��Ʈ (x������+ y�� ����)�� sumids�� ���Ѵ�.
		sumdis += sqrt((double)(point_set[i].x *point_set[i].x + point_set[i].y * point_set[i].y));
	}

	dis_scale = sqrt((double)2)*num / sumdis;
	nor_center.x = sumx * 1.0 / num;
	nor_center.y = sumy * 1.0 / num;
	vector<CvDPoint> edge_point_nor;


	for (i = 0; i < num; i++) {
		CvDPoint edge;
		//������ ���� ����ȭ��Ų��
		edge.x = (point_set[i].x - nor_center.x)*dis_scale;
		edge.y = (point_set[i].y - nor_center.y)*dis_scale;
		//   ����ȭ ��Ų x,y ��ǥ�� �ִ´�
		edge_point_nor.push_back(edge);
	}
	return edge_point_nor;
}

//�����Ͱ����� ǥ��ȭ ���ױ⶧���� ���������� ����ȭ ��Ű�� ��
// dis_scale, normalized�� �߽�, ���� ����Ʈ ����, ��������Ʈ��
vector<CvDPoint> normalize_edge_point(double &dis_scale, CvDPoint &nor_center, int ep_num, vector<CvDPoint> edge_point) {
	double sumx = 0, sumy = 0;
	double sumdis = 0;
	int i;

	// (1) ��� ��������Ʈ ���� ���ϱ� (x, y, �Ÿ�=ũ��)
	for (i = 0; i < ep_num; i++) {
		sumx += edge_point[i].x;
		sumy += edge_point[i].y;
		sumdis += sqrt((double)(edge_point[i].x *edge_point[i].x + edge_point[i].y * edge_point[i].y));
	}
	dis_scale = sqrt((double)2)*ep_num / sumdis;
	nor_center.x = sumx * 1.0 / ep_num; //x��հ�
	nor_center.y = sumy * 1.0 / ep_num; //y��հ�

	// (2) ����ȭ ����
	vector<CvDPoint> edge_point_nor;
	for (i = 0; i < ep_num; i++) {
		CvDPoint edge;
		edge.x = (edge_point[i].x - nor_center.x)*dis_scale; //���� x���� ǥ��ȭ ��Ű�°���
		edge.y = (edge_point[i].y - nor_center.y)*dis_scale;//���� y���� ǥ��ȭ ��Ű�� ����
		edge_point_nor.push_back(edge);
	}
	return edge_point_nor; // ����ȭ�� ������
}

//�ٽ� ����ȭ �ݴ�� �ϴ� ����
void denormalize_ellipse_param(double* par, double* normailized_par, double dis_scale, CvDPoint nor_center) {
	par[0] = normailized_par[0] / dis_scale;   //major or minor axis
	par[1] = normailized_par[1] / dis_scale;
	par[2] = normailized_par[2] / dis_scale + nor_center.x;   //ellipse center
	par[3] = normailized_par[3] / dis_scale + nor_center.y;
}

/*
1.�ʱ��� ���� �ּ��� ���� ����
2.�ϰ� ������ ������ Ȯ��
3.������ �Ķ���� �����ϴ� ������ �ݺ�(2>3 ��� )
4.
*/
// �� �˰��� ���� �ϴ� �κ� �帧 ������ �Լ�  
// pupil_fitting_inliers(�̹���, �ʺ�, ����, inlier �ִ� ����, ���������)
int* pupil_fitting_inliers(UINT8* pupil_image, int width, int height, int &return_max_inliers_num, vector<CvDPoint> edge_point) {
	int i;
	int ep_num = edge_point.size(); //��������Ʈ�� ������ ũ��	
	int ellipse_point_num = 5;		//number of point that needed to fit an ellipse (Ÿ������µ� �ʿ��� ���� ����)
	// ��������� ũ�Ⱑ Ÿ������ �ּ� ������ 5������ ���� ��
	if (ep_num < ellipse_point_num) {
		printf("RANSAC - Error! %d points are not enough to fit ellipse (���� 5���� ������) \n", ep_num);
		memset(pupil_param, 0, sizeof(pupil_param));
		return_max_inliers_num = 0;
		return NULL;
	}

	// Normalization  
	double dis_scale;
	CvDPoint nor_center; //x,y�� nor_center����
	// normalize_edge_point (dis_scale, nor_center, ��������Ʈ ����, ��������Ʈ����)
	vector<CvDPoint> edge_point_nor = normalize_edge_point(dis_scale, nor_center, ep_num, edge_point);

	//Ransac
	// ���� ������ŭ �޸𸮻���
	int *inliers_index = (int*)malloc(sizeof(int)*ep_num);
	int *max_inliers_index = (int*)malloc(sizeof(int)*ep_num);

	// 1.�ʱⰪ ����
	int ninliers = 0;
	int max_inliers = 0;
	int sample_num = 1000;   //number of sample
	int ransac_count = 0;
	double dis_threshold = sqrt(3.84)*dis_scale;
	double dis_error;

	// 0�� ���� �Ҵ� ���� ep_num��ŭ 0�� �ʱ�ȭ
	memset(inliers_index, int(0), sizeof(int)*ep_num);
	memset(max_inliers_index, int(0), sizeof(int)*ep_num);
	//memset() (a,b,c) �Ķ���Ͷ�� a�� b�� cũ�⸸ŭ ����ä���.

	int rand_index[5];
	double A[6][6];
	int M = 6, N = 6; //M is row; N is column
	for (i = 0; i < N; i++) {
		A[i][5] = 1;
		A[5][i] = 0;
	}
	/* 0  1  2  3  4  5
	0>[ ][ ][ ][ ][ ][1]
	1>[ ][ ][ ][ ][ ][1]
	2>[ ][ ][ ][ ][ ][1]
	3>[ ][ ][ ][ ][ ][1]
	4>[ ][ ][ ][ ][ ][1]
	5>[0][0][0][0][0][0]
	*/

	// svd�� ���� ������ / ���߹迭 ppa, ppu, ppv
	double **ppa = (double**)malloc(sizeof(double*)*M);
	double **ppu = (double**)malloc(sizeof(double*)*M);
	double **ppv = (double**)malloc(sizeof(double*)*N);

	// ���߹迭 A�� ppa�� �״�� ���� & ppu columnũ�⸸ŭ �� ����
	for (i = 0; i < M; i++) {
		ppa[i] = A[i]; // �� 
		ppu[i] = (double*)malloc(sizeof(double)*N);//��
	}

	// columnũ�⸸ŭ ppv[i]������
	for (i = 0; i < N; i++) {
		ppv[i] = (double*)malloc(sizeof(double)*N);
	}

	double pd[6];
	int min_d_index;
	double conic_par[6] = { 0 }; // ���� �Ķ���� 6�� �ʱ�ȭ 0
	double ellipse_par[5] = { 0 }; // Ÿ�� �Ķ���� 5�� �ʱ�ȭ 0
	double best_ellipse_par[5] = { 0 }; // ���� ������ Ÿ�� �Ķ���� 5�� �ʱ�ȭ 0
	double ratio;
	//���ø� �����̳����� while������
	while (sample_num > ransac_count) {
		//������ ���� ����
		get_5_random_num((ep_num - 1), rand_index);

		// normalize�� �� A[][]�� ����
		// Ÿ���� �����Ŀ��� x���鿡 rand_index[i]�� �� ��. �� �������� �� ������ ������ ������ ���ϱ� ���ؼ�
		for (i = 0; i < 5; i++) {
			A[i][0] = edge_point_nor[rand_index[i]].x * edge_point_nor[rand_index[i]].x;
			A[i][1] = edge_point_nor[rand_index[i]].x * edge_point_nor[rand_index[i]].y;
			A[i][2] = edge_point_nor[rand_index[i]].y * edge_point_nor[rand_index[i]].y;
			A[i][3] = edge_point_nor[rand_index[i]].x;
			A[i][4] = edge_point_nor[rand_index[i]].y;
		}

		//���� N/���ϱ� n 
		 /* 0  1  2  3  4  5
		 0>[N][N][N][n][n][1]
		 1>[N][N][N][n][n][1]
		 2>[N][N][N][n][n][1]
		 3>[N][N][N][n][n][1]
		 4>[N][N][N][n][n][1]
		 5>[0][0][0][0][0][0] */

		 //SVD����(Ư�̰����� �Լ�)�� ���� Ÿ�� �Ķ���� ����
		 // ����� pd�� �����
		svd(M, N, ppa, ppu, pd, ppv);
		min_d_index = 0;

		// pd�迭(svd�� ���� ���� �迭)�� �ּڰ� �ε��� ���ϱ� (min_d_index)
		for (i = 1; i < N; i++) {
			if (pd[i] < pd[min_d_index])
				min_d_index = i;
		}

		for (i = 0; i < N; i++)
			conic_par[i] = ppv[i][min_d_index];   //the column of v that corresponds to the smallest singular value, 
		 // which is the solution of the equations
		ninliers = 0;
		memset(inliers_index, 0, sizeof(int)*ep_num); // ���ǵȰ�(inliers) 0���� �ʱ�ȭ

		for (i = 0; i < ep_num; i++) {
			dis_error =
				conic_par[0] * edge_point_nor[i].x * edge_point_nor[i].x +
				conic_par[1] * edge_point_nor[i].x*edge_point_nor[i].y +
				conic_par[2] * edge_point_nor[i].y*edge_point_nor[i].y +
				conic_par[3] * edge_point_nor[i].x + conic_par[4] * edge_point_nor[i].y +
				conic_par[5];
			// fabs�� �Ǽ� ���� ���ϴ°� ex) -3.14 -> 3.14
			if (fabs(dis_error) < dis_threshold) {
				inliers_index[ninliers] = i;
				ninliers++;
				//2~3�� �ݺ� ������ ������ Ȯ���� ����
			}
		}

		// ���� ���� inlier�� �������� �� 
		// ���ǵȰ�(ninliers)���� �Ӱ谪���� �ִ밪�϶� - ���� ���� inlier
		if (ninliers > max_inliers) {
			if (solve_ellipse(conic_par, ellipse_par)) { // �ּ��ڽ¹� ����
				denormalize_ellipse_param(ellipse_par, ellipse_par, dis_scale, nor_center);
				ratio = ellipse_par[0] / ellipse_par[1]; // ������ 0,5���� 2������ ��

				// max���̶�� ����
				if (ellipse_par[2] > 0 &&
					ellipse_par[2] <= width - 1 &&
					ellipse_par[3] > 0 &&
					ellipse_par[3] <= height - 1 &&
					ratio > 0.5 && ratio < 2) {
					memcpy(max_inliers_index, inliers_index, sizeof(int)*ep_num);


					for (i = 0; i < 5; i++) {
						best_ellipse_par[i] = ellipse_par[i];
					}
					//�̶��� inliner���� max_inlier�� ����
					max_inliers = ninliers;
					//�˰��� �����߿� log�ϴ� �κ��ִµ� �ʹ� �������� ..���ػ���
					sample_num = (int)(log((double)(1 - 0.99)) / log(1.0 - pow(ninliers*1.0 / ep_num, 5)));
				}
			} // if(solve_ellipse(conic_par, ellipse_par) - �ּ��ڽ¹��� �̿��Ͽ� ���Է� Ÿ������.
		} // if(ninliers > maxinliers)
		ransac_count++;
		//sample_num 1000�̾��µ� 1500�� ������ �ʰ�
		if (ransac_count > 1500) {
			printf("RANSAC - Error! ransac_count exceed! ransac break! sample_num=%d, ransac_count=%d\n", sample_num, ransac_count);
			break;
		}
	} // while (sample_num > ransac_count)

	// ���� ���� Ÿ���� �Ķ������ �ʺ�� ���̰� �Ѵ� 0���� ŭ
	if (best_ellipse_par[0] > 0 && best_ellipse_par[1] > 0) {
		for (i = 0; i < 5; i++) {
			pupil_param[i] = best_ellipse_par[i];
			//�ְ��� Ÿ�� �Ķ���Ͱ���
		}
	}

	else {
		printf("RANSAC - Error! best_ellipse_par - width, height �� 0���� ���ų� ����\n");
		memset(pupil_param, 0, sizeof(pupil_param));
		max_inliers = 0;
		free(max_inliers_index);
		max_inliers_index = NULL;
	}

	//���� �Ҵ� ��� �����ϴ� ���
	for (i = 0; i < M; i++) {
		free(ppu[i]); free(ppv[i]);
	}

	free(ppu); free(ppv); free(ppa); free(inliers_index);
	return_max_inliers_num = max_inliers;
	return max_inliers_index; // ���� ����� Ÿ���� ���߿� �� ���� �� 
}

// -------------------------------------- Ķ�� ��� (�Ŀ� cali.cpp�� ����) ----------------
// (1) Ķ�� ����
void on_mouse_homo(int event, int x, int y, int flags, void *param) {
	switch (event) {
		// Ķ���극�̼� ������ �� 9�� ���� ������ ���� 
	case CV_EVENT_LBUTTONDOWN: {
		printf("screen ��ǥ: (%d %d) %d/9 \n", x, y, number_calibration_points_set); // Ķ���̹������� ���� ���콺�� ���� ��ġ
		homo_sub_capture_image = original_image.clone();

		Mat roi_gray_image;
		cvtColor(homo_sub_capture_image, roi_gray_image, CV_BGR2GRAY);

		if (starburst == NULL) {
			starburst = new StarBurst(roi_gray_image, CONTOUR_THRESHOLD);
		}

		starburst->draw_on = 0; // �׸��� ��� off

		info = starburst->Apply();
		printf("�����ݻ��� �߽���? %d %d \n", info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y);
		printf("������ �߽���? %d %d \n", info->PUPIL.center_of_x, info->PUPIL.center_of_y);

		diff_point.x = info->PUPIL.center_of_x - info->CORNEAL_REFLEX.center_of_x;
		diff_point.y = info->PUPIL.center_of_y - info->CORNEAL_REFLEX.center_of_y;

		diff_vector_point.push_back(diff_point);
		screen_vector_point.push_back(Point(x, y));

		if (number_calibration_points_set < CALIBRATIONPOINTS) {
			number_calibration_points_set++;   // Ƚ��
			printf("calibration points number: %d (total 9)\n", number_calibration_points_set);
		}
		break;
	}

							   // Ķ���극�̼� �Ϸ� ���� �� 9�� �� �� ����� ��. 

	case CV_EVENT_RBUTTONDOWN:
		Activate_Calibration();
		break;
	}
}



// Ķ���극�̼� �۵�

void Activate_Calibration() {
	// 9�� �ٸ��� ��
	if (number_calibration_points_set == CALIBRATIONPOINTS) {
		do_map2scene = !do_map2scene; // 0�� 1��
		view_cal_points = !view_cal_points; // 1�� 0����
		printf("ȣ��׷������\n");
		homo_matrix = findHomography(diff_vector_point, screen_vector_point, RANSAC);

		cout << screen_vector_point << endl;
		cout << diff_vector_point << endl;

		//cout << homo_matrix << endl;
	}

	// 9�� �ȸ��� ��
	else {
		diff_vector_point.pop_back();
		number_calibration_points_set--;
		printf("diff_vector_point.pop_back\n");
		printf("Attempt to activate calibration without a full set of points.\n");
	}
}


void timer() {
	ftime(&timebuffer);  // timebuffer��ä���
	ltime = timebuffer.time;  // time_t �����������´�
	milisec = timebuffer.millitm;  // milisec�����Ѵ�
	now = localtime(&ltime);  // time ������ä���
}
