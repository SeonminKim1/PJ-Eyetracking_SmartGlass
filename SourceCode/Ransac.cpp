
#include "data.h"
#include "svd.h"
/*Ransanc알고리즈 아웃라인
임의의 측정된 데이터엣 중에 노이즈 =거짓정보(outlier)/진실된 정보(inliner)가 있다
@outlier = 이상점 :데이터 분포에서 현저하게 벗어나 있는 관측값

outlier를제거하기위해서 데이터셋으로부터 수학적 모델을 찾아야하는데
코딩에서는 수학적 모델으로 대신하였다
초기 데이터를 최소로 사용하고 일관된 데이터의 집합을 확장해 나아가 반복적인 작업을 통해 해를 예측 하는 방법

*/
//------------ Ransac ellipse fitting -----------//
//동공 파라미터 배열 5개 0으로 초기화
double pupil_param[5] = { 0, 0, 0, 0, 0 };
Point final_Point[5];

// Randomly select 5 indeics
// (1) 샘플데이터 rand_num 개 뽑아서 모델의 파라미터를 구한다 ----//
// max_num = 엣지 사이즈-1 , int* rand_num = 크기 5의 배열
void get_5_random_num(int max_num, int* rand_num) {
	int rand_index = 0; //랜덤 인덱스값 = 0
	int r, i; // r은 랜덤값 변수, i는 반복문변수
	bool is_new = 1;  // 랜덤값 중복체크용
	//--------------------------------------
	//최댓값이 4이면 

	if (max_num == 4) {
		for (i = 0; i < 5; i++) {
			//rand_num배열 = [ 0 1 2 3 4 ]
			rand_num[i] = i;
			cout << rand_num[i] << endl;
		}
		return;
	}
	//----------------------------------------------
	//rand_index 초기값이 0이므로 while문을 돈다
	while (rand_index < 5) {
		is_new = 1;//구별값 =1 
		r = (int)((rand()*1.0 / RAND_MAX) * max_num); // RAND_MAX= 32767, 최대 max_num-1인 랜덤값 구함.

		// 이미 고른 랜덤값(엣지점) 중복체크 및 5개 안채워진 점이 있는지
		for (i = 0; i < rand_index; i++) {
			if (r == rand_num[i]) {
				//구별값 =0
				is_new = 0;
				break;
			}
		}

		// 새로운 랜덤값일때 
		if (is_new) {
			//new 값이 1이면 rand_index +1
			rand_num[rand_index] = r;
			rand_index++;
		}
	}
}

// (2) 최소자승법에 의한 타원 모델정합----------------------------------------//
/*여기서 최소자승법을 사용하는 이유는 대수적인 방법에 의해서 구한 타원은 주어진 데이터 점중에서 5개는 반드시 타원상에 있어야한다.
최소자승법을 이용한면 좀더 안정적인 결과를 얻을 수있다
하지만 데이터가 N>5일경우 p = (A'' * A)^-1 * A'' * B 로 표현이 가능 >>>> SVD를 활용
theta= 타원이 회전한 각도
원뿔방정식 conic equation: ax^2 + bxy + cy^2 + dx + ey + f = 0;
타원 방정식 ax^2 + cy^2 + cx + cy + theta = 0;
*/
// solve_ellipse
// conic_param[6] is the parameters of a conic {a, b, c, d, e, f}; conic equation: ax^2 + bxy + cy^2 + dx + ey + f = 0;
// ellipse_param[5] is the parameters of an ellipse {ellipse_a, ellipse_b, cx, cy, theta}; a & b is the major or minor axis; 
// cx & cy is the ellipse center; theta is the ellipse orientation
bool solve_ellipse(double* conic_param, double* ellipse_param) {
	//원뿔곡선방정식 conic equation: ax^2 + bxy + cy^2 + dx + ey + f = 0;
	//원뿔을 평면으로 자르면 타원/포물선/쌍곡선등 이차곡선으로 변하게 된다.

	//원뿔 방정식을 통해서 타원의 파라미터를 구하는 과정
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
	//타원 파라미터 구함

	//cout << "원뿔: " << a << b << c << d << e << f << theta << endl;
	//cout << "타원: "<< ct << st << ap << cp << cx << cy << val << scale_inv << endl;
	return true;
}

//CvDPoint구체는 x.y좌료를 말한다.
//벡터 x,y구조체를 normalize_point 정규화를 시킨다.
// (3) 값 정규화 과정 ------------------------//
vector<CvDPoint> normalize_point_set(vector<CvDPoint> &point_set, double &dis_scale, CvDPoint &nor_center, int num)
{
	double sumx = 0, sumy = 0;
	double sumdis = 0;
	//CvPoint *edge = point_set;
	int i;
	for (i = 0; i < num; i++) {
		//sumx,sumy에 point의 x,y 들을 각각 더한다.
		sumx += point_set[i].x;
		sumy += point_set[i].y;
		//루트 (x의제곱+ y의 제곱)을 sumids에 더한다.
		sumdis += sqrt((double)(point_set[i].x *point_set[i].x + point_set[i].y * point_set[i].y));
	}

	dis_scale = sqrt((double)2)*num / sumdis;
	nor_center.x = sumx * 1.0 / num;
	nor_center.y = sumy * 1.0 / num;
	vector<CvDPoint> edge_point_nor;


	for (i = 0; i < num; i++) {
		CvDPoint edge;
		//엣지값 또한 정규화시킨다
		edge.x = (point_set[i].x - nor_center.x)*dis_scale;
		edge.y = (point_set[i].y - nor_center.y)*dis_scale;
		//   정규화 시킨 x,y 좌표를 넣는다
		edge_point_nor.push_back(edge);
	}
	return edge_point_nor;
}

//데이터값들을 표준화 시켰기때문에 엣지값또한 정규화 시키는 것
// dis_scale, normalized한 중심, 엣지 포인트 갯수, 엣지포인트들
vector<CvDPoint> normalize_edge_point(double &dis_scale, CvDPoint &nor_center, int ep_num, vector<CvDPoint> edge_point) {
	double sumx = 0, sumy = 0;
	double sumdis = 0;
	int i;

	// (1) 평균 엣지포인트 값들 구하기 (x, y, 거리=크기)
	for (i = 0; i < ep_num; i++) {
		sumx += edge_point[i].x;
		sumy += edge_point[i].y;
		sumdis += sqrt((double)(edge_point[i].x *edge_point[i].x + edge_point[i].y * edge_point[i].y));
	}
	dis_scale = sqrt((double)2)*ep_num / sumdis;
	nor_center.x = sumx * 1.0 / ep_num; //x평균값
	nor_center.y = sumy * 1.0 / ep_num; //y평균값

	// (2) 정규화 진행
	vector<CvDPoint> edge_point_nor;
	for (i = 0; i < ep_num; i++) {
		CvDPoint edge;
		edge.x = (edge_point[i].x - nor_center.x)*dis_scale; //엣지 x값을 표준화 시키는과정
		edge.y = (edge_point[i].y - nor_center.y)*dis_scale;//엣지 y값을 표준화 시키는 과정
		edge_point_nor.push_back(edge);
	}
	return edge_point_nor; // 정규화된 엣지들
}

//다시 정규화 반대로 하는 과정
void denormalize_ellipse_param(double* par, double* normailized_par, double dis_scale, CvDPoint nor_center) {
	par[0] = normailized_par[0] / dis_scale;   //major or minor axis
	par[1] = normailized_par[1] / dis_scale;
	par[2] = normailized_par[2] / dis_scale + nor_center.x;   //ellipse center
	par[3] = normailized_par[3] / dis_scale + nor_center.y;
}

/*
1.초기의 값을 최소의 값을 설정
2.일관 데이터 집합을 확장
3.최적의 파라미터 에측하는 과정을 반복(2>3 계속 )
4.
*/
// 주 알고리즘 시작 하는 부분 흐름 위에는 함수  
// pupil_fitting_inliers(이미지, 너비, 높이, inlier 최대 갯수, 동공경계점)
int* pupil_fitting_inliers(UINT8* pupil_image, int width, int height, int &return_max_inliers_num, vector<CvDPoint> edge_point) {
	int i;
	int ep_num = edge_point.size(); //엣지포인트의 사이즈 크기	
	int ellipse_point_num = 5;		//number of point that needed to fit an ellipse (타원만드는데 필요한 점의 갯수)
	// 동공경계점 크기가 타원피팅 최소 갯수인 5개보다 적을 때
	if (ep_num < ellipse_point_num) {
		printf("RANSAC - Error! %d points are not enough to fit ellipse (점이 5개도 안찍힘) \n", ep_num);
		memset(pupil_param, 0, sizeof(pupil_param));
		return_max_inliers_num = 0;
		return NULL;
	}

	// Normalization  
	double dis_scale;
	CvDPoint nor_center; //x,y꼴 nor_center변수
	// normalize_edge_point (dis_scale, nor_center, 엣지포인트 갯수, 엣지포인트벡터)
	vector<CvDPoint> edge_point_nor = normalize_edge_point(dis_scale, nor_center, ep_num, edge_point);

	//Ransac
	// 엣지 개수만큼 메모리생성
	int *inliers_index = (int*)malloc(sizeof(int)*ep_num);
	int *max_inliers_index = (int*)malloc(sizeof(int)*ep_num);

	// 1.초기값 설정
	int ninliers = 0;
	int max_inliers = 0;
	int sample_num = 1000;   //number of sample
	int ransac_count = 0;
	double dis_threshold = sqrt(3.84)*dis_scale;
	double dis_error;

	// 0을 동적 할당 받은 ep_num만큼 0을 초기화
	memset(inliers_index, int(0), sizeof(int)*ep_num);
	memset(max_inliers_index, int(0), sizeof(int)*ep_num);
	//memset() (a,b,c) 파라미터라면 a를 b로 c크기만큼 가득채운다.

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

	// svd를 위한 변수들 / 이중배열 ppa, ppu, ppv
	double **ppa = (double**)malloc(sizeof(double*)*M);
	double **ppu = (double**)malloc(sizeof(double*)*M);
	double **ppv = (double**)malloc(sizeof(double*)*N);

	// 이중배열 A를 ppa에 그대로 복사 & ppu column크기만큼 열 생성
	for (i = 0; i < M; i++) {
		ppa[i] = A[i]; // 행 
		ppu[i] = (double*)malloc(sizeof(double)*N);//열
	}

	// column크기만큼 ppv[i]를생성
	for (i = 0; i < N; i++) {
		ppv[i] = (double*)malloc(sizeof(double)*N);
	}

	double pd[6];
	int min_d_index;
	double conic_par[6] = { 0 }; // 원뿔 파라미터 6개 초기화 0
	double ellipse_par[5] = { 0 }; // 타원 파라미터 5개 초기화 0
	double best_ellipse_par[5] = { 0 }; // 가장 젤좋은 타원 파라미터 5개 초기화 0
	double ratio;
	//샘플링 개수이내에서 while문돌림
	while (sample_num > ransac_count) {
		//임의의 랜덤 선택
		get_5_random_num((ep_num - 1), rand_index);

		// normalize한 거 A[][]로 복사
		// 타원의 방정식에서 x값들에 rand_index[i]가 들어간 것. 즉 랜덤으로 고른 변수들 가지고 방정식 비교하기 위해서
		for (i = 0; i < 5; i++) {
			A[i][0] = edge_point_nor[rand_index[i]].x * edge_point_nor[rand_index[i]].x;
			A[i][1] = edge_point_nor[rand_index[i]].x * edge_point_nor[rand_index[i]].y;
			A[i][2] = edge_point_nor[rand_index[i]].y * edge_point_nor[rand_index[i]].y;
			A[i][3] = edge_point_nor[rand_index[i]].x;
			A[i][4] = edge_point_nor[rand_index[i]].y;
		}

		//제곱 N/곱하기 n 
		 /* 0  1  2  3  4  5
		 0>[N][N][N][n][n][1]
		 1>[N][N][N][n][n][1]
		 2>[N][N][N][n][n][1]
		 3>[N][N][N][n][n][1]
		 4>[N][N][N][n][n][1]
		 5>[0][0][0][0][0][0] */

		 //SVD분해(특이값분해 함수)로 인해 타원 파라미터 결정
		 // 결과는 pd에 저장됨
		svd(M, N, ppa, ppu, pd, ppv);
		min_d_index = 0;

		// pd배열(svd를 통해 나온 배열)의 최솟값 인덱스 구하기 (min_d_index)
		for (i = 1; i < N; i++) {
			if (pd[i] < pd[min_d_index])
				min_d_index = i;
		}

		for (i = 0; i < N; i++)
			conic_par[i] = ppv[i][min_d_index];   //the column of v that corresponds to the smallest singular value, 
		 // which is the solution of the equations
		ninliers = 0;
		memset(inliers_index, 0, sizeof(int)*ep_num); // 진실된값(inliers) 0으로 초기화

		for (i = 0; i < ep_num; i++) {
			dis_error =
				conic_par[0] * edge_point_nor[i].x * edge_point_nor[i].x +
				conic_par[1] * edge_point_nor[i].x*edge_point_nor[i].y +
				conic_par[2] * edge_point_nor[i].y*edge_point_nor[i].y +
				conic_par[3] * edge_point_nor[i].x + conic_par[4] * edge_point_nor[i].y +
				conic_par[5];
			// fabs는 실수 절댓값 구하는것 ex) -3.14 -> 3.14
			if (fabs(dis_error) < dis_threshold) {
				inliers_index[ninliers] = i;
				ninliers++;
				//2~3번 반복 데이터 집합을 확장해 나감
			}
		}

		// 제일 좋은 inlier가 구해졌을 때 
		// 진실된값(ninliers)에서 임계값에서 최대값일때 - 제일 좋은 inlier
		if (ninliers > max_inliers) {
			if (solve_ellipse(conic_par, ellipse_par)) { // 최소자승법 적용
				denormalize_ellipse_param(ellipse_par, ellipse_par, dis_scale, nor_center);
				ratio = ellipse_par[0] / ellipse_par[1]; // 비율이 0,5부터 2사이일 때

				// max값이라고 결정
				if (ellipse_par[2] > 0 &&
					ellipse_par[2] <= width - 1 &&
					ellipse_par[3] > 0 &&
					ellipse_par[3] <= height - 1 &&
					ratio > 0.5 && ratio < 2) {
					memcpy(max_inliers_index, inliers_index, sizeof(int)*ep_num);


					for (i = 0; i < 5; i++) {
						best_ellipse_par[i] = ellipse_par[i];
					}
					//이때의 inliner값을 max_inlier로 결정
					max_inliers = ninliers;
					//알고리즘 과정중에 log하는 부분있는데 너무 수학적임 ..이해생략
					sample_num = (int)(log((double)(1 - 0.99)) / log(1.0 - pow(ninliers*1.0 / ep_num, 5)));
				}
			} // if(solve_ellipse(conic_par, ellipse_par) - 최소자승법을 이용하여 원뿔로 타원구함.
		} // if(ninliers > maxinliers)
		ransac_count++;
		//sample_num 1000이었는데 1500을 넘으면 초과
		if (ransac_count > 1500) {
			printf("RANSAC - Error! ransac_count exceed! ransac break! sample_num=%d, ransac_count=%d\n", sample_num, ransac_count);
			break;
		}
	} // while (sample_num > ransac_count)

	// 제일 좋은 타원형 파라미터의 너비와 높이가 둘다 0보다 큼
	if (best_ellipse_par[0] > 0 && best_ellipse_par[1] > 0) {
		for (i = 0; i < 5; i++) {
			pupil_param[i] = best_ellipse_par[i];
			//최고의 타원 파라미터값들
		}
	}

	else {
		printf("RANSAC - Error! best_ellipse_par - width, height 가 0보다 같거나 작음\n");
		memset(pupil_param, 0, sizeof(pupil_param));
		max_inliers = 0;
		free(max_inliers_index);
		max_inliers_index = NULL;
	}

	//동적 할당 모두 해제하는 모습
	for (i = 0; i < M; i++) {
		free(ppu[i]); free(ppv[i]);
	}

	free(ppu); free(ppv); free(ppa); free(inliers_index);
	return_max_inliers_num = max_inliers;
	return max_inliers_index; // 제일 검출된 타원모델 값중에 젤 좋은 값 
}

// -------------------------------------- 캘리 기능 (후에 cali.cpp로 갈꺼) ----------------
// (1) 캘리 진행
void on_mouse_homo(int event, int x, int y, int flags, void *param) {
	switch (event) {
		// 캘리브레이션 진행중 점 9개 찍을 때까지 진행 
	case CV_EVENT_LBUTTONDOWN: {
		printf("screen 좌표: (%d %d) %d/9 \n", x, y, number_calibration_points_set); // 캘리이미지에서 내가 마우스로 찍은 위치
		homo_sub_capture_image = original_image.clone();

		Mat roi_gray_image;
		cvtColor(homo_sub_capture_image, roi_gray_image, CV_BGR2GRAY);

		if (starburst == NULL) {
			starburst = new StarBurst(roi_gray_image, CONTOUR_THRESHOLD);
		}

		starburst->draw_on = 0; // 그리기 모드 off

		info = starburst->Apply();
		printf("각막반사의 중심은? %d %d \n", info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y);
		printf("동공의 중심은? %d %d \n", info->PUPIL.center_of_x, info->PUPIL.center_of_y);

		diff_point.x = info->PUPIL.center_of_x - info->CORNEAL_REFLEX.center_of_x;
		diff_point.y = info->PUPIL.center_of_y - info->CORNEAL_REFLEX.center_of_y;

		diff_vector_point.push_back(diff_point);
		screen_vector_point.push_back(Point(x, y));

		if (number_calibration_points_set < CALIBRATIONPOINTS) {
			number_calibration_points_set++;   // 횟수
			printf("calibration points number: %d (total 9)\n", number_calibration_points_set);
		}
		break;
	}

							   // 캘리브레이션 완료 했을 때 9개 점 다 찍었을 때. 

	case CV_EVENT_RBUTTONDOWN:
		Activate_Calibration();
		break;
	}
}



// 캘리브레이션 작동

void Activate_Calibration() {
	// 9개 다모였을 때
	if (number_calibration_points_set == CALIBRATIONPOINTS) {
		do_map2scene = !do_map2scene; // 0을 1로
		view_cal_points = !view_cal_points; // 1을 0으로
		printf("호모그래피행렬\n");
		homo_matrix = findHomography(diff_vector_point, screen_vector_point, RANSAC);

		cout << screen_vector_point << endl;
		cout << diff_vector_point << endl;

		//cout << homo_matrix << endl;
	}

	// 9개 안모였을 때
	else {
		diff_vector_point.pop_back();
		number_calibration_points_set--;
		printf("diff_vector_point.pop_back\n");
		printf("Attempt to activate calibration without a full set of points.\n");
	}
}


void timer() {
	ftime(&timebuffer);  // timebuffer를채운다
	ltime = timebuffer.time;  // time_t 정보를가져온다
	milisec = timebuffer.millitm;  // milisec를구한다
	now = localtime(&ltime);  // time 정보를채운다
}
