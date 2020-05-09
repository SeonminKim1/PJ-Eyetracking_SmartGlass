#include "data.h"
#include "StarBurstHelper.h"

// 생성자
StarBurstHelper::StarBurstHelper(Mat image) {
	// (1) 각막반사 사용 이미지 초기화 
	this->cr_image = NULL; this->cr_removed_image = NULL;
	this->cr_image = image;
	this->cr_removed_image = this->cr_image.clone();	// clone by cr_image

	// (2) 각 사용 변수들 초기화 
	// angle_delta - Δ각도 / angle_num - 각도 (360) / biggest_crar - 가장 큰 각막반사점 / cos_array, sin_array - cos 0도 sin 0도 ~ cos 359도 sin359도
	angle_delta = 1 * PI / 180;					// Δ각도-약 0.0174 - 1도씩 변할때마다 변하는 값
	angle_num = (int)(2 * PI / angle_delta);	// number of angle : 초깃값 360
	biggest_crar = (int)((cr_image.size().height / 10) / 2.5);	// bggest radius of r in Image : it would be lower than (height/10) -> 최소 반지름테스트를 15.44 까진 해보아야 한다.
	angle_array = (double*)malloc(angle_num * sizeof(double)); // 360개의 값들 0도일떄의 라디안 값 ~ 359도일때의 라디안 값
	sin_array = (double*)malloc(angle_num * sizeof(double)); // 360개의 값들 0도일떄의 값 ~ 359도일때의 값
	cos_array = (double*)malloc(angle_num * sizeof(double)); // 360개의 값들 0도일떄의 값 ~ 359도일때의 값

	// (3) cos, sin 배열 초기화 (1도부터 360도)
	// calculate sin, cos for each angle
	for (int i = 0; i < angle_num; i++) {
		angle_array[i] = i * angle_delta; // 1도~360도값 초기화
		cos_array[i] = cos(angle_array[i]); // cos 각도 넣었을 때 y축값 - cos함수 그려보면 1에서 -1 사이 
		sin_array[i] = sin(angle_array[i]); // sin 각도 넣었을 때 y축값 - sin함수 그려보면 1에서 -1사이
		//printf("%d %f %f %f\n", i, angle_array[i], cos_array[i], sin_array[i]);
	}
}

// 소멸자
StarBurstHelper::~StarBurstHelper(void) {}

Mat StarBurstHelper::CornealReflexRemove(StarBurstInfo &info) {
	if (cr_image.empty()) { //||
		printf("Error in StarBurst Helper Class::CornealReflexRemove Method->cr_image.empty\n");
	}
	// (1) 각막반사점 위치 찾음.
	CrDetect();

	// (2) 가장 적합한 반지름 구함.
	int fit_radius = this->getFitRadius();
	fit_radius = (int)(2.5*fit_radius); // 2.5배여야 커버할 수 있다.

	// (3) 구한 값들 토대로 헤더 초기화.
	info.CORNEAL_REFLEX.center_of_x = crx;
	info.CORNEAL_REFLEX.center_of_y = cry;
	info.CORNEAL_REFLEX.radius = crar;
	info.CORNEAL_REFLEX.fit_radius = fit_radius;
	printf("(정상) 각막반사점 정보: center_x= %d, center_y= %d, radius= %d, fit_radius= %d\n", crx, cry, crar, fit_radius);

	// (4) 구한 각막반사점 지우기.
	CrRemove(info);
	return this->cr_removed_image;
}

// 커널 탐지.
void StarBurstHelper::CrDetect(void) {
	Mat origin;
	int r = (cr_image.size().width - 1) / 2; // 초깃값 616 기준 307.5정도
	int biggest_crar = (int)((cr_image.size().height / 10) / 2.5); // 초깃값 386 기준 15.4정도
	int threshold;					// Threshold Value
	double min_value, max_value;	// pixel Value
	Point min_loc, max_loc;			// location

	// (1) minMaxLoc를 이용해서 가장 값이 작은 픽셀값과 위치와, 가장 값이 큰 픽셀값의 위치를 구함.
	// Finding Minium Pixel Value, Maxium Pixel Value and Where they Located?
	minMaxLoc(cr_image, &min_value, &max_value, &min_loc, &max_loc);
	IplImage *t_image = &IplImage(cr_image); // Iplimage로 변화ㅏㄴ
	IplImage *threshold_image = cvCloneImage(t_image);

	CvSeq* contour = NULL;
	CvMemStorage* storage = cvCreateMemStorage(0);

	// array for score : estimated contour
	double *scores = (double*)malloc(sizeof(double)*((int)max_value + 1));
	memset(scores, 0, sizeof(double)*((int)max_value + 1));

	// maxium area 
	int area, max_area, sum_area; // 가장 큰 지역값, 지역값들 전부 합친 것
	CvSeq *max_contour;

	// (2) 적응형 임계값을 이용해 max값부터 1까지 threshhold를 반복적용.
	// apply adaptive thresholding technique iteration
	for (threshold = (int)max_value; threshold >= 1; threshold--) {
		cvThreshold(t_image, threshold_image, threshold, 1, CV_THRESH_BINARY);

		// (3) findcontour를 이용해서 윤곽들을 다 구함. contour에 저장
		cvFindContours(threshold_image, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		max_area = 0;
		sum_area = 0;
		max_contour = contour;

		// (4) 구한 contour들(CvSeq* 형태)을 하나씩 접근하면서 최대넓이와 합을 구함.
		for (; contour != 0; contour = contour->h_next) {
			// 전체 넓이 합함
			area = contour->total + (int)(fabs(cvContourArea(contour, CV_WHOLE_SEQ)));
			sum_area += area;
			if (area > max_area) {
				max_area = area;
				max_contour = contour;
			}
		}

		// (5) 각 threshold에서 (1 ~ max_value) 매번 모든 넓이 합한거에서 최대넓이를 뺀 것을 scores 배열에 담음
		if (sum_area - max_area > 0) {
			scores[threshold - 1] = max_area / (sum_area - max_area);
			/*printf("max_area: %d, max_contour: %d, sum_area: %d; scores[%d]: %lf\n",
					max_area, max_contour->total, sum_area, threshold-1, scores[threshold-1]);      */
		}
		else // 첫번째 max_area는 scores에 담지 않음.
			continue;

		// (6) scores배열 비교 및 값 대입 기존 것 보다 이후에 들어온게 크면 값들 대입 및 반복 종료
		// 올바른 커널을 찾았을 떄
		if (scores[threshold - 1] - scores[threshold] < 0) {
			//found the corneal reflection
			crar = (int)sqrt(max_area / PI); // 커널의 반지름
			int sum_x = 0, sum_y = 0;
			CvPoint *point;
			for (int i = 0; i < max_contour->total; i++) {
				point = CV_GET_SEQ_ELEM(CvPoint, max_contour, i);
				sum_x += point->x;
				sum_y += point->y;
			}
			//printf("count %d\n", max_contour->total);
			//printf("sum_x %d sum_y %d \n", sum_x, sum_y);
			crx = sum_x / max_contour->total;
			cry = sum_y / max_contour->total;
			break;
		}
	}
	//printf("(정상)Corneal reflection) 최댓값 - max_value = %lf; 제일 좋은 임계값 - Best threshold = %d\n", max_value, threshold);
	// Reset Image Region Of Interest & Release variable
	free(scores);
	cvReleaseMemStorage(&storage);

	// final step to calculate result;
	if (crar > biggest_crar) {
		cry = crx = -1;
		crar = -1;
	}
}

// CR영역에서 가장 알맞은 radius( crar 값 구함) 값을 찾음.
int StarBurstHelper::getFitRadius(void) {
	// if Error Occur or Apply Not Preprocessing..
	if (crx == -1 || cry == -1 || crar == -1)
		return -1;

	// (1) biggest_crar = 최대로 클수 있는 가상 커널의 크기
	double *ratio = (double*)malloc((biggest_crar - crar + 1) * sizeof(double));
	int i, r, r_delta = 1;
	int x, y, x2, y2;
	double sum, sum2;
	for (r = crar; r <= biggest_crar; r++) {
		sum = 0; sum2 = 0;
		// (2) 360도 방향으로 내가 구한 커널 반지름에 1더하고 1뻇을 때의 픽셀값들의 합을 구함.
		// crar이 3이면 3일때 sum, sum2 다구하고
		for (i = 0; i < angle_num; i++) {
			x = (int)(crx + (r + r_delta)*cos_array[i]);
			y = (int)(cry + (r + r_delta)*sin_array[i]);
			x2 = (int)(crx + (r - r_delta)*cos_array[i]);
			y2 = (int)(cry + (r + r_delta)*sin_array[i]);

			if ((x >= 0 && y >= 0 && x < cr_image.size().width && y < cr_image.size().height) &&
				(x2 >= 0 && y2 >= 0 && x2 < cr_image.size().width && y2 < cr_image.size().height))
			{
				sum += *(cr_image.data + y * cr_image.size().width + x);
				sum2 += *(cr_image.data + y2 * cr_image.size().width + x2);
				//printf("sum sum2 %d %d \n", sum, sum2);
			}
		}

		// (3) ratio배열에 sum/sum2를 넣어서. 체크
		ratio[r - crar] = sum / sum2;
		if (r - crar >= 2) {
			// (4) ratio 배열에서 3개 항목이 순차적인 크기를 가지면 crar을 r-1로 리턴함. 
			if (ratio[r - crar - 2] < ratio[r - crar - 1] && ratio[r - crar] < ratio[r - crar - 1]) {
				free(ratio);
				//printf("(정상)Corneal Reflection -> getFitRadius -> r-crar>=2 인 경우 r-1리턴 r-1=%d\n", r - 1);
				return r - 1;
			}
		}
	}
	free(ratio);
	//printf("crar2 %d\n", crar);
	return crar;
}

// 구한 각막반사 지우기
void StarBurstHelper::CrRemove(StarBurstInfo info) {
	int q = 0;
	int crr = info.CORNEAL_REFLEX.fit_radius;

	// (1) 값 잘못 구했거나, 크기가 너무크거나에 대한 오류 체크.
	if (crx == -1 || cry == -1 || crr == -1) {
		printf("\n StarburstHelper->CrRemove->size error\n");
		return;
	}
	// ex) 반지름이 12인데  각막반사 위치가 11일 때 
	if (crx - crr < 0 || crx + crr > cr_removed_image.size().width || cry - crr < 0 || cry + crr > cr_removed_image.size().height) {
		printf("Corneal reflection -> CrRemove -> cr is too near the image border \n");
		info.CORNEAL_REFLEX.fit_radius = 1;
		crr = 1;
		printf("Corneal reflection -> CrRemove -> fit_radius %d 로 수정 \n", info.CORNEAL_REFLEX.fit_radius);
		//return;
	}

	// (2) 둘레의 픽셀값들(perimeter_pixel) 합 구하기 
	int i, r, r2, x, y;
	UINT8 *perimeter_pixel = (UINT8*)malloc(angle_num * sizeof(int));
	int sum = 0; double avg; // 주변 픽셀값 합, 평균

	for (i = 0; i < angle_num; i++) {
		// (3) 중심기준 각도별 반지름 길이 위치점에서의 픽셀값들 구하기
		x = (int)(crx + crr * cos_array[i]); // 중심기준 각도별 반지름길이 위치점에서의 x값 
		y = (int)(cry + crr * sin_array[i]); // 중심기준 각도별 반지름길이 위치점에서의 y값
		// get Adjust Pixel around center position 

		// (4) 둘레의 픽셀값들 합 구함.
		perimeter_pixel[i] = (UINT8)(*(cr_removed_image.data + (y*(cr_removed_image.size().width + q)) + x));
		sum += perimeter_pixel[i];
	}

	// (5) 둘레 픽셀값들에 대한 평균내기
	// average pixel value
	avg = sum * 1.0 / angle_num;

	// (6) cr_removed_image 픽셀값들 변화시키기
	// 반지름 1서부터 crr까지(1차 for문) 360도(2차 for문)
	for (r = 1; r < crr; r++) {
		r2 = crr - r;
		for (i = 0; i < angle_num; i++) {
			x = (int)(crx + r * cos_array[i]);
			y = (int)(cry + r * sin_array[i]);

			*(cr_removed_image.data + (y*(cr_removed_image.size().width + q)) + x) =
				(UINT8)((r2*1.0 / crr)*avg + (r*1.0 / crr)*perimeter_pixel[i]);
		}
	}
	//printf("여까지 잘됬어\n");
	free(perimeter_pixel);
	//printf("\nend ----------------------- CR REMOVE ----------------------------------\n");
}

