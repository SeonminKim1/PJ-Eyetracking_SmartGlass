#include "data.h"
#include "StarBurstHelper.h"

// ������
StarBurstHelper::StarBurstHelper(Mat image) {
	// (1) �����ݻ� ��� �̹��� �ʱ�ȭ 
	this->cr_image = NULL; this->cr_removed_image = NULL;
	this->cr_image = image;
	this->cr_removed_image = this->cr_image.clone();	// clone by cr_image

	// (2) �� ��� ������ �ʱ�ȭ 
	// angle_delta - �İ��� / angle_num - ���� (360) / biggest_crar - ���� ū �����ݻ��� / cos_array, sin_array - cos 0�� sin 0�� ~ cos 359�� sin359��
	angle_delta = 1 * PI / 180;					// �İ���-�� 0.0174 - 1���� ���Ҷ����� ���ϴ� ��
	angle_num = (int)(2 * PI / angle_delta);	// number of angle : �ʱ갪 360
	biggest_crar = (int)((cr_image.size().height / 10) / 2.5);	// bggest radius of r in Image : it would be lower than (height/10) -> �ּ� �������׽�Ʈ�� 15.44 ���� �غ��ƾ� �Ѵ�.
	angle_array = (double*)malloc(angle_num * sizeof(double)); // 360���� ���� 0���ϋ��� ���� �� ~ 359���϶��� ���� ��
	sin_array = (double*)malloc(angle_num * sizeof(double)); // 360���� ���� 0���ϋ��� �� ~ 359���϶��� ��
	cos_array = (double*)malloc(angle_num * sizeof(double)); // 360���� ���� 0���ϋ��� �� ~ 359���϶��� ��

	// (3) cos, sin �迭 �ʱ�ȭ (1������ 360��)
	// calculate sin, cos for each angle
	for (int i = 0; i < angle_num; i++) {
		angle_array[i] = i * angle_delta; // 1��~360���� �ʱ�ȭ
		cos_array[i] = cos(angle_array[i]); // cos ���� �־��� �� y�ప - cos�Լ� �׷����� 1���� -1 ���� 
		sin_array[i] = sin(angle_array[i]); // sin ���� �־��� �� y�ప - sin�Լ� �׷����� 1���� -1����
		//printf("%d %f %f %f\n", i, angle_array[i], cos_array[i], sin_array[i]);
	}
}

// �Ҹ���
StarBurstHelper::~StarBurstHelper(void) {}

Mat StarBurstHelper::CornealReflexRemove(StarBurstInfo &info) {
	if (cr_image.empty()) { //||
		printf("Error in StarBurst Helper Class::CornealReflexRemove Method->cr_image.empty\n");
	}
	// (1) �����ݻ��� ��ġ ã��.
	CrDetect();

	// (2) ���� ������ ������ ����.
	int fit_radius = this->getFitRadius();
	fit_radius = (int)(2.5*fit_radius); // 2.5�迩�� Ŀ���� �� �ִ�.

	// (3) ���� ���� ���� ��� �ʱ�ȭ.
	info.CORNEAL_REFLEX.center_of_x = crx;
	info.CORNEAL_REFLEX.center_of_y = cry;
	info.CORNEAL_REFLEX.radius = crar;
	info.CORNEAL_REFLEX.fit_radius = fit_radius;
	printf("(����) �����ݻ��� ����: center_x= %d, center_y= %d, radius= %d, fit_radius= %d\n", crx, cry, crar, fit_radius);

	// (4) ���� �����ݻ��� �����.
	CrRemove(info);
	return this->cr_removed_image;
}

// Ŀ�� Ž��.
void StarBurstHelper::CrDetect(void) {
	Mat origin;
	int r = (cr_image.size().width - 1) / 2; // �ʱ갪 616 ���� 307.5����
	int biggest_crar = (int)((cr_image.size().height / 10) / 2.5); // �ʱ갪 386 ���� 15.4����
	int threshold;					// Threshold Value
	double min_value, max_value;	// pixel Value
	Point min_loc, max_loc;			// location

	// (1) minMaxLoc�� �̿��ؼ� ���� ���� ���� �ȼ����� ��ġ��, ���� ���� ū �ȼ����� ��ġ�� ����.
	// Finding Minium Pixel Value, Maxium Pixel Value and Where they Located?
	minMaxLoc(cr_image, &min_value, &max_value, &min_loc, &max_loc);
	IplImage *t_image = &IplImage(cr_image); // Iplimage�� ��ȭ����
	IplImage *threshold_image = cvCloneImage(t_image);

	CvSeq* contour = NULL;
	CvMemStorage* storage = cvCreateMemStorage(0);

	// array for score : estimated contour
	double *scores = (double*)malloc(sizeof(double)*((int)max_value + 1));
	memset(scores, 0, sizeof(double)*((int)max_value + 1));

	// maxium area 
	int area, max_area, sum_area; // ���� ū ������, �������� ���� ��ģ ��
	CvSeq *max_contour;

	// (2) ������ �Ӱ谪�� �̿��� max������ 1���� threshhold�� �ݺ�����.
	// apply adaptive thresholding technique iteration
	for (threshold = (int)max_value; threshold >= 1; threshold--) {
		cvThreshold(t_image, threshold_image, threshold, 1, CV_THRESH_BINARY);

		// (3) findcontour�� �̿��ؼ� �������� �� ����. contour�� ����
		cvFindContours(threshold_image, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		max_area = 0;
		sum_area = 0;
		max_contour = contour;

		// (4) ���� contour��(CvSeq* ����)�� �ϳ��� �����ϸ鼭 �ִ���̿� ���� ����.
		for (; contour != 0; contour = contour->h_next) {
			// ��ü ���� ����
			area = contour->total + (int)(fabs(cvContourArea(contour, CV_WHOLE_SEQ)));
			sum_area += area;
			if (area > max_area) {
				max_area = area;
				max_contour = contour;
			}
		}

		// (5) �� threshold���� (1 ~ max_value) �Ź� ��� ���� ���Ѱſ��� �ִ���̸� �� ���� scores �迭�� ����
		if (sum_area - max_area > 0) {
			scores[threshold - 1] = max_area / (sum_area - max_area);
			/*printf("max_area: %d, max_contour: %d, sum_area: %d; scores[%d]: %lf\n",
					max_area, max_contour->total, sum_area, threshold-1, scores[threshold-1]);      */
		}
		else // ù��° max_area�� scores�� ���� ����.
			continue;

		// (6) scores�迭 �� �� �� ���� ���� �� ���� ���Ŀ� ���°� ũ�� ���� ���� �� �ݺ� ����
		// �ùٸ� Ŀ���� ã���� ��
		if (scores[threshold - 1] - scores[threshold] < 0) {
			//found the corneal reflection
			crar = (int)sqrt(max_area / PI); // Ŀ���� ������
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
	//printf("(����)Corneal reflection) �ִ� - max_value = %lf; ���� ���� �Ӱ谪 - Best threshold = %d\n", max_value, threshold);
	// Reset Image Region Of Interest & Release variable
	free(scores);
	cvReleaseMemStorage(&storage);

	// final step to calculate result;
	if (crar > biggest_crar) {
		cry = crx = -1;
		crar = -1;
	}
}

// CR�������� ���� �˸��� radius( crar �� ����) ���� ã��.
int StarBurstHelper::getFitRadius(void) {
	// if Error Occur or Apply Not Preprocessing..
	if (crx == -1 || cry == -1 || crar == -1)
		return -1;

	// (1) biggest_crar = �ִ�� Ŭ�� �ִ� ���� Ŀ���� ũ��
	double *ratio = (double*)malloc((biggest_crar - crar + 1) * sizeof(double));
	int i, r, r_delta = 1;
	int x, y, x2, y2;
	double sum, sum2;
	for (r = crar; r <= biggest_crar; r++) {
		sum = 0; sum2 = 0;
		// (2) 360�� �������� ���� ���� Ŀ�� �������� 1���ϰ� 1�P�� ���� �ȼ������� ���� ����.
		// crar�� 3�̸� 3�϶� sum, sum2 �ٱ��ϰ�
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

		// (3) ratio�迭�� sum/sum2�� �־. üũ
		ratio[r - crar] = sum / sum2;
		if (r - crar >= 2) {
			// (4) ratio �迭���� 3�� �׸��� �������� ũ�⸦ ������ crar�� r-1�� ������. 
			if (ratio[r - crar - 2] < ratio[r - crar - 1] && ratio[r - crar] < ratio[r - crar - 1]) {
				free(ratio);
				//printf("(����)Corneal Reflection -> getFitRadius -> r-crar>=2 �� ��� r-1���� r-1=%d\n", r - 1);
				return r - 1;
			}
		}
	}
	free(ratio);
	//printf("crar2 %d\n", crar);
	return crar;
}

// ���� �����ݻ� �����
void StarBurstHelper::CrRemove(StarBurstInfo info) {
	int q = 0;
	int crr = info.CORNEAL_REFLEX.fit_radius;

	// (1) �� �߸� ���߰ų�, ũ�Ⱑ �ʹ�ũ�ų��� ���� ���� üũ.
	if (crx == -1 || cry == -1 || crr == -1) {
		printf("\n StarburstHelper->CrRemove->size error\n");
		return;
	}
	// ex) �������� 12�ε�  �����ݻ� ��ġ�� 11�� �� 
	if (crx - crr < 0 || crx + crr > cr_removed_image.size().width || cry - crr < 0 || cry + crr > cr_removed_image.size().height) {
		printf("Corneal reflection -> CrRemove -> cr is too near the image border \n");
		info.CORNEAL_REFLEX.fit_radius = 1;
		crr = 1;
		printf("Corneal reflection -> CrRemove -> fit_radius %d �� ���� \n", info.CORNEAL_REFLEX.fit_radius);
		//return;
	}

	// (2) �ѷ��� �ȼ�����(perimeter_pixel) �� ���ϱ� 
	int i, r, r2, x, y;
	UINT8 *perimeter_pixel = (UINT8*)malloc(angle_num * sizeof(int));
	int sum = 0; double avg; // �ֺ� �ȼ��� ��, ���

	for (i = 0; i < angle_num; i++) {
		// (3) �߽ɱ��� ������ ������ ���� ��ġ�������� �ȼ����� ���ϱ�
		x = (int)(crx + crr * cos_array[i]); // �߽ɱ��� ������ ���������� ��ġ�������� x�� 
		y = (int)(cry + crr * sin_array[i]); // �߽ɱ��� ������ ���������� ��ġ�������� y��
		// get Adjust Pixel around center position 

		// (4) �ѷ��� �ȼ����� �� ����.
		perimeter_pixel[i] = (UINT8)(*(cr_removed_image.data + (y*(cr_removed_image.size().width + q)) + x));
		sum += perimeter_pixel[i];
	}

	// (5) �ѷ� �ȼ����鿡 ���� ��ճ���
	// average pixel value
	avg = sum * 1.0 / angle_num;

	// (6) cr_removed_image �ȼ����� ��ȭ��Ű��
	// ������ 1������ crr����(1�� for��) 360��(2�� for��)
	for (r = 1; r < crr; r++) {
		r2 = crr - r;
		for (i = 0; i < angle_num; i++) {
			x = (int)(crx + r * cos_array[i]);
			y = (int)(cry + r * sin_array[i]);

			*(cr_removed_image.data + (y*(cr_removed_image.size().width + q)) + x) =
				(UINT8)((r2*1.0 / crr)*avg + (r*1.0 / crr)*perimeter_pixel[i]);
		}
	}
	//printf("������ �߉��\n");
	free(perimeter_pixel);
	//printf("\nend ----------------------- CR REMOVE ----------------------------------\n");
}

