#include "data.h";
#include "StarBurst.h";
#include <process.h>

using namespace cv;
using namespace std;

// ������ �ü����� â
const char* original_wnd = "Original Image View";
const char* gray_wnd = "Gray Image";
const char* cr_wnd = "Corneal Reflection View";
const char* cr_removed_wnd = "Corneal Reflection Removed Image View";
const char* pupil_wnd = "Pupil View";
const char* ransac_wnd = "Ransac View";
const char* contour_th_wnd = "Contour_th_View";

// ������ Ķ�� â
const char* cal_points_9_wnd = "Homography Window";
const char* front_camera_wnd = "Front Camera Window";

// ������ ȣ��׷������ sub ĸ�� â
const char* homo_sub_capture_wnd = "Homography Sub Capture Window";

// �ü����� �̹���
Mat original_image;      // Original Image
Mat gray_image;         // Gray Image for Processing
Mat cr_image;           // Only Corneal Reflection Image without Background
Mat cr_removed_image;   // Corneal Reflection Removed Image
Mat pupil_image;      // Only Pupil Image without Background
Mat ransac_image;      // Merge All of Images of Result by Starburst (CR & Pupil Location)
Mat homo_sub_capture_image = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, Scalar(0));

// Ķ�� �̹��� 
Mat cal_points_9 = imread("calibrate.png", 1);
Mat front_original_image;

// ����� ����
Mat th_image;
vector<vector<Point>> contours;
int frame_num = 0; // frame_number
int eye_ok = 0; // ���� ���� ����
int now_onoff_number = 0; // on�̸� off message������ off�� on message������
int onoff_state = 0;
int eyecheck = 0;

//int eyeCheck = 0; // ���� ������ �� ���� ��ǥ Ȯ�ε��� �� flag
char correction; // 1������ �Ϸ� Ȯ��

int RightTerm = 0;
int LeftTerm = 0;
int TopTerm = 0;
int BotTerm = 0;
int KeepTerm = 0;

Point2f roi1, roi2;

Rect rect = Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);

// ��ü0
StarBurst *starburst;
StarBurstInfo* info;

// 1�� ���� ���� - ��������� �� ���� ��� + Contour ���� �߽��� ��� 
int CONTOUR_THRESHOLD = 23;

// 2�� ���� - Ķ���극�̼� ������
#define CALIBRATIONPOINTS    9
int number_calibration_points_set = 0;
int view_cal_points = 1; // �������϶��� flag
int do_map2scene = 0; // Ķ���극�̼� �Ϸ������ flag

Point2f front_pupil_point; // ó������ - ���� ���� �߽� ��ǥ
Point2f diff_point; // �����߽� - �����ݻ� ����Ʈ��
vector<Point2f> eye_point; // �� ��ǥ
vector<Point2f> screen_vector_point; // ���� ��ǥ
vector<Point2f> diff_vector_point; // ���̺��� �����͵�
vector<Point2f> real_screen_vector_point; // ���� ���� ��ǥ
vector<Point2f> real_diff_vector_point; // ���� ���̺��� �����͵�
Mat homo_matrix; // ȣ��׷��� ���

				 // �����޸� ����
HANDLE hfMemMap;
static LPSTR lpAddress1, lpAddress2;
struct timeb timebuffer;
struct tm *now;
time_t ltime;
int milisec;
int manipulate = 0; // ���� ���ۺ���

VideoCapture videoCapture("http://192.168.0.78:9090/?action=stream");
VideoCapture videoCapture2("http://192.168.0.78:9093/?action=stream");


// ���� ��� �Լ���
void timer();
void setGUI();
void unSetGUI();
void on_mouse_eyePoint(int event, int x, int y, int flags, void *param);
void on_mouse_homo(int event, int x, int y, int flags, void *param);
void on_mouse_center(int event, int x, int y, int flags, void *param);
//void Set_Calibration_Point(int x, int y);
void Activate_Calibration();
//void Zero_Calibration();
void init();
void refreshViews();
void Draw_Cross(Mat image, int centerx, int centery, int x_cross_length, int y_cross_length, Scalar color);
void Draw_Circle(Mat image, Point center, int radius, Scalar color, int thickness, int linetype);
int eye_ok_check(int frame_n);
int nMapWrite(LPSTR lpStr, int x, int y, int z);
void CamThread1(void *p);
void CamThread2(void *p);

void main() {
	// ���� ���� �޸𸮵�
	char buf[256];
	char ip_Address[100];
	int index = 0;

	if (!videoCapture.isOpened()) {
		printf("ù��°(eye) ī�޶� ���� �����ϴ�. \n"); return;
	}

	if (!videoCapture2.isOpened()) {
		printf("�ι���(scene)ī�޶� ���� �����ϴ�.\n"); return;
	}

	//printf("homo �̹��� �� ũ�� %d %d\n", cal_points_9.size().width, cal_points_9.size().height);
	// 1. �� ���� ����ϸ鼭 homography ��� �����
	namedWindow(original_wnd, WINDOW_AUTOSIZE);
	namedWindow(front_camera_wnd, 1);
	namedWindow(homo_sub_capture_wnd, 1);
	_beginthread(CamThread1, 0, (void*)(1));
	_beginthread(CamThread2, 0, (void*)(1));

	Sleep(100);

	//printf("%d %d %d %d")
	// 1�� Contour ���� (��� ���� ������ �� + ���� �߽��� ���ϱ�)
	while ((correction = waitKey(10)) != 'q') {
		if (!videoCapture.retrieve(original_image)) {
			printf("Capture1 Null\n");
			continue;
		}// �ü�

		original_image(rect).copyTo(original_image);
		moveWindow(original_wnd, 0, 0);
		imshow(original_wnd, original_image);
		cvtColor(original_image, gray_image, CV_BGR2GRAY);
		threshold(gray_image, th_image, CONTOUR_THRESHOLD, 255, THRESH_BINARY_INV);
		moveWindow("gray_image", 600, 0);
		imshow("gray_image", gray_image);
		moveWindow("th_image", 0, 480);
		imshow("th_image", th_image);

		setMouseCallback("gray_image", on_mouse_eyePoint);

		printf("��---------%d------------\n", CONTOUR_THRESHOLD);

	}

	destroyWindow("gray_image");
	destroyWindow("th_image");

	// 2�� Calibration ���� ( 9������ ������ǥ-��ũ����ǥ ĳġ �� Calibration) ����

	while (1) {
		// Ķ���극�̼� ����Ʈ �� ä������ do_map2scene = 1
		if (do_map2scene&&eyecheck) {
			destroyAllWindows(); // ��� ������ �ı��� �ٽ� ����
			break;
		}

		// Ķ���극�̼� ����Ʈ ���� �� ��ä������ ��-
		else {
			if (!videoCapture.retrieve(original_image)) continue;
			if (!videoCapture2.retrieve(front_original_image))continue;
			original_image(rect).copyTo(original_image);

			moveWindow(original_wnd, 0, 0);
			imshow(original_wnd, original_image);
			setMouseCallback(original_wnd, on_mouse_eyePoint);
			moveWindow(front_camera_wnd, 640, 0);
			moveWindow(homo_sub_capture_wnd, 0, 480);
			imshow(homo_sub_capture_wnd, homo_sub_capture_image);
			imshow(front_camera_wnd, front_original_image);
			setMouseCallback(front_camera_wnd, on_mouse_homo);
		} // else - Ķ�� point �� ��ä������ ��

		waitKey(10);

	}

	// 2. �ǽð� �ü��м�
	setGUI(); // destroyAllWindows()�� �� �ı������� �ٽ� ������ ����
			  // ���� ������

	frame_num = 0;
	eye_ok = 0;
	char c;
	// �� �����ӽ� �ݺ�

	while ((c = waitKey(10)) != 'q') {
		timer();

		// �ΰ��� ������ �Ϻ��ϰ� �޾����� ��
		if ((videoCapture.retrieve(original_image)) && (videoCapture2.retrieve(front_original_image))) {
			if (original_image.empty()) { printf("���� ��\n"); return; }
			init(); // resize �� �ʱ� �̹��� ����
			eye_ok = eye_ok_check(frame_num); // �� ������� üũ

											  // ����on message������ or ����off message ������
											  // on or off�޽��� ������ ������ �ٽ� 0����

			if (now_onoff_number == CONTROLTIME + 10) {
				if (onoff_state == 0) {
					onoff_state = 1; // ��ü ���� Ȯ��
					manipulate = 1;
					printf("Control on --------- %d \n", manipulate);
				}

				else {
					onoff_state = 0; // ��ü ���� ����
					LeftTerm = 0;	RightTerm = 0;
					TopTerm = 0;	BotTerm = 0;
					KeepTerm = 0;
					manipulate = 2;
					printf("Control off --------- %d \n", manipulate);
				}
				now_onoff_number = 0; // on or off �޽��� ������ ���� �ٽ� 0����
			}

			// ���� ���������� ������ �� (= ���� �ü������� ����Ǵ� �κ�)
			if (eye_ok != 0) {
				starburst = new StarBurst(gray_image, CONTOUR_THRESHOLD);
				starburst->draw_on = 1; // draw ��� Ȱ��ȭ
				starburst->ex_image = original_image.clone(); // Starburst������ ���� ���� ��.
				info = starburst->Apply();
				printf("Result --------- \n");
				printf("Corneal Pos(%d, %d) radius(%d)\n", info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y, info->CORNEAL_REFLEX.radius);
				printf("Pupil Pos(%d, %d) \n", info->PUPIL.center_of_x, info->PUPIL.center_of_y);

				// ��ü ���� �����ѻ����϶� ���� ���������� LeftTerm ++ ������ ���������� RightTerm++ ������ ���������� TopTerm++ �Ʒ����� ���������� BotTerm++

				if (onoff_state) {
					if (info->PUPIL.center_of_x < front_pupil_point.x - 5) LeftTerm++;
					if (info->PUPIL.center_of_x > front_pupil_point.x + 5) RightTerm++;
					if (info->PUPIL.center_of_y < front_pupil_point.y - 3) TopTerm++;
					if (info->PUPIL.center_of_y > front_pupil_point.y + 3) BotTerm++;
					KeepTerm++;
				}

				// KeepTerm�� �ٽ׿��� manipulate ���� ���� ���
				if (KeepTerm == CONTROLTIME) {
					if (LeftTerm >= CONTROLTIME2 && TopTerm >= CONTROLTIME2) {  //������ 11�ù����� ���� ��
						manipulate = 3;
						printf("left top sign --------- %d \n", manipulate);
					}
					else if (RightTerm >= CONTROLTIME2 && TopTerm >= CONTROLTIME2) { //�������� 1�� ������ ���� ��
						manipulate = 4;
						printf("right top sign --------- %d \n", manipulate);
					}
					else if (LeftTerm >= CONTROLTIME2 && BotTerm >= CONTROLTIME2) { //���ʾƷ� 7�� ������ ���� ��
						manipulate = 5;
						printf("left bot sign --------- %d \n", manipulate);
					}
					else if (RightTerm >= CONTROLTIME2 && BotTerm >= CONTROLTIME2) { //�����ʾƷ� 5�� ������ ���� ��
						manipulate = 6;
						printf("right bot sign --------- %d \n", manipulate);
					}

					LeftTerm = 0;	RightTerm = 0;
					TopTerm = 0;	BotTerm = 0;
					KeepTerm = 0;
				}

				// �����ݻ� ��ġ �׸���
				Draw_Cross(cr_image, info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y, 3, 3, Scalar(0, 0, 255));
				// ���� ���� ��ġ �׸���
				circle(ransac_image, Point(info->PUPIL.center_of_x, info->PUPIL.center_of_y), 2, Scalar(255, 0, 0), 3, 8);
				// Ÿ���׸���
				ellipse(ransac_image, Point(info->PUPIL.center_of_x, info->PUPIL.center_of_y), Size(info->PUPIL.width, info->PUPIL.height), info->PUPIL.theta * 180 / PI, 0, 360, MAGENTA, 2);
				// �� ���̺��� ���ϱ�

				diff_point.x = info->PUPIL.center_of_x - info->CORNEAL_REFLEX.center_of_x;
				diff_point.y = info->PUPIL.center_of_y - info->CORNEAL_REFLEX.center_of_y;

				Mat before = (Mat_<double>(3, 1) << diff_point.x, diff_point.y, 1);

				printf("�� ���̺��� ��ǥ : (%d, %d) \n", (int)diff_point.x, (int)diff_point.y);

				Mat real = homo_matrix * before;

				Point calib_point; // Ķ���극�̼ǵ� ��ǥ

				calib_point.x = real.at<double>(0) / real.at<double>(2);
				calib_point.y = real.at<double>(1) / real.at<double>(2);

				printf("Ķ���극�̼� �� ��ǥ : (%d, %d) \n", calib_point.x, calib_point.y);
				Draw_Cross(front_original_image, calib_point.x, calib_point.y, 7, 7, Scalar(0, 255, 255));

				// mmf

				nMapWrite(lpAddress1, calib_point.x, calib_point.y, manipulate);
				if (manipulate > 0) Sleep(1000);
				manipulate = 0;
			}
			refreshViews();
			frame_num++;
		} // �� ���� �����弭 ���� �޾ƿ�.
	} // �ι��� while((c = waitKey(30)) != 'q')
} // main


  // ���� ���� eyetracking �Ҹ��� ������. + �̹��� write�� ���� �߰� �˻� ����
int eye_ok_check(int frame_n) {
	// cvtColor(bgr->gray), GaussianBlur() ������ ����� ����
	threshold(gray_image, th_image, CONTOUR_THRESHOLD, 255, THRESH_BINARY_INV);
	findContours(th_image, contours, RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	if (contours.size() == 0) {
		now_onoff_number++;
		printf("�� �������� frame:%d now_onoff_number: %d\n", frame_n, now_onoff_number);
		return 0;
	}
	// �� �������϶�
	else {
		now_onoff_number = 0;
		return 1;
	}
}


void Draw_Circle(Mat image, Point center, int radius, Scalar color, int thickness, int linetype) {
	circle(image, center, radius, color, thickness, linetype);
	//circle(this->houghcircle_image, this->center, this->radius, Scalar(0, 0, 255), 3);
}


void CamThread1(void *p) {
	while (waitKey(10)) {
		videoCapture.grab();
	}
}


void CamThread2(void *p) {
	while (waitKey(10)) {
		videoCapture2.grab();
	}
}

// ------------------------------------------------------------------------ �ü� ���� ���
void setGUI() {
	namedWindow(original_wnd, WINDOW_AUTOSIZE);
	namedWindow(gray_wnd, 1);
	namedWindow(cr_wnd, 1);
	namedWindow(cr_removed_wnd, 1);
	namedWindow(pupil_wnd, 1);
	namedWindow(ransac_wnd, 1);
	namedWindow(front_camera_wnd, 1);

	moveWindow(original_wnd, 0, 0);
	moveWindow(gray_wnd, 320, 0);
	moveWindow(cr_wnd, 640, 0);
	moveWindow(cr_removed_wnd, 0, 240);
	moveWindow(pupil_wnd, 320, 240);
	moveWindow(ransac_wnd, 640, 240);
	moveWindow(front_camera_wnd, 0, 480);
}


// �̹��� ������ ����.

void unSetGUI() {
	original_image.release();
	gray_image.release();
	cr_image.release();
	cr_removed_image.release();
	pupil_image.release();
	ransac_image.release();
	front_original_image.release();
	//destroyAllWindows();
}


void init() {
	// original_image - resize�۾� �� gray_image�� ����
	original_image(rect).copyTo(original_image);
	cvtColor(original_image, gray_image, CV_BGR2GRAY);
	GaussianBlur(gray_image, gray_image, Size(3, 3), 0);
	cr_image = gray_image.clone();
	cr_removed_image = Mat(gray_image.rows, gray_image.cols, CV_8UC3, Scalar(0));
	pupil_image = original_image.clone();
	ransac_image = original_image.clone();
}


void refreshViews() {
	imshow(original_wnd, original_image);
	imshow(gray_wnd, gray_image);
	imshow(cr_wnd, cr_image);
	imshow(cr_removed_wnd, starburst->m_removed_image);
	imshow(pupil_wnd, pupil_image);
	imshow(ransac_wnd, ransac_image);
	//imshow(contour_th_wnd, th_image); // contour th_image
	// ���� ķ
	imshow(front_camera_wnd, front_original_image);
}

// --------------------------- Draw ��� ���� ----------------------------------
// ���ڰ� �׸��� center���� �¿�(x_cross_length) ����(x_cross_length)
void Draw_Cross(Mat image, int centerx, int centery, int x_cross_length, int y_cross_length, Scalar color) {
	Point pt1, pt2, pt3, pt4;

	pt1.x = centerx - x_cross_length; pt1.y = centery;
	pt2.x = centerx + x_cross_length; pt2.y = centery;
	pt3.x = centerx; pt3.y = centery - y_cross_length;
	pt4.x = centerx; pt4.y = centery + y_cross_length;

	line(image, pt1, pt2, color, 1, 8); // ���� �� �׸��� 
	line(image, pt3, pt4, color, 1, 8); // ���� �� �׸���
}



//---------------------���� ���� ���� ��ǥ Ȯ��-----------------------
void on_mouse_eyePoint(int event, int x, int y, int flags, void *param) {
	switch (event) {
		// ������ ������ �� ������ �߽� ��ǥ�� Ȯ���ϱ� ����
	case CV_EVENT_RBUTTONDOWN: {
		if (correction == 'q') {
			front_pupil_point.x = x;
			front_pupil_point.y = y;
			eyecheck = 1;
			printf("asdfsaf");
		}

		else {
			CONTOUR_THRESHOLD = gray_image.at<uchar>(y, x) + 10;
		}
		break;
	}

	case CV_EVENT_LBUTTONDOWN: {
		if (correction != 'q') {
			roi1.x = x;
			roi1.y = y;
		}
		break;
	}

	case CV_EVENT_LBUTTONUP: {
		if (correction != 'q') {
			roi2.x = x;
			roi2.y = y;
			rect = Rect(roi1, roi2);
		}
		break;
	}
	}
}


// ----------------------------------------- �����޸� -------------------------------------
// �����޸� �̿��ϴ� ������ Eyetracking Visual Studio�� YOLO Visual Studio�� ���� ������ ���ؼ�
// �����޸𸮺��� ���� (Memory Mapped File)
// mmf�� �̿��� ������ ����
// Handle �̶� ���μ����� �ʱ�ȭ�Ǿ��� �� �ٽ� Ŀ�� ������Ʈ�� ����� �� �ְ� �Ϸ��� �ü���� �ڵ� ���̺��� �Ҵ�
// ���� ���� ���� Ŀ�� ������Ʈ ��븻�� ���������� ����ϰ� �ϴ� ?
int nMapWrite(LPSTR lpStr, int x, int y, int z) {
	char szData[1024] = "\n";
	if (!hfMemMap) {
		CloseHandle(hfMemMap); // �ڵ� �ݳ�
	}

	// ������ ���� �����
	hfMemMap = CreateFileMapping((HANDLE)-1, NULL, PAGE_READWRITE, 0, 1024, "MemoryMapTest"); // HANDLE��, ���ϸ��μӼ�, ���Ϻ�ȣ�Ӽ�, �ƽɾ�ũ���ִ�, �ƽɾ�ũ���ּ�, IPName (name)

	if (hfMemMap == NULL)
		return -1; // ���ο�����Ʈ ������ -1
				   /*
				   // �̹� ���� ������Ʈ�� ������
				   if (GetLastError() == ERROR_ALREADY_EXISTS)
				   printf("�̹� ���� ������Ʈ�� �־��.\n");
				   */

	lpStr = (LPSTR)MapViewOfFile(hfMemMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);

	if (lpStr == NULL)
		return -2;

	if (x < 0 || x > 639) x = 639;
	if (y < 0 || y > 479) y = 479;
	// szData ���ۿ� ���� ���� strcpy�� IpStr���� ��.
	sprintf(szData, "%d", x * 10000 + y * 10 + z); // ���ۿ� ���� ����
	strcpy(lpStr, szData); strcat(lpStr, "\n"); // strcpy, strcat���� ���� �� �� ����.
	printf("SzData�� %s\n", szData);
	UnmapViewOfFile(lpStr); // ���� ����
	printf("�ü���ǥ (%d, %d) �ð�: (%d�� %d) ���� �Ϸ� \n\n\n", x, y, now->tm_sec, milisec);

	return 0;
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