#include "data.h";
#include "StarBurst.h";
#include <process.h>

using namespace cv;
using namespace std;

// 윈도우 시선추적 창
const char* original_wnd = "Original Image View";
const char* gray_wnd = "Gray Image";
const char* cr_wnd = "Corneal Reflection View";
const char* cr_removed_wnd = "Corneal Reflection Removed Image View";
const char* pupil_wnd = "Pupil View";
const char* ransac_wnd = "Ransac View";
const char* contour_th_wnd = "Contour_th_View";

// 윈도우 캘리 창
const char* cal_points_9_wnd = "Homography Window";
const char* front_camera_wnd = "Front Camera Window";

// 윈도우 호모그래피행렬 sub 캡쳐 창
const char* homo_sub_capture_wnd = "Homography Sub Capture Window";

// 시선추적 이미지
Mat original_image;      // Original Image
Mat gray_image;         // Gray Image for Processing
Mat cr_image;           // Only Corneal Reflection Image without Background
Mat cr_removed_image;   // Corneal Reflection Removed Image
Mat pupil_image;      // Only Pupil Image without Background
Mat ransac_image;      // Merge All of Images of Result by Starburst (CR & Pupil Location)
Mat homo_sub_capture_image = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, Scalar(0));

// 캘리 이미지 
Mat cal_points_9 = imread("calibrate.png", 1);
Mat front_original_image;

// 제어용 변수
Mat th_image;
vector<vector<Point>> contours;
int frame_num = 0; // frame_number
int eye_ok = 0; // 제어 동작 여부
int now_onoff_number = 0; // on이면 off message보내기 off면 on message보내기
int onoff_state = 0;
int eyecheck = 0;

//int eyeCheck = 0; // 정면 응시할 때 동공 좌표 확인됐을 때 flag
char correction; // 1차보정 완료 확인

int RightTerm = 0;
int LeftTerm = 0;
int TopTerm = 0;
int BotTerm = 0;
int KeepTerm = 0;

Point2f roi1, roi2;

Rect rect = Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);

// 객체0
StarBurst *starburst;
StarBurstInfo* info;

// 1차 보정 변수 - 사람마다의 눈 차이 잡기 + Contour 제어 중심점 잡기 
int CONTOUR_THRESHOLD = 23;

// 2차 보정 - 캘리브레이션 변수들
#define CALIBRATIONPOINTS    9
int number_calibration_points_set = 0;
int view_cal_points = 1; // 보정중일때의 flag
int do_map2scene = 0; // 캘리브레이션 완료됬을때 flag

Point2f front_pupil_point; // 처음시작 - 정면 동공 중심 좌표
Point2f diff_point; // 동공중심 - 각막반사 포인트들
vector<Point2f> eye_point; // 눈 좌표
vector<Point2f> screen_vector_point; // 정면 좌표
vector<Point2f> diff_vector_point; // 차이벡터 포인터들
vector<Point2f> real_screen_vector_point; // 리얼 정면 좌표
vector<Point2f> real_diff_vector_point; // 리얼 차이벡터 포인터들
Mat homo_matrix; // 호모그래피 행렬

				 // 공유메모리 변수
HANDLE hfMemMap;
static LPSTR lpAddress1, lpAddress2;
struct timeb timebuffer;
struct tm *now;
time_t ltime;
int milisec;
int manipulate = 0; // 제어 조작변수

VideoCapture videoCapture("http://192.168.0.78:9090/?action=stream");
VideoCapture videoCapture2("http://192.168.0.78:9093/?action=stream");


// 각종 사용 함수들
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
	// 공유 변수 메모리들
	char buf[256];
	char ip_Address[100];
	int index = 0;

	if (!videoCapture.isOpened()) {
		printf("첫번째(eye) 카메라를 열수 없습니다. \n"); return;
	}

	if (!videoCapture2.isOpened()) {
		printf("두번쨰(scene)카메라를 열수 없습니다.\n"); return;
	}

	//printf("homo 이미지 총 크기 %d %d\n", cal_points_9.size().width, cal_points_9.size().height);
	// 1. 눈 영상 출력하면서 homography 행렬 만들기
	namedWindow(original_wnd, WINDOW_AUTOSIZE);
	namedWindow(front_camera_wnd, 1);
	namedWindow(homo_sub_capture_wnd, 1);
	_beginthread(CamThread1, 0, (void*)(1));
	_beginthread(CamThread2, 0, (void*)(1));

	Sleep(100);

	//printf("%d %d %d %d")
	// 1차 Contour 보정 (사람 마다 맞춤형 눈 + 제어 중심점 정하기)
	while ((correction = waitKey(10)) != 'q') {
		if (!videoCapture.retrieve(original_image)) {
			printf("Capture1 Null\n");
			continue;
		}// 시선

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

		printf("색---------%d------------\n", CONTOUR_THRESHOLD);

	}

	destroyWindow("gray_image");
	destroyWindow("th_image");

	// 2차 Calibration 보정 ( 9점으로 동공좌표-스크린좌표 캐치 및 Calibration) 진행

	while (1) {
		// 캘리브레이션 포인트 다 채워지면 do_map2scene = 1
		if (do_map2scene&&eyecheck) {
			destroyAllWindows(); // 모든 윈도우 파괴후 다시 생성
			break;
		}

		// 캘리브레이션 포인트 아직 다 안채워졌을 때-
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
		} // else - 캘리 point 다 안채워졌을 때

		waitKey(10);

	}

	// 2. 실시간 시선분석
	setGUI(); // destroyAllWindows()로 다 파괴했으니 다시 윈도우 생성
			  // 제어 변수들

	frame_num = 0;
	eye_ok = 0;
	char c;
	// 한 프레임식 반복

	while ((c = waitKey(10)) != 'q') {
		timer();

		// 두개의 영상이 완벽하게 받아졌을 때
		if ((videoCapture.retrieve(original_image)) && (videoCapture2.retrieve(front_original_image))) {
			if (original_image.empty()) { printf("영상 끝\n"); return; }
			init(); // resize 및 초기 이미지 생성
			eye_ok = eye_ok_check(frame_num); // 눈 감겼는지 체크

											  // 전원on message보내기 or 전원off message 보내기
											  // on or off메시지 보내고 나서는 다시 0으로

			if (now_onoff_number == CONTROLTIME + 10) {
				if (onoff_state == 0) {
					onoff_state = 1; // 객체 제어 확보
					manipulate = 1;
					printf("Control on --------- %d \n", manipulate);
				}

				else {
					onoff_state = 0; // 객체 제어 해제
					LeftTerm = 0;	RightTerm = 0;
					TopTerm = 0;	BotTerm = 0;
					KeepTerm = 0;
					manipulate = 2;
					printf("Control off --------- %d \n", manipulate);
				}
				now_onoff_number = 0; // on or off 메시지 보내고 나서 다시 0으로
			}

			// 눈이 정상적으로 찍혔을 때 (= 실제 시선추적이 진행되는 부분)
			if (eye_ok != 0) {
				starburst = new StarBurst(gray_image, CONTOUR_THRESHOLD);
				starburst->draw_on = 1; // draw 기능 활성화
				starburst->ex_image = original_image.clone(); // Starburst과정을 보기 위한 것.
				info = starburst->Apply();
				printf("Result --------- \n");
				printf("Corneal Pos(%d, %d) radius(%d)\n", info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y, info->CORNEAL_REFLEX.radius);
				printf("Pupil Pos(%d, %d) \n", info->PUPIL.center_of_x, info->PUPIL.center_of_y);

				// 객체 제어 가능한상태일때 왼쪽 보고있으면 LeftTerm ++ 오른쪽 보고있으면 RightTerm++ 위쪽을 보고있으면 TopTerm++ 아래쪽을 보고있으면 BotTerm++

				if (onoff_state) {
					if (info->PUPIL.center_of_x < front_pupil_point.x - 5) LeftTerm++;
					if (info->PUPIL.center_of_x > front_pupil_point.x + 5) RightTerm++;
					if (info->PUPIL.center_of_y < front_pupil_point.y - 3) TopTerm++;
					if (info->PUPIL.center_of_y > front_pupil_point.y + 3) BotTerm++;
					KeepTerm++;
				}

				// KeepTerm이 다쌓여서 manipulate 조작 변수 기록
				if (KeepTerm == CONTROLTIME) {
					if (LeftTerm >= CONTROLTIME2 && TopTerm >= CONTROLTIME2) {  //왼쪽위 11시방향을 봤을 때
						manipulate = 3;
						printf("left top sign --------- %d \n", manipulate);
					}
					else if (RightTerm >= CONTROLTIME2 && TopTerm >= CONTROLTIME2) { //오른쪽위 1시 방향을 봤을 때
						manipulate = 4;
						printf("right top sign --------- %d \n", manipulate);
					}
					else if (LeftTerm >= CONTROLTIME2 && BotTerm >= CONTROLTIME2) { //왼쪽아래 7시 방향을 봤을 때
						manipulate = 5;
						printf("left bot sign --------- %d \n", manipulate);
					}
					else if (RightTerm >= CONTROLTIME2 && BotTerm >= CONTROLTIME2) { //오른쪽아래 5시 방향을 봤을 때
						manipulate = 6;
						printf("right bot sign --------- %d \n", manipulate);
					}

					LeftTerm = 0;	RightTerm = 0;
					TopTerm = 0;	BotTerm = 0;
					KeepTerm = 0;
				}

				// 각막반사 위치 그리기
				Draw_Cross(cr_image, info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y, 3, 3, Scalar(0, 0, 255));
				// 동공 최종 위치 그리기
				circle(ransac_image, Point(info->PUPIL.center_of_x, info->PUPIL.center_of_y), 2, Scalar(255, 0, 0), 3, 8);
				// 타원그리기
				ellipse(ransac_image, Point(info->PUPIL.center_of_x, info->PUPIL.center_of_y), Size(info->PUPIL.width, info->PUPIL.height), info->PUPIL.theta * 180 / PI, 0, 360, MAGENTA, 2);
				// 눈 차이벡터 구하기

				diff_point.x = info->PUPIL.center_of_x - info->CORNEAL_REFLEX.center_of_x;
				diff_point.y = info->PUPIL.center_of_y - info->CORNEAL_REFLEX.center_of_y;

				Mat before = (Mat_<double>(3, 1) << diff_point.x, diff_point.y, 1);

				printf("눈 차이벡터 좌표 : (%d, %d) \n", (int)diff_point.x, (int)diff_point.y);

				Mat real = homo_matrix * before;

				Point calib_point; // 캘리브레이션된 좌표

				calib_point.x = real.at<double>(0) / real.at<double>(2);
				calib_point.y = real.at<double>(1) / real.at<double>(2);

				printf("캘리브레이션 된 좌표 : (%d, %d) \n", calib_point.x, calib_point.y);
				Draw_Cross(front_original_image, calib_point.x, calib_point.y, 7, 7, Scalar(0, 255, 255));

				// mmf

				nMapWrite(lpAddress1, calib_point.x, calib_point.y, manipulate);
				if (manipulate > 0) Sleep(1000);
				manipulate = 0;
			}
			refreshViews();
			frame_num++;
		} // 두 개의 스레드서 영상 받아옴.
	} // 두번쨰 while((c = waitKey(30)) != 'q')
} // main


  // 현재 눈이 eyetracking 할만한 눈인지. + 이미지 write를 통한 추가 검사 가능
int eye_ok_check(int frame_n) {
	// cvtColor(bgr->gray), GaussianBlur() 까지는 진행된 상태
	threshold(gray_image, th_image, CONTOUR_THRESHOLD, 255, THRESH_BINARY_INV);
	findContours(th_image, contours, RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	if (contours.size() == 0) {
		now_onoff_number++;
		printf("눈 감았을때 frame:%d now_onoff_number: %d\n", frame_n, now_onoff_number);
		return 0;
	}
	// 눈 정상적일때
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

// ------------------------------------------------------------------------ 시선 추적 기능
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


// 이미지 윈도우 해제.

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
	// original_image - resize작업 및 gray_image에 복사
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
	// 정면 캠
	imshow(front_camera_wnd, front_original_image);
}

// --------------------------- Draw 기능 관련 ----------------------------------
// 십자가 그리기 center에서 좌우(x_cross_length) 상하(x_cross_length)
void Draw_Cross(Mat image, int centerx, int centery, int x_cross_length, int y_cross_length, Scalar color) {
	Point pt1, pt2, pt3, pt4;

	pt1.x = centerx - x_cross_length; pt1.y = centery;
	pt2.x = centerx + x_cross_length; pt2.y = centery;
	pt3.x = centerx; pt3.y = centery - y_cross_length;
	pt4.x = centerx; pt4.y = centery + y_cross_length;

	line(image, pt1, pt2, color, 1, 8); // 가로 선 그리기 
	line(image, pt3, pt4, color, 1, 8); // 세로 선 그리기
}



//---------------------정면 응시 동공 좌표 확인-----------------------
void on_mouse_eyePoint(int event, int x, int y, int flags, void *param) {
	switch (event) {
		// 정면을 응시할 때 동공의 중심 좌표를 확인하기 위함
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


// ----------------------------------------- 공유메모리 -------------------------------------
// 공유메모리 이용하는 이유는 Eyetracking Visual Studio와 YOLO Visual Studio의 변수 공유를 위해서
// 공유메모리변수 저장 (Memory Mapped File)
// mmf를 이용한 데이터 저장
// Handle 이란 프로세스가 초기화되었을 때 다시 커널 오브젝트를 사용할 수 있게 하려고 운영체제가 핸들 테이블을 할당
// 쉽게 말해 직접 커널 오브젝트 사용말고 간접적으로 사용하게 하는 ?
int nMapWrite(LPSTR lpStr, int x, int y, int z) {
	char szData[1024] = "\n";
	if (!hfMemMap) {
		CloseHandle(hfMemMap); // 핸들 반납
	}

	// 매핑할 파일 만들기
	hfMemMap = CreateFileMapping((HANDLE)-1, NULL, PAGE_READWRITE, 0, 1024, "MemoryMapTest"); // HANDLE형, 파일매핑속성, 파일보호속성, 맥심엄크기최대, 맥심엄크기최소, IPName (name)

	if (hfMemMap == NULL)
		return -1; // 매핑오프젝트 없으면 -1
				   /*
				   // 이미 매핑 오브젝트가 있으면
				   if (GetLastError() == ERROR_ALREADY_EXISTS)
				   printf("이미 매핑 오브젝트가 있어요.\n");
				   */

	lpStr = (LPSTR)MapViewOfFile(hfMemMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);

	if (lpStr == NULL)
		return -2;

	if (x < 0 || x > 639) x = 639;
	if (y < 0 || y > 479) y = 479;
	// szData 버퍼에 직접 쓰고 strcpy로 IpStr에도 씀.
	sprintf(szData, "%d", x * 10000 + y * 10 + z); // 버퍼에 직접 쓰고
	strcpy(lpStr, szData); strcat(lpStr, "\n"); // strcpy, strcat으로 쓰고 한 줄 구분.
	printf("SzData는 %s\n", szData);
	UnmapViewOfFile(lpStr); // 파일 종료
	printf("시선좌표 (%d, %d) 시간: (%d초 %d) 쓰기 완료 \n\n\n", x, y, now->tm_sec, milisec);

	return 0;
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