#include <opencv2\opencv.hpp>
#include <iostream>
#include <math.h>

#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480
#define S (IMAGE_WIDTH/8)
#define T (0.15f)

using namespace std;
using namespace cv;

void adaptiveThreshold(unsigned char* input, unsigned char* bin)
{
	unsigned long* integralImg = 0;
	int i, j;
	long sum = 0;
	int count = 0;
	int index;
	int x1, y1, x2, y2;
	int s2 = S / 2;

	// create the integral image
	integralImg = (unsigned long*)malloc(IMAGE_WIDTH*IMAGE_HEIGHT * sizeof(unsigned long*));

	for (i = 0; i<IMAGE_WIDTH; i++)
	{
		// reset this column sum
		sum = 0;

		for (j = 0; j<IMAGE_HEIGHT; j++)
		{
			index = j * IMAGE_WIDTH + i;

			sum += input[index];
			if (i == 0)
				integralImg[index] = sum;
			else
				integralImg[index] = integralImg[index - 1] + sum;
		}
	}

	// perform thresholding
	for (i = 0; i<IMAGE_WIDTH; i++)
	{
		for (j = 0; j<IMAGE_HEIGHT; j++)
		{
			index = j * IMAGE_WIDTH + i;

			// set the SxS region
			x1 = i - s2; x2 = i + s2;
			y1 = j - s2; y2 = j + s2;

			// check the border
			if (x1 < 0) x1 = 0;
			if (x2 >= IMAGE_WIDTH) x2 = IMAGE_WIDTH - 1;
			if (y1 < 0) y1 = 0;
			if (y2 >= IMAGE_HEIGHT) y2 = IMAGE_HEIGHT - 1;

			count = (x2 - x1)*(y2 - y1);

			// I(x,y)=s(x2,y2)-s(x1,y2)-s(x2,y1)+s(x1,x1)
			sum = integralImg[y2*IMAGE_WIDTH + x2] -
				integralImg[y1*IMAGE_WIDTH + x2] -
				integralImg[y2*IMAGE_WIDTH + x1] +
				integralImg[y1*IMAGE_WIDTH + x1];

			if ((long)(input[index] * count) < (long)(sum*(1.0 - T)))
				bin[index] = 0;
			else
				bin[index] = 255;
		}
	}

	free(integralImg);
}

/*
static void calcCenters(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.clear();
	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float((2 * j + i % 2)*squareSize), float(i*squareSize), 0));
}

static void saveparams(const string& filename, const Mat& cameraMatrix, const Mat& distCoeffs,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs, const const double& RMS)
{
	FileStorage fs(filename, FileStorage::WRITE);
	fs << "Calibrate_Accuracy" << RMS;
	fs << "Camera_Matrix" << cameraMatrix;
	fs << "Distortion_Coefficients" << distCoeffs;
	fs << "Rotation_Vector" << rvecs;
	fs << "Translation_vector" << tvecs;

	if (!rvecs.empty() && !tvecs.empty()) {

		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++) {
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);

			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		cvWriteComment(*fs, "Rotation vector + Translation vector", 0);
		fs << "extrinsic_parameters" << bigmat;
	}
	fs.release();
}

*/

int main()
{
	/*Size patternsize = Size(6, 5);//cantidad de anillos
	vector<Point3f> corners3D;
	vector<Point2f> corners2D;//findCirclesGrid guarda los puntos del tablero aqui
	vector<vector<Point2f>> coord2D;//Ubicacion de las esquinas detectadas en la imagen
	vector<vector<Point3f>> coord3D;//Ubicacion real de los puntos 3D*/
	
	string filePath = "E:\\FramesVideoEjemplo\\Frame";
	stringstream imgs;

	Mat imagen, gray, imgBlur, imgBin, imgCanny;
	IplImage* img, *imgGray, *iplImg;
	
	int threshold = 120; // Definimos el valor umbral
	int maxValue = 255; // Definimos el valor máximo
	int thresholdType = CV_THRESH_BINARY; // Definimos el tipo de binarización

	/*calcCenters(patternsize, 25, corners3D);
	bool found;
	*/

	for (int i = 0; i <= 24; i++) {
		imgs << filePath << i << ".jpg";
		imagen = imread(imgs.str().c_str());
		img = cvCloneImage(&(IplImage)imagen);
		imgs = stringstream();
		
		namedWindow("image", WINDOW_AUTOSIZE);
		cvShowImage("image", img);
		//imshow("image", imagen);

		imgGray = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
		cvCvtColor(img, imgGray, COLOR_BGR2GRAY);  
		
		gray = cvarrToMat(imgGray);
		GaussianBlur(gray, imgBlur, Size(3, 3), 0, 0, 0); //Aplicando filtro gausiano
		
		iplImg = cvCloneImage(&(IplImage)imgBlur);		
		adaptiveThreshold((unsigned char*)iplImg->imageData, (unsigned char*)iplImg->imageData);
				
		imgBin = cvarrToMat(iplImg);
		Canny(imgBin, imgCanny, 10, 100, 3, true);
		namedWindow("canny", WINDOW_AUTOSIZE);
		imshow("canny", imgCanny);
				
		//found = findCirclesGrid(imgCanny, patternsize, corners2D, CALIB_CB_ASYMMETRIC_GRID);
		/*
		if (found) {
			drawChessboardCorners(imagen, patternsize, Mat(corners2D), found);
			coord2D.push_back(corners2D);
			coord3D.push_back(corners3D);
		}
		

		Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
		Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
		vector<Mat> rvecs;
		vector<Mat> tvecs;
		
		double rms = calibrateCamera(coord3D, coord2D, imagen.size(), cameraMatrix,		
			distCoeffs, rvecs, tvecs,
			CALIB_FIX_PRINCIPAL_POINT +
			CALIB_FIX_ASPECT_RATIO +
			CALIB_ZERO_TANGENT_DIST
			, TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 2.22e-16));

		cout << "RMS: " << rms << endl;
		cout << "Camera matrix: " << cameraMatrix << endl;
		cout << "Distortion _coefficients: " << distCoeffs << endl;

		saveparams("E:\\DataCam.yml", cameraMatrix, distCoeffs,
			rvecs, tvecs, rms);
		*/
		moveWindow("image", 0, 0);		
		moveWindow("imgCanny", 360, 0); 
		waitKey(2000);
	}
	
	waitKey(0);
	return 0;
}
