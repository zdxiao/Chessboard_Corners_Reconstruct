#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

bool getChessboardCorner(const cv::Mat &img, std::vector<cv::Point2f> &corners, cv::Size patternSize);
void prepareProjectMat(cv::Mat &LK, cv::Mat &RK, cv::Mat &R, cv::Mat &T, cv::Mat &L_ProjectMat, cv::Mat &R_ProjectMat);
double calChessboradCubeSize(cv::Mat Corners3d);

int main(int argc, char* argv[])
{
	cv::Mat L = cv::imread("../images/L.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat R = cv::imread("../images/R.png", CV_LOAD_IMAGE_GRAYSCALE);
	std::vector<cv::Point2f> L_corners, R_corners;
	cv::Size patternSize(10, 13);
	// cv::Size patternSize(5, 8);
	// detect chessboard corners
	bool corners_got_L = getChessboardCorner(L, L_corners, patternSize);
	bool corners_got_R = getChessboardCorner(R, R_corners, patternSize);

	if(!corners_got_L || !corners_got_R)
	{
		std::cout << "Get chessboard corners error!" << std::endl;
		return -1;
	}

	cv::Mat L_ProjectMat, R_ProjectMat;

// parameter for Oseme Stereo Camera
	cv::Mat LK = (cv::Mat_<double>(3, 3) 
			<< 149.547298796355, 0, 319.773511280681, 
				0, 	149.645388214504, 240.264589337940,
				0, 			0, 				1);

	cv::Mat RK = (cv::Mat_<double>(3, 3) 
			<< 151.121296912028, 0, 320.030542369825, 
				0, 150.950134114643, 240.626477688915, 
				0, 			0, 				1);

	cv::Mat rotMatrix = (cv::Mat_<double>(3, 3) 
			<< 0.999912726924717, -0.00572856656604976, 0.0119047074334342, 
			   0.00579965007895173, 0.999965509194672, -0.00594512237062572, 
			   -0.0118702398012442, 0.00601364665891970, 0.999911462811044);

	cv::Mat translation = (cv::Mat_<double>(3, 1) 
			<< -0.0665096120636032, -0.00120864382661637, 0.000391898091312008);


// parameter for Etron Stereo Camera
	// cv::Mat LK = (cv::Mat_<double>(3, 3) 
	// 		<< 509.983914578054, 0, 331.812291248236, 
	// 			0, 512.761512597142, 195.791918460058, 
	// 			0, 0, 1);

	// cv::Mat RK = (cv::Mat_<double>(3, 3) 
	// 		<< 509.312568682217, 0, 334.563583071416, 
	// 			0, 511.176254380978, 197.095989202155, 
	// 			0, 0, 1);

	// cv::Mat rotMatrix = (cv::Mat_<double>(3, 3) 
	// 		<< 0.999970047063419, 0.000432714407725337, -0.00772772503554598, 
	// 		-0.000463515531983943, 0.999991954629630, -0.00398444843900750, 
	// 		0.00772593873488956, 0.00398791101365697, 0.999962202503881);

	// cv::Mat translation = (cv::Mat_<double>(3, 1) 
	// 		<< -0.119548030739185, -0.000513039243305221, -0.00208018401114427);


// f = 350
	// cv::Mat LK = (cv::Mat_<double>(3, 3) 
	// 		<< 3.4744467506766159e+02, 0., 3.2737382548703812e+02, 
	// 		0., 3.4808169498377572e+02, 2.5370914925404722e+02, 
	// 		0., 0., 1.);

	// cv::Mat RK = (cv::Mat_<double>(3, 3) 
	// 		<<  3.4862913717020223e+02, 0., 3.3290091646470989e+02, 
	// 		0., 3.4914531352030764e+02, 2.5116887426293255e+02,
 //        	0., 0., 1. );

	// cv::Mat rotMatrix = (cv::Mat_<double>(3, 3) 
	// 		<< 9.9992791167175199e-01, -5.6959739137493201e-03, 1.0570115464971636e-02, 
	// 		5.6871160601332802e-03, 9.9998345161255031e-01, 8.6787785360897709e-04, 
	// 		-1.0574883956220013e-02, -8.0770181632707503e-04, 9.9994375814197090e-01 );

	// cv::Mat translation = (cv::Mat_<double>(3, 1) 
	// 		<< -6.6172189056587699e-02, -2.8098628738460282e-04, 6.5226190408761110e-04);

	// cv::Mat LK = (cv::Mat_<double>(3, 3) 
	// 		<< 2.0854034432466037e+02, 0., 3.3193799843586658e+02, 
	// 		0., 2.0892269110439511e+02, 2.5464970862257988e+02, 
	// 		0., 0., 1. );

// f = 200
	// cv::Mat RK = (cv::Mat_<double>(3, 3) 
	// 		<<  2.0881593277768414e+02, 0., 3.3937980564397611e+02, 
	// 		0., 2.0912510328161829e+02, 2.5185311466219545e+02, 
	// 		0., 0., 1.);

	// cv::Mat rotMatrix = (cv::Mat_<double>(3, 3) 
	// 		<< 9.9992791167175199e-01, -5.6959739137493201e-03, 1.0570115464971636e-02, 
	// 		5.6871160601332802e-03, 9.9998345161255031e-01, 8.6787785360897709e-04, 
	// 		-1.0574883956220013e-02, -8.0770181632707503e-04, 9.9994375814197090e-01 );

	// cv::Mat translation = (cv::Mat_<double>(3, 1) 
	// 		<< -6.6172189056587699e-02, -2.8098628738460282e-04, 6.5226190408761110e-04);


	prepareProjectMat(LK, RK, rotMatrix, translation, L_ProjectMat, R_ProjectMat);

	int n_pnts = L_corners.size();

	// std::cout << n_pnts << std::endl;

	cv::Mat Corners3d(4, n_pnts, CV_64FC1);

	// std::cout << "L_P" << L_ProjectMat << std::endl << 
	// 	"R_P" << R_ProjectMat << std::endl;
	triangulatePoints(L_ProjectMat, R_ProjectMat, 
		cv::Mat(L_corners), cv::Mat(R_corners), Corners3d);

	// std::cout << Corners3d << std::endl;

	calChessboradCubeSize(Corners3d);

	return 0;
}

double calChessboradCubeSize(cv::Mat Corners3d)
{
	int n_pnts = Corners3d.cols;
	cv::Mat corners(3, n_pnts, Corners3d.type());
	for(int i = 0; i < 3; ++i)
	{
		cv::Mat tmp;
		divide(Corners3d.row(i), Corners3d.row(3), tmp);
		tmp.copyTo(corners.row(i));
	}
	std::cout << corners << std::endl;
	cv::Mat tmp1 = corners(cv::Rect(1, 0, n_pnts - 1, 3));
	cv::Mat tmp2 = corners(cv::Rect(0, 0, n_pnts - 1, 3));
	cv::Mat tmp3 = tmp1 - tmp2;
	// std::cout << tmp3.size() << std::endl;
	cv::Mat dis(1, n_pnts - 1, CV_64FC1);
	double sum = 0;
	int counter = 0;
	for(int i = 0; i < n_pnts - 1; ++i)
	{
		dis.at<double>(0, i) = norm(tmp3.col(i));
		if(dis.at<double>(0, i) < 0.09)
		{
			sum += dis.at<double>(0, i);
			counter++;
		}
		else
		{
			// std::cout << i << std::endl;
		}
		// std::cout << cv::norm(tmp3.col(i)) << " " << dis.at<double>(0, i) << std::endl;
	}
	double avg_size = sum / counter;

	std::cout << "dis" << std::endl << dis << std::endl;
	std::cout << "average size = " << avg_size << std::endl;
	return 0.0;
}

void prepareProjectMat(cv::Mat &LK, cv::Mat &RK, cv::Mat &R, cv::Mat &T, cv::Mat &L_ProjectMat, cv::Mat &R_ProjectMat)
{
	// Left
	cv::Mat normalP = (cv::Mat_<double>(3, 4) 
		<< 1, 0, 0, 0, 
		   0, 1, 0, 0,
		   0, 0, 1, 0);
	L_ProjectMat = LK * normalP;
	// Right
	cv::Mat P(3, 4, CV_64FC1);
	cv::Mat sub1 = P(cv::Rect(0, 0, 3, 3));
	R.copyTo(sub1);
	cv::Mat sub2 = P(cv::Rect(3, 0, 1, 3));
	T.copyTo(sub2);
	//std::cout << P << std::endl;
	R_ProjectMat = RK * P;
}

bool getChessboardCorner(const cv::Mat &img, std::vector<cv::Point2f> &corners, cv::Size patternSize)
{
	bool patternfound = findChessboardCorners(img, patternSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH
				 + CV_CALIB_CB_FAST_CHECK);
	//CV_CALIB_CB_NORMALIZE_IMAGE

	if(patternfound)
	{

		std::cout << "pattern found!" << std::endl;

		cornerSubPix(img, corners, cv::Size(5, 5), cv::Size(-1, -1),
		    cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	}
	else
	{
		std::cout << "pattern not found!" << std::endl;
		return false;
	}
	cv::Mat img_color;
	cvtColor(img, img_color, cv::COLOR_GRAY2BGR);
	drawChessboardCorners(img_color, patternSize, cv::Mat(corners), patternfound); 
	//resize(L_color, L_color, cv::Size(), 3.0, 3.0);
	cv::namedWindow("image", CV_WINDOW_NORMAL);
	imshow("image", img_color);
	cv::waitKey();
	return true;
}