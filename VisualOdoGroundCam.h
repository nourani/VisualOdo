/*
 * A demonstration of visual odometry for car-like vehicles using a downward
 * looking camera. Please refer to the publicaiton:
 *		Navid Nourani-Vatani and Paulo VK Borges
 *			Correlation-based Visual Odometry for Car-like Vehicles
 *			Journal of Field Robotics, September 2011
 *  Copyright (C) 2012  Navid Nourani-Vatani
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef   	_VISUALODOGROUNDCAM_H_
#define   	_VISUALODOGROUNDCAM_H_


// Std C/C++ libs
#include <stdio.h>
#include <stdlib.h>
#include <list>
#include <math.h>
#include <sys/time.h>
#include <dirent.h>
#include <errno.h>
// OpenCV libs
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

#include <small/linalg.hh>
#include <small/Pose3D.hh>

#ifdef USE_CIMG
// CImg lib
#define cimg_plugin <CImg/plugins/cimgIPL.h>
#include <CImg/CImg.h>
using namespace cimg_library;
#endif

// Music lib. Download FMIT src, make and take out libMusic
#ifdef USE_LIBMUSIC
#include <Music/Autocorrelation.h>
#include <Music/LPC.h>
#endif

using namespace std;


// Some Constants
#define MOTION_FORWARD	 1
#define MOTION_NONE		 0
#define MOTION_REVERSE	-1
#define	imuAvailable	false

/**
 * The maximum theoretical std dev is achieved in an image consisting of
 * 	black and white pixels resulting in a std.dev of 128 (in an 8bit image).
 * The maximum variance is the square of this of course.
 */
int 					  MAX_THEORETICAL_STDEV;
int 					  MAX_THEORETICAL_VARIANCE;
int 					  MAX_THEORETICAL_MAD;

// Global control variables
bool					  getFrameFromImages = true;	// We would like to be the filenames are the time the frames were grabbed
// using this format YYYYMMDD-HHMMSS-MMS. This feature is not implemented in this code, so we are using the system time instead
string					  fileDir = "/Linux/home/nou038/proj/VisualOdoGroundCam/test_data";
string 					  fileExt = "ppm";
vector<string>			  fileVec;

bool 					  done = false; /* thread termination flag */
bool					  show = true; /* display what's happening (really slows down everything! */
bool					  stepByStep = false;
bool					  writeFramesAsImage = false;
int 					  verbose = 0; /* verbosity level */

IplImage * prevDrawFrame = NULL;
IplImage * currDrawFrame = NULL;
CvFont font;

struct VisualOdo_t {

	// Theoretical maximum frame rate sets the maximum possible velocity (see main)
	float					  MAX_FRAME_RATE;
	float					  MAX_VEL;

	/*
	 * Template and quality measure
	 *  0 = Auto-correlation, 1 = Std Dev, 2 = Variance, 3 = MAD, Entropy = 4
	 */
	int						  templateQualityMeasure;
	int						  numTemplateTests;
	int						  templateWinLength;
	bool					  restrictSearch;
	double					  restrictFactor;
	int						  restrictedSearchArea;



	// Image frame, numbering and timing
	CvCapture 				* capture;
	IplImage				* frame; /* the most recent grabbed frame */
	unsigned long 			  frameNum; /* updated every time a frame is grabbed */
	unsigned long 			  startFrame; /* frames up to this frame are skipped */
	unsigned long 			  endFrame; /* frames after this frame are skipped */
	double 					  frameGrabTimeDouble;
	double					  resizeFactor; /* resize the input image to speed up even further */

	// Pose variables
	SMALL::Pose3D			  A3D;
	SMALL::Pose3D 			  poseCam;
	SMALL::Pose3D			  pose;
	double 					  vel;
	double					  velPrev;
	int 					  directionOfTravel;
	float					  zConst;
	// If IMU available we can set the offsets (in the cpp file)
	const static double		  imuOffsetRoll;
	const static double 	  imuOffsetPitch;
	const static double 	  imuOffsetYaw;


	// Forward prediciton variables
	int predictDeltaU;
	int predictDeltaV;
#ifdef USE_LIBMUSIC
	//
	static const int FILT_SIZE = 8;
	std::list<double> dUHist;
	std::list<double> dVHist;
	double xHist[filtSize];
	double yHist[filtSize];
	list<double>::iterator dUIter;
	list<double>::iterator dVIter;
#else
	// Kalman filter stuff
	CvRandState rng;
	CvKalman * kalmanU;
	CvKalman * kalmanV;
	// state
	CvMat * xU_k;
	CvMat * xV_k;
	// process noise
	CvMat * wU_k;
	CvMat * wV_k;
	// measurement
	CvMat * zU_k;
	CvMat * zV_k;
#endif


	VisualOdo_t() {
		templateQualityMeasure = 4;
		numTemplateTests = 3;
		templateWinLength = 92;
		restrictSearch = true;
		restrictFactor = 1.5;
		restrictedSearchArea = restrictFactor * templateWinLength;


		capture = NULL;
		frame = NULL;
		frameNum = 0;
		startFrame = 0;
		endFrame = (long)4e9;
		frameGrabTimeDouble = 0.0;
		resizeFactor = 1.0/1.0;

		directionOfTravel = MOTION_NONE;
	}

};

#endif 	    /* _VISUALODOGROUNDCAM_H_ */
