/*
Copyright 2010. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso.
Authors: Bernd Kitt, Andreas Geiger

libviso is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#ifndef _VISUALODOMETRY_H_
#define _VISUALODOMETRY_H_

#include "cvm.h"
#include "trifocalTensor.h"
#include <vector>

#define pi 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117068

class VisualOdometry
{
public:
  
  // constructor
  // initialState ................... initial state vector (6 dimensional: vx,vy,vz,omegax,omegay,omegaz; unit: 3x m/s, 3x rad/s)
  //                                  the state vector represents the transformation to be estimated, which takes a point
  //                                  in the previous left coordinate system and projects it into the current left coordinate system
  //                                  for more detailed specification of the rotation matrices see projectionMatricesFromMotion()
  // initialCovarianceState ......... initial 6x6 covariance matrix of the state vector
  // covarianceSystem ............... 6x6 system covariance matrix wrt. state vector
  // intrinsicCalibration_right ..... 3x3 matrix with intrinsics of right camera
  // intrinsicCalibration_left ...... 3x3 matrix with intrinsics of left camera
  // extrinsicRotation R............. 3x3 matrix with extrinsic rotation between left and right camera
  // extrinsicTranslation t ......... 3x1 matrix with extrinsic translation between left and right camera
  //                                  here [R|t] projects a point from the left coordinate system to the right one
  // param_ransac_samples ........... number of ransac samples to draw (if you have many outliers you might want to increase this)
  // param_ransac_inlier_distance ... euclidean distance in pixels for a projected correspondence to be considered as inlier
  // param_ispkf_max_iterations ..... limit of ISPKF iterations, usually only the first few frames need more than 3-5 iterations
  // param_ispkf_threshold .......... state-change threshold criteria for the ISPKF
	VisualOdometry(cvm::rmatrix initialState, cvm::srsmatrix initialCovarianceState, cvm::srsmatrix covarianceSystem,
                 cvm::srmatrix intrinsicCalibration_right, cvm::srmatrix intrinsicCalibration_left,
                 cvm::srmatrix extrinsicRotation, cvm::rmatrix extrinsicTranslation,
                 int param_ransac_samples=10,double param_ransac_inlier_distance=3,int param_ispkf_max_iterations=10,double param_ispkf_threshold=0.001);
  
  // Kalman prediction: call this before innovation
  void prediction();
  
  // Kalman innovation: call this after prediction
  // matchesLeftPrevious ............ image coordinates of left previous correspondences
  // matchesRightPrevious ........... image coordinates of right previous correspondences
  // matchesLeftCurrent ............. image coordinates of left current correspondences
  // matchesRightCurrent ............ image coordinates of right current correspondences
  // deltaT ......................... time between previous and current frame in seconds
  // varianceMeasurements ........... variance of measurements in pixel^2
	void innovation(const cvm::rmatrix &matchesLeftPrevious, const cvm::rmatrix &matchesRightPrevious,
                  const cvm::rmatrix &matchesLeftCurrent, const cvm::rmatrix &matchesRightCurrent,
                  double deltaT,double varianceMeasurements);

  // returns the current state
  cvm::rmatrix getState() { return state; }
  
  // returns the covariance of the current state
  cvm::srsmatrix getCovarianceState() { return covarianceState; }
  
  // returns the current 3x4 transformation matrix (projects from left previous to left current)
  cvm::rmatrix getTransformation() { return CAM_transformationMatrixLeft; }
	
  // returns a vector with all inliers after the last iteration
  std::vector<int> getInliers() { return ISPKF_featureIndices_inlierBest; }

private:
  
	unsigned int   dimensionState;
	cvm::rmatrix   state;
	cvm::srsmatrix covarianceState;
	cvm::srsmatrix covarianceSystem;

	// configuration stereo camera rig (assumed as constant)
	cvm::srmatrix CAMERARIG_extrinsicRotation;
	cvm::rmatrix  CAMERARIG_extrinsicTranslation;
	cvm::srmatrix CAMERARIG_intrinsicCalibration_right;
	cvm::srmatrix CAMERARIG_intrinsicCalibration_left;
	cvm::srmatrix CAMERARIG_essentialMatrix;
	cvm::srmatrix CAMERARIG_fundamentalMatrix;

	// projection matrices
	cvm::rmatrix CAM_projectionMatrixRightPrevious;
	cvm::rmatrix CAM_projectionMatrixLeftPrevious;
	cvm::rmatrix CAM_projectionMatrixRightCurrent;
	cvm::rmatrix CAM_projectionMatrixLeftCurrent;
  cvm::rmatrix CAM_transformationMatrixLeft;

	// trifocal tensors
	double trifocalTensorRight[3][3][3];
	double trifocalTensorLeft[3][3][3];

	// RANSAC parameters
	unsigned int RANSAC_featuresPerSample;
	double       RANSAC_maxEuclideanDistanceInlier;
	unsigned int RANSAC_numSamples;
	
	// ISPKF parameters
	double ISPKF_alpha;
	double ISPKF_beta;
	std::vector<int> ISPKF_featureIndices_inlier;
	std::vector<int> ISPKF_featureIndices_inlierBest;
	std::vector<int> ISPKF_featureIndices_usedForEstimation;
	double ISPKF_kappa;
	double ISPKF_lambda;
	unsigned int ISPKF_maxIterations;
	cvm::rmatrix ISPKF_measurements_current;      // 4 * numMatches
	cvm::rmatrix ISPKF_measurements_previous;     // 4 * numMatches
	cvm::rmatrix ISPKF_measurements_currentUsed;  // 4 * numMatchesUsed
	cvm::rmatrix ISPKF_measurements_previousUsed; // 4 * numMatchesUsed
	cvm::rmatrix ISPKF_sigmaPoints; // dimensionState x 2 * dimensionState + 1
	double ISPKF_terminationThreshold;
	cvm::rvector ISPKF_transferedPoints; // 4 * numMatchesUsed
	cvm::rmatrix ISPKF_transferedPointsAll; // 4 * numMatches x 1
	cvm::rmatrix ISPKF_transferedPointsAllSigmaPoints; // 4 * numMatchesUsed x numSigmaPoints
	double ISPKF_wc0;
	double ISPKF_wci;
	double ISPKF_wm0;
	double ISPKF_wmi;
	cvm::rmatrix ISPKF_meanMeasurements;
	cvm::srsmatrix ISPKF_covarianceMeasurements;
	cvm::rmatrix ISPKF_crossCovariance;

	// fixed size temporary variables (computeSigmaPoints)
	cvm::srmatrix temp_computeSigmaPoints_1; // dimensionState x dimensionState
	cvm::srmatrix temp_computeSigmaPoints_2; // dimensionState x dimensionState

	// fixed size temporary variables (innovation)
	cvm::rmatrix temp_innovation_stateLast; // dimensionsState x 1
	cvm::rmatrix temp_innovation_stateIteration; // dimensionState x 1

	// fixed size temporary variables (transferPoints)
	cvm::srmatrix temp_transferPoints_rightCurrent;
	cvm::srmatrix temp_transferPoints_leftCurrent;
	cvm::rmatrix temp_transferPoints_rightPrevious;
	cvm::rvector temp_transferPoints_epipolarLine; // 3 x 1
	cvm::rvector temp_transferPoints_perpendicularLine; // 3 x 1
	cvm::rvector temp_transferPoints_currentPoint_rightPreviousHomogeneous; // 3 x 1
	cvm::rvector temp_transferPoints_currentPoint_leftPreviousHomogeneous; // 3 x 1
	cvm::rvector temp_transferPoints_currentPoint_transferedRight; // 3 x 1
	cvm::rvector temp_transferPoints_currentPoint_transferedLeft; // 3 x 1

	void computeInlier(const cvm::rmatrix &measurementsCurrent, const cvm::rmatrix &transferedPoints);
	void computeMeanMeasurements();
	void computeCovarianceMeasurements();
	void computeCrossCovarianceMeasurements();
	void computeSigmaPoints(const cvm::rmatrix &state, const cvm::srsmatrix &covarianceState);
	bool isMember(const std::vector<int> &indexList, const int index);
	void projectionMatricesFromMotion(const cvm::rmatrix &motionVector, double deltaT, bool copyTransformation);
	cvm::srmatrix skew(const cvm::rmatrix &vector);
	cvm::rmatrix transferPoints(const cvm::rmatrix &measurementsPrevious);
};

#endif // _VISUALODOMETRY_H_
