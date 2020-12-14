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

#include "visualOdometry.h"

using namespace std;

VisualOdometry::VisualOdometry(cvm::rmatrix initialState, cvm::srsmatrix initialCovarianceState, cvm::srsmatrix covarianceSystem, cvm::srmatrix intrinsicCalibration_right, cvm::srmatrix intrinsicCalibration_left, cvm::srmatrix extrinsicRotation, cvm::rmatrix extrinsicTranslation,
                               int param_ransac_samples,double param_ransac_inlier_distance,int param_ispkf_max_iterations,double param_ispkf_threshold) {
  
	// get state dimension
	this->dimensionState = initialState.msize();

	// set parameters for estimation process
	this->covarianceState.resize(this->dimensionState);
	this->covarianceSystem.resize(this->dimensionState);
	this->state.resize(this->dimensionState, 1);

	this->covarianceState = initialCovarianceState;
	this->covarianceSystem = covarianceSystem;
	this->state = initialState;

	// initialize and set parameters of stereo camera rig
	this->CAMERARIG_extrinsicRotation.resize(3);
	this->CAMERARIG_extrinsicTranslation.resize(3, 1);
	this->CAMERARIG_intrinsicCalibration_right.resize(3);
	this->CAMERARIG_intrinsicCalibration_left.resize(3);
	this->CAMERARIG_essentialMatrix.resize(3);
	this->CAMERARIG_fundamentalMatrix.resize(3);

	this->CAMERARIG_extrinsicRotation          = extrinsicRotation;
	this->CAMERARIG_extrinsicTranslation       = extrinsicTranslation;
	this->CAMERARIG_intrinsicCalibration_right = intrinsicCalibration_right;
	this->CAMERARIG_intrinsicCalibration_left  = intrinsicCalibration_left;
	this->CAMERARIG_essentialMatrix            = skew(this->CAMERARIG_extrinsicTranslation) * this->CAMERARIG_extrinsicRotation;
	this->CAMERARIG_fundamentalMatrix          = this->CAMERARIG_intrinsicCalibration_left * this->CAMERARIG_essentialMatrix * ~this->CAMERARIG_intrinsicCalibration_right;

	// initialize and set projection matrices
	this->CAM_projectionMatrixRightPrevious.resize(3, 4);
	this->CAM_projectionMatrixLeftPrevious.resize(3, 4);
	this->CAM_projectionMatrixRightCurrent.resize(3, 4);
	this->CAM_projectionMatrixLeftCurrent.resize(3, 4);
  this->CAM_transformationMatrixLeft.resize(3, 4);

	cvm::rmatrix temp_rightPrevious_rotation   (this->CAM_projectionMatrixRightPrevious, 1, 1, 3, 3);
	cvm::rmatrix temp_rightPrevious_translation(this->CAM_projectionMatrixRightPrevious, 1, 4, 3, 1);
	cvm::rmatrix temp_leftPrevious_rotation    (this->CAM_projectionMatrixLeftPrevious, 1, 1, 3, 3);
	cvm::rmatrix temp_leftPrevious_translation (this->CAM_projectionMatrixLeftPrevious, 1, 4, 3, 1);

  temp_leftPrevious_rotation     = cvm::eye_real(3);
	temp_rightPrevious_rotation    = this->CAMERARIG_extrinsicRotation;
  temp_rightPrevious_translation = this->CAMERARIG_extrinsicTranslation;

	this->CAM_projectionMatrixRightPrevious = this->CAMERARIG_intrinsicCalibration_right * this->CAM_projectionMatrixRightPrevious;
	this->CAM_projectionMatrixLeftPrevious  = this->CAMERARIG_intrinsicCalibration_left  * this->CAM_projectionMatrixLeftPrevious;

  // set default RANSAC parameters
	this->RANSAC_featuresPerSample          = 3;
	this->RANSAC_numSamples                 = param_ransac_samples;
	this->RANSAC_maxEuclideanDistanceInlier = param_ransac_inlier_distance;

	// set default ISPKF parameters
	this->ISPKF_alpha  = 0.0001;
	this->ISPKF_beta   = 2.0;
	this->ISPKF_kappa  = 0.0;
	this->ISPKF_lambda = this->ISPKF_alpha * this->ISPKF_alpha * (this->dimensionState + this->ISPKF_kappa) - this->dimensionState;
	this->ISPKF_maxIterations = param_ispkf_max_iterations;
	this->ISPKF_sigmaPoints.resize(this->dimensionState, 2 * this->dimensionState + 1);
	this->ISPKF_terminationThreshold = param_ispkf_threshold;
	this->ISPKF_wc0 = this->ISPKF_lambda / (this->dimensionState + this->ISPKF_lambda) + (1 - this->ISPKF_alpha * this->ISPKF_alpha + this->ISPKF_beta);
	this->ISPKF_wci = 1 / (2 * (this->dimensionState + this->ISPKF_lambda));
	this->ISPKF_wm0 = this->ISPKF_lambda / (this->dimensionState + this->ISPKF_lambda);
	this->ISPKF_wmi = 1 / (2 * (this->dimensionState + this->ISPKF_lambda));

	// initialize temporary fixed size variables (computeSigmaPoints)
	this->temp_computeSigmaPoints_1.resize(this->dimensionState);
	this->temp_computeSigmaPoints_2.resize(this->dimensionState);

	// initialize temporary fixed size variables (innovation)
	this->temp_innovation_stateLast.resize(this->dimensionState, 1);
	this->temp_innovation_stateIteration.resize(this->dimensionState, 1);

	// initialize temporary fixed size variables (transferPoints)
	this->temp_transferPoints_rightCurrent.resize(4);
	this->temp_transferPoints_leftCurrent.resize(4);
	this->temp_transferPoints_rightPrevious.resize(2, 4);
	this->temp_transferPoints_epipolarLine.resize(3);
	this->temp_transferPoints_perpendicularLine.resize(3);
	this->temp_transferPoints_currentPoint_rightPreviousHomogeneous.resize(3);
	this->temp_transferPoints_currentPoint_leftPreviousHomogeneous.resize(3);
	this->temp_transferPoints_currentPoint_transferedRight.resize(3);
	this->temp_transferPoints_currentPoint_transferedLeft.resize(3);
}

void VisualOdometry::innovation(const cvm::rmatrix &matchesLeftPrevious, const cvm::rmatrix &matchesRightPrevious, const cvm::rmatrix &matchesLeftCurrent, const cvm::rmatrix &matchesRightCurrent, double deltaT, double varianceMeasurements) {

  // get number of matches in current frame
	unsigned int numMatches = matchesRightPrevious.msize();
  
  if (numMatches<6) {
    cout << "WARNING: At least 6 matches needed for estimating egomotion. Skipping this frame." << endl;
    return;
  }
  
  // clear list with best inliers
  ISPKF_featureIndices_inlierBest.clear();

	// define auxialiary variables
	bool ISPKF_exitIteration;
	unsigned int ISPKF_iterationCounter = 0;
	unsigned int RANSAC_currentIndex;
	unsigned int RANSAC_featureCounter = 0;
	unsigned int RANSAC_sampleCounter = 0;

	// resize temporary variables
	this->ISPKF_measurements_current.resize(4 * numMatches, 1);
	this->ISPKF_measurements_previous.resize(4 * numMatches, 1);
	this->ISPKF_measurements_currentUsed.resize(4 * this->RANSAC_featuresPerSample, 1);
	this->ISPKF_measurements_previousUsed.resize(4 * this->RANSAC_featuresPerSample, 1);
	this->ISPKF_transferedPoints.resize(4 * this->RANSAC_featuresPerSample);
	this->ISPKF_transferedPointsAll.resize(4 * numMatches, 1);
	this->ISPKF_transferedPointsAllSigmaPoints.resize(4 * this->RANSAC_featuresPerSample, 2 * this->dimensionState + 1);
	this->ISPKF_meanMeasurements.resize(4 * this->RANSAC_featuresPerSample, 1);
	this->ISPKF_covarianceMeasurements.resize(4 * this->RANSAC_featuresPerSample);
	this->ISPKF_crossCovariance.resize(this->dimensionState, 4 * this->RANSAC_featuresPerSample);

	// store measurements in current images
	for(unsigned int i = 0; i < numMatches; i++) {
    
		// previous image
		this->ISPKF_measurements_previous(2 * i + 1, 1) = matchesRightPrevious(i+1, 1);
		this->ISPKF_measurements_previous(2 * i + 2, 1) = matchesRightPrevious(i+1, 2);
		this->ISPKF_measurements_previous(2 * i + 1 + 2 * numMatches, 1) = matchesLeftPrevious(i+1, 1);
		this->ISPKF_measurements_previous(2 * i + 2 + 2 * numMatches, 1) = matchesLeftPrevious(i+1, 2);

		// current image
		this->ISPKF_measurements_current(2 * i + 1, 1) = matchesRightCurrent(i+1, 1);
		this->ISPKF_measurements_current(2 * i + 2, 1) = matchesRightCurrent(i+1, 2);
		this->ISPKF_measurements_current(2 * i + 1 + 2 * numMatches, 1) = matchesLeftCurrent(i+1, 1);
		this->ISPKF_measurements_current(2 * i + 2 + 2 * numMatches, 1) = matchesLeftCurrent(i+1, 2);
	}

	// iterative egomotion estimation
	while(RANSAC_sampleCounter < this->RANSAC_numSamples + 1) { // "+1" is the estimation with all inliers
    
		// reset variables
		ISPKF_exitIteration = false;
		ISPKF_iterationCounter = 0;

		// randomly chose indices of matches used for RANSAC or define used indices based on the best subset
		if(RANSAC_sampleCounter < this->RANSAC_numSamples) { // egomotion estimation based on randomly chosen matches

      // reset variables
			RANSAC_featureCounter = 0;
			this->ISPKF_featureIndices_usedForEstimation.clear();

			// select features used for RANSAC
			while(RANSAC_featureCounter < this->RANSAC_featuresPerSample) {
        
				// randomly chose and store feature index
				RANSAC_currentIndex = rand() % numMatches;

				if(!isMember(this->ISPKF_featureIndices_usedForEstimation, RANSAC_currentIndex)) {
          
					// store current index
					this->ISPKF_featureIndices_usedForEstimation.push_back(RANSAC_currentIndex);

					// increase feature counter
					RANSAC_featureCounter++;
				}
			}
      
		}	else { // egomotion estimation based on best subset
      
			// select features based on best inlier set
			this->ISPKF_featureIndices_usedForEstimation = this->ISPKF_featureIndices_inlierBest;

			// resize variables
			this->ISPKF_measurements_currentUsed.resize(4 * static_cast<int>(this->ISPKF_featureIndices_usedForEstimation.size()), 1);
			this->ISPKF_measurements_previousUsed.resize(4 * static_cast<int>(this->ISPKF_featureIndices_usedForEstimation.size()), 1);
			//this->ISPKF_transferedPointsUsed.resize(4 * static_cast<int>(this->ISPKF_featureIndices_usedForEstimation.size()));
			this->ISPKF_transferedPointsAllSigmaPoints.resize(4 * static_cast<int>(this->ISPKF_featureIndices_usedForEstimation.size()), 2 * this->dimensionState + 1);
			this->ISPKF_meanMeasurements.resize(4 * static_cast<int>(this->ISPKF_featureIndices_usedForEstimation.size()), 1);
			this->ISPKF_covarianceMeasurements.resize(4 * static_cast<int>(this->ISPKF_featureIndices_usedForEstimation.size()));
			this->ISPKF_crossCovariance.resize(this->dimensionState, 4 * static_cast<int>(this->ISPKF_featureIndices_usedForEstimation.size()));
		}

		// create measurement vectors (right/left previous/current) based on previously defined indices (either current subset or best subset)
		unsigned int numUsedFeatures = static_cast<int>(this->ISPKF_featureIndices_usedForEstimation.size());

		for(unsigned int i = 0; i < numUsedFeatures; i++) {
      
			// previous images
			this->ISPKF_measurements_previousUsed(2 * i + 1, 1) = matchesRightPrevious(this->ISPKF_featureIndices_usedForEstimation[i]+1, 1);
			this->ISPKF_measurements_previousUsed(2 * i + 2, 1) = matchesRightPrevious(this->ISPKF_featureIndices_usedForEstimation[i]+1, 2);
			this->ISPKF_measurements_previousUsed(2 * i + 1 + 2 * numUsedFeatures, 1) = matchesLeftPrevious(this->ISPKF_featureIndices_usedForEstimation[i]+1, 1);
			this->ISPKF_measurements_previousUsed(2 * i + 2 + 2 * numUsedFeatures, 1) = matchesLeftPrevious(this->ISPKF_featureIndices_usedForEstimation[i]+1, 2);

			// current images
			this->ISPKF_measurements_currentUsed(2 * i + 1, 1) = matchesRightCurrent(this->ISPKF_featureIndices_usedForEstimation[i]+1, 1);
			this->ISPKF_measurements_currentUsed(2 * i + 2, 1) = matchesRightCurrent(this->ISPKF_featureIndices_usedForEstimation[i]+1, 2);
			this->ISPKF_measurements_currentUsed(2 * i + 1 + 2 * numUsedFeatures, 1) = matchesLeftCurrent(this->ISPKF_featureIndices_usedForEstimation[i] +1, 1);
			this->ISPKF_measurements_currentUsed(2 * i + 2 + 2 * numUsedFeatures, 1) = matchesLeftCurrent(this->ISPKF_featureIndices_usedForEstimation[i] +1, 2);
		}

		// store state before iteration
		this->temp_innovation_stateIteration = this->state;

		// perform iteration
		while(!ISPKF_exitIteration) {
      
			// store previous state
			this->temp_innovation_stateLast = this->temp_innovation_stateIteration;

			// reset variables
			this->ISPKF_featureIndices_inlier.clear();
			this->ISPKF_sigmaPoints.set(0.0);

			// compute sigma points from current (iterated) state
			computeSigmaPoints(this->temp_innovation_stateIteration, this->covarianceState);

			// reset transformed points
			this->ISPKF_transferedPointsAllSigmaPoints.set(0.0);

			// compute predicted measurements for each sigma point
			for(unsigned int i = 0; i < 2 * this->dimensionState + 1; i++) {
        
				// get current sigma point from list
				cvm::rmatrix currentSigmaPoint(this->ISPKF_sigmaPoints, 1, i + 1, this->dimensionState, 1);

				// compute projection matrices of current cameras based on egomotion (i.e. current sigma point)
				projectionMatricesFromMotion(currentSigmaPoint,deltaT,false);

        if (numUsedFeatures>0) {

  				// get storage for transfered points for current sigma point
				  cvm::rmatrix transferedPointsForCurrentSigmaPoint(this->ISPKF_transferedPointsAllSigmaPoints, 1, i + 1, 4 * numUsedFeatures, 1);

				  // transfer matches from previous images into new images (using point-line-point transfer)
				  transferedPointsForCurrentSigmaPoint = transferPoints(this->ISPKF_measurements_previousUsed);
        }
			}

			// compute mean and covariance of transfered points
			computeMeanMeasurements();
			computeCovarianceMeasurements();
      
			// compute cross-covariance
			computeCrossCovarianceMeasurements();

			// get length of measurement vector
			unsigned int lengthMeasurements = this->ISPKF_meanMeasurements.size();

      if (lengthMeasurements==0) return;

			// compute iterated state
			cvm::rmatrix tempMatrixUpdate1(this->dimensionState, 1);
			cvm::rmatrix tempMatrixUpdate2(this->dimensionState, 1);
			cvm::rmatrix tempMatrixUpdate3(lengthMeasurements, 1);
			cvm::rmatrix tempMatrixUpdate4(lengthMeasurements, 1);
			cvm::rmatrix tempMatrixUpdate5(lengthMeasurements, 1);
			cvm::rmatrix tempMatrixUpdate6(this->dimensionState, 1);

			cvm::rmatrix transferedPointsMean(this->ISPKF_transferedPointsAllSigmaPoints, 1, 1, lengthMeasurements, 1);
      cvm::srsmatrix covarianceMeasurementsUsed(varianceMeasurements * cvm::eye_real(lengthMeasurements));
			this->ISPKF_covarianceMeasurements = this->ISPKF_covarianceMeasurements + covarianceMeasurementsUsed;

			tempMatrixUpdate1 = this->state - this->temp_innovation_stateIteration;
			tempMatrixUpdate2 = this->covarianceState.inv() * tempMatrixUpdate1;
			tempMatrixUpdate3 = ~this->ISPKF_crossCovariance * tempMatrixUpdate2;
			tempMatrixUpdate4 = this->ISPKF_measurements_currentUsed - transferedPointsMean - tempMatrixUpdate3;
			tempMatrixUpdate5 = this->ISPKF_covarianceMeasurements.inv() * tempMatrixUpdate4;
			tempMatrixUpdate6 = this->ISPKF_crossCovariance * tempMatrixUpdate5;

			this->temp_innovation_stateIteration = this->state + tempMatrixUpdate6;

			// check if at least one termination criterion is fulfilled:
      // state change between two iterations smaller than predefined threshold
			if((abs(this->temp_innovation_stateIteration(1, 1) - this->temp_innovation_stateLast(1, 1)) < this->ISPKF_terminationThreshold) &&
         (abs(this->temp_innovation_stateIteration(2, 1) - this->temp_innovation_stateLast(2, 1)) < this->ISPKF_terminationThreshold) &&
         (abs(this->temp_innovation_stateIteration(3, 1) - this->temp_innovation_stateLast(3, 1)) < this->ISPKF_terminationThreshold) &&
         (abs(this->temp_innovation_stateIteration(4, 1) - this->temp_innovation_stateLast(4, 1)) < this->ISPKF_terminationThreshold) &&
         (abs(this->temp_innovation_stateIteration(5, 1) - this->temp_innovation_stateLast(5, 1)) < this->ISPKF_terminationThreshold) &&
         (abs(this->temp_innovation_stateIteration(6, 1) - this->temp_innovation_stateLast(6, 1)) < this->ISPKF_terminationThreshold))
				ISPKF_exitIteration = true;
      
      // maximum iterations reached
			if(ISPKF_iterationCounter == this->ISPKF_maxIterations)
				ISPKF_exitIteration = true;

			// increase iteration counter
			ISPKF_iterationCounter++;

			// compute reprojection error for all measurements (only if RANSAC is active and iteration has finished)
			if((RANSAC_sampleCounter < this->RANSAC_numSamples) && (ISPKF_exitIteration)) {

				// compute projection matrices for iterated egomotion estimation
				projectionMatricesFromMotion(this->temp_innovation_stateIteration,deltaT,false);

				// transfer all measurements into current images using last iterated egomotion estimation
				this->ISPKF_transferedPointsAll = transferPoints(this->ISPKF_measurements_previous);
				
				// compute inliers
				computeInlier(this->ISPKF_measurements_current, this->ISPKF_transferedPointsAll);

				// get number of inlier in current inlier set
				unsigned int numInlierCurrent = static_cast<unsigned int>(ISPKF_featureIndices_inlier.size());

				// check and store inlier
				if(numInlierCurrent > this->ISPKF_featureIndices_inlierBest.size())
					this->ISPKF_featureIndices_inlierBest = this->ISPKF_featureIndices_inlier;
			}
		}

		// increase sample counter
		RANSAC_sampleCounter++;
	}

	// get length of measurement vector
	unsigned int lengthMeasurements = this->ISPKF_meanMeasurements.size();

	// update state and set transformation (equals state at the end of the final iteration process)
	this->state = this->temp_innovation_stateIteration;
  projectionMatricesFromMotion(this->state,deltaT,true);

	// update covariance after update
	cvm::srmatrix  tempMatrixUpdateCovariance1(lengthMeasurements);
	cvm::srsmatrix covarianceMeasurementsUsed(varianceMeasurements * cvm::eye_real(lengthMeasurements));
	cvm::srmatrix  covarianceStateTemp(6);

	tempMatrixUpdateCovariance1 = this->ISPKF_covarianceMeasurements + covarianceMeasurementsUsed;
	cvm::srmatrix temp = this->ISPKF_crossCovariance * tempMatrixUpdateCovariance1.inv() * ~this->ISPKF_crossCovariance;

	for(unsigned int i = 0; i < this->dimensionState; i++)
		for(unsigned int j = i; j < this->dimensionState; j++)
			this->covarianceState.set(i + 1, j + 1, this->covarianceState(i + 1, j + 1) - temp(i + 1, j + 1));
}

void VisualOdometry::prediction() {
  
	// predict state (constant velocity)
	this->state = this->state;

	// predict covariance of the state
	this->covarianceState = this->covarianceState + this->covarianceSystem;
}

void VisualOdometry::computeInlier(const cvm::rmatrix &measurementsCurrent, const cvm::rmatrix &transferedPoints) {
  
	// get number of features
	unsigned int numFeatures = measurementsCurrent.msize() / 4;

	// compute euclidean distance (int both current images)
	double euclideanDistanceRight;
	double euclideanDistanceLeft;
	double distanceWidthRight;
	double distanceHeightRight;
	double distanceWidthLeft;
	double distanceHeightLeft;

	for(unsigned int i = 0; i < numFeatures; i++)	{
    
		// compute distances in each direction
		distanceWidthRight = transferedPoints(2 * i + 1, 1) - measurementsCurrent(2 * i + 1, 1);
		distanceHeightRight = transferedPoints(2 * i + 2, 1) - measurementsCurrent(2 * i + 2, 1);
		distanceWidthLeft = transferedPoints(2 * i + 1 + 2 * numFeatures, 1) - measurementsCurrent(2 * i + 1 + 2 * numFeatures, 1);
		distanceHeightLeft = transferedPoints(2 * i + 2 + 2 * numFeatures, 1) - measurementsCurrent(2 * i + 2 + 2 * numFeatures, 1);

		// compute euclidean distance in current right image
		euclideanDistanceRight = sqrt(distanceWidthRight * distanceWidthRight + distanceHeightRight * distanceHeightRight);
		
		// compute euclidean distance in current left image
		euclideanDistanceLeft = sqrt(distanceWidthLeft * distanceWidthLeft + distanceHeightLeft * distanceHeightLeft);

		// check and store current index (if inlier)
		if((euclideanDistanceRight <= this->RANSAC_maxEuclideanDistanceInlier) && (euclideanDistanceLeft <= this->RANSAC_maxEuclideanDistanceInlier))
			this->ISPKF_featureIndices_inlier.push_back(i);
	}
}

void VisualOdometry::computeMeanMeasurements() {
  
	// reset mean
	this->ISPKF_meanMeasurements.set(0.0);

	// get length of transfered points
	unsigned int lengthTransferedPoints = this->ISPKF_transferedPointsAllSigmaPoints.msize();

  if (lengthTransferedPoints==0) return;

	// compute mean
	for(unsigned int i = 0; i < 2 * this->dimensionState + 1; i++) {
		// get transfered points for current sigma point
		cvm::rmatrix transferedPointsCurrent(this->ISPKF_transferedPointsAllSigmaPoints, 1, i + 1, lengthTransferedPoints, 1);

		if(i == 0)
			this->ISPKF_meanMeasurements = this->ISPKF_meanMeasurements + this->ISPKF_wm0 * transferedPointsCurrent;
		else
			this->ISPKF_meanMeasurements = this->ISPKF_meanMeasurements + this->ISPKF_wmi * transferedPointsCurrent;
	}
}

void VisualOdometry::computeCovarianceMeasurements() {
  
	// reset covariance
	this->ISPKF_covarianceMeasurements.set(0.0);

	// get length of trasfered points
	unsigned int lengthTransferedPoints = this->ISPKF_transferedPointsAllSigmaPoints.msize();

  if (lengthTransferedPoints==0) return;

	// compute covariance
	for(unsigned int i = 0; i < 2 * this->dimensionState + 1; i++) {
    
		// extract transfered points for current sigma point
		cvm::rmatrix transferedPointsCurrent(this->ISPKF_transferedPointsAllSigmaPoints, 1, i + 1, lengthTransferedPoints, 1);

		// compute current difference between current transfered points and mean of transfered points
		cvm::rmatrix difference(lengthTransferedPoints, 1);
		difference = transferedPointsCurrent - this->ISPKF_meanMeasurements;

		// compute covariance for current difference
		cvm::srsmatrix currentCovarianceMatrix(lengthTransferedPoints);

    // rank 1 update
    currentCovarianceMatrix.syrk(1.0,difference(1),0.0);

		// compute covariance matrix
		if(i == 0)
			this->ISPKF_covarianceMeasurements = this->ISPKF_covarianceMeasurements + this->ISPKF_wc0 * currentCovarianceMatrix;
		else
			this->ISPKF_covarianceMeasurements = this->ISPKF_covarianceMeasurements + this->ISPKF_wci * currentCovarianceMatrix;
	}
}

void VisualOdometry::computeCrossCovarianceMeasurements() {
	// reset cross-coveariance
	this->ISPKF_crossCovariance.set(0.0);

	// get length of trasfered points
	unsigned int lengthTransferedPoints = this->ISPKF_transferedPointsAllSigmaPoints.msize();

  if (lengthTransferedPoints==0) return;

	// compute cross-covariance matrix
	for(unsigned int i = 0; i < 2 * this->dimensionState + 1; i++) {
		// extract current sigma point
		cvm::rmatrix currentSigmaPoint(this->ISPKF_sigmaPoints, 1, i + 1, this->dimensionState, 1);

		// compute difference between current sigma point and predicted state
		cvm::rmatrix differenceState(this->dimensionState, 1);
		differenceState = currentSigmaPoint - this->temp_innovation_stateIteration;

		// extract transfered points corresponding to current sigma point
		cvm::rmatrix currentTransferedPoints(this->ISPKF_transferedPointsAllSigmaPoints, 1, i + 1, lengthTransferedPoints, 1);

		// compute difference between transfered points for current sigma point and mean of transfered points
		cvm::rmatrix differenceTransferedPoints(lengthTransferedPoints, 1);
		differenceTransferedPoints = currentTransferedPoints - this->ISPKF_meanMeasurements;

		// compute cross-covariance matrix
		if(i == 0)
			this->ISPKF_crossCovariance = this->ISPKF_crossCovariance + this->ISPKF_wc0 * differenceState * ~differenceTransferedPoints;
		else
			this->ISPKF_crossCovariance = this->ISPKF_crossCovariance + this->ISPKF_wci * differenceState * ~differenceTransferedPoints;
	}

}

void VisualOdometry::computeSigmaPoints(const cvm::rmatrix &state, const cvm::srsmatrix &covarianceState) {
  
	// compute sigma points
	this->temp_computeSigmaPoints_1 = sqrt(this->dimensionState + this->ISPKF_lambda) * covarianceState.cholesky().transpose();

	for(unsigned int i = 0; i < this->dimensionState; i++)
		for(unsigned int j = 0; j < this->dimensionState; j++)
			this->temp_computeSigmaPoints_2(i + 1, j + 1) = state(i + 1, 1);

	for(unsigned int i = 0; i < this->dimensionState; i++) {
		for(unsigned int j = 0; j < 2 * this->dimensionState + 1; j++) {
			if(j == 0)
				this->ISPKF_sigmaPoints(i + 1, j + 1) = state(i + 1, 1);
			else if((j > 0) && (j <= this->dimensionState))
				this->ISPKF_sigmaPoints(i + 1, j + 1) = this->temp_computeSigmaPoints_2(i + 1, j) + this->temp_computeSigmaPoints_1(i + 1, j);
			else if((j > this->dimensionState) && (j <= 2 * this->dimensionState))
				this->ISPKF_sigmaPoints(i + 1, j + 1) = this->temp_computeSigmaPoints_2(i + 1, j - this->dimensionState) - this->temp_computeSigmaPoints_1(i + 1, j - this->dimensionState);
		}
	}
}

bool VisualOdometry::isMember(const std::vector<int> &indexList, const int index) {
  
	// check if "index" is member of "indexList"
	for(unsigned int i = 0; i < indexList.size(); i++) 
		if(indexList[i] == index)
			return true;

	return false;
}

void VisualOdometry::projectionMatricesFromMotion(const cvm::rmatrix &motionVector, double deltaT,bool copyTransformation) {
  
	// compute parameters from motion
	double deltaX = motionVector(1, 1) * deltaT;
	double deltaY = motionVector(2, 1) * deltaT;
	double deltaZ = motionVector(3, 1) * deltaT;
	double angleX = motionVector(4, 1) * deltaT;
	double angleY = motionVector(5, 1) * deltaT;
	double angleZ = motionVector(6, 1) * deltaT;
  
	// compute rotation matrix from motion
	double cX = cos(angleX);
	double cY = cos(angleY);
	double cZ = cos(angleZ);
	double sX = sin(angleX);
	double sY = sin(angleY);
	double sZ = sin(angleZ);

	double Rx_data[] = {1, 0, 0, 0, cX, sX, 0, -sX, cX};
	double Ry_data[] = {cY, 0, -sY, 0, 1, 0, sY, 0, cY};
	double Rz_data[] = {cZ, sZ, 0, -sZ, cZ, 0, 0, 0, 1};

	cvm::rmatrix Rx(Rx_data, 3, 3);
	cvm::rmatrix Ry(Ry_data, 3, 3);
	cvm::rmatrix Rz(Rz_data, 3, 3);

	cvm::rmatrix rotationFromMotion(3, 3);
	rotationFromMotion = Rz * Rx * Ry;
  
	// compute translation vector from motion
	cvm::rmatrix translationFromMotion(3, 1);
	translationFromMotion(1, 1) = deltaX;
	translationFromMotion(2, 1) = deltaY;
	translationFromMotion(3, 1) = deltaZ;

	// allocate submatrices
	cvm::rmatrix temp_rightCurrent_rotation   (this->CAM_projectionMatrixRightCurrent, 1, 1, 3, 3);
	cvm::rmatrix temp_rightCurrent_translation(this->CAM_projectionMatrixRightCurrent, 1, 4, 3, 1);
	cvm::rmatrix temp_leftCurrent_rotation    (this->CAM_projectionMatrixLeftCurrent,  1, 1, 3, 3);
	cvm::rmatrix temp_leftCurrent_translation (this->CAM_projectionMatrixLeftCurrent,  1, 4, 3, 1);

  // projects points from prev right to current right
  temp_rightCurrent_rotation    = this->CAMERARIG_extrinsicRotation * rotationFromMotion;
  temp_rightCurrent_translation = this->CAMERARIG_extrinsicRotation * translationFromMotion + this->CAMERARIG_extrinsicTranslation;
  
  // projects points from prev right to current left
  temp_leftCurrent_rotation    = rotationFromMotion;
  temp_leftCurrent_translation = translationFromMotion;
  
  // copy to private variables
  if (copyTransformation)
    this->CAM_transformationMatrixLeft = this->CAM_projectionMatrixLeftCurrent;

  // compute projection matrices of current stereo camera rig
	this->CAM_projectionMatrixRightCurrent = this->CAMERARIG_intrinsicCalibration_right * this->CAM_projectionMatrixRightCurrent;
	this->CAM_projectionMatrixLeftCurrent  = this->CAMERARIG_intrinsicCalibration_left * this->CAM_projectionMatrixLeftCurrent;
}

cvm::rmatrix VisualOdometry::transferPoints(const cvm::rmatrix &measurementsPrevious) {
  
  // get number of feature correspondences
	unsigned int numMatches = measurementsPrevious.size() / 4;
  
  // create output vector
	cvm::rmatrix transferedPoints(4 * numMatches, 1);
  
	// compute trifocal tensors from camera matrices
	TrifocalTensor T_right(this->CAM_projectionMatrixRightPrevious, this->CAM_projectionMatrixLeftPrevious, this->CAM_projectionMatrixRightCurrent);
	TrifocalTensor T_left(this->CAM_projectionMatrixRightPrevious, this->CAM_projectionMatrixLeftPrevious, this->CAM_projectionMatrixLeftCurrent);

	// copy feature correspondences
	cvm::rmatrix featRP(2 * numMatches, 1);
	cvm::rmatrix featLP(2 * numMatches, 1);
	for(unsigned int i = 0; i < 2 * numMatches; i++) {
		featRP(i + 1, 1) = measurementsPrevious(i + 1, 1);
		featLP(i + 1, 1) = measurementsPrevious(i + 1 + 2 * numMatches, 1);
	}

	// create fundamental matrix of stereo camera rig (assumed to be constant)
	FundamentalMatrix F_rig(this->CAMERARIG_intrinsicCalibration_left, this->CAMERARIG_intrinsicCalibration_right,
                          this->CAMERARIG_extrinsicTranslation, this->CAMERARIG_extrinsicRotation);

  // point-line-point transfer
	cvm::rvector transRight = T_right.pointLinePointTransfer(featRP, featLP, F_rig);
	cvm::rvector transLeft  = T_left.pointLinePointTransfer(featRP, featLP, F_rig);
  
	// copy transfered points
	for(unsigned int i = 0; i < 2 * numMatches; i++) {
		transferedPoints(i + 1, 1) = transRight(i + 1);
		transferedPoints(i + 1 + 2 * numMatches, 1) = transLeft(i + 1);
	}

	return transferedPoints;
}

cvm::srmatrix VisualOdometry::skew(const cvm::rmatrix &vector) {
  
	// create output matrix
	cvm::srmatrix skewMatrix(3);

	// check dimension of input vector
	if(vector.msize() != 3) {
		std::cout << "Input vector has to be a 3-vector!" << std::endl;
		return skewMatrix;
	}

	// convert vector into skew-symmetric matrix
	skewMatrix(1, 2) = -vector(3, 1);
	skewMatrix(1, 3) = +vector(2, 1);
	skewMatrix(2, 1) = +vector(3, 1);
	skewMatrix(2, 3) = -vector(1, 1);
	skewMatrix(3, 1) = -vector(2, 1);
	skewMatrix(3, 2) = +vector(1, 1);

	// return skew-symmetric matrix
	return skewMatrix;
}
