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

#include "trifocalTensor.h"

using namespace std;

TrifocalTensor::TrifocalTensor(cvm::rmatrix &cameraMatrixA, cvm::rmatrix &cameraMatrixB, cvm::rmatrix &cameraMatixC) {
  
	// create temporary variables
	cvm::rmatrix  temp1(2, 4);
	cvm::srmatrix temp(4);

	// compute trifocal tensor from projection matrices
	for(int i = 0; i < 3; i++) {
    
		// extract relevant parts of projection matrix 1
		if(i == 0) {
			cvm::rmatrix temp1a(cameraMatrixA, 2, 1, 1, 4);
			cvm::rmatrix temp1b(cameraMatrixA, 3, 1, 1, 4);
			temp1.assign(1, 1, temp1a);
			temp1.assign(2, 1, temp1b);
		} else if(i == 1) {
			cvm::rmatrix temp1a(cameraMatrixA, 1, 1, 1, 4);
			cvm::rmatrix temp1b(cameraMatrixA, 3, 1, 1, 4);
			temp1.assign(1, 1, temp1a);
			temp1.assign(2, 1, temp1b);
		} else if(i == 2) {
			cvm::rmatrix temp1a(cameraMatrixA, 1, 1, 1, 4);
			cvm::rmatrix temp1b(cameraMatrixA, 2, 1, 1, 4);
			temp1.assign(1, 1, temp1a);
			temp1.assign(2, 1, temp1b);
		}
    
		for(int q = 0; q < 3; q++) {
      
			// extract relevant parts of projection matrix 2
			cvm::rmatrix temp2(cameraMatrixB, q + 1, 1, 1, 4);
			
			for(int r = 0; r < 3; r++) {
        
				// extract relevant parts of projection matrix 3
				cvm::rmatrix temp3(cameraMatixC, r + 1, 1, 1, 4);

				// define temporary matrix
				temp.assign(1, 1, temp1);
				temp.assign(3, 1, temp2);
				temp.assign(4, 1, temp3);
        
        // compute determinant via eigenvalue decomposition
        // since temp.det() internally throws an exception which
        // might be catched by MATLAB leading to crashes
        cvm::cvector v(4);
        v.eig(temp);
        double det = (v(1)*v(2)*v(3)*v(4)).real();
        
				// set entry of trifocal tensor
				this->tensorData[i][q][r] = pow(-1.0, double(i+1+1)) * det;
			}
		}
	}
}

cvm::rvector TrifocalTensor::pointLinePointTransfer(cvm::rmatrix &featuresA, cvm::rmatrix &featuresB, FundamentalMatrix &fundamentalMatrixRig) {
  
	// get number of features
	int numFeatures = featuresA.msize() / 2;

	// create output data
	cvm::rvector transferedPoints(numFeatures * 2);

	//create temporary variables
	cvm::rvector currentPointA(3);
	cvm::rvector currentPointB(3);
	cvm::rvector transferedPoint(3);
	cvm::rvector epipolarLine(3);
	cvm::rvector perpendicularLine(3);

	currentPointA(3) = 1;
	currentPointB(3) = 1;

	// transfer points into third view
	for(int k = 0; k < numFeatures; k++) {
    
		// reset variables
		transferedPoint.set(0.0);

		// extract current points
		currentPointA(1) = featuresA(2 * k + 1, 1);
		currentPointA(2) = featuresA(2 * k + 2, 1);
		currentPointB(1) = featuresB(2 * k + 1, 1);
		currentPointB(2) = featuresB(2 * k + 2, 1);

		// compute epipolar line in second view
		epipolarLine = fundamentalMatrixRig.computeEpipolarLine(currentPointA, 2);

		// compute perpendicular line
		perpendicularLine(1) =  epipolarLine(2);
		perpendicularLine(2) = -epipolarLine(1);
		perpendicularLine(3) = -currentPointB(1) * epipolarLine(2) + currentPointB(2) * epipolarLine(1);

		// transfer point
		for(int i = 0; i < 3; i++)
			for(int q = 0; q < 3; q++)
				for(int r = 0; r < 3; r++)
					transferedPoint(i + 1) += currentPointA(q + 1) * perpendicularLine(r + 1) * this->tensorData[q][r][i];

		// normalize transfered point
		transferedPoint = transferedPoint / transferedPoint(3);

		// store transfered point in output vector
		transferedPoints(2 * k + 1) = transferedPoint(1);
		transferedPoints(2 * k + 2) = transferedPoint(2);
	}

	// return transfered points
	return transferedPoints;
}

void TrifocalTensor::resetTrifocalTensor(cvm::rmatrix &cameraMatrixA, cvm::rmatrix &cameraMatrixB, cvm::rmatrix &cameraMatixC) {
  
	// create temporary variables
	cvm::rmatrix temp1(2, 4);
	cvm::srmatrix temp(4);

	// compute trifocal tensor from projection matrices
	for(int i = 0; i < 3; i++) {
    
		// extract relevant parts of projection matrix 1
		if(i == 0) {
			cvm::rmatrix temp1a(cameraMatrixA, 2, 1, 1, 4);
			cvm::rmatrix temp1b(cameraMatrixA, 3, 1, 1, 4);
			temp1.assign(1, 1, temp1a);
			temp1.assign(2, 1, temp1b);
		} else if(i == 1) {
			cvm::rmatrix temp1a(cameraMatrixA, 1, 1, 1, 4);
			cvm::rmatrix temp1b(cameraMatrixA, 3, 1, 1, 4);
			temp1.assign(1, 1, temp1a);
			temp1.assign(2, 1, temp1b);
		}	else if(i == 2)	{
			cvm::rmatrix temp1a(cameraMatrixA, 1, 1, 1, 4);
			cvm::rmatrix temp1b(cameraMatrixA, 2, 1, 1, 4);
			temp1.assign(1, 1, temp1a);
			temp1.assign(2, 1, temp1b);
		}

		for(int q = 0; q < 3; q++) {
      
			// extract relevant parts of projection matrix 2
			cvm::rmatrix temp2(cameraMatrixB, q + 1, 1, 1, 4);
			
			for(int r = 0; r < 3; r++) {
        
				// extract relevant parts of projection matrix 3
				cvm::rmatrix temp3(cameraMatixC, r + 1, 1, 1, 4);

				// define temporary matrix
				temp.assign(1, 1, temp1);
				temp.assign(3, 1, temp2);
				temp.assign(4, 1, temp3);
        
        // compute determinant via eigenvalue decomposition
        // since temp.det() internally throws an exception which
        // might be catched by MATLAB leading to crashes
        cvm::cvector v(4);
        v.eig(temp);
        double det = (v(1)*v(2)*v(3)*v(4)).real();

				// set entry of trifocal tensor
				this->tensorData[i][q][r] = pow(-1, double(i + 1 + 1)) * det;
			}
		}
	}
}
