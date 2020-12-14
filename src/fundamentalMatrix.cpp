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

#include "fundamentalMatrix.h"

FundamentalMatrix::FundamentalMatrix(cvm::srmatrix &intrinsicsA, cvm::srmatrix &intrinsicsB, cvm::rmatrix &extrinsicTranslation, cvm::srmatrix &extrinsicRotation) {
  
	//create data storage
	this->matrixData.resize(3);

	// compute essential matrix
	cvm::srmatrix essentialMatrix = skew(extrinsicTranslation) * extrinsicRotation;

	// compute fundamental matrix
	this->matrixData = intrinsicsB * essentialMatrix * ~intrinsicsA;
}

cvm::rvector FundamentalMatrix::computeEpipolarLine(cvm::rvector &point, int viewID) {
  
	// create output data
	cvm::rvector line(3);

	// compute epipolar line
	if(viewID == 1)
		line.mult(this->matrixData, point);
	else if(viewID == 2)
		line.mult(~this->matrixData, point);
	else
		std::cout << "Invalid selection of viewID!" << std::endl;

	// return epipolar line
	return line;
}

cvm::srmatrix FundamentalMatrix::getMatrixData() {
  
	// return matrix data
	return this->matrixData;
}

cvm::srmatrix FundamentalMatrix::skew(cvm::rmatrix &vector) {
  
	// create output data
	cvm::srmatrix skewMatrix(3);

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
