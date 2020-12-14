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

#ifndef _TRIFOCALTENSOR_H_
#define _TRIFOCALTENSOR_H_

#include "cvm.h"
#include "fundamentalMatrix.h"

class TrifocalTensor
{
public: // protected attributes
	double tensorData[3][3][3];

public: // constructors
	TrifocalTensor(cvm::rmatrix &cameraMatrixA, cvm::rmatrix &cameraMatrixB, cvm::rmatrix &cameraMatixC);

public: // public methods
	cvm::rvector pointLinePointTransfer(cvm::rmatrix &featuresA, cvm::rmatrix &featuresB, FundamentalMatrix &fundamentalMatrixRig);
	void resetTrifocalTensor(cvm::rmatrix &cameraMatrixA, cvm::rmatrix &cameraMatrixB, cvm::rmatrix &cameraMatixC);
};

#endif // _TRIFOCALTENSOR_H_