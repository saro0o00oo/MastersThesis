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

#ifndef _FUNDAMENTALMATRIX_H_
#define _FUNDAMENTALMATRIX_H_

#include "cvm.h"

class FundamentalMatrix
{
protected: // protected attributes
	cvm::srmatrix matrixData;

public: // constructors
	FundamentalMatrix(cvm::srmatrix &intrinsicsA, cvm::srmatrix &intrinsicsB, cvm::rmatrix &extrinsicTranslation, cvm::srmatrix &extrinsicRotation);

public: // public methods
	cvm::rvector computeEpipolarLine(cvm::rvector &point, int viewID);
	cvm::srmatrix getMatrixData();

protected: // protected methods
	cvm::srmatrix skew(cvm::rmatrix &vector);
};

#endif // _FUNDAMENTALMATRIX_H_