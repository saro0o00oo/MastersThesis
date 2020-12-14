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

#include "mex.h"
#include <iostream>
#include <string.h>
#include "visualOdometry.h"

using namespace std;

static VisualOdometry *VO;

void mexFunction (int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {

  // read command
  char command[128];
  mxGetString(prhs[0],command,128);

  // init
  if (!strcmp(command,"init")) {
    
    // check arguments
    if(nrhs!=1+6)
      mexErrMsgTxt("6 parameters required (initialState,initialCovState,covSystem,intrinsicCalibration,extrinsicRotation,extrinsicTranslation).");
    if(!mxIsDouble(prhs[1]) || mxGetM(prhs[1])!=6 || mxGetN(prhs[1])!=1)
      mexErrMsgTxt("Input initialState must be a 6x1 double vector.");
    if(!mxIsDouble(prhs[2]) || mxGetM(prhs[2])!=6 || mxGetN(prhs[2])!=6)
      mexErrMsgTxt("Input initialCovState must be a 6x6 double matrix.");
    if(!mxIsDouble(prhs[3]) || mxGetM(prhs[3])!=6 || mxGetN(prhs[3])!=6)
      mexErrMsgTxt("Input covSystem must be a 6x6 double matrix.");
    if(!mxIsDouble(prhs[4]) || mxGetM(prhs[4])!=3 || mxGetN(prhs[4])!=3)
      mexErrMsgTxt("Input intrinsicCalibration must be a 3x3 double matrix.");
    if(!mxIsDouble(prhs[5]) || mxGetM(prhs[5])!=3 || mxGetN(prhs[5])!=3)
      mexErrMsgTxt("Input extrinsicRotation must be a 3x3 double matrix.");
    if(!mxIsDouble(prhs[6]) || mxGetM(prhs[6])!=3 || mxGetN(prhs[6])!=1)
      mexErrMsgTxt("Input extrinsicTranslation must be a 3x1 double vector.");
    
    // create matrices which point to the input
    cvm::rmatrix   initialState((double*)mxGetPr(prhs[1]),6,1);
    cvm::srsmatrix initialCovState((double*)mxGetPr(prhs[2]),6);
    cvm::srsmatrix covSystem((double*)mxGetPr(prhs[3]),6);
    cvm::srmatrix  intrinsicCalibration((double*)mxGetPr(prhs[4]),3);
    cvm::srmatrix  extrinsicRotation((double*)mxGetPr(prhs[5]),3);
    cvm::rmatrix   extrinsicTranslation((double*)mxGetPr(prhs[6]),3,1);
    
    // TODO right/left calibration
    VO = new VisualOdometry(initialState,initialCovState,covSystem,intrinsicCalibration,
                            intrinsicCalibration,extrinsicRotation,extrinsicTranslation);

  // close
  } else if (!strcmp(command,"close")) {
    delete VO;

  // update via observations
  } else if (!strcmp(command,"update")) {
    
    // check arguments
    if(nrhs!=1+3)
      mexErrMsgTxt("3 inputs required (deltaT,p_matched,varianceMeasurements).");
    if(!mxIsDouble(prhs[1]) || mxGetM(prhs[1])*mxGetN(prhs[1])!=1)
      mexErrMsgTxt("Input deltaT must be a double scalar.");
    if(!mxIsSingle(prhs[2]) || mxGetM(prhs[2])!=8)
      mexErrMsgTxt("Input p_matched must be a 8xN single matrix.");
    if(!mxIsDouble(prhs[3]) || mxGetM(prhs[3])*mxGetN(prhs[3])!=1)
      mexErrMsgTxt("Input varianceMeasurements must be a double scalar.");
    
    // get pointers
    double deltaT    = *((double*)mxGetPr(prhs[1]));
    float* p_matched =    (float*)mxGetPr(prhs[2]);
    int    N         =             mxGetN(prhs[2]);
    
    // copy matches to 4 single cvm matrices
    cvm::rmatrix matchesLeftPrevious (N,2);
    cvm::rmatrix matchesRightPrevious(N,2);
    cvm::rmatrix matchesLeftCurrent  (N,2);
    cvm::rmatrix matchesRightCurrent (N,2);
    for (int i=0; i<N; i++) {
      matchesLeftPrevious(i+1,1)  = p_matched[i*8+0];
      matchesLeftPrevious(i+1,2)  = p_matched[i*8+1];
      matchesRightPrevious(i+1,1) = p_matched[i*8+2];
      matchesRightPrevious(i+1,2) = p_matched[i*8+3];
      matchesLeftCurrent(i+1,1)   = p_matched[i*8+4];
      matchesLeftCurrent(i+1,2)   = p_matched[i*8+5];
      matchesRightCurrent(i+1,1)  = p_matched[i*8+6];
      matchesRightCurrent(i+1,2)  = p_matched[i*8+7];
    }
    
    // variance of measurements
    double varianceMeasurements = *((double*)mxGetPr(prhs[3]));
    
    // perform prediction and innovation
    VO->prediction();
    VO->innovation(matchesLeftPrevious,matchesRightPrevious,
                   matchesLeftCurrent,matchesRightCurrent,
                   deltaT,varianceMeasurements);

  // read transformation
  } else if (!strcmp(command,"gettransformation")) {
    
    // check arguments
    if(nlhs!=1)
      mexErrMsgTxt("1 output required (transformation).");
    
    // create outputs
    const int dims[] = {3,4};
    plhs[0] = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL);
    cvm::rmatrix transformation((double*)mxGetPr(plhs[0]),3,4);
    
    // get state
    transformation = VO->getTransformation();

  // get inliers
  } else if (!strcmp(command,"getinliers")) {
    
    // check arguments
    if(nlhs!=1)
      mexErrMsgTxt("1 output required (inliers).");
    
    // get inliers
    std::vector<int> inliers = VO->getInliers();
    const int dims[] = {1,inliers.size()};
    plhs[0] = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL);
    double *inliers_mex = (double*)mxGetPr(plhs[0]);
    for (int i=0; i<inliers.size(); i++)
      inliers_mex[i] = inliers[i]+1; // convert to matlab index (start with 1)
  
  // unknown command
  } else {
    mexPrintf("Unknown command: %s\n",command);
  }
  
}
