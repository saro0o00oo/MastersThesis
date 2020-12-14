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

#ifndef __MATCHER_H__
#define __MATCHER_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <emmintrin.h>
#include <algorithm>
#include <vector>

// Define fixed-width datatypes for Visual Studio projects
#ifndef _MSC_VER
  #include <stdint.h>
#else
  typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif

class Matcher {

friend class MatcherFriends;

public:

    // constructor (with default parameters)
    Matcher(int32_t param_nms_n                  = 5,    // non-max-suppression: min. distance between maxima
            int32_t param_nms_tau                = 50,   // non-max-suppression: interest point threshold
            int32_t param_sobel_kernel           = 5,    // sobel kernel size (3 or 5)
            int32_t param_match_binsize          = 100,  // matching bin width/height (affects efficiency)
            int32_t param_match_radius           = 200,  // matching radius (du/dv)
            int32_t param_match_disp_tolerance   = 1,    // du tolerance for stereo matches
            float   param_match_uniqueness       = 0.9,  // ratio between best and second best match
            int32_t param_outlier_binsize        = 0,    // outlier removal: 0=triangulation method, otherwise bin size of binning method
            int32_t param_outlier_disp_tolerance = 10,   // outlier removal: disparity tolerance wrt. median disparity
            int32_t param_outlier_flow_tolerance = 10)   // outlier removal: flow tolerance wrt. median flow
                                                       : param_nms_n(param_nms_n),
                                                         param_nms_tau(param_nms_tau),
                                                         param_sobel_kernel(param_sobel_kernel),
                                                         param_match_binsize(param_match_binsize),
                                                         param_match_radius(param_match_radius),
                                                         param_match_disp_tolerance(param_match_disp_tolerance),
                                                         param_match_uniqueness(param_match_uniqueness),
                                                         param_outlier_binsize (param_outlier_binsize ),
                                                         param_outlier_disp_tolerance(param_outlier_disp_tolerance),
                                                         param_outlier_flow_tolerance(param_outlier_flow_tolerance) {

      // init match ring buffer to zero
      m1p = 0; n1p = 0; I_du1p = 0; I_dv1p = 0;
      m2p = 0; n2p = 0; I_du2p = 0; I_dv2p = 0;
      m1c = 0; n1c = 0; I_du1c = 0; I_dv1c = 0;
      m2c = 0; n2c = 0; I_du2c = 0; I_dv2c = 0;
      
      // margin needed to compute descriptor
      margin = 5;
    }

    // deconstructor
    ~Matcher() {
      if (m1p) _mm_free(m1p);
      if (I_du1p)  free(I_du1p);
      if (I_dv1p)  free(I_dv1p);
      if (m2p) _mm_free(m2p);
      if (I_du2p)  free(I_du2p);
      if (I_dv2p)  free(I_dv2p);
      if (m1c) _mm_free(m1c);
      if (I_du1c)  free(I_du1c);
      if (I_dv1c)  free(I_dv1c);
      if (m2c) _mm_free(m2c);
      if (I_du2c)  free(I_du2c);
      if (I_dv2c)  free(I_dv2c);
    }

    // structure for storing matches
    struct p_match {
      float u1p,v1p; // u,v-coordinates in previous left  image
      float u2p,v2p; // u,v-coordinates in previous right image
      float u1c,v1c; // u,v-coordinates in current  left  image
      float u2c,v2c; // u,v-coordinates in current  right image
      p_match(){}
      p_match(float u1p,float v1p,float u2p,float v2p,
              float u1c,float v1c,float u2c,float v2c):
              u1p(u1p),v1p(v1p),u2p(u2p),v2p(v2p),
              u1c(u1c),v1c(v1c),u2c(u2c),v2c(v2c) {}
    };

    // computes features from both images and pushes them back to a ringbuffer,
    // which interally stores the features of the current and previous image pair
    // input: I1,I2 ............. pointers to first byte of left and right image (row-aligned)
    //                            image data may range [0..255], but must be of type int16_t
    //        dims  ............. dims[0] = image width, dims[1] = image height
    void computeFeaturesFromImagePair (int16_t *I1,int16_t* I2,const int32_t* dims);

    // match features in ring buffer (current and previous frame)
    // output: vector<p_match> .... vector of feature matches
    void matchFeatures();

    // feature bucketing: keeps only max_features per bucket, where the domain
    // is split into buckets of size (bucket_width,bucket_height)
    void bucketFeatures(int32_t max_features,float bucket_width,float bucket_height);

    std::vector<p_match>* getMatches() { return &p_matched; }

private:

    // structure for storing interest points
    struct maximum {
      int32_t u;   // u-coordinate
      int32_t v;   // v-coordinate
      int32_t val; // value
      int32_t c;   // class
      int32_t d1,d2,d3,d4,d5,d6,d7,d8;
      maximum() {}
      maximum(int32_t u,int32_t v,int32_t val,int32_t c):u(u),v(v),val(val),c(c){}
    };

    // computes the address offset for coordinates u,v of an image of given width
    inline int32_t getAddressOffsetImage (const int32_t& u,const int32_t& v,const int32_t& width) {
      return v*width+u;
    }

    void convolve (int16_t* I_in,int16_t* I_filter,int16_t* I_out,const int32_t* dims,const int32_t* dims_filter);

    // Alexander Neubeck and Luc Van Gool: Efficient Non-Maximum Suppression, ICPR'06 algorithm 4
    void nonMaximumSuppression (int16_t* I,int16_t* M,const int32_t* dims,std::vector<Matcher::maximum> &maxima,bool pos,int32_t c);

    // descriptor functions
    void createSobelImages (int16_t* I,const int32_t* dims,uint8_t* I_sobel_du,uint8_t* I_sobel_dv);
    inline void computeDescriptor (uint8_t* I_du,uint8_t* I_dv,const int32_t &width,const int32_t &u,const int32_t &v,uint8_t *desc_addr);
    void computeDescriptors (int16_t* I,const int32_t* dims,std::vector<Matcher::maximum> &maxima,uint8_t* &I_du,uint8_t* &I_dv);

    // compute sparse set of features from image
    // inputs:  I ........ image
    //          dims ..... image dimensions [width,height]
    //          n ........ non-max neighborhood
    //          tau ...... non-max threshold
    // outputs: max ...... vector with maxima [u,v,value,class,descriptor (128 bits)]
    //          I_du ..... gradient in horizontal direction
    //          I_dv ..... gradient in vertical direction
    // WARNING: max,I_du,I_dv has to be freed by yourself!
    void computeFeatures (int16_t *I,const int32_t* dims,int32_t* &max,int32_t &num,uint8_t* &I_du,uint8_t* &I_dv);

    // matching functions
    void createIndexvector (int32_t* m,int32_t n,std::vector<int32_t> *k,const int32_t &u_bin_num,const int32_t &v_bin_num);
    inline bool findMatch (int32_t* m1,const int32_t &i1,int32_t* m2,const int32_t &step_size,
                           std::vector<int32_t> *k2,const int32_t &u_bin_num,const int32_t &v_bin_num,
                           int32_t& min_ind,__m128i &xmm1,__m128i& xmm2,__m128i &xmm3,__m128i& xmm4,bool flow);
    void quadMatchMaxima ();

    // outlier removal functions
    static bool sortMatchFlowU (const p_match& lhs, const p_match& rhs) { return lhs.u1c-lhs.u1p < rhs.u1c-rhs.u1p; }
    static bool sortMatchFlowV (const p_match& lhs, const p_match& rhs) { return lhs.v1c-lhs.v1p < rhs.v1c-rhs.v1p; }
    static bool sortMatchDisp  (const p_match& lhs, const p_match& rhs) { return lhs.u1p-lhs.u2p < rhs.u1p-rhs.u2p; }
    void removeOutliersBinningMethod ();
    void removeOutliersTriangulationMethod ();
    float parabolicFitting (uint8_t* I_du1,uint8_t* I_dv1,uint8_t* I_du2,uint8_t* I_dv2,int32_t* dims,int32_t u1,int32_t v1,int32_t u2,int32_t v2,uint8_t* desc_buffer);
    void subpixelRefinement ();

    // private variables
    int32_t param_nms_n;
    int32_t param_nms_tau;
    int32_t param_sobel_kernel;
    int32_t param_match_binsize;
    int32_t param_match_radius;
    int32_t param_match_disp_tolerance;
    float   param_match_uniqueness;
    int32_t param_outlier_binsize;
    int32_t param_outlier_disp_tolerance;
    int32_t param_outlier_flow_tolerance;
    int32_t margin;
    int32_t *m1p,*m2p,*m1c,*m2c;
    int32_t n1p,n2p,n1c,n2c;
    uint8_t *I_du1p,*I_du2p,*I_du1c,*I_du2c;
    uint8_t *I_dv1p,*I_dv2p,*I_dv1c,*I_dv2c;
    int32_t dims_p[2],dims_c[2];

    std::vector<Matcher::p_match> p_matched;
};

#endif
