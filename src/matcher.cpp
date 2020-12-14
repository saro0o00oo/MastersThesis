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

#include "matcher.h"
#include "triangle.h"

using namespace std;

//////////////////////
// PUBLIC FUNCTIONS //
//////////////////////

void Matcher::computeFeaturesFromImagePair (int16_t *I1,int16_t* I2,const int32_t* dims) {

  // copy current features to last frame
  if (m1p)    _mm_free(m1p);
  if (I_du1p) free(I_du1p);
  if (I_dv1p) free(I_dv1p);
  if (m2p)    _mm_free(m2p);
  if (I_du2p) free(I_du2p);
  if (I_dv2p) free(I_dv2p);
  m1p = m1c; n1p = n1c; I_du1p = I_du1c; I_dv1p = I_dv1c;
  m2p = m2c; n2p = n2c; I_du2p = I_du2c; I_dv2p = I_dv2c;
  dims_p[0] = dims_c[0];
  dims_p[1] = dims_c[1];

  // compute new features for current frame
  computeFeatures(I1,dims,m1c,n1c,I_du1c,I_dv1c);
  computeFeatures(I2,dims,m2c,n2c,I_du2c,I_dv2c);
  
  // set new dims
  dims_c[0] = dims[0];
  dims_c[1] = dims[1];
}

void Matcher::matchFeatures() {

  // if all images contain features, match them
  if (m1p!=0 && n1p!=0 && m2p!=0 && n2p!=0 && m1c!=0 && n1c!=0 && m2c!=0 && n2c!=0) {
    
    // clear old matches
    p_matched.clear();

    // match features
    quadMatchMaxima();

    // remove outliers
    if (param_outlier_binsize>0)
      removeOutliersBinningMethod();
    else
      removeOutliersTriangulationMethod();
    
    // refine u-coordinates for more accurate stereo measurements
    subpixelRefinement();
  }
}

void Matcher::bucketFeatures(int32_t max_features,float bucket_width,float bucket_height) {

  // find max values
  float u_max = 0;
  float v_max = 0;
  for (vector<p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    if (it->u1p>u_max) u_max=it->u1p;
    if (it->v1p>v_max) v_max=it->v1p;
  }

  // allocate number of buckets needed
  int32_t bucket_cols = floor(u_max/bucket_width)+1;
  int32_t bucket_rows = floor(v_max/bucket_height)+1;
  vector<p_match> *buckets = new vector<p_match>[bucket_cols*bucket_rows];

  // assign matches to their buckets
  for (vector<p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
    int32_t u = floor(it->u1p/bucket_width);
    int32_t v = floor(it->v1p/bucket_height);
    buckets[v*bucket_cols+u].push_back(*it);
  }
  
  // refill p_matched from buckets
  p_matched.clear();
  for (int32_t i=0; i<bucket_cols*bucket_rows; i++) {
    
    // shuffle bucket indices randomly
    std::random_shuffle(buckets[i].begin(),buckets[i].end());
    
    // add up to max_features features from this bucket to p_matched
    int32_t k=0;
    for (vector<p_match>::iterator it=buckets[i].begin(); it!=buckets[i].end(); it++) {
      p_matched.push_back(*it);
      k++;
      if (k>=max_features)
        break;
    }
  }

  // free buckets
  delete []buckets;
}

///////////////////////
// PRIVATE FUNCTIONS //
///////////////////////

void Matcher::nonMaximumSuppression (int16_t* I,int16_t* M,const int32_t* dims,vector<Matcher::maximum> &maxima,bool pos,int32_t c) {
  
  // extract parameters
  int32_t w   = dims[0];
  int32_t h   = dims[1];
  int32_t n   = param_nms_n;
  int32_t tau = param_nms_tau;
  
  // loop variables
  int32_t mi,mj;
  int32_t mval,cval;
  
  for (int32_t i=n+margin; i<w-n-margin;i+=n+1) {
    for (int32_t j=n+margin; j<h-n-margin;j+=n+1) {
      
      mi   = i;
      mj   = j;
      mval = *(I+getAddressOffsetImage(mi,mj,w));
      
      for (int32_t i2=i; i2<=i+n; i2++) {
        for (int32_t j2=j; j2<=j+n; j2++) {
          cval = *(I+getAddressOffsetImage(i2,j2,w));
          if (pos ? cval>mval : cval<mval) {
            mi   = i2;
            mj   = j2;
            mval = cval;
          }
        }
      }
      
      for (int32_t i2=mi-n; i2<=min(mi+n,w-1); i2++) {
        for (int32_t j2=mj-n; j2<=min(mj+n,h-1); j2++) {
          if (i2<i || i2>i+n || j2<j || j2>j+n) {
            cval = *(I+getAddressOffsetImage(i2,j2,w));
            if (pos ? cval>mval : cval<mval)
              goto failed;
          }
        }
      }
      
      // add maximum
      if ( (pos ? mval>=tau : mval<=tau) && *(M+getAddressOffsetImage(mi,mj,w))==0 ) {
        maxima.push_back(Matcher::maximum(mi,mj,mval,c));
        *(M+getAddressOffsetImage(mi,mj,w)) = 1;
      }
      failed:;
    }
  }
}

void Matcher::convolve (int16_t* I_in,int16_t* I_filter,int16_t* I_out,const int32_t* dims,const int32_t* dims_filter) {
  
  // get image width and height  
  int32_t width  = dims[0];
  int32_t height = dims[1];
  
  // get filter width and height  
  int32_t width_filter  = dims_filter[0];
  int32_t height_filter = dims_filter[1];
  
  // compute non-processable margin
  int32_t u_margin = (int32_t)floor((float)width_filter/2.0);
  int32_t v_margin = (int32_t)floor((float)height_filter/2.0);
  
  // allocate image pointers
  int16_t **p_img = (int16_t**)malloc(width_filter*height_filter*sizeof(int16_t*));
  int16_t *I_out_p;

  // loop variables
  int32_t u,v,u2,v2,i,k;

  // for all rows do
  for (v=v_margin; v<height-v_margin; v++) {

    // init image pointers (left->right => top->bottom)
    k=0;
    for (v2=-v_margin; v2<-v_margin+height_filter; v2++)
      for (u2=0; u2<width_filter; u2++)
        *(p_img+k++) = I_in+(v+v2)*width+u2;
    
    // pointer to current row of filter image
    I_out_p = I_out+v*width+u_margin;
    
    // for all columns do
    for (u=u_margin; u<width-u_margin; u++) {
      for (i=0; i<k; i++) {
        *I_out_p += *(I_filter+i) * *p_img[i]++;
      }
      I_out_p++;
    }
  }

  free(p_img);
}

void Matcher::createSobelImages (int16_t* I,const int32_t* dims,uint8_t* I_sobel_du,uint8_t* I_sobel_dv) {
  
  // get image width and height  
  int32_t width  = dims[0];
  int32_t height = dims[1];

  // allocate memory-aligned temporary buffer
  int32_t buffer_size = (width*height/8+1)*sizeof(__m128);
  int16_t* buffer  = (int16_t*)malloc(buffer_size);
  int16_t* buffer2 = (int16_t*)malloc(buffer_size);
  
  // create sobel kernel
  int16_t f1[5]; int32_t dims_f1[2];
  int16_t f2[5]; int32_t dims_f2[2];

  // 5x5
  if (param_sobel_kernel==5) {

    // filter kernel
    f1[0] = +1; f1[1] = +4; f1[2] = +6; f1[3] = +4; f1[4] = +1;
    f2[0] = +1; f2[1] = +2; f2[2] = +0; f2[3] = -2; f2[4] = -1;
    int16_t divisor = 48;

    // du
    dims_f1[0] = 1; dims_f1[1] = 5;
    dims_f2[0] = 5; dims_f2[1] = 1;
    memset(buffer,0,buffer_size);
    memset(buffer2,0,buffer_size);
    convolve(I,f1,buffer,dims,dims_f1);
    convolve(buffer,f2,buffer2,dims,dims_f2);
    for (int32_t i=0; i<height*width; i++)
      *(I_sobel_du+i) = min(max(*(buffer2+i)/divisor+128,0),255);

    // dv
    dims_f1[0] = 5; dims_f1[1] = 1;
    dims_f2[0] = 1; dims_f2[1] = 5;
    memset(buffer,0,buffer_size);
    memset(buffer2,0,buffer_size);
    convolve(I,f1,buffer,dims,dims_f1);
    convolve(buffer,f2,buffer2,dims,dims_f2);
    for (int32_t i=0; i<height*width; i++)
      *(I_sobel_dv+i) = min(max(*(buffer2+i)/divisor+128,0),255);

  // 3x3
  } else {

    // filter kernel
    f1[0] = +1; f1[1] = +2; f1[2] = +1;
    f2[0] = +1; f2[1] = +0; f2[2] = -1;
    int16_t divisor = 4;

    // du
    dims_f1[0] = 1; dims_f1[1] = 3;
    dims_f2[0] = 3; dims_f2[1] = 1;
    memset(buffer,0,buffer_size);
    memset(buffer2,0,buffer_size);
    convolve(I,f1,buffer,dims,dims_f1);
    convolve(buffer,f2,buffer2,dims,dims_f2);
    for (int32_t i=0; i<height*width; i++)
      *(I_sobel_du+i) = min(max(*(buffer2+i)/divisor+128,0),255);

    // dv
    dims_f1[0] = 3; dims_f1[1] = 1;
    dims_f2[0] = 1; dims_f2[1] = 3;
    memset(buffer,0,buffer_size);
    memset(buffer2,0,buffer_size);
    convolve(I,f1,buffer,dims,dims_f1);
    convolve(buffer,f2,buffer2,dims,dims_f2);
    for (int32_t i=0; i<height*width; i++)
      *(I_sobel_dv+i) = min(max(*(buffer2+i)/divisor+128,0),255);
  }
    
  // free buffer
  free(buffer);
  free(buffer2);
}

// note: descriptor must be computed within margin!
inline void Matcher::computeDescriptor (uint8_t* I_du,uint8_t* I_dv,const int32_t &width,const int32_t &u,const int32_t &v,uint8_t *desc_addr) {
  uint32_t k = 0;
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u-3,v-1,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u-3,v-1,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u-3,v+1,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u-3,v+1,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u-1,v-1,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u-1,v-1,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u-1,v+1,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u-1,v+1,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u+1,v-1,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u+1,v-1,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u+1,v+1,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u+1,v+1,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u+3,v-1,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u+3,v-1,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u+3,v+1,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u+3,v+1,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u-1,v-5,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u-1,v-5,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u-1,v+5,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u-1,v+5,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u+1,v-5,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u+1,v-5,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u+1,v+5,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u+1,v+5,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u-5,v-4,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u-5,v-4,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u-5,v+4,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u-5,v+4,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u+5,v-4,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u+5,v-4,width));
  *(desc_addr+k++) = *(I_du+getAddressOffsetImage(u+5,v+4,width));
  *(desc_addr+k++) = *(I_dv+getAddressOffsetImage(u+5,v+4,width));
}

void Matcher::computeDescriptors (int16_t* I,const int32_t* dims,std::vector<Matcher::maximum> &maxima,uint8_t* &I_du,uint8_t* &I_dv) {

  // get image width and height  
  int32_t width  = dims[0];
  int32_t height = dims[1];
  
  // create sobel images
  I_du = (uint8_t*)malloc(width*height*sizeof(uint8_t*));
  I_dv = (uint8_t*)malloc(width*height*sizeof(uint8_t*));
  createSobelImages(I,dims,I_du,I_dv);
  
  // loop variables
  int32_t u,v,k;
  uint8_t *desc_addr;
  
  // for all maxima do
  for (vector<Matcher::maximum>::iterator it=maxima.begin(); it!=maxima.end(); it++) {
    u = (*it).u;
    v = (*it).v;
    desc_addr = (uint8_t*)(&((*it).d1));
    computeDescriptor(I_du,I_dv,width,u,v,desc_addr);    
  }
}

void Matcher::computeFeatures (int16_t *I,const int32_t* dims,int32_t* &max,int32_t &num,uint8_t* &I_du,uint8_t* &I_dv) {

  // get image width and height  
  int32_t width  = dims[0];
  int32_t height = dims[1];

  // return value
  vector<Matcher::maximum> maxima;

  // create interest point filter kernels
  int16_t F1[16] = {-1,-1,-1,-1,-1,+3,+3,-1,-1,+3,+3,-1,-1,-1,-1,-1};
  int16_t F2[16] = {-1,-1,+1,+1,-1,-1,+1,+1,+1,+1,-1,-1,+1,+1,-1,-1};
  const int32_t dims_filter[] = {4,4};

  // allocate memory for filtered images
  int16_t *IF1 = (int16_t*)calloc(width*height,sizeof(int16_t));
  int16_t *IF2 = (int16_t*)calloc(width*height,sizeof(int16_t));

  // filter images
  convolve(I,F1,IF1,dims,dims_filter);
  convolve(I,F2,IF2,dims,dims_filter);

  // extract maxima via non-maximum suppression
  int16_t *M = (int16_t*)calloc(width*height,sizeof(int16_t));
  nonMaximumSuppression(IF1,M,dims,maxima,true, 0);
  nonMaximumSuppression(IF1,M,dims,maxima,false,1);
  nonMaximumSuppression(IF2,M,dims,maxima,true, 2);
  nonMaximumSuppression(IF2,M,dims,maxima,false,3);
  free(M);

  // add descriptors
  computeDescriptors(I,dims,maxima,I_du,I_dv);

  // release filter images
  free(IF1);
  free(IF2);

  // return zero if no feature point has been found  
  num = maxima.size();
  if (num==0) {
    max = 0;
    return;
  }

  // return result as aligned memory
  max = (int32_t*)_mm_malloc(sizeof(Matcher::maximum)*num,16);
  int32_t k=0;
  for (vector<Matcher::maximum>::iterator it=maxima.begin(); it!=maxima.end(); it++) {
    *(max+k++) = it->u;    *(max+k++) = it->v;    *(max+k++) = it->val;  *(max+k++) = it->c;
    *(max+k++) = it->d1;   *(max+k++) = it->d2;   *(max+k++) = it->d3;   *(max+k++) = it->d4;
    *(max+k++) = it->d5;   *(max+k++) = it->d6;   *(max+k++) = it->d7;   *(max+k++) = it->d8;
  }
}

void Matcher::createIndexvector (int32_t* m,int32_t n,vector<int32_t> *k,const int32_t &u_bin_num,const int32_t &v_bin_num) {

  // descriptor step size
  int32_t step_size = sizeof(Matcher::maximum)/4;
  
  // for all points do
  for (int32_t i=0; i<n; i++) {
    
    // extract coordinates and class
    int32_t u = *(m+step_size*i+0);
    int32_t v = *(m+step_size*i+1);
    int32_t c = *(m+step_size*i+3);
    
    // compute number of bins
    int32_t u_bin = min((int32_t)floor((float)u/(float)param_match_binsize),u_bin_num-1);
    int32_t v_bin = min((int32_t)floor((float)v/(float)param_match_binsize),v_bin_num-1);
    
    // save index
    k[(c*v_bin_num+v_bin)*u_bin_num+u_bin].push_back(i);
  }
}

inline bool Matcher::findMatch (int32_t* m1,const int32_t &i1,int32_t* m2,const int32_t &step_size,vector<int32_t> *k2,const int32_t &u_bin_num,const int32_t &v_bin_num,int32_t& min_ind,__m128i &xmm1,__m128i& xmm2,__m128i &xmm3,__m128i& xmm4,bool flow) {
  
  min_ind = 0;
  int32_t min_cost_1 = 10000000;
  int32_t min_cost_2 = 10000000;
  int32_t cost;
  int32_t u1 = *(m1+step_size*i1+0);
  int32_t v1 = *(m1+step_size*i1+1);
  int32_t c1 = *(m1+step_size*i1+3);
  int32_t u2,v2;
  int32_t du,dv;
  xmm1 = _mm_load_si128((__m128i*)(m1+step_size*i1+4));
  xmm2 = _mm_load_si128((__m128i*)(m1+step_size*i1+8));
  
  // compute bins of interest
  int32_t u_bin_1 = max(min((int32_t)floor((float)(u1-param_match_radius)/(float)param_match_binsize),u_bin_num-1),0);
  int32_t u_bin_2 = max(min((int32_t)floor((float)(u1+param_match_radius)/(float)param_match_binsize),u_bin_num-1),0);
  int32_t v_bin_1 = max(min((int32_t)floor((float)(v1-param_match_radius)/(float)param_match_binsize),v_bin_num-1),0);
  int32_t v_bin_2 = max(min((int32_t)floor((float)(v1+param_match_radius)/(float)param_match_binsize),v_bin_num-1),0);
  
  // for all bins of interest do
  for (int32_t u_bin=u_bin_1; u_bin<=u_bin_2; u_bin++) {
    for (int32_t v_bin=v_bin_1; v_bin<=v_bin_2; v_bin++) {
      int32_t k_ind = (c1*v_bin_num+v_bin)*u_bin_num+u_bin;
      for (vector<int32_t>::iterator i2_it=k2[k_ind].begin(); i2_it!=k2[k_ind].end(); i2_it++) {
        u2 = *(m2+step_size*(*i2_it)+0);
        v2 = *(m2+step_size*(*i2_it)+1);
        du = abs(u1-u2);
        dv = abs(v1-v2);
        if (du<param_match_radius && dv<param_match_radius && (flow || dv<=param_match_disp_tolerance)) {
          xmm3 = _mm_load_si128((__m128i*)(m2+step_size*(*i2_it)+4));
          xmm4 = _mm_load_si128((__m128i*)(m2+step_size*(*i2_it)+8));
          xmm3 = _mm_sad_epu8(xmm1,xmm3);
          xmm4 = _mm_sad_epu8(xmm2,xmm4);
          cost = _mm_extract_epi16(xmm3,0)+_mm_extract_epi16(xmm3,4)+_mm_extract_epi16(xmm4,0)+_mm_extract_epi16(xmm4,4);
          if (cost<min_cost_1) {
            min_ind    = *i2_it;
            min_cost_1 = cost;
          } else if (cost<min_cost_2) {
            min_cost_2 = cost;
          }
        }
      }
    }
  }
  
  // check uniqueness criterion
  if ((float)min_cost_1<param_match_uniqueness*(float)min_cost_2)
    return true;
  else
    return false;
}

void Matcher::quadMatchMaxima () {

  // descriptor step size
  int32_t step_size = sizeof(Matcher::maximum)/4;
  
  // find u_max, v_max
  int32_t u_max = 0;
  int32_t v_max = 0;
  for (int32_t i=0; i<n1p; i++) {
    if (*(m1p+step_size*i+0)>u_max)
      u_max = *(m1p+step_size*i+0);
    if (*(m1p+step_size*i+1)>v_max)
      v_max = *(m1p+step_size*i+0);
  }
  
  // compute number of bins
  int32_t u_bin_num = (int32_t)floor((float)u_max/(float)param_match_binsize);
  int32_t v_bin_num = (int32_t)floor((float)v_max/(float)param_match_binsize);
  int32_t bin_num = 4*v_bin_num*u_bin_num;
  
  // allocate memory for index vectors (needed for efficient search)
  vector<int32_t> *k1p = new vector<int32_t>[bin_num];
  vector<int32_t> *k2p = new vector<int32_t>[bin_num];
  vector<int32_t> *k1c = new vector<int32_t>[bin_num];
  vector<int32_t> *k2c = new vector<int32_t>[bin_num];

  // create index vectors (fill k1p,...)
  createIndexvector(m1p,n1p,k1p,u_bin_num,v_bin_num);
  createIndexvector(m2p,n2p,k2p,u_bin_num,v_bin_num);
  createIndexvector(m1c,n1c,k1c,u_bin_num,v_bin_num);
  createIndexvector(m2c,n2c,k2c,u_bin_num,v_bin_num);
  
  // loop variables
  int32_t i1p,i2p,i1c,i2c,i1p2;
  int32_t u1p,v1p,u2p,v2p,u1c,v1c,u2c,v2c;
  int32_t dp,dc;
  __m128i xmm1,xmm2,xmm3,xmm4;
  
  // for all points in 
  for (i1p=0; i1p<n1p; i1p++) {
    
    // match point in circle
    if (!findMatch(m1p,i1p,m2p,step_size,k2p,u_bin_num,v_bin_num,i2p, xmm1,xmm2,xmm3,xmm4,false)) continue;
    if (!findMatch(m2p,i2p,m2c,step_size,k2c,u_bin_num,v_bin_num,i2c, xmm1,xmm2,xmm3,xmm4,true )) continue;
    if (!findMatch(m2c,i2c,m1c,step_size,k1c,u_bin_num,v_bin_num,i1c, xmm1,xmm2,xmm3,xmm4,false)) continue;
    if (!findMatch(m1c,i1c,m1p,step_size,k1p,u_bin_num,v_bin_num,i1p2,xmm1,xmm2,xmm3,xmm4,true )) continue;
    
    // circle closure success?
    if (i1p2==i1p) {
      u1p = *(m1p+step_size*i1p+0); v1p = *(m1p+step_size*i1p+1);
      u2p = *(m2p+step_size*i2p+0); v2p = *(m2p+step_size*i2p+1);
      u1c = *(m1c+step_size*i1c+0); v1c = *(m1c+step_size*i1c+1);
      u2c = *(m2c+step_size*i2c+0); v2c = *(m2c+step_size*i2c+1);
      dp = u1p-u2p;
      dc = u1c-u2c;
      
      // if disparities are positive
      if (dp>0 && dc>0) {
        
        // pull matches to same scanline
        v1p = (v1p+v2p)/2; v2p = v1p;
        v1c = (v1c+v2c)/2; v2c = v1c;
        
        // add match to vector
        p_matched.push_back(Matcher::p_match(u1p,v1p,u2p,v2p,u1c,v1c,u2c,v2c));
      }
    }
  }

  // free memory
  delete []k1p;
  delete []k2p;
  delete []k1c;
  delete []k2c;
}

void Matcher::removeOutliersBinningMethod () {
  
  // find u_max, v_max
  int32_t u_max = 0;
  int32_t v_max = 0;
  for (vector<Matcher::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
    if (it->u1p>u_max)
      u_max = it->u1p;
    if (it->v1p>v_max)
      v_max = it->v1p;
  }
  
  // compute number of bins
  int32_t u_bin_num = (int32_t)floor((float)u_max/(float)param_outlier_binsize);
  int32_t v_bin_num = (int32_t)floor((float)v_max/(float)param_outlier_binsize);
  int32_t bin_num   = u_bin_num*v_bin_num;

  // put all matches into bins and clear p_matched
  vector<Matcher::p_match> *p_matched_bin = new vector<Matcher::p_match>[bin_num];
  for (vector<Matcher::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
    int32_t u_bin = min((int32_t)floor((float)(*it).u1p/(float)param_outlier_binsize),u_bin_num-1);
    int32_t v_bin = min((int32_t)floor((float)(*it).v1p/(float)param_outlier_binsize),v_bin_num-1);
    p_matched_bin[v_bin*u_bin_num+u_bin].push_back(*it);
  }
  p_matched.clear();
  
  // for all bins: put inliers into p_matched
  for (int32_t u_bin=0; u_bin<u_bin_num; u_bin++) {
    for (int32_t v_bin=0; v_bin<v_bin_num; v_bin++) {
      int32_t ind = v_bin*u_bin_num+u_bin;
      if (p_matched_bin[ind].size()>=2) {
        
        // median element
        int32_t median = (p_matched_bin[ind].size()-1)/2;
        
        // median flow u
        sort(p_matched_bin[ind].begin(),p_matched_bin[ind].end(),sortMatchFlowU);
        int32_t med_flow_u = p_matched_bin[ind][median].u1c-p_matched_bin[ind][median].u1p;
        
        // median flow v
        sort(p_matched_bin[ind].begin(),p_matched_bin[ind].end(),sortMatchFlowV);
        int32_t med_flow_v = p_matched_bin[ind][median].v1c-p_matched_bin[ind][median].v1p;
        
        // median disp
        sort(p_matched_bin[ind].begin(),p_matched_bin[ind].end(),sortMatchDisp);
        int32_t med_disp = p_matched_bin[ind][median].u1p-p_matched_bin[ind][median].u2p;
        
        // add all inlier elements of this bin to filtered matches
        if (p_matched_bin[ind].size()>2) {
          for (uint32_t i=0; i<p_matched_bin[ind].size(); i++) {
            int32_t flow_u = p_matched_bin[ind][i].u1c-p_matched_bin[ind][i].u1p;
            int32_t flow_v = p_matched_bin[ind][i].v1c-p_matched_bin[ind][i].v1p;
            int32_t disp   = p_matched_bin[ind][i].u1p-p_matched_bin[ind][i].u2p;
            
            if (abs(disp-med_disp)<param_outlier_disp_tolerance && abs(flow_u-med_flow_u)+abs(flow_v-med_flow_v)<param_outlier_flow_tolerance)
              p_matched.push_back(p_matched_bin[ind][i]);
          }
        }
 
      }
    }
  }

  // free memory
  delete []p_matched_bin;
}

void Matcher::removeOutliersTriangulationMethod () {

  // input/output structure for triangulation
  struct triangulateio in, out;

  // inputs
  in.numberofpoints = p_matched.size();
  in.pointlist = (float*)malloc(in.numberofpoints*2*sizeof(float));
  int32_t k=0;
  
  // create copy of p_matched, init vector with number of support points
  // and fill triangle point vector for delaunay triangulation
  vector<Matcher::p_match> p_matched_vec;  
  vector<int32_t> num_support;
  for (vector<Matcher::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
    p_matched_vec.push_back(*it);
    num_support.push_back(0);
    in.pointlist[k++] = it->u1p;
    in.pointlist[k++] = it->v1p;
  }
  
  // input parameters
  in.numberofpointattributes = 0;
  in.pointattributelist      = NULL;
  in.pointmarkerlist         = NULL;
  in.numberofsegments        = 0;
  in.numberofholes           = 0;
  in.numberofregions         = 0;
  in.regionlist              = NULL;
  
  // outputs
  out.pointlist              = NULL;
  out.pointattributelist     = NULL;
  out.pointmarkerlist        = NULL;
  out.trianglelist           = NULL;
  out.triangleattributelist  = NULL;
  out.neighborlist           = NULL;
  out.segmentlist            = NULL;
  out.segmentmarkerlist      = NULL;
  out.edgelist               = NULL;
  out.edgemarkerlist         = NULL;

  // do triangulation (z=zero-based, n=neighbors, Q=quiet, B=no boundary markers)
  // attention: not using the B switch or using the n switch creates a memory leak (=> use valgrind!)
  char parameters[] = "zQB";
  triangulate(parameters, &in, &out, NULL);
  
  // for all triangles do
  for (int32_t i=0; i<out.numberoftriangles; i++) {
    
    // extract triangle corner points
    int32_t p1 = out.trianglelist[i*3+0];
    int32_t p2 = out.trianglelist[i*3+1];
    int32_t p3 = out.trianglelist[i*3+2];

    // 1. corner disparity and flow
    int32_t p1_flow_u = p_matched_vec[p1].u1c-p_matched_vec[p1].u1p;
    int32_t p1_flow_v = p_matched_vec[p1].v1c-p_matched_vec[p1].v1p;
    int32_t p1_disp   = p_matched_vec[p1].u1p-p_matched_vec[p1].u2p;
    
    // 2. corner disparity and flow
    int32_t p2_flow_u = p_matched_vec[p2].u1c-p_matched_vec[p2].u1p;
    int32_t p2_flow_v = p_matched_vec[p2].v1c-p_matched_vec[p2].v1p;
    int32_t p2_disp   = p_matched_vec[p2].u1p-p_matched_vec[p2].u2p;
    
    // 3. corner disparity and flow
    int32_t p3_flow_u = p_matched_vec[p3].u1c-p_matched_vec[p3].u1p;
    int32_t p3_flow_v = p_matched_vec[p3].v1c-p_matched_vec[p3].v1p;
    int32_t p3_disp   = p_matched_vec[p3].u1p-p_matched_vec[p3].u2p;

    // consistency of 1. edge
    if (abs(p1_disp-p2_disp)<param_outlier_disp_tolerance && abs(p1_flow_u-p2_flow_u)+abs(p1_flow_v-p2_flow_v)<param_outlier_flow_tolerance) {
      num_support[p1]++;
      num_support[p2]++;
    }
    
    // consistency of 2. edge
    if (abs(p2_disp-p3_disp)<param_outlier_disp_tolerance && abs(p2_flow_u-p3_flow_u)+abs(p2_flow_v-p3_flow_v)<param_outlier_flow_tolerance) {
      num_support[p2]++;
      num_support[p3]++;
    }
    
    // consistency of 3. edge
    if (abs(p1_disp-p3_disp)<param_outlier_disp_tolerance && abs(p1_flow_u-p3_flow_u)+abs(p1_flow_v-p3_flow_v)<param_outlier_flow_tolerance) {
      num_support[p1]++;
      num_support[p3]++;
    }
  }
  
  // refill p_matched
  p_matched.clear();
  for (int i=0; i<in.numberofpoints; i++)
    if (num_support[i]>=2*2)
      p_matched.push_back(p_matched_vec[i]);
  
  // free memory used for triangulation
  free(in.pointlist);
  free(out.pointlist);
  free(out.trianglelist);
}

float Matcher::parabolicFitting(uint8_t* I_du1,uint8_t* I_dv1,uint8_t* I_du2,uint8_t* I_dv2,int32_t* dims,int32_t u1,int32_t v1,int32_t u2,int32_t v2,uint8_t* desc_buffer) {
  
  // get image width and height  
  int32_t width  = dims[0];
  int32_t height = dims[1];
  
  // check if parabolic fitting can be done (descriptors are within margin)
  if (u2-2<margin || u2+2>width-1-margin)
    return -1;
  
  // compute reference descriptor
  __m128i xmm1,xmm2,xmm3,xmm4;
  computeDescriptor(I_du1,I_dv1,width,u1,v1,desc_buffer);
  xmm1 = _mm_load_si128((__m128i*)(desc_buffer+0 ));
  xmm2 = _mm_load_si128((__m128i*)(desc_buffer+16));
  
  // compute cost vector
  int32_t cost[5];
  for (int32_t du=-2; du<=+2; du++) {
    computeDescriptor(I_du2,I_dv2,width,u2+du,v2,desc_buffer+32);
    xmm3 = _mm_load_si128((__m128i*)(desc_buffer+32));
    xmm4 = _mm_load_si128((__m128i*)(desc_buffer+48));
    xmm3 = _mm_sad_epu8(xmm1,xmm3);
    xmm4 = _mm_sad_epu8(xmm2,xmm4);
    cost[du+2] = _mm_extract_epi16(xmm3,0)+_mm_extract_epi16(xmm3,4)+_mm_extract_epi16(xmm4,0)+_mm_extract_epi16(xmm4,4);
  }
  
  // compute minimum
  int32_t min_ind  = 0;
  int32_t min_cost = cost[0];
  for (int32_t i=1; i<4; i++) {
    if (cost[i]<min_cost) {
      min_ind   = i;
      min_cost  = cost[i];
    }
  }
  
  // if minimum is at borders => this seems to be a bad match
  // (at least we cannot compute subpixel accuracy)
  if (min_ind==0 || min_ind==4)
    return -1;
  
  // cost function must have minimum in between
  if (cost[min_ind-1]<cost[min_ind] || cost[min_ind+1]<cost[min_ind])
    return -1;
  
  // return parabolic fit
  // see for example: I. Haller: Real-time semi-global dense stereo solution
  // with improved sub-pixel accuracy, IV 2010
  float a = cost[min_ind-1]-cost[min_ind+1];
  float b = 2*(cost[min_ind-1]+cost[min_ind+1]-2*cost[min_ind]);
  if (fabs(b)<0.0001)
    return -1;
  else
    return u2+min_ind-2+a/b;
}

void Matcher::subpixelRefinement () {
  
  // allocate some aligned memory (2*32 bytes for 2 descriptors)
  uint8_t* desc_buffer = (uint8_t*)_mm_malloc(sizeof(uint8_t)*32*2,16);
  
  // copy vector
  vector<Matcher::p_match> p_matched_ = p_matched;
  p_matched.clear();
  
  // for all matches do
  for (vector<Matcher::p_match>::iterator it=p_matched_.begin(); it!=p_matched_.end(); it++) {
    float u2p_fit = parabolicFitting(I_du1p,I_dv1p,I_du2p,I_dv2p,dims_p,it->u1p,it->v1p,it->u2p,it->v2p,desc_buffer);
    float u2c_fit = parabolicFitting(I_du1c,I_dv1c,I_du2c,I_dv2c,dims_c,it->u1c,it->v1c,it->u2c,it->v2c,desc_buffer);
    if (u2p_fit>=0 && u2c_fit>=0) {
      it->u2p = u2p_fit;
      it->u2c = u2c_fit;
      p_matched.push_back(*it);
    }
  }
  
  // free memory
  _mm_free(desc_buffer);
}


