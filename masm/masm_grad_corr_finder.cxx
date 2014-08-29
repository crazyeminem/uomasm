/* 
* Copyright 2013-2014 Tim Cootes
* This fileis part of UoMASM
* UoMASM is free software: you can redistribute it and/or modify it under the terms of the 
* GNU General Public License as published by the Free Software Foundation, either version 3  
* of the License, or (at your option) any later version.
*
* UoMASM is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; without even the 
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
* See the GNU General Public License for more details. You should have recieved a copy of the licence 
* along with UoMASM. If not, see <http://www.gnu.org/licenses/>
*/
//:
// \file
// \brief Search using cross-correlation of gradient information.
// \author Tim Cootes

#include "masm_grad_corr_finder.h"

#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>
#include <vimt/vimt_image_2d_of.h>
#include <msm/msm_wt_mat_2d.h>
#include <vcl_cassert.h>
#include <vcl_algorithm.h>


//=======================================================================
// Dflt ctor
//=======================================================================

masm_grad_corr_finder::masm_grad_corr_finder()
  : gc_model_(0), search_ni_(5), search_nj_(5)
{
}

//=======================================================================
// Destructor
//=======================================================================

masm_grad_corr_finder::~masm_grad_corr_finder()
{
}

//: Define model data for this searcher
void masm_grad_corr_finder::set_model(const masm_point_finder_model& model)
{
  assert(model.is_a()=="masm_grad_corr_model");
  gc_model_ = &(static_cast<const masm_grad_corr_model&>(model));
  
  search_ni_=gc_model_->search_ni();
  search_nj_=gc_model_->search_nj();
}

//: Define search area in terms of steps either side of base point
//  Units of pose.u()
void masm_grad_corr_finder::set_search_area(unsigned ni, unsigned nj)
{
  search_ni_=ni;
  search_nj_=nj;
}
  
//: Current model data for this searcher
const masm_point_finder_model& masm_grad_corr_finder::model()
{
  assert(gc_model_!=0);
  return *gc_model_;
}

// Dot product of two 2-plane image patches, divided by number of pixels
// Assumes planestep is 1, so element (i,j,p) is im1[p+2*i+j*jstep1] etc
template<class T>
inline float mean_corr(const T* im1, const float* im2,
                        vcl_ptrdiff_t jstep1, vcl_ptrdiff_t jstep2,
                        unsigned ni, unsigned nj)
{
  float sum1=0.0;
  unsigned ni2=2*ni;
  for (unsigned j=0;j<nj;++j,im1+=jstep1,im2+=jstep2)
    for (unsigned i=0;i<ni2;++i)
    {
      sum1+=im1[i]*im2[i];
    }
  unsigned n=ni*nj;

  return sum1/n;
}

//: Fit at given point and scale/orientation in image.
// \param image: Image to be searched
// \param base_pose: Position and scale/orientation defining initial point around which to search
double masm_grad_corr_finder::fit_at_point(const vimt_image_2d& image, const mfpf_pose& base_pose)
{
  assert(gc_model_!=0);

  unsigned kni = gc_model_->kernel().ni();
  unsigned knj = gc_model_->kernel().nj();

  vil_image_view<float> sample;
  sample_region(image,base_pose,0,0,sample);
  return 1.0-mean_corr(gc_model_->kernel().top_left_ptr(),
                   &sample(1,1), 
                   gc_model_->kernel().jstep(),
                   sample.jstep(),
                   kni, knj);
 }

//: Sample region required to do search.
//  Computes unit gradient direction at each point.
void masm_grad_corr_finder::sample_region(const vimt_image_2d& image, 
                                    const mfpf_pose& base_pose,                     
                                    unsigned search_ni, unsigned search_nj,
                                    vil_image_view<float>& sample)
{
  assert(gc_model_!=0);
  
  unsigned kni = gc_model_->kernel().ni();
  unsigned knj = gc_model_->kernel().nj();
  
  masm_sample_grad_region(image,base_pose,kni,knj,gc_model_->ref_pt(),
                          search_ni,search_nj,sample);
}


//: Search the given image to find best position for the point
// \param image: Image to be searched
// \param base_pose: Position and scale/orientation defining initial point around which to search
// \param new_pt: New position found by the search
// \param fit: Value describing quality of match (the smaller the better)
void masm_grad_corr_finder::search(const vimt_image_2d& image, const mfpf_pose& base_pose,
                             vgl_point_2d<double>& new_pt, double& fit)
{
  assert(gc_model_!=0);
  
  vil_image_view<float> sample;
  sample_region(image,base_pose,search_ni_,search_nj_,sample);
  
  int ni=1+2*search_ni_;
  int nj=1+2*search_nj_;
  unsigned kni = gc_model_->kernel().ni();
  unsigned knj = gc_model_->kernel().nj();

  const float* k = gc_model_->kernel().top_left_ptr();
  const float* s_row = &sample(1,1);  // Borders are zero
  vcl_ptrdiff_t s_jstep = sample.jstep();
  vcl_ptrdiff_t k_jstep = gc_model_->kernel().jstep();

  float best_r=-9e9f;
  int best_i=-1,best_j=-1;
  for (int j=0;j<nj;++j,s_row+=s_jstep)
  {
    const float *s=s_row;
    for (int i=0;i<ni;++i,s+=2)
    {
      float r = mean_corr(s,k,s_jstep,k_jstep,kni,knj);
      if (r>best_r) { best_r=r; best_i=i; best_j=j; }
    }
  }

  // Compute position of best point
  new_pt = base_pose(best_i-double(search_ni_),best_j-double(search_nj_));
  fit = 1.0 - best_r;
}

//: Search the given image to find best position for the point
// \param image: Image to be searched
// \param base_pose: Position and scale/orientation defining initial point around which to search
// \param new_pt: New position found by the search
// \param wt_mat: Describes inverse co-variance of estimate of position
void masm_grad_corr_finder::search2(const vimt_image_2d& image, const mfpf_pose& base_pose,
                              vgl_point_2d<double>& new_pt, msm_wt_mat_2d& wt_mat)
{
  assert(gc_model_!=0);
  
  vil_image_view<float> sample;
  sample_region(image,base_pose,search_ni_,search_nj_,sample);
  
  int ni=1+2*search_ni_;
  int nj=1+2*search_nj_;
  unsigned kni = gc_model_->kernel().ni();
  unsigned knj = gc_model_->kernel().nj();

  const float* k = gc_model_->kernel().top_left_ptr();
  const float* s_row = &sample(1,1);   // Borders are zero
  vcl_ptrdiff_t s_jstep = sample.jstep();
  vcl_ptrdiff_t k_jstep = gc_model_->kernel().jstep();

  float best_r=9e9f;
  int best_i=-1,best_j=-1;
  
  vil_image_view<float> fit_im(ni,nj);
  for (int j=0;j<nj;++j,s_row+=s_jstep)
  {
    const float *s=s_row;
    for (int i=0;i<ni;++i,s+=2)
    {
      float r = 1.0f-mean_corr(s,k,s_jstep,k_jstep,kni,knj);
      fit_im(i,j)=r;
      if (r<best_r) { best_r=r; best_i=i; best_j=j; }
    }
  }

  // Compute position of best point
  new_pt = base_pose(best_i-double(search_ni_),best_j-double(search_nj_));
  
  // Estimate weights, assuming that the shape of the fit surface is a quadratic
  // aligned with the axes.  Allowing arbitrary alignment would be possible, but
  // more complicated.
  // Could also fit 1D quadratic to estimate position to sub-pixel accuracy here.
  
  double wi=0,wj=0;
  if (ni>1)
  {
    if (best_i==0) 
      wi = fit_im(1,best_j)-best_r;
    else if (best_i==(ni-1)) 
      wi = fit_im(ni-2,best_j)-best_r;
    else
      wi = 0.5*(fit_im(best_i-1,best_j)+fit_im(best_i+1,best_j))-best_r;
  }
  
  if (nj>0)
  {
    if (best_j==0) 
      wj = fit_im(best_i,1)-best_r;
    else if (best_j==(nj-1)) 
      wj = fit_im(best_i,nj-2)-best_r;
    else
      wj = 0.5*(fit_im(best_i,best_j-1)+fit_im(best_i,best_j+1))-best_r;
  }

  wt_mat = msm_wt_mat_2d(wi,0,wj);

  // wt_mat is in the local co-ordinate frame.
  // Transform into the world frame to take account of scale/rot applied by the frame_pose
  // W' = T'WT where T = inverse of scale/rot defined in base_pose
  double s2 = base_pose.sqr_scale();
  wt_mat = wt_mat.transform_by(base_pose.u().x()/s2,-base_pose.u().y()/s2);
}

//=======================================================================
// Method: is_a
//=======================================================================

vcl_string masm_grad_corr_finder::is_a() const
{
  return vcl_string("masm_grad_corr_finder");
}

  //: Create a copy on the heap and return base class pointer
masm_grad_corr_finder* masm_grad_corr_finder::clone() const
{
  return new masm_grad_corr_finder(*this);
}

  //: Print class to os
void masm_grad_corr_finder::print_summary(vcl_ostream& os) const
{
}