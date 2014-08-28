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
// \brief Search using normalised cross correlation.
// \author Tim Cootes

#include "masm_ncc_finder.h"

#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>
#include <mfpf/mfpf_pose.h>
#include <vil/vil_resample_bilin.h>
#include <vimt/vimt_image_2d_of.h>
#include <msm/msm_wt_mat_2d.h>
#include <vcl_cassert.h>
#include <vcl_algorithm.h>


//=======================================================================
// Dflt ctor
//=======================================================================

masm_ncc_finder::masm_ncc_finder()
  : ncc_model_(0), search_ni_(5), search_nj_(5)
{
}

//=======================================================================
// Destructor
//=======================================================================

masm_ncc_finder::~masm_ncc_finder()
{
}

//: Define model data for this searcher
void masm_ncc_finder::set_model(const masm_point_finder_model& model)
{
  assert(model.is_a()=="masm_ncc_finder_model");
  ncc_model_ = &(static_cast<const masm_ncc_finder_model&>(model));
  
  search_ni_=ncc_model_->search_ni();
  search_nj_=ncc_model_->search_nj();
}

//: Define search area in terms of steps either side of base point
//  Units of pose.u()
void masm_ncc_finder::set_search_area(unsigned ni, unsigned nj)
{
  search_ni_=ni;
  search_nj_=nj;
}
  
//: Current model data for this searcher
const masm_point_finder_model& masm_ncc_finder::model()
{
  assert(ncc_model_!=0);
  return *ncc_model_;
}

// Assumes im2[i] has zero mean and unit length as a vector
// Assumes element (i,j) is im1[i+j*jstep1] etc
template<class T>
inline float norm_corr(const T* im1, const float* im2,
                        vcl_ptrdiff_t jstep1, vcl_ptrdiff_t jstep2,
                        unsigned ni, unsigned nj)
{
  float sum1=0.0,sum2=0.0,sum_sq=0.0;
  for (unsigned j=0;j<nj;++j,im1+=jstep1,im2+=jstep2)
    for (unsigned i=0;i<ni;++i)
    {
      sum1+=im1[i]*im2[i];
      sum2+=im1[i];
      sum_sq+=im1[i]*im1[i];
    }
  unsigned n=ni*nj;
  float mean = sum2/n;
  float ss = vcl_max(1e-6f,sum_sq-n*mean*mean);
  float s = vcl_sqrt(ss);

  return sum1/s;
}

template<class T>
float evaluate(const vimt_image_2d_of<T>& image,
               const vgl_point_2d<double>& p,
               const vgl_vector_2d<double>& u,
               const vgl_point_2d<double>& ref_pt,
               const vil_image_view<float>& kernel)
{
  vgl_vector_2d<double> v(-u.y(),u.x());

  vil_image_view<float> sample;

  const vgl_point_2d<double> p0 = p-ref_pt.x()*u-ref_pt.y()*v;

  const vimt_transform_2d& s_w2i = image.world2im();
  vgl_point_2d<double> im_p0 = s_w2i(p0);
  vgl_vector_2d<double> im_u = s_w2i.delta(p0, u);
  vgl_vector_2d<double> im_v = s_w2i.delta(p0, v);

  vil_resample_bilin(image.image(),sample,
                     im_p0.x(),im_p0.y(),  im_u.x(),im_u.y(),
                     im_v.x(),im_v.y(),
                     kernel.ni(),kernel.nj());

  return 1.0f-norm_corr(sample.top_left_ptr(),kernel.top_left_ptr(),
                        sample.jstep(),kernel.jstep(),
                        kernel.ni(),kernel.nj());
}

//: Fit at given point and scale/orientation in image.
// \param image: Image to be searched
// \param base_pose: Position and scale/orientation defining initial point around which to search
double masm_ncc_finder::fit_at_point(const vimt_image_2d& image, const mfpf_pose& base_pose)
{
  assert(ncc_model_!=0);
  switch (image.image_base().pixel_format())
  {
    case VIL_PIXEL_FORMAT_BYTE:
      return evaluate(static_cast<const vimt_image_2d_of<vxl_byte>&>(image),base_pose.p(),base_pose.u(),
                      ncc_model_->ref_pt(),ncc_model_->kernel());
      break;
    case VIL_PIXEL_FORMAT_FLOAT:
      return evaluate(static_cast<const vimt_image_2d_of<float>&>(image),base_pose.p(),base_pose.u(),
                      ncc_model_->ref_pt(),ncc_model_->kernel());
      break;
    default:
      vcl_cerr<<"masm_ncc_finder::fit_at_point Unknown pixel format."<<vcl_endl;
      abort();
  }
  return 0;
}

//: Sample region required to do search.
void masm_ncc_finder::sample_region(const vimt_image_2d& image, 
                                    const mfpf_pose& base_pose, vil_image_view<float>& sample)
{
  assert(ncc_model_!=0);
  
  unsigned kni = ncc_model_->kernel().ni();
  unsigned knj = ncc_model_->kernel().nj();
  int nsi = 2*search_ni_ + kni;
  int nsj = 2*search_nj_ + knj;

  vgl_vector_2d<double> u1=base_pose.u();
  vgl_vector_2d<double> v1(-u1.y(),u1.x());
  const vgl_point_2d<double> p0 = base_pose.p()-(search_ni_+ncc_model_->ref_pt().x())*u1
                                               -(search_nj_+ncc_model_->ref_pt().y())*v1;

  const vimt_transform_2d& s_w2i = image.world2im();
  vgl_point_2d<double> im_p0 = s_w2i(p0);
  vgl_vector_2d<double> im_u = s_w2i.delta(p0, u1);
  vgl_vector_2d<double> im_v = s_w2i.delta(p0, v1);
  
  switch (image.image_base().pixel_format())
  {
    case VIL_PIXEL_FORMAT_BYTE:
      vil_resample_bilin(static_cast<const vimt_image_2d_of<vxl_byte>&>(image).image(),sample,
                     im_p0.x(),im_p0.y(),  im_u.x(),im_u.y(), im_v.x(),im_v.y(), nsi,nsj);
      break;
    case VIL_PIXEL_FORMAT_FLOAT:
      vil_resample_bilin(static_cast<const vimt_image_2d_of<float>&>(image).image(),sample,
                     im_p0.x(),im_p0.y(),  im_u.x(),im_u.y(), im_v.x(),im_v.y(), nsi,nsj);
      break;
    default:
      vcl_cerr<<"masm_ncc_finder::fit_at_point Unknown pixel format."<<vcl_endl;
      abort();
  }
 
}


//: Search the given image to find best position for the point
// \param image: Image to be searched
// \param base_pose: Position and scale/orientation defining initial point around which to search
// \param new_pt: New position found by the search
// \param fit: Value describing quality of match (the smaller the better)
void masm_ncc_finder::search(const vimt_image_2d& image, const mfpf_pose& base_pose,
                             vgl_point_2d<double>& new_pt, double& fit)
{
  assert(ncc_model_!=0);
  
  vil_image_view<float> sample;
  sample_region(image,base_pose,sample);
  
  int ni=1+2*search_ni_;
  int nj=1+2*search_nj_;
  unsigned kni = ncc_model_->kernel().ni();
  unsigned knj = ncc_model_->kernel().nj();

  const float* k = ncc_model_->kernel().top_left_ptr();
  const float* s = sample.top_left_ptr();
  vcl_ptrdiff_t s_jstep = sample.jstep();
  vcl_ptrdiff_t k_jstep = ncc_model_->kernel().jstep();

  float best_r=-9e99;
  int best_i=-1,best_j=-1;
  for (int j=0;j<nj;++j,s+=s_jstep)
  {
    for (int i=0;i<ni;++i)
    {
      float r = norm_corr(s+i,k,s_jstep,k_jstep,kni,knj);
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
void masm_ncc_finder::search2(const vimt_image_2d& image, const mfpf_pose& base_pose,
                              vgl_point_2d<double>& new_pt, msm_wt_mat_2d& wt_mat)
{
  assert(ncc_model_!=0);
  
  vil_image_view<float> sample;
  sample_region(image,base_pose,sample);
  
  int ni=1+2*search_ni_;
  int nj=1+2*search_nj_;
  unsigned kni = ncc_model_->kernel().ni();
  unsigned knj = ncc_model_->kernel().nj();

  const float* k = ncc_model_->kernel().top_left_ptr();
  const float* s = sample.top_left_ptr();
  vcl_ptrdiff_t s_jstep = sample.jstep();
  vcl_ptrdiff_t k_jstep = ncc_model_->kernel().jstep();

  float best_r=9e99;
  int best_i=-1,best_j=-1;
  
  vil_image_view<float> fit_im(ni,nj);
  for (int j=0;j<nj;++j,s+=s_jstep)
  {
    for (int i=0;i<ni;++i)
    {
      float r = 1.0-norm_corr(s+i,k,s_jstep,k_jstep,kni,knj);
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

vcl_string masm_ncc_finder::is_a() const
{
  return vcl_string("masm_ncc_finder");
}

  //: Create a copy on the heap and return base class pointer
masm_ncc_finder* masm_ncc_finder::clone() const
{
  return new masm_ncc_finder(*this);
}

  //: Print class to os
void masm_ncc_finder::print_summary(vcl_ostream& os) const
{
}