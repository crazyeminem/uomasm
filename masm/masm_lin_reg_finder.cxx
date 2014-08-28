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
// \brief Search for point using linear regression on patch around current point.
// \author Tim Cootes

#include "masm_lin_reg_finder.h"

#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>
#include <mfpf/mfpf_pose.h>
#include <vil/vil_resample_bilin.h>
#include <vimt/vimt_image_2d_of.h>
#include <msm/msm_wt_mat_2d.h>
#include <vcl_cassert.h>
#include <vcl_algorithm.h>

#include <mfpf/mfpf_sample_region.h>
#include <mfpf/mfpf_norm_vec.h>
#include <mbl/mbl_stats_1d.h>

//=======================================================================
// Dflt ctor
//=======================================================================

masm_lin_reg_finder::masm_lin_reg_finder()
  : lin_reg_model_(0), search_ni_(5), search_nj_(5)
{
}

//=======================================================================
// Destructor
//=======================================================================

masm_lin_reg_finder::~masm_lin_reg_finder()
{
}

//: Define model data for this searcher
void masm_lin_reg_finder::set_model(const masm_point_finder_model& model)
{
  assert(model.is_a()=="masm_lin_reg_model");
  lin_reg_model_ = &(static_cast<const masm_lin_reg_model&>(model));
  
  search_ni_=lin_reg_model_->search_ni();
  search_nj_=lin_reg_model_->search_nj();
}

//: Define search area in terms of steps either side of base point
//  Units of pose.u()
void masm_lin_reg_finder::set_search_area(unsigned ni, unsigned nj)
{
  search_ni_=ni;
  search_nj_=nj;
}
  
//: Current model data for this searcher
const masm_point_finder_model& masm_lin_reg_finder::model()
{
  assert(lin_reg_model_!=0);
  return *lin_reg_model_;
}

//: Fit at given point and scale/orientation in image.
// \param image: Image to be searched
// \param base_pose: Position and scale/orientation defining initial point around which to search
double masm_lin_reg_finder::fit_at_point(const vimt_image_2d& image, const mfpf_pose& base_pose)
{
  assert(lin_reg_model_!=0);

  vgl_vector_2d<double> u1=base_pose.u();
  vgl_vector_2d<double> v1(-u1.y(),u1.x());

  unsigned np=image.image_base().nplanes();
  
  // Set up sample area with interleaved planes (ie planestep==1)
  vil_image_view<float> sample(lin_reg_model_->roi_ni(),lin_reg_model_->roi_nj(),1,np);

  const vgl_point_2d<double>& ref_pt = lin_reg_model_->ref_pt();

  const vgl_point_2d<double> p0 = base_pose.p()-ref_pt.x()*u1-ref_pt.y()*v1;

  const vimt_transform_2d& s_w2i = image.world2im();
  vgl_point_2d<double> im_p0 = s_w2i(p0);
  vgl_vector_2d<double> im_u = s_w2i.delta(p0, u1);
  vgl_vector_2d<double> im_v = s_w2i.delta(p0, v1);

  switch (image.image_base().pixel_format())
  {
    case VIL_PIXEL_FORMAT_BYTE:
      vil_resample_bilin(static_cast<const vimt_image_2d_of<vxl_byte>&>(image).image(),sample,
                     im_p0.x(),im_p0.y(),  im_u.x(),im_u.y(),
                     im_v.x(),im_v.y(), lin_reg_model_->roi_ni(),lin_reg_model_->roi_nj());
      break;
    case VIL_PIXEL_FORMAT_FLOAT:
       vil_resample_bilin(static_cast<const vimt_image_2d_of<float>&>(image).image(),sample,
                     im_p0.x(),im_p0.y(),  im_u.x(),im_u.y(),
                     im_v.x(),im_v.y(), lin_reg_model_->roi_ni(),lin_reg_model_->roi_nj());
      break;
    default:
      vcl_cerr<<"masm_lin_reg_finder::fit_at_point Unknown pixel format."<<vcl_endl;
      abort();
  }

  vnl_vector<double> v(lin_reg_model_->n_pixels()*sample.nplanes());
  mfpf_sample_region(sample.top_left_ptr(),sample.jstep(),
                     np,lin_reg_model_->roi(),v);

  if (lin_reg_model_->norm_method()==1) mfpf_norm_vec(v);
  
  vgl_vector_2d<double> dp = lin_reg_model_->offset_prediction(v);
  return dp.length();  // Small is good!
}

//: Sample region required to do search.
void masm_lin_reg_finder::sample_region(const vimt_image_2d& image, 
                                    const mfpf_pose& base_pose, vil_image_view<float>& sample)
{
  assert(lin_reg_model_!=0);
  
  unsigned kni = lin_reg_model_->roi_ni();
  unsigned knj = lin_reg_model_->roi_nj();
  int nsi = 2*search_ni_ + kni;
  int nsj = 2*search_nj_ + knj;

  vgl_vector_2d<double> u1=base_pose.u();
  vgl_vector_2d<double> v1(-u1.y(),u1.x());
  const vgl_point_2d<double> p0 = base_pose.p()-(search_ni_+lin_reg_model_->ref_pt().x())*u1
                                               -(search_nj_+lin_reg_model_->ref_pt().y())*v1;

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
      vcl_cerr<<"masm_lin_reg_finder::fit_at_point Unknown pixel format."<<vcl_endl;
      abort();
  }
  // WARNING: Currently only deals with single plane images.
}


//: Search the given image to find best position for the point
// \param image: Image to be searched
// \param base_pose: Position and scale/orientation defining initial point around which to search
// \param new_pt: New position found by the search
// \param fit: Value describing quality of match (the smaller the better)
void masm_lin_reg_finder::search(const vimt_image_2d& image, const mfpf_pose& base_pose,
                             vgl_point_2d<double>& new_pt, double& fit)
{
  assert(lin_reg_model_!=0);
  
  vil_image_view<float> sample;
  sample_region(image,base_pose,sample);
  
  int ni=1+2*search_ni_;
  int nj=1+2*search_nj_;
  unsigned kni = lin_reg_model_->roi_ni();
  unsigned knj = lin_reg_model_->roi_nj();

  const float* s = sample.top_left_ptr();
  vcl_ptrdiff_t s_jstep = sample.jstep();
  
  unsigned np=image.image_base().nplanes();
  vnl_vector<double> v(lin_reg_model_->n_pixels()*np);

  // Use prediction from every nearby point
  vgl_vector_2d<double> sum_dp(0,0);
  for (int j=0;j<nj;++j,s+=s_jstep)
  {
    for (int i=0;i<ni;++i)
    {
      mfpf_sample_region(s+i*np,s_jstep,np,lin_reg_model_->roi(),v);
      if (lin_reg_model_->norm_method()==1) mfpf_norm_vec(v);
      
      vgl_vector_2d<double> dp = lin_reg_model_->offset_prediction(v);
      dp += vgl_vector_2d<double>(i-double(search_ni_),j-double(search_nj_));
      sum_dp += dp;
    }
  }
  
  // Compute the mean
  sum_dp/=(ni*nj);

  // Compute position of best point
  new_pt = base_pose(sum_dp.x(),sum_dp.y());
  fit = 1.0;  // Should calculate this properly somehow.
}

//: Search the given image to find best position for the point
// \param image: Image to be searched
// \param base_pose: Position and scale/orientation defining initial point around which to search
// \param new_pt: New position found by the search
// \param wt_mat: Describes inverse co-variance of estimate of position
void masm_lin_reg_finder::search2(const vimt_image_2d& image, const mfpf_pose& base_pose,
                              vgl_point_2d<double>& new_pt, msm_wt_mat_2d& wt_mat)
{
  assert(lin_reg_model_!=0);
  
  vil_image_view<float> sample;
  sample_region(image,base_pose,sample);
  
  int ni=1+2*search_ni_;
  int nj=1+2*search_nj_;
  unsigned kni = lin_reg_model_->roi_ni();
  unsigned knj = lin_reg_model_->roi_nj();

  const float* s = sample.top_left_ptr();
  vcl_ptrdiff_t s_jstep = sample.jstep();
  
  unsigned np=image.image_base().nplanes();
  vnl_vector<double> v(lin_reg_model_->n_pixels()*np);

  // Compute stats of predictions along axes (eventually replace with full covar.)
  mbl_stats_1d dx_stats,dy_stats;
  
  for (int j=0;j<nj;++j,s+=s_jstep)
  {
    for (int i=0;i<ni;++i)
    {
      mfpf_sample_region(s+i*np,s_jstep,np,lin_reg_model_->roi(),v);
      if (lin_reg_model_->norm_method()==1) mfpf_norm_vec(v);
      vgl_vector_2d<double> dp = lin_reg_model_->offset_prediction(v);
// vcl_cout<<"From ("<<i<<","<<j<<") : "<<dp.x()<<","<<dp.y()<<vcl_endl;

      dx_stats.obs(dp.x()+i-double(search_ni_));
      dy_stats.obs(dp.y()+j-double(search_nj_));
    }
  }

  // Compute position of best point
  new_pt = base_pose(dx_stats.mean(),dy_stats.mean());
  
  // Estimate weights, assuming that the shape of the fit surface is a quadratic
  // aligned with the axes.
  
  double wi=1,wj=1;
  if (dx_stats.nObs()>1)
  {
    wi = 1.0/vcl_max(0.1,dx_stats.variance());
    wj = 1.0/vcl_max(0.1,dy_stats.variance());
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

vcl_string masm_lin_reg_finder::is_a() const
{
  return vcl_string("masm_lin_reg_finder");
}

  //: Create a copy on the heap and return base class pointer
masm_lin_reg_finder* masm_lin_reg_finder::clone() const
{
  return new masm_lin_reg_finder(*this);
}

  //: Print class to os
void masm_lin_reg_finder::print_summary(vcl_ostream& os) const
{
}