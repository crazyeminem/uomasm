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
// \brief Search for point using the PDF of region around point.
// \author Tim Cootes

#include "masm_region_pdf_finder.h"

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

//=======================================================================
// Dflt ctor
//=======================================================================

masm_region_pdf_finder::masm_region_pdf_finder()
  : region_pdf_model_(0), search_ni_(5), search_nj_(5)
{
}

//=======================================================================
// Destructor
//=======================================================================

masm_region_pdf_finder::~masm_region_pdf_finder()
{
}

//: Define model data for this searcher
void masm_region_pdf_finder::set_model(const masm_point_finder_model& model)
{
  assert(model.is_a()=="masm_region_pdf_model");
  region_pdf_model_ = &(static_cast<const masm_region_pdf_model&>(model));
  
  search_ni_=region_pdf_model_->search_ni();
  search_nj_=region_pdf_model_->search_nj();
}

//: Define search area in terms of steps either side of base point
//  Units of pose.u()
void masm_region_pdf_finder::set_search_area(unsigned ni, unsigned nj)
{
  search_ni_=ni;
  search_nj_=nj;
}
  
//: Current model data for this searcher
const masm_point_finder_model& masm_region_pdf_finder::model()
{
  assert(region_pdf_model_!=0);
  return *region_pdf_model_;
}

//: Fit at given point and scale/orientation in image.
// \param image: Image to be searched
// \param base_pose: Position and scale/orientation defining initial point around which to search
double masm_region_pdf_finder::fit_at_point(const vimt_image_2d& image, const mfpf_pose& base_pose)
{
  assert(region_pdf_model_!=0);

  vgl_vector_2d<double> u1=base_pose.u();
  vgl_vector_2d<double> v1(-u1.y(),u1.x());

  unsigned np=image.image_base().nplanes();
  
  // Set up sample area with interleaved planes (ie planestep==1)
  vil_image_view<float> sample(region_pdf_model_->roi_ni(),region_pdf_model_->roi_nj(),1,np);

  const vgl_point_2d<double>& ref_pt = region_pdf_model_->ref_pt();

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
                     im_v.x(),im_v.y(), region_pdf_model_->roi_ni(),region_pdf_model_->roi_nj());
      break;
    case VIL_PIXEL_FORMAT_FLOAT:
       vil_resample_bilin(static_cast<const vimt_image_2d_of<float>&>(image).image(),sample,
                     im_p0.x(),im_p0.y(),  im_u.x(),im_u.y(),
                     im_v.x(),im_v.y(), region_pdf_model_->roi_ni(),region_pdf_model_->roi_nj());
      break;
    default:
      vcl_cerr<<"masm_region_pdf_finder::fit_at_point Unknown pixel format."<<vcl_endl;
      abort();
  }

  vnl_vector<double> v(region_pdf_model_->n_pixels()*sample.nplanes());
  mfpf_sample_region(sample.top_left_ptr(),sample.jstep(),
                     np,region_pdf_model_->roi(),v);

  if (region_pdf_model_->norm_method()==1) mfpf_norm_vec(v);
  return float(-1*region_pdf_model_->pdf().log_p(v));
}

//: Sample region required to do search.
void masm_region_pdf_finder::sample_region(const vimt_image_2d& image, 
                                    const mfpf_pose& base_pose, vil_image_view<float>& sample)
{
  assert(region_pdf_model_!=0);
  
  unsigned kni = region_pdf_model_->roi_ni();
  unsigned knj = region_pdf_model_->roi_nj();
  int nsi = 2*search_ni_ + kni;
  int nsj = 2*search_nj_ + knj;

  vgl_vector_2d<double> u1=base_pose.u();
  vgl_vector_2d<double> v1(-u1.y(),u1.x());
  const vgl_point_2d<double> p0 = base_pose.p()-(search_ni_+region_pdf_model_->ref_pt().x())*u1
                                               -(search_nj_+region_pdf_model_->ref_pt().y())*v1;

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
      vcl_cerr<<"masm_region_pdf_finder::fit_at_point Unknown pixel format."<<vcl_endl;
      abort();
  }
  // WARNING: Currently only deals with single plane images.
}


//: Search the given image to find best position for the point
// \param image: Image to be searched
// \param base_pose: Position and scale/orientation defining initial point around which to search
// \param new_pt: New position found by the search
// \param fit: Value describing quality of match (the smaller the better)
void masm_region_pdf_finder::search(const vimt_image_2d& image, const mfpf_pose& base_pose,
                             vgl_point_2d<double>& new_pt, double& fit)
{
  assert(region_pdf_model_!=0);
  
  vil_image_view<float> sample;
  sample_region(image,base_pose,sample);
  
  int ni=1+2*search_ni_;
  int nj=1+2*search_nj_;
  unsigned kni = region_pdf_model_->roi_ni();
  unsigned knj = region_pdf_model_->roi_nj();

  const float* s = sample.top_left_ptr();
  vcl_ptrdiff_t s_jstep = sample.jstep();
  
  unsigned np=image.image_base().nplanes();
  vnl_vector<double> v(region_pdf_model_->n_pixels()*np);


  float best_r=9e9f;
  int best_i=-1,best_j=-1;
  for (int j=0;j<nj;++j,s+=s_jstep)
  {
    for (int i=0;i<ni;++i)
    {
      mfpf_sample_region(s+i*np,s_jstep,np,region_pdf_model_->roi(),v);
      if (region_pdf_model_->norm_method()==1) mfpf_norm_vec(v);
      float r = float(-1*region_pdf_model_->pdf().log_p(v));
      if (r<best_r) { best_r=r; best_i=i; best_j=j; }
    }
  }

  // Compute position of best point
  new_pt = base_pose(best_i-double(search_ni_),best_j-double(search_nj_));
  fit = best_r;
}

//: Search the given image to find best position for the point
// \param image: Image to be searched
// \param base_pose: Position and scale/orientation defining initial point around which to search
// \param new_pt: New position found by the search
// \param wt_mat: Describes inverse co-variance of estimate of position
void masm_region_pdf_finder::search2(const vimt_image_2d& image, const mfpf_pose& base_pose,
                              vgl_point_2d<double>& new_pt, msm_wt_mat_2d& wt_mat)
{
  assert(region_pdf_model_!=0);
  
  vil_image_view<float> sample;
  sample_region(image,base_pose,sample);
  
  int ni=1+2*search_ni_;
  int nj=1+2*search_nj_;
  unsigned kni = region_pdf_model_->roi_ni();
  unsigned knj = region_pdf_model_->roi_nj();

  const float* s = sample.top_left_ptr();
  vcl_ptrdiff_t s_jstep = sample.jstep();
  
  unsigned np=image.image_base().nplanes();
  vnl_vector<double> v(region_pdf_model_->n_pixels()*np);

  vil_image_view<float> fit_im(ni,nj);

  float best_r=9e9f;
  int best_i=-1,best_j=-1;
  for (int j=0;j<nj;++j,s+=s_jstep)
  {
    for (int i=0;i<ni;++i)
    {
      mfpf_sample_region(s+i*np,s_jstep,np,region_pdf_model_->roi(),v);
      if (region_pdf_model_->norm_method()==1) mfpf_norm_vec(v);
      float r = float(-1*region_pdf_model_->pdf().log_p(v));
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

vcl_string masm_region_pdf_finder::is_a() const
{
  return vcl_string("masm_region_pdf_finder");
}

  //: Create a copy on the heap and return base class pointer
masm_region_pdf_finder* masm_region_pdf_finder::clone() const
{
  return new masm_region_pdf_finder(*this);
}

  //: Print class to os
void masm_region_pdf_finder::print_summary(vcl_ostream& os) const
{
}