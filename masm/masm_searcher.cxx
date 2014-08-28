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
#include "masm_searcher.h"
//:
// \file
// \brief Representation of an instance of an Active Shape Model.
// \author Tim Cootes

#include <vcl_iostream.h>
#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_io.h>

#include <vcl_cstdlib.h>  // for vcl_atoi() & vcl_abort()
#include <vimt/vimt_image_2d.h>

//=======================================================================
// Dflt ctor
//=======================================================================

masm_searcher::masm_searcher()
  : model_(0),max_its_(5),use_wt_mat_in_search_(false),wt_for_fixed_(100)
{
}

//: Set up model (retains pointer to model)
masm_searcher::masm_searcher(const masm_model& model)
  : model_(0)
{
  set_asm(model);
}

//=======================================================================
// Destructor
//=======================================================================

masm_searcher::~masm_searcher()
{
}

//: Set up model (retains pointer to model)
void masm_searcher::set_asm(const masm_model& model)
{
  model_ = &model;
  set_shape_model(model.shape_model());
  unsigned n_pts = model.shape_model().size();
  finders_.resize(n_pts);
  for (unsigned i=0;i<n_pts;++i) finders_[i]=model.finder(i).create_finder();
  
  wt_for_fixed_=model.wt_for_fixed();
}

//: When true, use estimate of covariance in point positions when updating model
//  Allows points to slide along boundaries more easily.
void masm_searcher::set_use_wt_mat_in_search(bool b)
{
  use_wt_mat_in_search_=b;
}

//: Set weight to use for fixed points
void masm_searcher::set_wt_for_fixed(double w)
{
  wt_for_fixed_=w;
}
  
//: Select best level for searching around pose with finder
//  Selects pyramid level with pixel sizes best matching
//  the model pixel size at given pose.
unsigned image_level(const mfpf_pose& pose, const vimt_image_pyramid& im_pyr)
{
  double model_pixel_size = pose.scale();
  double rel_size0 = model_pixel_size/im_pyr.base_pixel_width();

  double log_step = vcl_log(im_pyr.scale_step());

  // Round level down, to work with slightly higher res. image.
  int level = int(vcl_log(rel_size0)/log_step);
  if      (level<im_pyr.lo()) return im_pyr.lo();
  else if (level>im_pyr.hi()) return im_pyr.hi();
  else                        return level;
}


//: Perform local search for each point that isn't fixed (ie those for which fixed[i]==false)
//  Results recorded in found_points_
void masm_searcher::find_points(const vimt_image_pyramid& image_pyr, const vcl_vector<bool>& fixed)
{
  unsigned n_points = points().size();
  assert(fixed.size()==n_points);
  vcl_vector<mfpf_pose> frame_pose(n_points);
  model().frame_maker().create_frames(*this,frame_pose);
  found_points_.set_size(n_points);

  vgl_point_2d<double> new_p;
  double fit;
  
  unsigned L=image_level(frame_pose[0],image_pyr);  // Assume all frame_poses define same scale.
  const vimt_image_2d& imageL = static_cast<const vimt_image_2d&>(image_pyr(L));
  
  for (unsigned j=0;j<n_points;++j)
  {
    if (fixed[j]) continue;  // Ignore fixed points
    finders_[j]->search(imageL,frame_pose[j],new_p,fit);
    found_points_.set_point(j,new_p.x(),new_p.y());
  }
}

//: Perform one update step (search then fit) 
void masm_searcher::update_step(const vimt_image_pyramid& image_pyr)
{
  vcl_vector<bool> fixed(points().size(),false);
  find_points(image_pyr,fixed);
  fit_to_points(found_points_);
}

//: Perform search on given image, without constraints.
//  Calls update_step(image_pyr) at most max_its times.
void masm_searcher::search(const vimt_image_pyramid& image_pyr)
{
  vcl_vector<bool> none_fixed;
  for (unsigned i=0;i<max_its_;++i)
  {
    if (use_wt_mat_in_search_)
      update_with_wt_mat(image_pyr,points(),none_fixed);
    else
      update_step(image_pyr);
  }

}


//: Perform one update step (search for best local matches, then fit shape), allowing fixed points
//  If fixed[i]==true, then don't search for point i, assume it to be at fixed_points[i]
//  Note that on completion points()[i] will be the best shape model approximation to fixed_points[i],
//  but won't necessarily be exactly that value.
void masm_searcher::update_with_fixed(const vimt_image_pyramid& image_pyr,
                         const msm_points& fixed_points,
                         const vcl_vector<bool>& fixed)
{
  find_points(image_pyr,fixed);
  vnl_vector<double> wts(fixed.size(),1.0);
  for (unsigned i=0;i<fixed.size();++i)
  {
    if (fixed[i])
    {
      vgl_point_2d<double> fp = fixed_points[i];
      found_points_.set_point(i,fp.x(),fp.y());
      wts[i]=wt_for_fixed_; 
    }
  }
  
  fit_to_points_wt(found_points_,wts);
}

//: Perform search, allowing fixed points
//  If fixed[i]==true, then don't search for point i, assume it to be at fixed_points[i]
//  Note that on completion points()[i] will be the best shape model approximation to fixed_points[i],
//  but won't necessarily be exactly that value.
void masm_searcher::search_with_fixed(const vimt_image_pyramid& image_pyr,
                         const msm_points& fixed_points,
                         const vcl_vector<bool>& fixed)
{
  for (unsigned i=0;i<max_its_;++i)
  {
    if (use_wt_mat_in_search_)
      update_with_wt_mat(image_pyr,fixed_points,fixed);
    else
      update_with_fixed(image_pyr,fixed_points,fixed);
  }
}


//: Perform one update step (search for best local matches, then fit shape), taking uncertainty into account
//  Local model search includes estimate of uncertainty, encoded in a weight matrix (inverse covar).
//  If fixed[i]==true, then don't search for point i, assume it to be at fixed_points[i]
//  Note that on completion points()[i] will be the best shape model approximation to fixed_points[i],
//  but won't necessarily be exactly that value.
void masm_searcher::update_with_wt_mat(const vimt_image_pyramid& image_pyr,
                         const msm_points& fixed_points,
                         const vcl_vector<bool>& fixed)
{
  unsigned n_points = points().size();
  assert(fixed.size()==n_points || fixed.size()==0);
  vcl_vector<mfpf_pose> frame_pose(n_points);
  model().frame_maker().create_frames(*this,frame_pose);
  found_points_.set_size(n_points);
  
  vcl_vector<msm_wt_mat_2d> wt_mat(n_points);

  vgl_point_2d<double> new_p;
  
  unsigned L=image_level(frame_pose[0],image_pyr);  // Assume all frame_poses define same scale.
  const vimt_image_2d& imageL = static_cast<const vimt_image_2d&>(image_pyr(L));
  
  for (unsigned j=0;j<n_points;++j)
  {
    if (fixed.size()>0 && fixed[j])
    {
      new_p = fixed_points[j];
      wt_mat[j] = msm_wt_mat_2d(wt_for_fixed_,0,wt_for_fixed_);  
    }
    else finders_[j]->search2(imageL,frame_pose[j],new_p,wt_mat[j]);
    
    found_points_.set_point(j,new_p.x(),new_p.y());
  }

  fit_to_points_wt_mat(found_points_,wt_mat);
}

//=======================================================================
// Method: is_a
//=======================================================================

vcl_string masm_searcher::is_a() const
{
  return vcl_string("masm_searcher");
}

//=======================================================================
// Method: print
//=======================================================================

  // required if data is present in this class
void masm_searcher::print_summary(vcl_ostream& os) const
{
}

//=======================================================================
// Method: save
//=======================================================================

  // required if data is present in this class
void masm_searcher::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,short(1));  // Version number
  msm_shape_instance::b_write(bfs);
  vsl_b_write(bfs,max_its_); 
  vsl_b_write(bfs,use_wt_mat_in_search_); 
  vsl_b_write(bfs,wt_for_fixed_);
}

//=======================================================================
// Method: load
//=======================================================================

  // required if data is present in this class
void masm_searcher::b_read(vsl_b_istream& bfs)
{
  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      msm_shape_instance::b_read(bfs);
      vsl_b_read(bfs,max_its_); 
      vsl_b_read(bfs,use_wt_mat_in_search_); 
      vsl_b_read(bfs,wt_for_fixed_); 
      break;
    default:
      vcl_cerr << "masm_searcher::b_read() :\n"
               << "Unexpected version number " << version << vcl_endl;
      bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
      return;
  }
}


//=======================================================================
// Associated function: operator<<
//=======================================================================

void vsl_b_write(vsl_b_ostream& bfs, const masm_searcher& b)
{
  b.b_write(bfs);
}

//=======================================================================
// Associated function: operator>>
//=======================================================================

void vsl_b_read(vsl_b_istream& bfs, masm_searcher& b)
{
  b.b_read(bfs);
}

//=======================================================================
// Associated function: operator<<
//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const masm_searcher& b)
{
  os << b.is_a() << ": ";
  vsl_indent_inc(os);
  b.print_summary(os);
  vsl_indent_dec(os);
  return os;
}

//: Stream output operator for class reference
void vsl_print_summary(vcl_ostream& os,const masm_searcher& b)
{
 os << b;
}
