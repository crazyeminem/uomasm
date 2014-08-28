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

#ifndef masm_lin_reg_finder_h_
#define masm_lin_reg_finder_h_

//:
// \file
// \brief Search for point using linear regression on patch around current point.
// \author Tim Cootes


#include <masm/masm_point_finder.h>
#include <masm/masm_lin_reg_model.h>

//: Search for point using linear regression on patch around current point.
//  Associated with a masm_lin_reg_model containing the data.
class masm_lin_reg_finder : public masm_point_finder
{
protected:
  //: Model containing data for this finder
  const masm_lin_reg_model* lin_reg_model_;
  
  //: Number of steps either side to search along direction pose.u() (in steps of u)
  unsigned search_ni_;
  //: Number of steps either side to search along direction orthogonal to pose.u()
  unsigned search_nj_;
  
  //: Sample region required to do search.
  void sample_region(const vimt_image_2d& image, const mfpf_pose& base_pose, vil_image_view<float>& sample);
  
public:

  //: Dflt ctor
  masm_lin_reg_finder();

  //: Destructor
  virtual ~masm_lin_reg_finder();
  
  //: Define model data for this searcher
  virtual void set_model(const masm_point_finder_model&);
  
  //: Define search area in terms of steps either side of base point
  //  Units of pose.u()
  virtual void set_search_area(unsigned ni, unsigned nj);

  //: Number of steps either side to search along direction pose.u() (in steps of u)
  int search_ni() const { return search_ni_; }
  
  //: Number of steps either side to search along direction orthogonal to pose.u()
  int search_nj() const { return search_nj_; }
  
  //: Current model data for this searcher
  virtual const masm_point_finder_model& model() ;

  //: Fit at given point and scale/orientation in image.
  // \param image: Image to be searched
  // \param base_pose: Position and scale/orientation defining initial point around which to search
  virtual double fit_at_point(const vimt_image_2d& image, const mfpf_pose& base_pose);

  //: Search the given image to find best position for the point
  // \param image: Image to be searched
  // \param base_pose: Position and scale/orientation defining initial point around which to search
  // \param new_pt: New position found by the search
  // \param fit: Value describing quality of match (the smaller the better)
  virtual void search(const vimt_image_2d& image, const mfpf_pose& base_pose,
                      vgl_point_2d<double>& new_pt, double& fit);

  //: Search the given image to find best position for the point
  // \param image: Image to be searched
  // \param base_pose: Position and scale/orientation defining initial point around which to search
  // \param new_pt: New position found by the search
  // \param wt_mat: Describes inverse co-variance of estimate of position
  virtual void search2(const vimt_image_2d& image, const mfpf_pose& base_pose,
                       vgl_point_2d<double>& new_pt, msm_wt_mat_2d& wt_mat);

  //: Name of the class
  virtual vcl_string is_a() const;

  //: Create a copy on the heap and return base class pointer
  virtual masm_lin_reg_finder* clone() const;

  //: Print class to os
  virtual void print_summary(vcl_ostream& os) const;

};

#endif


