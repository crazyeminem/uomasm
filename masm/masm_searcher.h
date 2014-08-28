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

#ifndef masm_searcher_h_
#define masm_searcher_h_
//:
// \file
// \brief Representation of an instance of an Active Shape Model.
// \author Tim Cootes

/*
Comments:
- Measure change in update_step
- Test for convergence?
- Record uncertainty on found points and use.
*/

#include <msm/msm_shape_instance.h>
#include <masm/masm_model.h>
#include <masm/masm_point_finder.h>
#include <vimt/vimt_image_pyramid.h>

//: Representation of an instance of an Active Shape Model.
//  Contains shape model parameters and the parameters of
//  the global (model to world) transformation.
//  Includes functions to fit instances to sets of points,
//  to generate sets of points, and to search an image.
//
//  By default, all shape parameters are used and params() returns 
//  a vector of length equal to the full number of shape modes.
//  To use fewer modes, create a parameter vector with the desired
//  number, and call set_params(b).
//
//  points() and model_points() use lazy evaluation
class masm_searcher : public msm_shape_instance
{
 private:
  //: Pointer to external model
  const masm_model* model_;
  
  vcl_vector<mbl_cloneable_ptr<masm_point_finder> > finders_;
  
  //: Maximum number of iterations to run during search.
  unsigned max_its_;
  
  //: When true, use estimate of covariance in point positions when updating model
  //  Allows points to slide along boundaries more easily.
  bool use_wt_mat_in_search_;
  
  //: Weight to use for fixed points
  double wt_for_fixed_;

  //: Points found by last call to find_points()
  msm_points found_points_;

  //: Perform local search for each point that isn't fixed (ie those for which fixed[i]==false)
  //  Results recorded in found_points_
  void find_points(const vimt_image_pyramid& image_pyr, const vcl_vector<bool>& fixed);

 public:

  // Dflt ctor
  masm_searcher();

  //: Set up model (retains pointer to model)
  masm_searcher(const masm_model& model);

  // Destructor
  ~masm_searcher();

  //: Set up model (retains pointer to model)
  void set_asm(const masm_model& model);
  
  //: Maximum number of iterations to run during search.
  unsigned max_its() const { return max_its_; }

  //: Maximum number of iterations to run during search.
  void set_max_its(unsigned max_its) { max_its_=max_its; }

  //: When true, use estimate of covariance in point positions when updating model
  //  Allows points to slide along boundaries more easily.
  void set_use_wt_mat_in_search(bool b);
  
  //: Set weight to use for fixed points
  void set_wt_for_fixed(double w);

  //: Pointer to current shape model
  const masm_model* model_ptr() const
  {
    return model_;
  }
  
  //: Return true if model defined.
  bool got_model() const { return model_!=0; }

  //: Reference to current model
  const masm_model& model() const 
  { return *model_; }

  //: Points found by last call to find_points()
  const msm_points& found_points() const { return found_points_; }

  //: Perform one update step (search for best local matches, then fit shape) 
  void update_step(const vimt_image_pyramid& image_pyr);

  //: Perform search on given image, without constraints.
  //  Applies an update step at most max_its times.
  //  If use_wt_mat_in_search_ then takes covariance of local matches into account.
  void search(const vimt_image_pyramid& image_pyr);
  
  //: Perform one update step (search for best local matches, then fit shape), allowing fixed points
  //  If fixed[i]==true, then don't search for point i, assume it to be at fixed_points[i]
  //  Note that on completion points()[i] will be the best shape model approximation to fixed_points[i],
  //  but won't necessarily be exactly that value.
  void update_with_fixed(const vimt_image_pyramid& image_pyr,
                         const msm_points& fixed_points,
                         const vcl_vector<bool>& fixed);
  
  //: Perform one update step (search for best local matches, then fit shape), taking uncertainty into account
  //  Local model search includes estimate of uncertainty, encoded in a weight matrix (inverse covar).
  //  If fixed[i]==true, then don't search for point i, assume it to be at fixed_points[i]
  //  Note that on completion points()[i] will be the best shape model approximation to fixed_points[i],
  //  but won't necessarily be exactly that value.
  void update_with_wt_mat(const vimt_image_pyramid& image_pyr,
                         const msm_points& fixed_points,
                         const vcl_vector<bool>& fixed);

  //: Perform search, allowing fixed points
  //  If fixed[i]==true, then don't search for point i, assume it to be at fixed_points[i]
  //  Note that on completion points()[i] will be the best shape model approximation to fixed_points[i],
  //  but won't necessarily be exactly that value.
  //  If use_wt_mat_in_search_ then takes covariance of local matches into account.
  void search_with_fixed(const vimt_image_pyramid& image_pyr,
                         const msm_points& fixed_points,
                         const vcl_vector<bool>& fixed);

  //: Name of the class
  vcl_string is_a() const;

  //: Print class to os
  void print_summary(vcl_ostream& os) const;

  //: Save class to binary file stream
  void b_write(vsl_b_ostream& bfs) const;

  //: Load class from binary file stream
  void b_read(vsl_b_istream& bfs);
};


//: Binary file stream output operator for class reference
void vsl_b_write(vsl_b_ostream& bfs, const masm_searcher& pts);


//: Binary file stream input operator for class reference
void vsl_b_read(vsl_b_istream& bfs, masm_searcher& pts);

//: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const masm_searcher& pts);

//: Stream output operator for class reference
void vsl_print_summary(vcl_ostream& os,const masm_searcher& pts);

#endif // masm_searcher_h_
