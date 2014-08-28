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

#ifndef masm_grad_corr_model_h_
#define masm_grad_corr_model_h_

//:
// \file
// \brief Data for normalised cross-correlation model
// \author Tim Cootes

#include <masm/masm_point_finder_model.h>
#include <vimt/vimt_image_2d_of.h>
#include <mfpf/mfpf_pose.h>

//: Data for cross-correlation of gradient information.
class masm_grad_corr_model : public masm_point_finder_model
{
protected:
  //: Kernel reference point (in kernel_ image frame)
  vgl_point_2d<double> ref_pt_;

  //: Filter kernel to search with
  //  Kernel assumed to be two plane image containing x and y gradient information.
  vil_image_view<float> kernel_;
  
  //: Number of steps either side to search along direction pose.u() (in steps of u)
  unsigned search_ni_;
  //: Number of steps either side to search along direction orthogonal to pose.u()
  unsigned search_nj_;

public:

  //: Dflt ctor
  masm_grad_corr_model();

  //: Destructor
  virtual ~masm_grad_corr_model();
  
  //: Creates a finder on the heap and sets this as its model
  virtual masm_point_finder* create_finder() const;
  
  //: Define kernel and the reference point within it.
  void set(const vil_image_view<float>& kernel, const vgl_point_2d<double>& ref_pt);

  //: Define search area in terms of steps either side of base point
  //  Units of pose.u()
  virtual void set_search_area(unsigned ni, unsigned nj);

  //: Filter kernel to search with
  const vil_image_view<float>& kernel() const { return kernel_; }

  //: Kernel reference point (in kernel_ image frame)
  const vgl_point_2d<double>& ref_pt() const { return ref_pt_; }

  //: Number of steps either side to search along direction pose.u() (in steps of u)
  int search_ni() const { return search_ni_; }
  
  //: Number of steps either side to search along direction orthogonal to pose.u()
  int search_nj() const { return search_nj_; }

  //: Generate points in ref frame that represent boundary
  //  Points of a contour around the shape.
  //  Used for display purposes.  Join the points with an open
  //  contour to get a representation.
  virtual void get_outline(vcl_vector<vgl_point_2d<double> >& pts) const;

  //: Name of the class
  virtual vcl_string is_a() const;

  //: Create a copy on the heap and return base class pointer
  virtual masm_point_finder_model* clone() const;

  //: Print class to os
  virtual void print_summary(vcl_ostream& os) const;

  //: Save class to binary file stream
  virtual void b_write(vsl_b_ostream& bfs) const;

  //: Load class from binary file stream
  virtual void b_read(vsl_b_istream& bfs);

};

//: Sample region required to do search.
//  Computes unit gradient direction at each point.
void masm_sample_grad_region(const vimt_image_2d& image, 
                                    const mfpf_pose& base_pose,
                                    unsigned kernel_ni, unsigned kernel_nj,
                                    const vgl_point_2d<double>& ref_pt,
                                    unsigned search_ni, unsigned search_nj,
                                    vil_image_view<float>& sample);


#endif


