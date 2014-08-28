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

#ifndef masm_point_finder_h_
#define masm_point_finder_h_

//:
// \file
// \brief Base for classes which search for individual model points.
// \author Tim Cootes

#include <vcl_vector.h>
#include <vcl_string.h>
#include <vcl_cassert.h>
#include <vcl_memory.h>
#include <vsl/vsl_binary_io.h>

#include <mfpf/mfpf_pose.h>

class vimt_image_2d;
class masm_point_finder_model;
class msm_wt_mat_2d;

//: Base for classes which search for individual model points.
//  Each is associated with a masm_point_finder_model containing the data.
class masm_point_finder
{
protected:

public:

  //: Dflt ctor
  masm_point_finder();

  //: Destructor
  virtual ~masm_point_finder();
  
  //: Define model data for this searcher
  virtual void set_model(const masm_point_finder_model&)=0;
  
  //: Current model data for this searcher
  virtual const masm_point_finder_model& model() = 0;
  
  //: Define search area in terms of steps either side of base point
  //  Units of pose.u()
  virtual void set_search_area(unsigned ni, unsigned nj)=0;

  //: Fit at given point and scale/orientation in image.
  // \param image: Image to be searched
  // \param base_pose: Position and scale/orientation defining initial point around which to search
  virtual double fit_at_point(const vimt_image_2d& image, const mfpf_pose& base_pose) = 0;

  //: Search the given image to find best position for the point
  // \param image: Image to be searched
  // \param base_pose: Position and scale/orientation defining initial point around which to search
  // \param new_pt: New position found by the search
  // \param fit: Value describing quality of match (the smaller the better)
  virtual void search(const vimt_image_2d& image, const mfpf_pose& base_pose,
                      vgl_point_2d<double>& new_pt, double& fit) = 0;

  //: Search the given image to find best position for the point
  // \param image: Image to be searched
  // \param base_pose: Position and scale/orientation defining initial point around which to search
  // \param new_pt: New position found by the search
  // \param wt_mat: Describes inverse co-variance of estimate of position
  virtual void search2(const vimt_image_2d& image, const mfpf_pose& base_pose,
                       vgl_point_2d<double>& new_pt, msm_wt_mat_2d& wt_mat) = 0;

  //: Name of the class
  virtual vcl_string is_a() const;

  //: Create a copy on the heap and return base class pointer
  virtual masm_point_finder* clone() const = 0;

  //: Print class to os
  virtual void print_summary(vcl_ostream& os) const =0;

};

//: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const masm_point_finder& b);

//: Stream output operator for class pointer
vcl_ostream& operator<<(vcl_ostream& os,const masm_point_finder* b);

#endif


