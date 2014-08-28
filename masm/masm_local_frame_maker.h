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

#ifndef masm_local_frame_maker_h_
#define masm_local_frame_maker_h_
//:
// \file
// \author Tim Cootes
// \brief Local frame defined as normal to curve at point

#include <msm/msm_shape_instance.h>
#include <msm/msm_curve.h>
#include <mfpf/mfpf_pose.h>

//: Local frame defined as normal to curve at point.
//  Generates aligned frame in reference frame for every
//  point, then applies model pose transformation.
//  Main axis is normal to curve through the point.
// 
//  Define tangent at point i as line between end0_[i] and end1_[i].
//  If end0_[i]==end1_[i] then use vector (1,0) in ref. frame.
class masm_local_frame_maker
{
 private:
  //: Scale step in reference frame
  double ref_scale_step_;

  //: Define tangent at point i as line between end0_[i] and end1_[i]
  vcl_vector<unsigned> end0_;

  //: Define tangent at point i as line between end0_[i] and end1_[i]
  vcl_vector<unsigned> end1_;

  //: Returns true if direction defined at point i
  bool defined(unsigned i) { return (end0_[i]!=0 || end1_[i]!=0); }

 public:

  ~masm_local_frame_maker() {}

  //: Define step size between samples in reference frame
  void set_ref_scale_step(double);

  //: Set up definitions of directions from the curves.
  //  Where multiple curves pass through a point, the direction
  //  is defined by the first in the list.
  void set_from_curves(unsigned n_points, const msm_curves& curves);

  //: Frame in ref. is aligned with x axis.
  //  For each point in instance.points(), generate one local frame.
  //  Generally origin will be the point itself.
  void create_default_frames(msm_shape_instance& instance,
                             vcl_vector<mfpf_pose>& frame_pose) const;


  //: Create a local frame for each point, defined using an mfpf_pose
  //  For each point in instance.points(), generate one local frame.
  //  Generally origin will be the point itself.
  void create_frames(msm_shape_instance& instance,
                             vcl_vector<mfpf_pose>& frame_pose) const;

  //: Print class to os
  void print_summary(vcl_ostream& os) const;

  //: Save class to binary file stream
  void b_write(vsl_b_ostream& bfs) const;

  //: Load class from binary file stream
  void b_read(vsl_b_istream& bfs);
};

//: Binary file stream output operator for class reference
void vsl_b_write(vsl_b_ostream& bfs, const masm_local_frame_maker& b);

//: Binary file stream input operator for class reference
void vsl_b_read(vsl_b_istream& bfs, masm_local_frame_maker& b);

//: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const masm_local_frame_maker& b);

#endif // masm_local_frame_maker_h_
