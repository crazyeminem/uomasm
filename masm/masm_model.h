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

#ifndef masm_model_h_
#define masm_model_h_
//:
// \file
// \author Tim Cootes
// \brief Active Shape Model

#include <msm/msm_shape_model.h>
#include <masm/masm_local_frame_maker.h>
#include <masm/masm_point_finder_model.h>
#include <mbl/mbl_cloneable_ptr.h>

//: Active Shape Model.
//  Contains the local models etc for a single resolution ASM.
//  For a multi-resolution ASM, use a sequence of such models.
class masm_model
{
 private:
  //: Shape model object
  msm_shape_model shape_model_;

  //: Object to create local frames
  // These define where to search for each point
  masm_local_frame_maker frame_maker_;

  //: Set of objects to perform local search
  vcl_vector<mbl_cloneable_ptr<masm_point_finder_model> > finders_;

  //: Weight to use for fixed points
  double wt_for_fixed_;

 public:
  //: Default constructor
  masm_model();

  //: Destructor
  ~masm_model() {}

  //: Set up the model. Clones are taken of the finders.
  void set(const msm_shape_model& shape_model,
           const masm_local_frame_maker& frame_maker,
           const vcl_vector<masm_point_finder_model*>& finder);

  //: Set up the shape model and frame maker - finders undefined.
  void set(const msm_shape_model& shape_model,
           const masm_local_frame_maker& frame_maker);
  
  //: Takes responsibility for finders on the heap
  //  Don't delete them externally!
  void take_finders(vcl_vector<masm_point_finder_model*>& finders);

  //: Shape model
  const msm_shape_model& shape_model() const 
  { return shape_model_; }

  //: Object to create local frames
  // These define where to search for each point
  const masm_local_frame_maker& frame_maker() const
  { return frame_maker_; }

  //: Point finder for point i
  const masm_point_finder_model& finder(unsigned i) const
  {
    return finders_[i];
  }

  //: Weight to use for fixed points
  double wt_for_fixed() const { return wt_for_fixed_; }
  
  //: Set weight to use for fixed points
  void set_wt_for_fixed(double w);

  //: Print class to os
  void print_summary(vcl_ostream& os) const;

  //: Save class to binary file stream
  void b_write(vsl_b_ostream& bfs) const;

  //: Load class from binary file stream
  void b_read(vsl_b_istream& bfs);
};

//: Binary file stream output operator for class reference
void vsl_b_write(vsl_b_ostream& bfs, const masm_model& b);

//: Binary file stream input operator for class reference
void vsl_b_read(vsl_b_istream& bfs, masm_model& b);

//: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const masm_model& b);


#endif // masm_model_h_
