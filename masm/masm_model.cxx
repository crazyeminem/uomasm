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
// \author Tim Cootes
// \brief Single Resolution Active Shape Model

#include "masm_model.h"
#include <msm/msm_shape_model.h>
#include <vsl/vsl_binary_loader.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_exception.h>
#include <vsl/vsl_indent.h>

//=======================================================================
  //: Default constructor
masm_model::masm_model()
  : wt_for_fixed_(100)
{
}

//: Set up the model
void masm_model::set(const msm_shape_model& shape_model,
           const masm_local_frame_maker& frame_maker,
           const vcl_vector<masm_point_finder_model*>& finder)
{
  assert(finder.size()==shape_model.size());
  shape_model_ = shape_model;
  frame_maker_ = frame_maker;
  finders_.resize(finder.size());
  for (unsigned i=0;i<finder.size();++i)
    finders_[i]=finder[i]->clone();
}

//: Set up the shape model and frame maker - finders undefined.
void masm_model::set(const msm_shape_model& shape_model,
           const masm_local_frame_maker& frame_maker)
{
  shape_model_ = shape_model;
  frame_maker_ = frame_maker;
}

//: Takes responsibility for finders on the heap
//  Don't delete them externally!
void masm_model::take_finders(vcl_vector<masm_point_finder_model*>& finders)
{
  assert(finders.size()==shape_model_.size());
  finders_.resize(finders.size());
  for (unsigned i=0;i<finders.size();++i)
    finders_[i]=finders[i];
}

//: Set weight to use for fixed points
void masm_model::set_wt_for_fixed(double w)
{
  wt_for_fixed_=w;
}
//=======================================================================
void masm_model::print_summary(vcl_ostream& os) const
{
  os<<"\n";
  vsl_indent_inc(os);
  os<<vsl_indent()<<"shape_model: "<<shape_model_<<vcl_endl;
  os<<vsl_indent()<<"frame_maker: "<<frame_maker_<<vcl_endl;
  os<<vsl_indent()<<"n_finders: "<<finders_.size()<<vcl_endl;
  if (finders_.size()>0)
    os<<vsl_indent()<<"First finder: "<<finders_[0]<<vcl_endl;
  vsl_indent_dec(os);
}

//: Save class to binary file stream
void masm_model::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,short(1));  // version number
  vsl_b_write(bfs,shape_model_);
  vsl_b_write(bfs,frame_maker_);
  vsl_b_write(bfs,unsigned(finders_.size()));
  for (unsigned i=0;i<finders_.size();++i)
    vsl_b_write(bfs,finders_[i]);
  vsl_b_write(bfs,wt_for_fixed_);
}


//: Load class from binary file stream
void masm_model::b_read(vsl_b_istream& bfs)
{
  short version;
  unsigned n;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      vsl_b_read(bfs,shape_model_);
      vsl_b_read(bfs,frame_maker_);
      vsl_b_read(bfs,n);
      finders_.resize(n);
      for (unsigned i=0;i<n;++i) vsl_b_read(bfs,finders_[i]);
      vsl_b_read(bfs,wt_for_fixed_); 
      break;
    default:
      vcl_cerr << "masm_model::b_read() :\n"
               << "Unexpected version number " << version << vcl_endl;
      bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
      return;
  }
}


//: Binary file stream output operator for class reference
void vsl_b_write(vsl_b_ostream& bfs, const masm_model& b)
{ b.b_write(bfs); }

//: Binary file stream input operator for class reference
void vsl_b_read(vsl_b_istream& bfs, masm_model& b)
{ b.b_read(bfs); }

//: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const masm_model& b)
{
  b.print_summary(os);
  return os;
}


