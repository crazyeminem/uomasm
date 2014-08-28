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
// \brief Holds a set of shape AAM objects, to be applied sequentially.

#include <vcl_cstdlib.h>

#include "masm_model_series.h"

#include <vil/vil_math.h>

#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>
#include <vsl/vsl_vector_io.h>
#include <vcl_algorithm.h>

//=======================================================================
// Dflt ctor
//=======================================================================
masm_model_series::masm_model_series()
{
}


//=======================================================================
// Destructor
//=======================================================================
masm_model_series::~masm_model_series()
{
}

//: Partially set up the model (predictors are empty).
//  Takes copies of each object.
void masm_model_series::set_aams(const vcl_vector<masm_model>& asm1)
{
  asm_=asm1;
}

//=======================================================================
// Method: is_a
//=======================================================================
vcl_string masm_model_series::is_a() const
{
  return vcl_string("masm_model_series");
}

//: Print class to os
void masm_model_series::print_summary(vcl_ostream& os) const
{
  os<<"n_models: "<<asm_.size()<<vcl_endl;
  vsl_indent_inc(os);
  for (unsigned i=0;i<asm_.size();++i)
  {
    os<<vsl_indent()<<i
      <<") ASM: { "<<asm_[i]<<" }"<<vcl_endl;
  }
  vsl_indent_dec(os);
}

//=======================================================================
// Method: save
//=======================================================================

void masm_model_series::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,short(1));  // Version number

  vsl_b_write(bfs,unsigned(asm_.size()));
  for (unsigned i=0;i<asm_.size();++i) 
    vsl_b_write(bfs,asm_[i]);
}

//=======================================================================
// Method: load
//=======================================================================

void masm_model_series::b_read(vsl_b_istream& bfs)
{
  if (!bfs) return;
  short version;
  unsigned n;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      vsl_b_read(bfs,n); asm_.resize(n);
      for (unsigned i=0;i<n;++i)
        vsl_b_read(bfs,asm_[i]);
      break;
    default:
      vcl_cerr << "I/O ERROR: vsl_b_read(vsl_b_istream&) \n";
      vcl_cerr << "           Unknown version number "<< version << vcl_endl;
      bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
      return;
  }
}

//=======================================================================
// Associated function: operator<<
//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const masm_model_series& b)
{
  os << b.is_a() << ": ";
  vsl_indent_inc(os);
  b.print_summary(os);
  vsl_indent_dec(os);
  return os;
}

  //: Binary file stream output operator for class reference
void vsl_b_write(vsl_b_ostream& bfs, const masm_model_series& b)
{
  b.b_write(bfs);
}

  //: Binary file stream input operator for class reference
void vsl_b_read(vsl_b_istream& bfs, masm_model_series& b)
{
  b.b_read(bfs);
}


