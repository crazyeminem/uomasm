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

#ifndef masm_model_series_h_
#define masm_model_series_h_

//:
// \file
// \author Tim Cootes
// \brief Holds a set of ASM objects, to be applied sequentially.

#include <masm/masm_model.h>

//: Holds a set of  ASM objects, to be applied sequentially.
//  Each is assumed to manipulate the same number of points.
class masm_model_series 
{
protected:
  //: Sequence of individual ASMs
  vcl_vector<masm_model> asm_;

public:
  //: Dflt ctor
  masm_model_series();

  //: Destructor
  virtual ~masm_model_series();

  //: Set up internal ASM objects
  void set_aams(const vcl_vector<masm_model>& asm1);

  //: Number of models.
  size_t size() const { return asm_.size(); }

  //: Set to n empty aams (used during initialisation)
  void resize(unsigned n) { asm_.resize(n); }

  //: Access to ASM i
  const masm_model& operator()(unsigned i) const
  { assert(i<asm_.size()); return asm_[i]; }

  //: Access to ASM i - use with care.
  masm_model& operator()(unsigned i)
  { assert(i<asm_.size()); return asm_[i]; }

  //: Access to last ASM
  const masm_model& last_asm() const
  { assert(asm_.size()>0); return asm_[asm_.size()-1]; }

  //: Name of the class
  virtual vcl_string is_a() const;

  //: Print class to os
  virtual void print_summary(vcl_ostream& os) const;

  //: Save class to binary file stream
  virtual void b_write(vsl_b_ostream& bfs) const;

  //: Load class from binary file stream
  virtual void b_read(vsl_b_istream& bfs);
};

//: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const masm_model_series& b);

//: Binary file stream output operator for class reference
void vsl_b_write(vsl_b_ostream& bfs, const masm_model_series& b);

//: Binary file stream input operator for class reference
void vsl_b_read(vsl_b_istream& bfs, masm_model_series& b);

#endif // masm_model_series_h_


