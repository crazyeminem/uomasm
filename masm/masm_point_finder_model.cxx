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
// \brief Base for classes containing data for masm_point_finders
// \author Tim Cootes


#include "masm_point_finder_model.h"

#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>

#include <vcl_cassert.h>

#include <vsl/vsl_binary_loader.txx>

//=======================================================================
// Dflt ctor
//=======================================================================

masm_point_finder_model::masm_point_finder_model()
{
}

//=======================================================================
// Destructor
//=======================================================================

masm_point_finder_model::~masm_point_finder_model()
{
}


//=======================================================================
// Method: is_a
//=======================================================================

vcl_string masm_point_finder_model::is_a() const
{
  return vcl_string("masm_point_finder_model");
}

//: Allows derived class to be loaded by base-class pointer
void vsl_add_to_binary_loader(const masm_point_finder_model& b)
{
  vsl_binary_loader<masm_point_finder_model>::instance().add(b);
}

//=======================================================================
// Associated function: operator<<
//=======================================================================

void vsl_b_write(vsl_b_ostream& bfs, const masm_point_finder_model& b)
{
  b.b_write(bfs);
}

//=======================================================================
// Associated function: operator>>
//=======================================================================

void vsl_b_read(vsl_b_istream& bfs, masm_point_finder_model& b)
{
  b.b_read(bfs);
}

//=======================================================================
// Associated function: operator<< 
//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const masm_point_finder_model& b)
{
  os << b.is_a() << ": ";
  vsl_indent_inc(os);
  b.print_summary(os);
  vsl_indent_dec(os);
  return os;
}

//=======================================================================
// Associated function: operator<< 
//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const masm_point_finder_model* b)
{
  if (b)
  return os << *b;
  else
  return os << "No masm_point_finder_model defined.";
}

VSL_BINARY_LOADER_INSTANTIATE(masm_point_finder_model);
