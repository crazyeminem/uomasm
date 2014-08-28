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
// \brief Base for classes which search for individual model points.
// \author Tim Cootes


#include "masm_point_finder.h"
#include <vsl/vsl_indent.h>
#include <vcl_cassert.h>


//=======================================================================
// Dflt ctor
//=======================================================================

masm_point_finder::masm_point_finder()
{
}

//=======================================================================
// Destructor
//=======================================================================

masm_point_finder::~masm_point_finder()
{
}

//=======================================================================
// Method: is_a
//=======================================================================

vcl_string masm_point_finder::is_a() const
{
  return vcl_string("masm_point_finder");
}

//=======================================================================
// Associated function: operator<< 
//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const masm_point_finder& b)
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

vcl_ostream& operator<<(vcl_ostream& os,const masm_point_finder* b)
{
  if (b)
  return os << *b;
  else
  return os << "No masm_point_finder defined.";
}
