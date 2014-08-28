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
// \brief Base for objects which build masm_point_finder_models.

#include "masm_point_finder_builder.h"
#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>
#include <mbl/mbl_cloneables_factory.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_exception.h>

#include <vsl/vsl_binary_loader.txx>
#include <mbl/mbl_cloneables_factory.h>

//: Indicate the index of this point (may be used to label points or data)
void masm_point_finder_builder::set_point_index(unsigned)
{
}

//: Define existing finder, which may contribute to new model.
//  When defined, new model may include information from the old finder.
//  For instance, it may be a specialised version of the old finder.
//  Default baseclass behaviour is to ignore old finder.
void masm_point_finder_builder::set_old_finder(const masm_point_finder_model*)
{
}

//=======================================================================

void vsl_add_to_binary_loader(const masm_point_finder_builder& b)
{
  vsl_binary_loader<masm_point_finder_builder>::instance().add(b);
}

//=======================================================================

void vsl_b_write(vsl_b_ostream& bfs, const masm_point_finder_builder& b)
{
  b.b_write(bfs);
}

//=======================================================================
//: Initialise from a text stream.
void masm_point_finder_builder::config_from_stream(vcl_istream &is)
{
  vcl_string s = mbl_parse_block(is);
  if (s.empty() || s=="{}") return;

  mbl_exception_parse_error x(
    this->is_a() + " expects no properties in initialisation,\n"
    "But the following properties were given:\n" + s);
  mbl_exception_error(x);
}

//: Override some parameters using information in s
//  Used to specialise a copy of the default builder
void masm_point_finder_builder::reconfig_from_string(const vcl_string& s)
{
  vcl_cout<<is_a()<<"::reconfig_from_string() not implemented."<<vcl_endl;
  vcl_cout<<"Unable to reconfigure with: "<<s<<vcl_endl;
}

//=======================================================================
//: Create a conrete derived object, from a text specification.
vcl_auto_ptr<masm_point_finder_builder> masm_point_finder_builder::create_from_stream(vcl_istream &is)
{
  vcl_string name;
  is >> name;

  vcl_auto_ptr<masm_point_finder_builder> ps =
    mbl_cloneables_factory<masm_point_finder_builder>::get_clone(name);

  ps -> config_from_stream(is);
  return ps;
}

//=======================================================================

void vsl_b_read(vsl_b_istream& bfs, masm_point_finder_builder& b)
{
  b.b_read(bfs);
}

//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const masm_point_finder_builder& b)
{
  os << b.is_a() << ": ";
  vsl_indent_inc(os);
  b.print_summary(os);
  vsl_indent_dec(os);
  return os;
}

//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const masm_point_finder_builder* b)
{
  if (b)
    return os << *b;
  else
    return os << "No masm_point_finder_builder defined.";
}

//=======================================================================
//: Stream output operator for class reference
void vsl_print_summary(vcl_ostream& os,const masm_point_finder_builder& b)
{
  os << b;
}

//=======================================================================
//: Stream output operator for class reference
void vsl_print_summary(vcl_ostream& os,const masm_point_finder_builder* b)
{
  if (b)
    os << *b;
  else
    os << vsl_indent() << "No masm_point_finder_builder defined.";
}

VSL_BINARY_LOADER_INSTANTIATE(masm_point_finder_builder);
MBL_CLONEABLES_FACTORY_INSTANTIATE(masm_point_finder_builder);

