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

#ifndef masm_point_finder_model_h_
#define masm_point_finder_model_h_

//:
// \file
// \brief Base for classes containing data for masm_point_finders
// \author Tim Cootes

#include <vcl_vector.h>
#include <vcl_string.h>
#include <vcl_cassert.h>
#include <vcl_memory.h>
#include <vsl/vsl_binary_io.h>
#include <vgl/vgl_point_2d.h>

class masm_point_finder;

//: Base for classes containing data for masm_point_finders.
//  Derived classes hold data and can create associated searchers, which actually do the work.
//  Multiple searchers can use the same model.
class masm_point_finder_model
{
protected:
  
public:

  //: Dflt ctor
  masm_point_finder_model();

  //: Destructor
  virtual ~masm_point_finder_model();
  
  //: Creates a finder on the heap and sets this as its model
  virtual masm_point_finder* create_finder() const=0;
  
  //: Generate points in ref frame that represent boundary
  //  Points of a contour around the shape.
  //  Used for display purposes.  Join the points with an open
  //  contour to get a representation.
  virtual void get_outline(vcl_vector<vgl_point_2d<double> >& pts) const=0;

  //: Name of the class
  virtual vcl_string is_a() const;

  //: Create a copy on the heap and return base class pointer
  virtual masm_point_finder_model* clone() const = 0;

  //: Print class to os
  virtual void print_summary(vcl_ostream& os) const =0;

  //: Save class to binary file stream
  virtual void b_write(vsl_b_ostream& bfs) const=0;

  //: Load class from binary file stream
  virtual void b_read(vsl_b_istream& bfs)=0;

};

//: Allows derived class to be loaded by base-class pointer
void vsl_add_to_binary_loader(const masm_point_finder_model& b);

//: Binary file stream output operator for class reference
void vsl_b_write(vsl_b_ostream& bfs, const masm_point_finder_model& b);

//: Binary file stream input operator for class reference
void vsl_b_read(vsl_b_istream& bfs, masm_point_finder_model& b);

//: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const masm_point_finder_model& b);

//: Stream output operator for class pointer
vcl_ostream& operator<<(vcl_ostream& os,const masm_point_finder_model* b);

#endif


