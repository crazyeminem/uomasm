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

#ifndef masm_point_finder_builder_h_
#define masm_point_finder_builder_h_

//:
// \file
// \author Tim Cootes
// \brief Base for objects which build masm_point_finders.

#include <vcl_string.h>
#include <vcl_memory.h>
#include <vcl_iosfwd.h>
#include <vsl/vsl_fwd.h>
#include <vimt/vimt_image_2d_of.h>
#include <msm/msm_points.h>

class masm_point_finder_model;
class mfpf_pose;

//: Base for objects which build masm_point_finders.
class masm_point_finder_builder
{
 public:

  virtual ~masm_point_finder_builder() {}

  //: Creates a new model on the heap
  virtual masm_point_finder_model* new_finder() const=0;


  //: Indicate the index of this point (may be used to label points or data)
  virtual void set_point_index(unsigned i);
    
  //: Define existing finder, which may contribute to new model.
  //  When defined, new model may include information from the old finder.
  //  For instance, it may be a specialised version of the old finder.
  //  Default baseclass behaviour is to ignore old finder.
  virtual void set_old_finder(const masm_point_finder_model*);

  //: Discard all samples. Indicate how many samples to expect.
  virtual void clear(unsigned n_samples)=0;

  //: Add a sample from the given image
  virtual void add_sample(const vimt_image_2d& image, const mfpf_pose& base_pose)=0;

  //: Build model with current set of samples
  virtual void build(masm_point_finder_model&)=0;

  //: Name of the class
  virtual vcl_string is_a() const = 0;

  //: Create a copy on the heap and return base class pointer
  virtual masm_point_finder_builder* clone() const = 0;

  //: Print class to os
  virtual void print_summary(vcl_ostream& os) const=0;

  //: Save class to binary file stream
  virtual void b_write(vsl_b_ostream& bfs) const = 0;

  //: Load class from binary file stream
  virtual void b_read(vsl_b_istream& bfs) = 0;

  //: Create a concrete masm_point_finder_builder-derived object, from a text specification.
  static vcl_auto_ptr<masm_point_finder_builder> create_from_stream(vcl_istream &is);

  //: Initialise from a text stream.
  virtual void config_from_stream(vcl_istream &is);

  //: Override some parameters using information in s
  //  Used to specialise a copy of the default builder
  virtual void reconfig_from_string(const vcl_string& s);
};

//: Allows derived class to be loaded by base-class pointer
//  A loader object exists which is invoked by calls
//  of the form "vsl_b_read(bfs,base_ptr);".  
//  This loads derived class objects from the disk, 
//  places them on the heap and returns a base class pointer.
//  In order to work the loader object requires
//  an instance of each derived class that might be
//  found.  This function gives the model class to
//  the appropriate loader.
void vsl_add_to_binary_loader(const masm_point_finder_builder& b);

//: Binary file stream output operator for class reference
void vsl_b_write(vsl_b_ostream& bfs, const masm_point_finder_builder& b);

//: Binary file stream input operator for class reference
void vsl_b_read(vsl_b_istream& bfs, masm_point_finder_builder& b);

//: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const masm_point_finder_builder& b);

//: Stream output operator for class pointer
vcl_ostream& operator<<(vcl_ostream& os,const masm_point_finder_builder* b);

//: Stream output operator for class reference
void vsl_print_summary(vcl_ostream& os,const masm_point_finder_builder& b);

//: Stream output operator for class reference
void vsl_print_summary(vcl_ostream& os,const masm_point_finder_builder* b);


#endif // masm_point_finder_builder_h_


