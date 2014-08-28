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

#ifndef masm_ncc_finder_builder_h_
#define masm_ncc_finder_builder_h_
//:
// \file
// \author Tim Cootes
// \brief Builds a masm_ncc_finder.

#include <masm/masm_point_finder_builder.h>

//: Builds a masm_ncc_finder.
class masm_ncc_finder_builder : public masm_point_finder_builder
{
 private:
  //: Width of template
  unsigned ni_;
  //: Height of template
  unsigned nj_;
  
  //: Kernel reference point (in kernel_ image frame)
  vgl_point_2d<double> ref_pt_;

  //: When true assumed to search for structure only well defined in 1D
  //  For instance, when searching for an edge.
  bool is_1d_;
  
  //: Number of steps either side to search along direction pose.u() (in steps of u)
  unsigned search_ni_;
  //: Number of steps either side to search along direction orthogonal to pose.u()
  unsigned search_nj_;

  vil_image_view<float> sum_;
  unsigned n_egs_;

 public:

  masm_ncc_finder_builder();

  virtual ~masm_ncc_finder_builder() {}
  
  //: Define size of template (ref point to be in centre)
  void set_kernel_size(unsigned ni, unsigned nj);
  
  //: Number of steps either side to search along direction pose.u() (in steps of u)
  unsigned search_ni() const { return search_ni_; }
  
  //: Number of steps either side to search along direction orthogonal to pose.u()
  unsigned search_nj() const { return search_nj_; }


  //: Creates a new model on the heap
  virtual masm_point_finder_model* new_finder() const;
    
  //: Discard all samples. Indicate how many samples to expect.
  virtual void clear(unsigned n_samples);

  //: Add a sample from the given image
  virtual void add_sample(const vimt_image_2d& image, const mfpf_pose& base_pose);

  //: Build model with current set of samples
  virtual void build(masm_point_finder_model&);

  //: Name of the class
  virtual vcl_string is_a() const;

  //: Create a copy on the heap and return base class pointer
  virtual masm_point_finder_builder* clone() const;

  //: Print class to os
  virtual void print_summary(vcl_ostream& os) const;

  //: Save class to binary file stream
  virtual void b_write(vsl_b_ostream& bfs) const;

  //: Load class from binary file stream
  virtual void b_read(vsl_b_istream& bfs);

  //: Initialise from a text stream.
  // Expects input such as: { ni: 9 nj: 9 }
  virtual void config_from_stream(vcl_istream &is);

  //: Override some parameters using information in s
  //  Used to specialise a copy of the default builder
  virtual void reconfig_from_string(const vcl_string& s);
};

#endif // masm_ncc_finder_builder_h_
