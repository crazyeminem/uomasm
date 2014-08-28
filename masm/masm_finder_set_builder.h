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

#ifndef masm_finder_set_builder_h_
#define masm_finder_set_builder_h_
//:
// \file
// \author Tim Cootes
// \brief Container for objects which can build a set of masm_point_finder objects.

#include <masm/masm_point_finder_builder.h>
#include <mbl/mbl_cloneable_ptr.h>
#include <vcl_map.h>
#include <vimt/vimt_image_pyramid.h>

//: Container for objects which can build a set of masm_point_finder objects.
//
// Text parameter format:
// \verbatim
//  {
//    default_builder: masm_ncc_builder { ni: 11 nj: 11 }
//    override_params: {
//      params: { index: 0 data: { ni: 7 nj: 13 } }
//      params: { index: { 2 5 : 7 } data: { ni: 9 nj: 11 } }
//    }
//  }
// \endverbatim
class masm_finder_set_builder
{
 private:
  //: Default builder type
  mbl_cloneable_ptr<masm_point_finder_builder> default_builder_;

  //: Parameter strings used to over-ride some of the builders
  vcl_map<unsigned,vcl_string> builder_params_;

  //: Set of builders for point finders
  vcl_vector<mbl_cloneable_ptr<masm_point_finder_builder> > builder_;

  //: Set up builder_ and n_pts copies of the default_builder_
  void set_as_default(unsigned n_pts);

 public:

  masm_finder_set_builder();

  virtual ~masm_finder_set_builder() {}

  unsigned n_points() const { return builder_.size(); }
  
  //: Define existing finders, which may contribute to new finders
  //  When defined, the sampling and building may take account of the
  //  existing model (eg it may personalise it to supplied data).
  //  If no existing searcher so defined, build from scratch (the default)
  virtual void set_old_finders(const vcl_vector<const masm_point_finder_model*>& old_finders);

  //: Indicate how many samples to expect
  virtual void set_n_samples(unsigned n);

  //: Discard all samples and indicate number of points and number of samples to expect
  virtual void clear(unsigned n_pts, unsigned n_samples);

  //: Add a sample from the given image
  //  Points are in world coordinates.
  virtual void add_sample(const vimt_image_pyramid& image_pyr, const vcl_vector<mfpf_pose>& pose);

  //: Build finders with current set of samples
  virtual void build(vcl_vector<masm_point_finder_model*>&);

  //: Print class to os
  virtual void print_summary(vcl_ostream& os) const;

  //: Save class to binary file stream
  virtual void b_write(vsl_b_ostream& bfs) const;

  //: Load class from binary file stream
  virtual void b_read(vsl_b_istream& bfs);

  //: Initialise from a text stream.
  // Expects input such as: "{ default_builder: hclm_norm_corr_builder { ni: 7 nj: 7 } }"
  virtual void config_from_stream(vcl_istream &is);
};

//: Binary file stream output operator for class reference
inline void vsl_b_write(vsl_b_ostream& bfs, const masm_finder_set_builder& b)
{ b.b_write(bfs); }

//: Binary file stream input operator for class reference
inline void vsl_b_read(vsl_b_istream& bfs, masm_finder_set_builder& b)
{ b.b_read(bfs); }

//: Stream output operator for class reference
inline vcl_ostream& operator<<(vcl_ostream& os,const masm_finder_set_builder& b)
{
  b.print_summary(os);
  return os;
}


#endif // masm_finder_set_builder_h_
