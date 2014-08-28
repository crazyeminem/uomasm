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

#ifndef masm_region_pdf_builder_h_
#define masm_region_pdf_builder_h_
//:
// \file
// \author Tim Cootes
// \brief Builds a masm_pdf_region_model.

#include <masm/masm_point_finder_builder.h>
#include <vpdfl/vpdfl_builder_base.h>
#include <mbl/mbl_cloneable_ptr.h>
#include <mbl/mbl_chord.h>

//: Builds a masm_ncc_finder.
class masm_region_pdf_builder : public masm_point_finder_builder
{
 private:
  
  //: Kernel reference point (in kernel_ image frame)
  vgl_point_2d<double> ref_pt_;

  //: String defining shape of region, eg "box" or "ellipse"
  vcl_string shape_;

  //: Chords defining the region of interest
  vcl_vector<mbl_chord> roi_;

  //: Size of bounding box of region of interest
  unsigned roi_ni_;
  //: Size of bounding box of region of interest
  unsigned roi_nj_;

  //: Number of pixels in region
  unsigned n_pixels_;
  
  //: Number of planes expected in image
  unsigned n_planes_;

  //: Builder for PDf for profile vector
  mbl_cloneable_ptr<vpdfl_builder_base> pdf_builder_;

  //: Which normalisation to use (0=none, 1=linear)
  short norm_method_;

  //: When true assumed to search for structure only well defined in 1D
  //  For instance, when searching for an edge.
  bool is_1d_;
  
  //: Number of steps either side to search along direction pose.u() (in steps of u)
  unsigned search_ni_;
  //: Number of steps either side to search along direction orthogonal to pose.u()
  unsigned search_nj_;


  //: Samples added in calls to add_example()
  vcl_vector<vnl_vector<double> > data_;
    
  //: Define model region as an ni x nj box
  void set_as_box(unsigned ni, unsigned nj,
                  const vgl_point_2d<double>& ref_pt);

  //: Define model region as an ellipse with radii ri, rj
  //  Ref. point in centre.
  void set_as_ellipse(double ri, double rj);

  //: Parse stream to set up as a box shape.
  // Expects: "{ ni: 3 nj: 5 ref_x: 1.0 ref_y: 2.0 }
  void config_as_box(vcl_istream &is);

  //: Parse stream to set up as an ellipse shape.
  // Expects: "{ ri: 2.1 rj: 5.2 }
  void config_as_ellipse(vcl_istream &is);


 public:

  masm_region_pdf_builder();

  virtual ~masm_region_pdf_builder() {}
  
  //: Define model region as an ni x nj box
  void set_as_box(unsigned ni, unsigned nj,
                  const vgl_point_2d<double>& ref_pt,
                  const vpdfl_builder_base& builder);

  //: Define model region as an ni x nj box
  //  Ref. point in centre.
  void set_as_box(unsigned ni, unsigned nj,
                  const vpdfl_builder_base& builder);

  //: Define model region as an ellipse with radii ri, rj
  //  Ref. point in centre.
  void set_as_ellipse(double ri, double rj,
                      const vpdfl_builder_base& builder);

  //: Number of steps either side to search along direction pose.u() (in steps of u)
  unsigned search_ni() const { return search_ni_; }
  
  //: Number of steps either side to search along direction orthogonal to pose.u()
  unsigned search_nj() const { return search_nj_; }

  //: Builder for PDF
  vpdfl_builder_base& pdf_builder() { return pdf_builder_; }


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
  // Expects input such as: 
  // \verbatim
  // { shape: box { ni: 9 nj: 9 }
  //   norm: linear 
  //   pdf_builder: vpdfl_axis_gaussian_builder { } 
  //   search_ni: 5 search_nj: 5
  // }
  // \endverbatim
  // The shape parameter can also be "ellipse { ri: 7.5 rj: 5.3 }"
  virtual void config_from_stream(vcl_istream &is);

  //: Override some parameters using information in s
  //  Used to specialise a copy of the default builder
  virtual void reconfig_from_string(const vcl_string& s);
};

#endif // masm_region_pdf_builder_h_
