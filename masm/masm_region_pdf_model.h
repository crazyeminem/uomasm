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

#ifndef masm_region_pdf_model_h_
#define masm_region_pdf_model_h_

//:
// \file
// \brief Data for finder which evaluates the PDF of region around point
// \author Tim Cootes

#include <masm/masm_point_finder_model.h>
#include <vil/vil_image_view.h>
#include <vgl/vgl_point_2d.h>
#include <vpdfl/vpdfl_pdf_base.h>
#include <mbl/mbl_cloneable_ptr.h>
#include <mbl/mbl_chord.h>

//: Data for finder which evaluates the PDF of region around point.
class masm_region_pdf_model : public masm_point_finder_model
{
protected:
  //: Kernel reference point (in kernel_ image frame)
  vgl_point_2d<double> ref_pt_;
  
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

  //: PDf for vector sampled over ROI
  mbl_cloneable_ptr<vpdfl_pdf_base> pdf_;

  //: Which normalisation to use (0=none, 1=linear)
  short norm_method_;

  //: Number of steps either side to search along direction pose.u() (in steps of u)
  unsigned search_ni_;
  //: Number of steps either side to search along direction orthogonal to pose.u()
  unsigned search_nj_;

public:

  //: Dflt ctor
  masm_region_pdf_model();

  //: Destructor
  virtual ~masm_region_pdf_model();
  
  //: Creates a finder on the heap and sets this as its model
  virtual masm_point_finder* create_finder() const;
  
  //: Define region and PDF of region
  void set(const vcl_vector<mbl_chord>& roi, unsigned n_planes,
           const vgl_point_2d<double>& ref_pt,
           const vpdfl_pdf_base& pdf,
           short norm_method=1);
  
  //: Define search area in terms of steps either side of base point
  //  Units of pose.u()
  virtual void set_search_area(unsigned ni, unsigned nj);

  //: Kernel reference point (in kernel_ image frame)
  const vgl_point_2d<double>& ref_pt() const { return ref_pt_; }

  //: PDf for region vector
  const vpdfl_pdf_base& pdf() const { return pdf_; }

  //: Chords defining the region of interest
  const vcl_vector<mbl_chord>& roi() const { return roi_; }

  //: Size of bounding box of region of interest
  unsigned roi_ni() const { return roi_ni_; }
  //: Size of bounding box of region of interest
  unsigned roi_nj() const { return roi_nj_; }

  //: Number of pixels in region
  unsigned n_pixels() const { return n_pixels_; }

  //: Which normalisation to use (0=none, 1=linear)
  short norm_method() const { return norm_method_; }

  //: Number of planes expected in image
  unsigned n_planes() const { return n_planes_; }

  //: Number of steps either side to search along direction pose.u() (in steps of u)
  int search_ni() const { return search_ni_; }
  
  //: Number of steps either side to search along direction orthogonal to pose.u()
  int search_nj() const { return search_nj_; }

  //: Generate points in ref frame that represent boundary
  //  Points of a contour around the shape.
  //  Used for display purposes.  Join the points with an open
  //  contour to get a representation.
  virtual void get_outline(vcl_vector<vgl_point_2d<double> >& pts) const;

  //: Name of the class
  virtual vcl_string is_a() const;

  //: Create a copy on the heap and return base class pointer
  virtual masm_point_finder_model* clone() const;

  //: Print class to os
  virtual void print_summary(vcl_ostream& os) const;

  //: Save class to binary file stream
  virtual void b_write(vsl_b_ostream& bfs) const;

  //: Load class from binary file stream
  virtual void b_read(vsl_b_istream& bfs);

};

#endif


