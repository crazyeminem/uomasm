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
// \brief Data for normalised cross-correlation model
// \author Tim Cootes


#include "masm_ncc_finder_model.h"

#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>
#include <vil/io/vil_io_image_view.h>
#include <vgl/io/vgl_io_point_2d.h>

#include <vcl_cassert.h>
#include <masm/masm_ncc_finder.h>

//=======================================================================
// Dflt ctor
//=======================================================================

masm_ncc_finder_model::masm_ncc_finder_model()
  : search_ni_(5),search_nj_(5)
{
}

//=======================================================================
// Destructor
//=======================================================================

masm_ncc_finder_model::~masm_ncc_finder_model()
{
}

//: Define kernel and the reference point within it.
void masm_ncc_finder_model::set(const vil_image_view<float>& kernel, const vgl_point_2d<double>& ref_pt)
{
  kernel_.deep_copy(kernel);
  ref_pt_ = ref_pt;
}

//: Define search area in terms of steps either side of base point
//  Units of pose.u()
void masm_ncc_finder_model::set_search_area(unsigned ni, unsigned nj)
{
  search_ni_=ni;
  search_nj_=nj;
}

//: Creates a finder on the heap and sets this as its model
masm_point_finder* masm_ncc_finder_model::create_finder() const
{
  masm_ncc_finder* finder = new masm_ncc_finder();
  finder->set_model(*this);
  return finder;
}

//: Generate points in ref frame that represent boundary
//  Points of a contour around the shape.
//  Used for display purposes.  Join the points with an open
//  contour to get a representation.
void masm_ncc_finder_model::get_outline(vcl_vector<vgl_point_2d<double> >& pts) const
{
  pts.resize(7);
  int roi_ni=kernel_.ni();
  int roi_nj=kernel_.nj();
  vgl_vector_2d<double> r(ref_pt_.x(),ref_pt_.y());
  pts[0]=vgl_point_2d<double>(0,roi_nj-1)-r;
  pts[1]=vgl_point_2d<double>(0,0);
  pts[2]=vgl_point_2d<double>(roi_ni-1,roi_nj-1)-r;
  pts[3]=vgl_point_2d<double>(0,roi_nj-1)-r;
  pts[4]=vgl_point_2d<double>(0,0)-r;
  pts[5]=vgl_point_2d<double>(roi_ni-1,0)-r;
  pts[6]=vgl_point_2d<double>(roi_ni-1,roi_nj-1)-r;
}
  
//=======================================================================
// Method: is_a
//=======================================================================

vcl_string masm_ncc_finder_model::is_a() const
{
  return vcl_string("masm_ncc_finder_model");
}


//: Create a copy on the heap and return base class pointer
masm_point_finder_model* masm_ncc_finder_model::clone() const
{
  return new masm_ncc_finder_model(*this);
}

//=======================================================================
// Method: print
//=======================================================================

void masm_ncc_finder_model::print_summary(vcl_ostream& os) const
{
  os<<"{  size: "<<kernel_.ni()<<" x "<<kernel_.nj()
    <<" ref:("<<ref_pt_.x()<<","<<ref_pt_.y()<<")"
    <<" search_ni: "<<search_ni_<<" search_nj: "<<search_nj_<<" }";
}

void masm_ncc_finder_model::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,short(1));  // Version number
  vsl_b_write(bfs,kernel_);
  vsl_b_write(bfs,ref_pt_);
  vsl_b_write(bfs,search_ni_);
  vsl_b_write(bfs,search_nj_);
}

void masm_ncc_finder_model::b_read(vsl_b_istream& bfs)
{
  if (!bfs) return;
  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      vsl_b_read(bfs,kernel_);
      vsl_b_read(bfs,ref_pt_);
      vsl_b_read(bfs,search_ni_);
      vsl_b_read(bfs,search_nj_);
      break;
    default:
      vcl_cerr << "I/O ERROR: vsl_b_read(vsl_b_istream&)\n"
               << "           Unknown version number "<< version << vcl_endl;
      bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
      return;
  }
}
