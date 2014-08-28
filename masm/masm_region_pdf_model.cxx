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
// \brief Data for finder which evaluates the PDF of region around point
// \author Tim Cootes


#include "masm_region_pdf_model.h"

#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>
#include <vsl/vsl_vector_io.h>
#include <vil/io/vil_io_image_view.h>
#include <vgl/io/vgl_io_point_2d.h>

#include <vcl_cassert.h>
#include <masm/masm_region_pdf_finder.h>

//=======================================================================
// Dflt ctor
//=======================================================================

masm_region_pdf_model::masm_region_pdf_model()
  : search_ni_(5),search_nj_(5),n_planes_(0),norm_method_(1)
{
}

//=======================================================================
// Destructor
//=======================================================================

masm_region_pdf_model::~masm_region_pdf_model()
{
}

//: Define region and PDF of region
void masm_region_pdf_model::set(const vcl_vector<mbl_chord>& roi,
                          unsigned n_planes,
                          const vgl_point_2d<double>& ref_pt,
                          const vpdfl_pdf_base& pdf,
                          short norm_method)
{
  pdf_ = pdf.clone();
  n_planes_ = n_planes;

  // Check bounding box
  if (roi.size()==0) { roi_ni_=0; roi_nj_=0; return; }
  int ilo=roi[0].start_x(), ihi=roi[0].end_x();
  int jlo=roi[0].y(), jhi=roi[0].y();

  for (unsigned k=1;k<roi.size();++k)
  {
    if (roi[k].start_x()<ilo) ilo=roi[k].start_x();
    if (roi[k].end_x()>ihi)   ihi=roi[k].end_x();
    if (roi[k].y()<jlo) jlo=roi[k].y();
    if (roi[k].y()>jhi) jhi=roi[k].y();
  }
  roi_ni_=1+ihi-ilo;
  roi_nj_=1+jhi-jlo;

  // Apply offset of (-ilo,-jlo) to ensure bounding box is +ive
  ref_pt_=ref_pt - vgl_vector_2d<double>(ilo,jlo);

  roi_.resize(roi.size());
  n_pixels_=0;
  for (unsigned k=0;k<roi.size();++k)
  {
    roi_[k]= mbl_chord(roi[k].start_x()-ilo,
                       roi[k].end_x()-ilo,   roi[k].y()-jlo);
    n_pixels_+=1+roi[k].end_x()-roi[k].start_x();
  }

  assert(norm_method>=0 && norm_method<=1);
  norm_method_ = norm_method;
}


//: Define search area in terms of steps either side of base point
//  Units of pose.u()
void masm_region_pdf_model::set_search_area(unsigned ni, unsigned nj)
{
  search_ni_=ni;
  search_nj_=nj;
}

//: Creates a finder on the heap and sets this as its model
masm_point_finder* masm_region_pdf_model::create_finder() const
{
  masm_region_pdf_finder* finder = new masm_region_pdf_finder();
  finder->set_model(*this);
  return finder;
}

//: Generate points in ref frame that represent boundary
//  Points of a contour around the shape.
//  Used for display purposes.  Join the points with an open
//  contour to get a representation.
void masm_region_pdf_model::get_outline(vcl_vector<vgl_point_2d<double> >& pts) const
{
  pts.resize(7);
  vgl_vector_2d<double> r(ref_pt_.x(),ref_pt_.y());
  pts[0]=vgl_point_2d<double>(0,roi_nj_-1)-r;
  pts[1]=vgl_point_2d<double>(0,0);
  pts[2]=vgl_point_2d<double>(roi_ni_-1,roi_nj_-1)-r;
  pts[3]=vgl_point_2d<double>(0,roi_nj_-1)-r;
  pts[4]=vgl_point_2d<double>(0,0)-r;
  pts[5]=vgl_point_2d<double>(roi_ni_-1,0)-r;
  pts[6]=vgl_point_2d<double>(roi_ni_-1,roi_nj_-1)-r;
}
  
//=======================================================================
// Method: is_a
//=======================================================================

vcl_string masm_region_pdf_model::is_a() const
{
  return vcl_string("masm_region_pdf_model");
}


//: Create a copy on the heap and return base class pointer
masm_point_finder_model* masm_region_pdf_model::clone() const
{
  return new masm_region_pdf_model(*this);
}

//=======================================================================
// Method: print
//=======================================================================

void masm_region_pdf_model::print_summary(vcl_ostream& os) const
{
  os << "{  size: "<<roi_ni_<<" x "<<roi_nj_
     << " n_pixels: "<<n_pixels_
     << " ref_pt: ("<<ref_pt_.x()<<','<<ref_pt_.y()<<')'<<'\n';
  vsl_indent_inc(os);
  if (norm_method_==0) os<<vsl_indent()<<"norm: none"<<'\n';
  else                 os<<vsl_indent()<<"norm: linear"<<'\n';
  os <<vsl_indent()<< "PDF: ";
  if (pdf_.ptr()==0) os << "--"<<vcl_endl; else os << pdf_<<'\n';
  os<<vsl_indent();
  vsl_indent_dec(os);
  os<<vsl_indent()<<'}';
}

void masm_region_pdf_model::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,short(1));  // Version number
  vsl_b_write(bfs,roi_);
  vsl_b_write(bfs,roi_ni_);
  vsl_b_write(bfs,roi_nj_);
  vsl_b_write(bfs,n_pixels_);
  vsl_b_write(bfs,pdf_);
  vsl_b_write(bfs,ref_pt_);
  vsl_b_write(bfs,norm_method_);
  vsl_b_write(bfs,n_planes_);
  vsl_b_write(bfs,search_ni_);
  vsl_b_write(bfs,search_nj_);
}

void masm_region_pdf_model::b_read(vsl_b_istream& bfs)
{
  if (!bfs) return;
  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      vsl_b_read(bfs,roi_);
      vsl_b_read(bfs,roi_ni_);
      vsl_b_read(bfs,roi_nj_);
      vsl_b_read(bfs,n_pixels_);
      vsl_b_read(bfs,pdf_);
      vsl_b_read(bfs,ref_pt_);
      vsl_b_read(bfs,norm_method_);
      vsl_b_read(bfs,n_planes_);
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
