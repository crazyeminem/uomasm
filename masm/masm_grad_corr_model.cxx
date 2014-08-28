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


#include "masm_grad_corr_model.h"

#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>
#include <vil/io/vil_io_image_view.h>
#include <vgl/io/vgl_io_point_2d.h>

#include <vil/vil_resample_bilin.h>
#include <vil/algo/vil_sobel_3x3.h>

#include <vcl_cassert.h>
#include <masm/masm_grad_corr_finder.h>

//=======================================================================
// Dflt ctor
//=======================================================================

masm_grad_corr_model::masm_grad_corr_model()
  : search_ni_(5),search_nj_(5)
{
}

//=======================================================================
// Destructor
//=======================================================================

masm_grad_corr_model::~masm_grad_corr_model()
{
}

//: Define kernel and the reference point within it.
void masm_grad_corr_model::set(const vil_image_view<float>& kernel, const vgl_point_2d<double>& ref_pt)
{
  kernel_.deep_copy(kernel);
  ref_pt_ = ref_pt;
}

//: Define search area in terms of steps either side of base point
//  Units of pose.u()
void masm_grad_corr_model::set_search_area(unsigned ni, unsigned nj)
{
  search_ni_=ni;
  search_nj_=nj;
}

//: Creates a finder on the heap and sets this as its model
masm_point_finder* masm_grad_corr_model::create_finder() const
{
  masm_grad_corr_finder* finder = new masm_grad_corr_finder();
  finder->set_model(*this);
  return finder;
}

//: Generate points in ref frame that represent boundary
//  Points of a contour around the shape.
//  Used for display purposes.  Join the points with an open
//  contour to get a representation.
void masm_grad_corr_model::get_outline(vcl_vector<vgl_point_2d<double> >& pts) const
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

vcl_string masm_grad_corr_model::is_a() const
{
  return vcl_string("masm_grad_corr_model");
}


//: Create a copy on the heap and return base class pointer
masm_point_finder_model* masm_grad_corr_model::clone() const
{
  return new masm_grad_corr_model(*this);
}

//=======================================================================
// Method: print
//=======================================================================

void masm_grad_corr_model::print_summary(vcl_ostream& os) const
{
  os<<"{  size: "<<kernel_.ni()<<" x "<<kernel_.nj()
    <<" ref:("<<ref_pt_.x()<<","<<ref_pt_.y()<<")"
    <<" search_ni: "<<search_ni_<<" search_nj: "<<search_nj_<<" }";
}

void masm_grad_corr_model::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,short(1));  // Version number
  vsl_b_write(bfs,kernel_);
  vsl_b_write(bfs,ref_pt_);
  vsl_b_write(bfs,search_ni_);
  vsl_b_write(bfs,search_nj_);
}

void masm_grad_corr_model::b_read(vsl_b_istream& bfs)
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

//: Sample region required to do search.
//  Computes unit gradient direction at each point.
void masm_sample_grad_region(const vimt_image_2d& image, 
                                    const mfpf_pose& base_pose,
                                    unsigned kernel_ni, unsigned kernel_nj,
                                    const vgl_point_2d<double>& ref_pt,
                                    unsigned search_ni, unsigned search_nj,
                                    vil_image_view<float>& sample)
{  
  unsigned kni = kernel_ni;
  unsigned knj = kernel_nj;
  int nsi = 2*search_ni + kni +2;  // Sample 2 wider region to allow for gradient.
  int nsj = 2*search_nj + knj +2;

  vgl_vector_2d<double> u1=base_pose.u();
  vgl_vector_2d<double> v1(-u1.y(),u1.x());
  const vgl_point_2d<double> p0 = base_pose.p()-(1+search_ni+ref_pt.x())*u1
                                               -(1+search_nj+ref_pt.y())*v1;

  const vimt_transform_2d& s_w2i = image.world2im();
  vgl_point_2d<double> im_p0 = s_w2i(p0);
  vgl_vector_2d<double> im_u = s_w2i.delta(p0, u1);
  vgl_vector_2d<double> im_v = s_w2i.delta(p0, v1);
  
  vil_image_view<float> patch;
  switch (image.image_base().pixel_format())
  {
    case VIL_PIXEL_FORMAT_BYTE:
      vil_resample_bilin(static_cast<const vimt_image_2d_of<vxl_byte>&>(image).image(),patch,
                     im_p0.x(),im_p0.y(),  im_u.x(),im_u.y(), im_v.x(),im_v.y(), nsi,nsj);
      break;
    case VIL_PIXEL_FORMAT_FLOAT:
      vil_resample_bilin(static_cast<const vimt_image_2d_of<float>&>(image).image(),patch,
                     im_p0.x(),im_p0.y(),  im_u.x(),im_u.y(), im_v.x(),im_v.y(), nsi,nsj);
      break;
    default:
      vcl_cerr<<"masm_grad_corr_finder::fit_at_point Unknown pixel format."<<vcl_endl;
      abort();
  }
  
  // Compute gradients in patch using Sobel
  sample=vil_image_view<float>(nsi,nsj,1,2);  // Interleaved planes, so planestep=1, istep=2
  vil_sobel_3x3(patch,sample);

  // Normalise each gradient to unit length
  float* sam = &sample(1,1);
  vcl_ptrdiff_t jstep = sample.jstep();
  unsigned nsi2=nsi-2;
  unsigned nsj2=nsj-2;
  for (unsigned j=0;j<nsj2;++j,sam+=jstep)
  {
    float *gx=sam;
    float *gy=sam+1;
    for (unsigned i=0;i<nsi2;++i, gx+=2,gy+=2)
    {
      float g2 = (*gx * *gx) + (*gy * *gy);
      if (g2>1e-6) { float g=vcl_sqrt(g2); *gx/=g; *gy/=g; }
    }
  }
}
