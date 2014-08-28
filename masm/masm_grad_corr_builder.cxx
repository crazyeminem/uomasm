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
// \brief Builds model for a masm_grad_corr_finder.

#include "masm_grad_corr_builder.h"
#include <vsl/vsl_binary_loader.h>
#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_exception.h>
#include <vul/vul_string.h>
#include <vcl_cassert.h>

#include <vil/vil_crop.h>
#include <vil/vil_math.h>
#include <vnl/vnl_math.h>
#include <vil/vil_resample_bilin.h>
#include <masm/masm_grad_corr_model.h>
#include <vgl/io/vgl_io_point_2d.h>
#include <mfpf/mfpf_pose.h>

//=======================================================================
masm_grad_corr_builder::masm_grad_corr_builder()
  : ni_(5),nj_(5),is_1d_(false),search_ni_(5),search_nj_(5)
{
}

//: Define size of template (ref point to be in centre)
void masm_grad_corr_builder::set_kernel_size(unsigned ni, unsigned nj)
{
  ni_=ni;
  nj_=nj;
  ref_pt_ = vgl_point_2d<double>(0.5*(ni-1.0),0.5*(nj-1.0));
}

  //: Creates a new model on the heap
masm_point_finder_model* masm_grad_corr_builder::new_finder() const
{
  return new masm_grad_corr_model();
}

//: Discard all samples.
void masm_grad_corr_builder::clear(unsigned)
{
  n_egs_=0;
  sum_ = vil_image_view<float>(ni_,nj_,1,2);  // Set up two plane interleaved image
  sum_.fill(0.0);
}


//: Add a sample from the given image
void masm_grad_corr_builder::add_sample(const vimt_image_2d& image, const mfpf_pose& pose)
{
  vgl_vector_2d<double> u = pose.u();
  vgl_vector_2d<double> v(-u.y(),u.x());

  vil_image_view<float> sample;
  masm_sample_grad_region(image,pose,ni_,nj_,ref_pt_,0,0,sample);
  
  // Crop sample to remove border pixels, which are zero.
  vil_math_image_sum(sum_,vil_crop(sample,1,ni_,1,nj_),sum_);
  n_egs_++;
}


  //: Build model with current set of samples
void masm_grad_corr_builder::build(masm_point_finder_model& finder)
{
  assert(finder.is_a()=="masm_grad_corr_model");
  masm_grad_corr_model& gc_finder 
    = static_cast<masm_grad_corr_model&>(finder);

  vil_math_scale_values(sum_,1.0/n_egs_);
  
  // Set to the mean patch - scaling and normalisation done in set()
  gc_finder.set(sum_,ref_pt_);
  gc_finder.set_search_area(search_ni_,search_nj_);
}


//=======================================================================
//: Print class to os
void masm_grad_corr_builder::print_summary(vcl_ostream& os) const
{
  os<<"{ ni: "<<ni_<<" nj: "<<nj_<<" is_1d: ";
  if (is_1d_) os<<"true "; else os<<"false ";
  os<<" search_ni: "<<search_ni_<<" search_nj: "<<search_nj_;
  os<<" }";
}

//: Save class to binary file stream
void masm_grad_corr_builder::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,short(1)); // Version number
  vsl_b_write(bfs,ni_);
  vsl_b_write(bfs,nj_);
  vsl_b_write(bfs,ref_pt_);
  vsl_b_write(bfs,is_1d_);
  vsl_b_write(bfs,search_ni_);
  vsl_b_write(bfs,search_nj_);
}


//: Load class from binary file stream
void masm_grad_corr_builder::b_read(vsl_b_istream& bfs)
{
  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      vsl_b_read(bfs,ni_);
      vsl_b_read(bfs,nj_);
      vsl_b_read(bfs,ref_pt_);
      vsl_b_read(bfs,is_1d_);
      vsl_b_read(bfs,search_ni_);
      vsl_b_read(bfs,search_nj_);
      break;
    default:
      vcl_cerr << "masm_grad_corr_builder::b_read() :\n"
               << "Unexpected version number " << version << vcl_endl;
      bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
      return;
  }
}

vcl_string masm_grad_corr_builder::is_a() const
{
  return vcl_string("masm_grad_corr_builder");
}

//: Create a copy on the heap and return base class pointer
masm_point_finder_builder* masm_grad_corr_builder::clone() const
{
  return new masm_grad_corr_builder(*this);
}

//: Initialise from a text stream.
void masm_grad_corr_builder::config_from_stream(vcl_istream &is)
{
  vcl_string s = mbl_parse_block(is);

  vcl_istringstream ss(s);
  mbl_read_props_type props = mbl_read_props_ws(ss);

  ni_ = vul_string_atoi(props.get_required_property("ni"));
  nj_ = vul_string_atoi(props.get_required_property("nj"));
  double rx = vul_string_atof(props.get_optional_property("ref_x","999"));
  if (rx>998) rx = 0.5*(ni_-1.0);
  double ry = vul_string_atof(props.get_optional_property("ref_y","999"));
  if (ry>998) ry = 0.5*(nj_-1.0);
  
  ref_pt_ = vgl_point_2d<double>(rx,ry);
  
  is_1d_ = vul_string_to_bool(props.get_optional_property("is_1d","false"));
  
  search_ni_ = vul_string_atoi(props.get_optional_property("search_ni","5"));
  search_nj_ = vul_string_atoi(props.get_optional_property("search_nj","5"));


  mbl_read_props_look_for_unused_props(
      "masm_grad_corr_builder::config_from_stream", props, mbl_read_props_type());
}

void masm_grad_corr_builder::reconfig_from_string(const vcl_string& param_str)
{
//  vcl_string s = mbl_parse_block(param_str);

  vcl_istringstream ss(param_str);
  mbl_read_props_type props = mbl_read_props_ws(ss);

  if (props.find("ni")!=props.end())
    ni_ = vul_string_atoi(props.get_required_property("ni"));
  if (props.find("nj")!=props.end())
    nj_ = vul_string_atoi(props.get_required_property("nj"));
  if (props.find("search_ni")!=props.end())
    search_ni_ = vul_string_atoi(props.get_required_property("search_ni"));
  if (props.find("search_nj")!=props.end())
    search_nj_ = vul_string_atoi(props.get_required_property("search_nj"));
  
  double rx = 0.5*(ni_-1.0);
  if (props.find("ref_x")!=props.end())
    rx = vul_string_atof(props.get_required_property("ref_x"));
  
  double ry = 0.5*(nj_-1.0);
  if (props.find("ref_y")!=props.end())
    ry = vul_string_atof(props.get_required_property("ref_y"));
  
  ref_pt_ = vgl_point_2d<double>(rx,ry);
  
  if (props.find("is_1d")!=props.end())
    is_1d_ = vul_string_to_bool(props.get_required_property("is_1d"));

  mbl_read_props_look_for_unused_props(
      "masm_grad_corr_builder::reconfig_from_string", props, mbl_read_props_type());

  vcl_cout<<"Reconfigured builder to "<<*this<<vcl_endl;
}

