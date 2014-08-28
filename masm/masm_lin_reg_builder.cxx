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
// \brief Builds a masm_lin_reg_model.

#include "masm_lin_reg_builder.h"
#include <vsl/vsl_binary_loader.h>
#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_exception.h>
#include <vul/vul_string.h>
#include <vcl_cassert.h>

#include <vil/vil_resample_bilin.h>
#include <masm/masm_lin_reg_model.h>
#include <vgl/io/vgl_io_point_2d.h>
#include <mfpf/mfpf_pose.h>
#include <vsl/vsl_vector_io.h>

#include <mfpf/mfpf_sample_region.h>
#include <mfpf/mfpf_norm_vec.h>
#include <vsl/vsl_indent.h>
#include <vnl/algo/vnl_svd.h>

//=======================================================================
masm_lin_reg_builder::masm_lin_reg_builder()
  : n_pixels_(0),norm_method_(1),n_planes_(1),is_1d_(false),
    max_di_(2.5),max_dj_(2.5), search_ni_(1),search_nj_(1)
{
  rand_.reseed(473827);
}

  //: Creates a new model on the heap
masm_point_finder_model* masm_lin_reg_builder::new_finder() const
{
  return new masm_lin_reg_model();
}

//: Define model region as an ni x nj box
void masm_lin_reg_builder::set_as_box(unsigned ni, unsigned nj,
                                         const vgl_point_2d<double>& ref_pt)
{
  roi_ni_=ni; roi_nj_=nj;
  n_pixels_ = ni*nj;

  // Set ROI to be a box
  roi_.resize(nj);
  for (unsigned j=0;j<nj;++j) roi_[j]=mbl_chord(0,ni-1,j);

  ref_pt_=ref_pt;
}


//: Define model region as an ni x nj box
void masm_lin_reg_builder::set_as_box(unsigned ni, unsigned nj)
{
  set_as_box(ni,nj, vgl_point_2d<double>(0.5*(ni-1),0.5*(nj-1)));
}

//: Define model region as an ellipse with radii ri, rj
//  Ref. point in centre.
void masm_lin_reg_builder::set_as_ellipse(double ri, double rj)
{
  ri+=1e-6; rj+=1e-6;
  int ni=int(ri);
  int nj=int(rj);
  roi_.resize(0);
  n_pixels_=0;
  for (int j = -nj;j<=nj;++j)
  {
    // Find start and end of line of pixels inside disk
    int x = int(ri*vcl_sqrt(1.0-j*j/(rj*rj)));
    roi_.push_back(mbl_chord(ni-x,ni+x,nj+j));
    n_pixels_+=2*x+1;
  }

  ref_pt_=vgl_point_2d<double>(ni,nj);
  roi_ni_=2*ni+1;
  roi_nj_=2*nj+1;
}

//: Define number of random displacements to make around each example
void masm_lin_reg_builder::set_n_random(unsigned n)
{
  n_random_=n;
}

//: Define maximum displacement used during training
void masm_lin_reg_builder::set_max_displacement(double max_di, double max_dj)
{
  max_di_=max_di;
  max_dj_=max_dj;
}

//: Discard all samples.
void masm_lin_reg_builder::clear(unsigned)
{
  data_.resize(0);
}

//: Add a sample from the given image, displaced by (dx,dy) in pose frame
void masm_lin_reg_builder::add_one_sample(const vimt_image_2d& image, const mfpf_pose& pose, double dx, double dy)
{
  vgl_vector_2d<double> u = pose.u();
  vgl_vector_2d<double> v(-u.y(),u.x());

  unsigned np=image.image_base().nplanes();
  
  if (data_.size()==0) n_planes_=np;
  else assert(n_planes_==np);
  
  // Set up sample area with interleaved planes (ie planestep==1)
  vil_image_view<float> sample(roi_ni_,roi_nj_,1,np);

  const vgl_point_2d<double> p0 = pose.p() +(dx-ref_pt_.x())*u + (dy-ref_pt_.y())*v;

  const vimt_transform_2d& s_w2i = image.world2im();
  vgl_point_2d<double> im_p0 = s_w2i(p0);
  vgl_vector_2d<double> im_u = s_w2i.delta(p0, u);
  vgl_vector_2d<double> im_v = s_w2i.delta(p0, v);

  switch (image.image_base().pixel_format())
  {
    case VIL_PIXEL_FORMAT_BYTE:
      vil_resample_bilin(static_cast<const vimt_image_2d_of<vxl_byte>&>(image).image(),sample,
                     im_p0.x(),im_p0.y(),  im_u.x(),im_u.y(), im_v.x(),im_v.y(), roi_ni_,roi_nj_);
      break;
    case VIL_PIXEL_FORMAT_FLOAT:
      vil_resample_bilin(static_cast<const vimt_image_2d_of<float>&>(image).image(),sample,
                     im_p0.x(),im_p0.y(),  im_u.x(),im_u.y(), im_v.x(),im_v.y(), roi_ni_,roi_nj_);
      break;
    default:
      vcl_cerr<<"masm_lin_reg_model::fit_at_point Unknown pixel format."<<vcl_endl;
      abort();
  }
  
  vnl_vector<double> vec(n_pixels_*n_planes_);
  mfpf_sample_region(sample.top_left_ptr(),sample.jstep(),
                     np,roi_,vec);

  if (norm_method_==1) mfpf_norm_vec(vec);
  data_.push_back(vec);
  
  // Record offset required to move back to true position
  vnl_vector<double> offset(2); offset[0]=-dx; offset[1]=-dy;
  true_offset_.push_back(offset);
}

//: Add a sample from the given image
// Note: Currently doesn't include local rotation/scale
void masm_lin_reg_builder::add_sample(const vimt_image_2d& image, const mfpf_pose& pose)
{
  // Add sample at true position
  add_one_sample(image,pose,0.0,0.0);
  
  for (unsigned i=0;i<n_random_;++i)
  {
    double dx = rand_.drand64(-max_di_,max_di_);
    double dy = rand_.drand64(-max_dj_,max_dj_);
    
    // Add symmetric samples
    add_one_sample(image,pose,dx,dy);
    add_one_sample(image,pose,-dx,-dy); 
  }
}

inline vnl_vector<double> get_mean(const vcl_vector<vnl_vector<double> >& x)
{
  if (x.size()==0) return vnl_vector<double>();
  vnl_vector<double> sum= x[0];
  for (unsigned i=1;i<x.size();++i) sum+=x[i];
  return sum/x.size();
}

//: M += vec1*vec2.transpose();
static void update_with_outer_product(vnl_matrix<double>& M,
                               const vnl_vector<double>& vec1,
                               const vnl_vector<double>& vec2)
{
  assert(M.rows()==vec1.size());
  assert(M.columns()==vec2.size());

  double** m = M.data_array();
  const double* v1 = vec1.data_block();
  const double* v2 = vec2.data_block();
  const unsigned n2=vec2.size();
  for (unsigned int i=0;i<vec1.size();++i)
  {
    double *row = m[i];
    double v1i = v1[i];
    for (unsigned j=0;j<n2;++j)
      row[j] += v1i*v2[j];
  }
}

//: Build model with current set of samples
// Assume more samples than variables in input vector
void masm_lin_reg_builder::build(masm_point_finder_model& model)
{
  assert(model.is_a()=="masm_lin_reg_model");
  masm_lin_reg_model& lin_reg_model 
    = static_cast<masm_lin_reg_model&>(model);
  
  // Perform regression to compute weight vectors
  vnl_vector<double> in_mean = get_mean(data_);
  vnl_vector<double> out_mean = get_mean(true_offset_);

  unsigned n1 = in_mean.size();
  unsigned n2 = out_mean.size();
  vnl_matrix<double> XtX(n1,n1),XtY(n1,n2);
  XtX.fill(0.0);
  XtY.fill(0.0);

  vnl_vector<double> xi,yi;
  for (unsigned i=0;i<data_.size();++i)
  {
    xi = (data_[i])-in_mean;
    yi = (true_offset_[i])-out_mean;
    update_with_outer_product(XtX,xi,xi);
    update_with_outer_product(XtY,xi,yi);
  }
  
  vnl_svd<double> svd(XtX,-1.0e-8);
  vnl_matrix<double> R=svd.solve(XtY);
  
  // y=R'(x-x_mean)+y_mean
  // y=R'x+(y_mean-R'.x_mean);

  vnl_vector<double> rx=R.get_column(0);
  double rx0 = out_mean(0) - dot_product(rx,in_mean);
  vnl_vector<double> ry=R.get_column(1);
  double ry0 = out_mean(1) - dot_product(ry,in_mean);
  
  lin_reg_model.set(roi_,n_planes_,ref_pt_,rx,rx0, ry,ry0,norm_method_);
  lin_reg_model.set_search_area(search_ni_,search_nj_);
  
  // Tidy up
  data_.resize(0);
}


//=======================================================================
//: Print class to os
void masm_lin_reg_builder::print_summary(vcl_ostream& os) const
{
  os<<"{ ni: "<<roi_ni_<<" nj: "<<roi_nj_<<"\n";
  vsl_indent_inc(os);

  os<<vsl_indent()<<" is_1d: ";
  if (is_1d_) os<<"true "; else os<<"false ";
  if (norm_method_==0) os<<vsl_indent()<<"norm: none"<<'\n';
  else                 os<<vsl_indent()<<"norm: linear"<<'\n';
  os <<vsl_indent()<< "n_random: "<<n_random_<<" max_di: "<<max_di_<<" max_dj: "<<max_dj_<<vcl_endl;
  os<<vsl_indent()<<" search_ni: "<<search_ni_<<" search_nj: "<<search_nj_;
  os<<vsl_indent()<<" }";
  vsl_indent_dec(os);
}

//: Save class to binary file stream
void masm_lin_reg_builder::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,short(1)); // Version number
  vsl_b_write(bfs,roi_);
  vsl_b_write(bfs,roi_ni_);
  vsl_b_write(bfs,roi_nj_);
  vsl_b_write(bfs,n_pixels_);
  vsl_b_write(bfs,ref_pt_);
  vsl_b_write(bfs,is_1d_);
  vsl_b_write(bfs,n_random_);
  vsl_b_write(bfs,max_di_);
  vsl_b_write(bfs,max_dj_);
  vsl_b_write(bfs,norm_method_);
  vsl_b_write(bfs,search_ni_);
  vsl_b_write(bfs,search_nj_);
}


//: Load class from binary file stream
void masm_lin_reg_builder::b_read(vsl_b_istream& bfs)
{
  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      vsl_b_read(bfs,roi_);
      vsl_b_read(bfs,roi_ni_);
      vsl_b_read(bfs,roi_nj_);
      vsl_b_read(bfs,n_pixels_);
      vsl_b_read(bfs,ref_pt_);
      vsl_b_read(bfs,is_1d_);
      vsl_b_read(bfs,n_random_);
      vsl_b_read(bfs,max_di_);
      vsl_b_read(bfs,max_dj_);
      vsl_b_read(bfs,norm_method_);
      vsl_b_read(bfs,search_ni_);
      vsl_b_read(bfs,search_nj_);
      break;
    default:
      vcl_cerr << "masm_lin_reg_builder::b_read() :\n"
               << "Unexpected version number " << version << vcl_endl;
      bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
      return;
  }
}

vcl_string masm_lin_reg_builder::is_a() const
{
  return vcl_string("masm_lin_reg_builder");
}

//: Create a copy on the heap and return base class pointer
masm_point_finder_builder* masm_lin_reg_builder::clone() const
{
  return new masm_lin_reg_builder(*this);
}

//: Parse stream to set up as a box shape.
// Expects: "{ ni: 3 nj: 5 ref_x: 1.0 ref_y: 2.0 }
void masm_lin_reg_builder::config_as_box(vcl_istream &is)
{
  // Cycle through string and produce a map of properties
  vcl_string s = mbl_parse_block(is);
  vcl_istringstream ss(s);
  mbl_read_props_type props = mbl_read_props_ws(ss);

  unsigned ni=5,nj=5;

  if (props.find("ni")!=props.end())
  {
    ni=vul_string_atoi(props["ni"]);
    props.erase("ni");
  }
  if (props.find("nj")!=props.end())
  {
    nj=vul_string_atoi(props["nj"]);
    props.erase("nj");
  }

  double ref_x,ref_y;
  if (props.find("ref_x")!=props.end())
  {
     ref_x=vul_string_atof(props.get_required_property("ref_x"));
  }
  else ref_x=0.5*(ni-1.0);

  if (props.find("ref_y")!=props.end())
  {
    ref_y=vul_string_atof(props.get_required_property("ref_y"));
  }
  else ref_y=0.5*(nj-1.0);

  // Check for unused props
  mbl_read_props_look_for_unused_props(
      "masm_lin_reg_builder::config_as_box",
      props, mbl_read_props_type());

  set_as_box(ni,nj,vgl_point_2d<double>(ref_x,ref_y));
}

//: Parse stream to set up as an ellipse shape.
// Expects: "{ ri: 2.1 rj: 5.2 }
void masm_lin_reg_builder::config_as_ellipse(vcl_istream &is)
{
  // Cycle through string and produce a map of properties
  vcl_string s = mbl_parse_block(is);
  vcl_istringstream ss(s);
  mbl_read_props_type props = mbl_read_props_ws(ss);

  double ri = vul_string_atof(props.get_optional_property("ri","3.1"));
  double rj = vul_string_atof(props.get_optional_property("rj","3.1"));

  // Check for unused props
  mbl_read_props_look_for_unused_props(
      "masm_lin_reg_builder::config_as_ellipse",
      props, mbl_read_props_type());

  set_as_ellipse(ri,rj);
}


//: Initialise from a text stream.
void masm_lin_reg_builder::config_from_stream(vcl_istream &is)
{
  vcl_string s = mbl_parse_block(is);

  vcl_istringstream ss(s);
  mbl_read_props_type props = mbl_read_props_ws(ss);
  
  if (props.find("shape")!=props.end())
  {
    vcl_istringstream shape_s(props["shape"]);
    shape_s>>shape_;
    if (shape_=="box")
    {
      // Parse parameters after box
      config_as_box(shape_s);
    }
    else
    if (shape_=="ellipse")
    {
      // Parse parameters after ellipse
      config_as_ellipse(shape_s);
    }
    else throw mbl_exception_parse_error("Unknown shape: "+shape_);

    props.erase("shape");
  }

  if (props.find("norm")!=props.end())
  {
    if (props["norm"]=="none") norm_method_=0;
    else
    if (props["norm"]=="linear") norm_method_=1;
    else throw mbl_exception_parse_error("Unknown norm: "+props["norm"]);

    props.erase("norm");
  }
  
  is_1d_ = vul_string_to_bool(props.get_optional_property("is_1d","false"));
  
  n_random_ = vul_string_atoi(props.get_optional_property("n_random","9"));
  max_di_ = vul_string_atof(props.get_optional_property("max_di","3.5"));
  max_dj_ = vul_string_atof(props.get_optional_property("max_dj","3.5"));
  
  search_ni_ = vul_string_atoi(props.get_optional_property("search_ni","5"));
  search_nj_ = vul_string_atoi(props.get_optional_property("search_nj","5"));


  mbl_read_props_look_for_unused_props(
      "masm_lin_reg_builder::config_from_stream", props, mbl_read_props_type());
}

void masm_lin_reg_builder::reconfig_from_string(const vcl_string& param_str)
{
  vcl_istringstream ss(param_str);
  mbl_read_props_type props = mbl_read_props_ws(ss);
  
  if (props.find("shape")!=props.end())
  {
    vcl_istringstream shape_s(props["shape"]);
    shape_s>>shape_;
    if (shape_=="box")
    {
      // Parse parameters after box
      config_as_box(shape_s);
    }
    else
    if (shape_=="ellipse")
    {
      // Parse parameters after ellipse
      config_as_ellipse(shape_s);
    }
    else throw mbl_exception_parse_error("Unknown shape: "+shape_);

    props.erase("shape");
  }

  if (props.find("norm")!=props.end())
  {
    if (props["norm"]=="none") norm_method_=0;
    else
    if (props["norm"]=="linear") norm_method_=1;
    else throw mbl_exception_parse_error("Unknown norm: "+props["norm"]);

    props.erase("norm");
  }

  if (props.find("n_random")!=props.end())
    n_random_ = vul_string_atoi(props.get_required_property("n_random"));
  if (props.find("max_di")!=props.end())
    max_di_ = vul_string_atof(props.get_required_property("max_di"));
  if (props.find("max_dj")!=props.end())
    max_dj_ = vul_string_atof(props.get_required_property("max_dj"));

  if (props.find("search_ni")!=props.end())
    search_ni_ = vul_string_atoi(props.get_required_property("search_ni"));
  if (props.find("search_nj")!=props.end())
    search_nj_ = vul_string_atoi(props.get_required_property("search_nj"));
    
  if (props.find("is_1d")!=props.end())
    is_1d_ = vul_string_to_bool(props.get_required_property("is_1d"));

  mbl_read_props_look_for_unused_props(
      "masm_lin_reg_builder::reconfig_from_string", props, mbl_read_props_type());

  vcl_cout<<"Reconfigured builder to "<<*this<<vcl_endl;
}

