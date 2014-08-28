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
// \brief Container for objects which can build a set of masm_point_finder objects.

#include "masm_finder_set_builder.h"
#include <vsl/vsl_binary_loader.h>
#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_parse_keyword_list.h>
#include <mbl/mbl_parse_int_list.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_exception.h>
#include <vul/vul_string.h>
#include <vcl_cassert.h>
#include <vnl/vnl_math.h>
#include <vcl_iterator.h>

#include <vil/vil_math.h>
#include <vil/vil_crop.h>
#include <vil/vil_plane.h>
#include <masm/masm_point_finder_model.h>
#include <mfpf/mfpf_pose.h>

//=======================================================================
masm_finder_set_builder::masm_finder_set_builder()
{
}


//: Define existing finders, which may contribute to new finders
//  When defined, the sampling and building may take account of the
//  existing model (eg it may personalise it to supplied data).
//  If no existing searcher so defined, build from scratch (the default)
void masm_finder_set_builder::set_old_finders(const vcl_vector<const masm_point_finder_model*>& old_finders)
{
  if (old_finders.size()==0)
  {
    for (unsigned i=0;i<builder_.size();++i)
      builder_[i]->set_old_finder(0);
    return;
  }
 
  for (unsigned i=0;i<builder_.size();++i)
      builder_[i]->set_old_finder(old_finders[i]);
}

  //: Indicate how many samples to expect
void masm_finder_set_builder::set_n_samples(unsigned n)
{
  for (unsigned i=0;i<builder_.size();++i)
  {
    builder_[i]->set_point_index(i);
    builder_[i]->clear(n);
  }
}

//: Set up builder_ and n_pts copies of the default_builder_
void masm_finder_set_builder::set_as_default(unsigned n_pts)
{
  // Set up all builders as copies of default_builder
  builder_.resize(n_pts);
  for (unsigned i=0;i<n_pts;++i) 
  {
    builder_[i] = *default_builder_;
    builder_[i]->set_point_index(i);

    // Check if extra parameters exist for this point.
    if (builder_params_.find(i)!=builder_params_.end())
      builder_[i]->reconfig_from_string(builder_params_[i]);
  }
}

//: Discard all samples and indicate number of points and number of samples to expect
void masm_finder_set_builder::clear(unsigned n_pts, unsigned n_samples)
{
  if (builder_.size()!=n_pts) set_as_default(n_pts);

  for (unsigned i=0;i<builder_.size();++i)
    builder_[i]->clear(n_samples);
}

//: Select best level for searching around pose with finder
//  Selects pyramid level with pixel sizes best matching
//  the model pixel size at given pose.
inline unsigned image_level(const mfpf_pose& pose, double step_size,
                     const vimt_image_pyramid& im_pyr)
{
  double model_pixel_size = step_size*pose.scale();
  double rel_size0 = model_pixel_size/im_pyr.base_pixel_width();

  double log_step = vcl_log(im_pyr.scale_step());

  // Round level down, to work with slightly higher res. image.
  int level = int(vcl_log(rel_size0)/log_step);
  if (level<im_pyr.lo()) return im_pyr.lo();
  if (level>im_pyr.hi()) return im_pyr.hi();
  return level;
}



//: Add a sample from the given image
//  Points are in world coordinates.
void masm_finder_set_builder::add_sample(const vimt_image_pyramid& image_pyr,
                                         const vcl_vector<mfpf_pose>& pose)
{
  for (unsigned j=0;j<n_points();++j)
  {
    // Select relevant level from pyramid
    unsigned L=image_level(pose[j],1.0,image_pyr);
    const vimt_image_2d& imageL
          = static_cast<const vimt_image_2d&>(image_pyr(L));

    builder_[j]->add_sample(imageL,pose[j]);
  }
}

  //: Build model with current set of samples
void masm_finder_set_builder::build(vcl_vector<masm_point_finder_model*>& finder)
{
  if (finder.size()!=n_points())
  {
    finder.resize(n_points());
    for (unsigned i=0;i<n_points();++i)
    {
      finder[i]=builder_[i]->new_finder();
    }
  }
  
  for (unsigned i=0;i<n_points();++i)
  {
    builder_[i]->build(*finder[i]);
  }
}


//=======================================================================
//: Print class to os
void masm_finder_set_builder::print_summary(vcl_ostream& os) const
{
  os<<" default_builder: ";
  if (default_builder_.isDefined()) os<<*default_builder_<<"\n";
  else os<<"-"<<vcl_endl;
  if (builder_params_.size()>0)
  {
    os<<"override_params: {\n";
    vcl_map<unsigned,vcl_string>::const_iterator it = builder_params_.begin();
    for (;it!=builder_params_.end();++it)
    {
      os<<"  params: { index: "<<it->first<<" data: "<<it->second<<" }\n";
    }
    os<<"}"<<vcl_endl;
  }
}

//: Save class to binary file stream
void masm_finder_set_builder::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,short(1)); // Version number
  vcl_cout<<"masm_finder_set_builder::b_write() Not implemented."<<vcl_endl;
}


//: Load class from binary file stream
void masm_finder_set_builder::b_read(vsl_b_istream& bfs)
{
  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      break;
    default:
      vcl_cerr << "masm_finder_set_builder::b_read() :\n"
               << "Unexpected version number " << version << vcl_endl;
      bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
      return;
  }
}

//: Assumes str is either an integer "35", or a set "{ 34 56 23 : 26 }"
//  Extracts ints
inline vcl_vector<unsigned> get_ints(const vcl_string& str)
{
  vcl_vector<unsigned> int_list;
  if (str[0]=='{')
  {
    vcl_stringstream ss(str);
    mbl_parse_int_list(ss,vcl_back_inserter(int_list),unsigned());
  }
  else
  {
    int_list.resize(1);
    int_list[0]=vul_string_atoi(str);
  }
  return int_list;
}

//: Initialise from a text stream.
// Expects input such as: "{ default_builder: masm_ncc_finder_builder { ni: 7 nj: 7 } }"
void masm_finder_set_builder::config_from_stream(vcl_istream &is)
{
  vcl_string s = mbl_parse_block(is);

  vcl_istringstream ss(s);
  mbl_read_props_type props = mbl_read_props_ws(ss);

  {
    vcl_istringstream sps(props.get_required_property("default_builder"));
    vcl_auto_ptr<masm_point_finder_builder> builder
           =masm_point_finder_builder::create_from_stream(sps);
    default_builder_=builder->clone();
  }

  vcl_string override_params = props.get_optional_property("override_params","");
  if (override_params!="")
  {
    vcl_istringstream data_s(override_params);
    vcl_vector<vcl_string> builder_params;
    mbl_parse_keyword_list(data_s,"params:",builder_params);

    // Parse each element in turn
    for (unsigned i=0;i<builder_params.size();++i)
    {
      vcl_istringstream bss(builder_params[i]);
      mbl_read_props_type bprops = mbl_read_props_ws(bss);
      vcl_vector<unsigned> index = get_ints(bprops.get_required_property("index"));
      vcl_string param_data = bprops.get_required_property("data");;
      for (unsigned j=0;j<index.size();++j)
      {
        // Check for repeats
        if (builder_params_.find(index[j])!=builder_params_.end())
        {
          vcl_ostringstream err_ss;
          err_ss<<"masm_finder_set_builder index of "<<index[j]<<" is repeated.";
          throw mbl_exception_parse_error(err_ss.str());
        }

        builder_params_[index[j]]=param_data;
      }
    }

  }

  mbl_read_props_look_for_unused_props(
      "masm_finder_set_builder::config_from_stream", props, mbl_read_props_type());
}


