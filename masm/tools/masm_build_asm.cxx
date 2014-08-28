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
// \brief Tool to build an Active Shape Model from data in files.
// \author Tim Cootes

/*
This is only a single resolution model.
Typically a sequence of models, at increasing resolutions, will give better results.
*/

#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_exception.h>
#include <mbl/mbl_parse_colon_pairs_list.h>
#include <mbl/mbl_parse_int_list.h>
#include <vul/vul_arg.h>
#include <vul/vul_string.h>
#include <vcl_sstream.h>
#include <vcl_fstream.h>
#include <vcl_string.h>
#include <vcl_iterator.h>
#include <vsl/vsl_quick_file.h>

#include <msm/msm_shape_model_builder.h>
#include <msm/msm_curve.h>

#include <masm/masm_model.h>
#include <masm/masm_builder.h>

#include <mbl/mbl_cloneables_factory.h>
#include <msm/msm_add_all_loaders.h>

#include <msdi/msdi_marked_images_from_files.h>
#include <msdi/msdi_reflected_marked_images.h>

#include <masm/masm_bldr_add_all_loaders.h>
#include <masm/masm_point_finder_builder.h>

/*
Parameter file format:
<START FILE>
//: File to save model to
asm_path: face.masm

asm_builder: {
  //: Aligner for shape model
  aligner: msm_similarity_aligner

  //: Object to apply limits to parameters
  param_limiter: msm_ellipsoid_limiter { accept_prop: 0.98 }

  // Maximum number of shape modes
  max_modes: 99

  // Proportion of shape variation to explain
  var_prop: 0.95


  curves_path: /home/bim/images/faces/tfc_train1/models/face_28pts.crvs

  // Sampling step size is set so that there are frame_width such steps across the mean shape.
  frame_width: 50

  // Define the type of builder for the local models
  finder_builders: { 
    default_builder: masm_ncc_finder_builder { ni: 11 nj: 11 search_ni: 5 search_nj: 3 }
    override_params: {
      params: { index: 14 data: { ni: 13 nj: 13 } } 
      params: { index: { 0 1 } data: { ni: 13 nj: 13 } }
      params: { index: { 4 : 7 } data: { ni: 13 nj: 13 } }
    }
  }

}

//: Define renumbering required under reflection
//  If defined, a reflected version of each shape is included in build
reflection_symmetry: { 1 0  3 2   7 6 5 4  13 12 11 10 9 8  14  16 15 17 18 19   21 20   23 22  26 27  24 25 }

// Use both original and the reflection
only_reflect: false

image_dir: /home/images/
points_dir: /home/points/
images: {
  image1.pts : image1.jpg
  image2.pts : image2.jpg
}

<END FILE>
*/

void print_usage()
{
  vcl_cout << "masm_build_asm -p param_file\n"
           << "Builds an Active Shape Model from the supplied data.\n"
           << vcl_endl;

  vul_arg_display_usage_and_exit();
}

//: Structure to hold parameters
struct tool_params
{
  // Object to build the ASM
  masm_builder asm_builder;

  //: Directory containing images
  vcl_string image_dir;

  //: Directory containing points
  vcl_string points_dir;

  //: File to save ASM to
  vcl_string asm_path;
  
  //: Define renumbering required under reflection
  //  If defined, a reflected version of each shape is included in build
  vcl_vector<unsigned> reflection_symmetry;

  //: When true, only use reflection. When false, use both reflection and original.
  bool only_reflect;

  //: List of image names
  vcl_vector<vcl_string> image_names;

  //: List of points file names
  vcl_vector<vcl_string> points_names;

  //: Parse named text file to read in data
  //  Throws a mbl_exception_parse_error if fails
  void read_from_file(const vcl_string& path);
};

//: Parse named text file to read in data
//  Throws a mbl_exception_parse_error if fails
void tool_params::read_from_file(const vcl_string& path)
{
  vcl_ifstream ifs(path.c_str());
  if (!ifs)
  {
    vcl_string error_msg = "Failed to open file: "+path;
    throw (mbl_exception_parse_error(error_msg));
  }

  mbl_read_props_type props = mbl_read_props_ws(ifs);

  image_dir=props.get_optional_property("image_dir","./");
  points_dir=props.get_optional_property("points_dir","./");
  asm_path=props.get_optional_property("asm_path",
                                       "shape_aam.bfs");

  {
    vcl_string builder_str 
       = props.get_required_property("asm_builder");
    vcl_stringstream ss(builder_str);
    asm_builder.config_from_stream(ss,"");
  }
 
  vcl_string ref_sym_str
       =props.get_optional_property("reflection_symmetry","");
  reflection_symmetry.resize(0);
  if (ref_sym_str!="")
  {
    vcl_stringstream ss(ref_sym_str);
    mbl_parse_int_list(ss, vcl_back_inserter(reflection_symmetry),
                       unsigned());
  }

  only_reflect=vul_string_to_bool(props.get_optional_property("only_reflect","false"));

  mbl_parse_colon_pairs_list(props.get_required_property("images"),
                             points_names,image_names);

  // Don't look for unused props so can use a single common parameter file.
}

int main(int argc, char** argv)
{
  vul_arg<vcl_string> param_path("-p","Parameter filename");
  vul_arg_parse(argc,argv);

  msm_add_all_loaders();
  masm_bldr_add_all_loaders();

  if (param_path()=="")
  {
    print_usage();
    return 0;
  }

  tool_params params;
  try 
  { 
    params.read_from_file(param_path()); 
  }
  catch (mbl_exception_parse_error& e)
  {
    vcl_cerr<<"Error: "<<e.what()<<vcl_endl;
    return 1;
  }

  
  //: Create objects to load in points and images
  msdi_marked_images_from_files marked_files;

//  marked_files.pyr_builder().set_max_levels(params.max_im_pyr_levels);
  marked_files.set(params.image_dir,
                    params.image_names,
                    params.points_dir,
                    params.points_names);

  msdi_marked_images *marked_images = &marked_files;

  msdi_reflected_marked_images reflected_data(marked_files,
                                              params.reflection_symmetry,
                                              params.only_reflect);
  if (params.reflection_symmetry.size()>0)
  {
    marked_images=&reflected_data;
    vcl_cout<<"Including reflections: "<<2*params.image_names.size()<<" examples."<<vcl_endl;
  }


  masm_model new_model;
  
  params.asm_builder.build(*marked_images,new_model);

  vcl_cout<<"Active Shape Model: "<<new_model<<vcl_endl;

  if (vsl_quick_file_save(new_model,params.asm_path))
  {
    vcl_cout<<"Saved ASM to "
            <<params.asm_path<<vcl_endl;
  }
  else
  {
    vcl_cerr<<"Failed to save to "<<params.asm_path<<vcl_endl;
    return 3;
  }

  return 0;
}
