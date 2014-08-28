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
// \brief Tool to run a series of ASMs on one or more images.
// \author Tim Cootes

#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_exception.h>
#include <mbl/mbl_parse_string_list.h>
#include <vul/vul_arg.h>
#include <vul/vul_string.h>
#include <vul/vul_file.h>
#include <vcl_sstream.h>
#include <vcl_fstream.h>
#include <vcl_string.h>
#include <vsl/vsl_quick_file.h>

#include <msm/msm_curve.h>
#include <msm/utils/msm_draw_shape_to_eps.h>

#include <masm/masm_model_series.h>
#include <masm/masm_series_searcher.h>

#include <mbl/mbl_cloneables_factory.h>
#include <msm/msm_add_all_loaders.h>

#include <masm/masm_add_all_loaders.h>

#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vil/vil_convert.h>
#include <vimt/vimt_image_2d_of.h>
#include <vimt/vimt_image_pyramid.h>
#include <vimt/vimt_gaussian_pyramid_builder_2d.h>
#include <vil/algo/vil_gauss_reduce.h>
#include <vil/vil_resample_bilin.h>

/*
Parameter file format:
<START FILE>

//: File containing ASM series
asm_path: face.masm

curves_path: face_front.crvs

// Width of output image (input will be scaled to this)
//  If zero, don't generate output images
out_image_width: 128

// Directory to save output frames to
output_dir: ./asm_results/

// Directory to save the final points to
output_points_dir: ./asm_points/

image_dir: /home/images/
points_dir: /home/points/
images: {
  image1.jpg
  image2.jpg
}

<END FILE>
*/

void print_usage()
{
  vcl_cout << "masm_series_search -p param_file\n"
           << "Tool to run a sequence of ASMs on one or more images.\n"
           << vcl_endl;

  vul_arg_display_usage_and_exit();
}

//: Structure to hold parameters
struct tool_params
{

  //: Directory containing images
  vcl_string image_dir;

  //: Directory containing points
  vcl_string points_dir;

  //: File to save ASM to
  vcl_string asm_path;

  vcl_string curves_path;

  //: Width of output image (input will be scaled to this)
  unsigned out_image_width;

  // Directory to save output frames to
  vcl_string output_dir;

  // Directory to save the final points to
  vcl_string output_points_dir;

  //: List of image names
  vcl_vector<vcl_string> image_names;

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

  curves_path=props["curves_path"];

  image_dir=props.get_optional_property("image_dir","./");
  points_dir=props.get_optional_property("points_dir","./");
  asm_path=props.get_optional_property("asm_path",
                                       "shape_aam.bfs");
  output_dir=props.get_optional_property("output_dir","./asm_frames/");
  output_points_dir=props.get_optional_property("output_points_dir",
                                                 "./asm_points/");
  out_image_width=vul_string_atoi(props.get_optional_property("out_image_width","128"));

  mbl_parse_string_list(props.get_required_property("images"),
                        image_names);

  // Don't look for unused props so can use a single common parameter file.
}

//: Rescales image so it has given width
//  scale is the scaling factor applied
void create_output_image(const vil_image_view<vxl_byte>& image,
                         unsigned im_width,
                         vil_image_view<vxl_byte>& new_image,
                         double& scale)
{
  // First shrink the image by factors of two until it's about right
  unsigned ni=image.ni();
  vil_image_view<vxl_byte> tmp_im = image;
  while (tmp_im.ni()>4*im_width)
  {
    vil_gauss_reduce_121(tmp_im,new_image);
    tmp_im=new_image;
  }
  scale = double(im_width)/ni;
  unsigned im_height = unsigned(0.5+scale*image.nj());
  vil_resample_bilin(tmp_im,new_image,im_width,im_height);
}

int main(int argc, char** argv)
{
  vul_arg<vcl_string> param_path("-p","Parameter filename");
  vul_arg_parse(argc,argv);

  msm_add_all_loaders();
  masm_add_all_loaders();

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

  msm_curves curves;
  if (!curves.read_text_file(params.curves_path))
    vcl_cerr<<"Failed to read in curves from "<<params.curves_path<<vcl_endl;

  masm_model_series model;

  if (!vsl_quick_file_load(model,params.asm_path))
  {
    vcl_cerr<<"Failed to load ASM sequence from "
        <<params.asm_path<<vcl_endl;
    return 2;
  }

  vcl_cout<<"Model: "<<model<<vcl_endl;

  masm_series_searcher asm_searcher(model);

  unsigned n_images = params.image_names.size();
  vimt_gaussian_pyramid_builder_2d<float> pyr_builder;

  vil_image_view<vxl_byte> output_image;
  double out_scale;

  if (params.out_image_width>0)
    vul_file::make_directory( params.output_dir );

  for (unsigned i=0;i<n_images;++i)
  {
    // Load in image
    vil_image_view<vxl_byte> byte_image;
    vcl_string image_path = params.image_dir+"/"+params.image_names[i];
    byte_image = vil_load(image_path.c_str());
    if (byte_image.size()==0)
    {
      vcl_cerr<<"Failed to load in image from "<<image_path<<vcl_endl;
      return 2;
    }

    // Convert to float image and build pyramid
    vimt_image_2d_of<float> image;
    if (byte_image.nplanes()==1)
      vil_convert_cast(byte_image,image.image());
    else
      vil_convert_planes_to_grey(byte_image,image.image());

    vimt_image_pyramid image_pyr;
    pyr_builder.build(image_pyr,image);

    asm_searcher(0).set_to_mean();
    asm_searcher(0).set_pose(model(0).shape_model().default_pose());

    if (params.out_image_width>0)
      create_output_image(byte_image,params.out_image_width,
                          output_image,out_scale);

    vcl_string im_name
            =vul_file::strip_extension(params.image_names[i]);

    asm_searcher.search(image_pyr);
   
    if (params.out_image_width>0)
    {
      vcl_stringstream out_path;
      out_path<<params.output_dir<<"/"<<im_name<<"_asm.eps";
      vcl_string out_dir=vul_file::dirname(out_path.str());
      vul_file::make_directory_path(out_dir);
      mbl_eps_writer writer;
      writer.open(out_path.str().c_str(),output_image,1.0,1.0);
      writer.set_scaling(out_scale);
//      writer.set_colour("red");
//      msm_draw_shape_to_eps(writer,new_points,curves);
      writer.set_colour("green");
      msm_draw_shape_to_eps(writer,asm_searcher.points(),curves);
//      writer.set_colour("yellow");
//      msm_draw_points_to_eps(writer,asm_searcher.found_points(),2);
      writer.close();
      vcl_cout<<"Overlay result saved to : "<<out_path.str()<<vcl_endl;
    }

    // Save the points to the output_points_dir
    vcl_stringstream out_pts_path;
    out_pts_path<<params.output_points_dir<<"/"<<im_name<<"_asm.pts";
    vcl_string out_dir=vul_file::dirname(out_pts_path.str());
    vul_file::make_directory_path(out_dir);
    if (asm_searcher.points().write_text_file(out_pts_path.str()))
      vcl_cout<<"Points saved to "<<out_pts_path.str()<<vcl_endl;
    else
      vcl_cout<<"Failed to save points to "
              <<out_pts_path.str()<<vcl_endl;
  }

  if (params.out_image_width>0)
    vcl_cout<<"Frames saved to "<<params.output_dir<<vcl_endl;
  return 0;
}
