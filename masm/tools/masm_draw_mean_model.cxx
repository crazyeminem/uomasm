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
// \brief Tool to write eps file showing mean model + local model boxes
// \author Tim Cootes

#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_exception.h>
#include <mbl/mbl_parse_colon_pairs_list.h>
#include <vul/vul_arg.h>
#include <vul/vul_string.h>
#include <vcl_sstream.h>
#include <vcl_fstream.h>
#include <vcl_string.h>
#include <vcl_algorithm.h>
#include <vsl/vsl_quick_file.h>

#include <msm/msm_shape_model.h>
#include <msm/msm_shape_instance.h>
#include <msm/msm_curve.h>

#include <mbl/mbl_cloneables_factory.h>
#include <msm/msm_add_all_loaders.h>
#include <msm/utils/msm_draw_shape_to_eps.h>
#include <msm/utils/msm_shape_mode_view.h>
#include <masm/masm_model.h>

#include <masm/masm_add_all_loaders.h>


/*
Parameter file format:
<START FILE>
//: File to load model from
asm_path: model.masm

curves_path: face_front.crvs

//: Radius of points to display (if <0, then don't draw points)
point_radius: 2

// Set this to "none" if no backgroud required (eg if this is going to be imported into a LaTeX document)
background_colour: white

line_colour: black
point_colour: red
local_model_colour: black

// Approximate width of region to display shape
width: 100

eps_path: asm_mean.eps
<END FILE>
*/

void print_usage()
{
  vcl_cout << "masm_draw_mean_model -p param_file\n"
           << "Tool to write eps file showing mean model + local model boxes.\n"
           << vcl_endl;

  vul_arg_display_usage_and_exit();
}

//: Structure to hold parameters
struct tool_params
{
  //: Path to shape model
  vcl_string asm_path;
  vcl_string curves_path;

  //: Approximate width of region to display shape
  double width;

  //: Path for output
  vcl_string eps_path;

  //: Set this to "none" if no backgroud required (eg if this is going to be imported into a LaTeX document)
  vcl_string background_colour;
  
  //: Line colour
  vcl_string line_colour;

  //: Point colour
  vcl_string point_colour;

  //: Local model colour
  vcl_string local_model_colour;

  //: Radius of points to display (if <0, then don't draw points)
  double point_radius;


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
  point_radius=vul_string_atof(props.get_optional_property("point_radius","1.5"));
  width=vul_string_atof(props.get_optional_property("width","100"));
  background_colour=props.get_optional_property("background_colour","none");
  line_colour=props.get_optional_property("line_colour","black");
  point_colour=props.get_optional_property("point_colour","red");
  local_model_colour=props.get_optional_property("local_model_colour","blue");
  eps_path=props.get_optional_property("eps_path","./");
  asm_path=props.get_optional_property("asm_path",
                                       "model.masm");


  // Don't look for unused props so can use a single common parameter file.
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

  masm_model model;

  if (!vsl_quick_file_load(model,params.asm_path))
  {
    vcl_cerr<<"Failed to load ASM from "
        <<params.asm_path<<vcl_endl;
    return 2;
  }

  vcl_cout<<"Model: "<<model<<vcl_endl;

  msm_curves curves;
  if (!curves.read_text_file(params.curves_path))
    vcl_cerr<<"Failed to read in curves from "<<params.curves_path<<vcl_endl;

  msm_shape_instance sm_inst(model.shape_model());
  msm_points points = sm_inst.points();
  vgl_box_2d<double> bounds = points.bounds();

  unsigned n_points = points.size();

  // Generate points defining each RoI
  vcl_vector<msm_points> local_points(n_points);
  vcl_vector<mfpf_pose> frame_pose(n_points);
  model.frame_maker().create_frames(sm_inst,frame_pose);
  vcl_vector<vgl_point_2d<double> > pts;
  for (unsigned i=0;i<n_points;++i)
  {
    model.finder(i).get_outline(pts);
    for (unsigned j=0;j<pts.size();++j) 
      pts[j]=frame_pose[i](pts[j]);
    local_points[i].set_points(pts);
    // Update total bounding box
    if (pts.size()>0) bounds.add(local_points[i].bounds());
  }

  // Scaling required to end up with desired width
  double s = 0.95*params.width/(1e-3+bounds.width());
  int win_height=int(s*bounds.height()/0.95+0.5);

  vgl_point_2d<double> c=bounds.centroid();
  double dx=0.5*params.width-s*c.x();
  double dy=0.5*win_height-s*c.y();

  // Transform all the points into viewing window
  points.transform_by(s,0,dx,dy);
  for (unsigned i=0;i<n_points;++i)
    local_points[i].transform_by(s,0,dx,dy);

  mbl_eps_writer writer(params.eps_path.c_str(),
                        params.width,win_height);
  
  if (params.background_colour!="none")
  {
    writer.set_colour(params.background_colour);
    writer.ofs()<<"clippath fill\n";
  }

  // Draw regions for each local model
  writer.set_colour(params.local_model_colour);
  for (unsigned i=0;i<n_points;++i)
  {
    local_points[i].get_points(pts);
    bool filled=true;
    writer.draw_polygon(pts,true,filled);
  }
  
  // Draw lines
  writer.set_colour(params.line_colour);
  msm_draw_shape_to_eps(writer,points,curves);

  // Draw model points
  writer.set_colour(params.point_colour);
  if (params.point_radius>0)
    msm_draw_points_to_eps(writer,points,
                           params.point_radius);

  writer.close();
  vcl_cout<<"Saved EPS to "<<params.eps_path<<vcl_endl;

  return 0;
}
