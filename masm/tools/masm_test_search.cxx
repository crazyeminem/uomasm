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
// \brief Tool to run an ASM series on multiple images and evaluate its performance
// Requires images and sets of points (to compare results against)
// \author Tim Cootes

#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_exception.h>
#include <mbl/mbl_parse_colon_pairs_list.h>
#include <mbl/mbl_parse_int_list.h>
#include <vul/vul_arg.h>
#include <vul/vul_string.h>
#include <vul/vul_file.h>
#include <vcl_sstream.h>
#include <vcl_fstream.h>
#include <vcl_string.h>
#include <vsl/vsl_quick_file.h>
#include <vcl_iterator.h>

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

#include <mbl/mbl_sample_stats_1d.h>


/*
Parameter file format:
<START FILE>

//: File containing ASM sequence
asm_path: face.masm

curves_path: face_front.crvs

// Index of ref point 0, 1 defining reference length.
ref_pt0_index: 0   ref_pt1_index: 1

// Directory to save the final points to, or "-" if to be ignored.
output_points_dir: -

//: Defines which points are initialised from initial_points.
//  If initial_pt_index[i]>=0, then initialise using points[initial_pt_index[i]]=initial_points[i]
//  If empty, then use all points
initial_pt_index: { 31 36 }


// Directory containing images
image_dir: /home/images/

// Directory containing `true' points
points_dir: /home/points/

// Directory containing points used to initialise the model
initial_points_dir: /home/init_points/

images: {
  image1.pts : image1.jpg
  image2.pts : image2.jpg
}

<END FILE>
*/

void print_usage()
{
  vcl_cout << "masm_test_search -p param_file\n"
           << "Tool to run a sequence of ASMs a set of images.\n"
           << "Initialises with mean shape at correct pose (or pose of supplied points).\n"
           << "Runs search and compares with supplied true points.\n"
           << vcl_endl;

  vul_arg_display_usage_and_exit();
}

//: Structure to hold parameters
struct tool_params
{

  //: Directory containing images
  vcl_string image_dir;

  //: Directory containing true points
  vcl_string points_dir;

  // Directory containing points used to initialise the model
  vcl_string initial_points_dir;

  //: File to save ASM to
  vcl_string asm_path;

  vcl_string curves_path;
  
  int ref_pt0_index, ref_pt1_index;
  
  //: Defines which points are initialised from initial_points.
  //  If initial_pt_index[i]>=0, then initialise using points[initial_pt_index[i]]=initial_points[i]
  //  If empty, then use all points
  vcl_vector<int> initial_pt_index;


  // Directory to save the final points to
  vcl_string output_points_dir;

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

  curves_path=props["curves_path"];

  image_dir=props.get_optional_property("image_dir","./");
  points_dir=props.get_optional_property("points_dir","./");
  initial_points_dir=props.get_optional_property("initial_points_dir","./");
  asm_path=props.get_optional_property("asm_path",
                                       "shape_aam.bfs");
  
  ref_pt0_index=vul_string_atoi(props.get_optional_property("ref_pt0_index","-1"));
  ref_pt1_index=vul_string_atoi(props.get_optional_property("ref_pt1_index","-1"));
  
  vcl_string init_pts_str
       =props.get_optional_property("initial_pt_index","");
  if (init_pts_str=="-") init_pts_str="";
  initial_pt_index.resize(0);
  if (init_pts_str!="")
  {
    vcl_stringstream ss(init_pts_str);
    mbl_parse_int_list(ss, vcl_back_inserter(initial_pt_index),int());
  }

  output_points_dir=props.get_optional_property("output_points_dir", "-");
  if (output_points_dir=="-") output_points_dir="";
  
  mbl_parse_colon_pairs_list(props.get_required_property("images"),
                             points_names,image_names);

  // Don't look for unused props so can use a single common parameter file.
}

double ref_length_from_points(const msm_points& points, int i0, int i1)
{
  if (i0<0 || i0>=points.size()) return 0;
  if (i1<0 || i1>=points.size()) return 0;
  return (points[i1]-points[i0]).length();
}

//: Calculate distance between two point sets
//  Compares each points1[i] with either points2[pt_subset[i]] or
//  points2[i] if no subset supplied.
void calc_point_distances(const msm_points& points1, // labelled
                          const msm_points& points2,    // predicted
                          const vcl_vector<int>& pt_subset,
                          vnl_vector<double>& d)
{
  d.set_size(points1.size());
  d.fill(0.0);

  if (pt_subset.size()==0)
  {
    if (points1.size()!=points2.size()) return;
    // Can't compute distances: Sets different sizes

    for (unsigned i=0;i<points1.size();++i)
      d[i]=(points1[i]-points2[i]).length();
  }
  else
  {
    for (unsigned i=0;i<points1.size();++i)
      d[i]=(points1[i]-points2[pt_subset[i]]).length();
  }
}

//: Display stats for a given quantity
inline void print_summary(vcl_ostream& os, const mbl_sample_stats_1d& stats)
{
  if (stats.n_samples()==0) { os<<"No samples."<<vcl_endl; return; }
  os<<"mean: "<<stats.mean()<<" se: "<<stats.stdError()<<" sd: "<<stats.sd()
    <<" med: "<<stats.median()<<" 90%: "<<stats.quantile(0.90)
    <<" 95%: "<<stats.quantile(0.95)<<" 99%: "<<stats.quantile(0.99)<<vcl_endl;
}

// Initialise the shape using the given points, which are a subset of all the points.
void initialise_model(msm_shape_instance& shape, 
                      const msm_points& initial_points, 
                      const vcl_vector<int>& initial_pt_index)
{
  if (initial_pt_index.size()==0)
    shape.fit_to_points(initial_points);
  else
  {
    // initial_points defines a subset of the model points
    unsigned n=shape.points().size();
    msm_points new_points(n);
    vnl_vector<double> wts(n,0.0);
    for (unsigned j=0;j<initial_pt_index.size();++j)
    {
      assert(initial_pt_index[j]<n);
      if (initial_pt_index[j]>=0)
      {
        new_points.set_point(initial_pt_index[j],initial_points[j].x(),initial_points[j].y());
        wts[initial_pt_index[j]]=1.0;
      }
    }
    // Use prior to avoid ill-conditioning when fitting to few points.
    bool use_prior = shape.ref_shape().use_prior();
    shape.ref_shape().set_use_prior(true);
    shape.fit_to_points_wt(new_points,wts);
    shape.ref_shape().set_use_prior(use_prior);
  }
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
  
  vcl_vector<int> pt_subset;  // Empty. For future expansion
  vnl_vector<double> pt_dist;  // To contain distances between results and true points
  
  // Statistics of mean distance before searching
  mbl_sample_stats_1d start_mean_d_stats,start_mean_rel_d_stats;

  // Statistics of mean distance after searching
  mbl_sample_stats_1d end_mean_d_stats,end_mean_rel_d_stats;


  for (unsigned i=0;i<n_images;++i)
  {
    if (i%10==9) { vcl_cout<<"."; vcl_cout.flush(); }
    
    // Load in true points
    msm_points true_points;
    vcl_string true_points_path = params.points_dir+"/"+params.points_names[i];
    if (!true_points.read_text_file(true_points_path))
      vcl_cout<<"Failed to load points from "<<true_points_path<<vcl_endl;
    
    // Load in initial points
    msm_points initial_points;
    if (params.initial_points_dir=="") initial_points = true_points;
    else
    {
      vcl_string initial_points_path = params.initial_points_dir+"/"+params.points_names[i];
      if (!initial_points.read_text_file(initial_points_path))
        vcl_cout<<"Failed to load points from "<<initial_points_path<<vcl_endl;
    }
    
    double ref_length = ref_length_from_points(true_points,params.ref_pt0_index,params.ref_pt1_index);
    
    if (i==0) 
    {
      vcl_cout<<"Loaded "<<true_points.size()<<" true points."<<vcl_endl;
      vcl_cout<<"  ref_length: "<<ref_length<<vcl_endl;
    }
   
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
    initialise_model(asm_searcher(0),initial_points,params.initial_pt_index);    
    asm_searcher(0).set_to_mean();
    
    // Compute initial stats
    calc_point_distances(true_points,asm_searcher(0).points(),pt_subset,pt_dist);
    start_mean_d_stats.add_sample(pt_dist.mean());
    if (ref_length>0) start_mean_rel_d_stats.add_sample(100*pt_dist.mean()/ref_length);

    vcl_string im_name
            =vul_file::strip_extension(params.image_names[i]);

    asm_searcher.search(image_pyr);
   
    if (params.output_points_dir!="")
    {
      // Save the points to the output_points_dir
      vcl_stringstream out_pts_path;
      out_pts_path<<params.output_points_dir<<"/"<<im_name<<"_asm.pts";
      vcl_string out_dir=vul_file::dirname(out_pts_path.str());
      vul_file::make_directory_path(out_dir);
      if (!asm_searcher.points().write_text_file(out_pts_path.str()))
        vcl_cout<<"Failed to save points to "<<out_pts_path.str()<<vcl_endl;
    }
    
    // Compute final stats
    calc_point_distances(true_points,asm_searcher.points(),pt_subset,pt_dist);
    end_mean_d_stats.add_sample(pt_dist.mean());
    if (ref_length>0) end_mean_rel_d_stats.add_sample(100*pt_dist.mean()/ref_length);

  }
  vcl_cout<<vcl_endl;

  vcl_cout<<"Initial mean distance (pixels)  : ";
  print_summary(vcl_cout,start_mean_d_stats);
  vcl_cout<<"Final mean distance  (pixels)   : ";
  print_summary(vcl_cout,end_mean_d_stats);

  if (start_mean_rel_d_stats.n_samples()>0)
  {
    vcl_cout<<"Initial mean rel. distance (%)    : ";
    print_summary(vcl_cout,start_mean_rel_d_stats);
  }
  if (end_mean_rel_d_stats.n_samples()>0)
  {
    vcl_cout<<"Final mean rel. distance  (%)     : ";
    print_summary(vcl_cout,end_mean_rel_d_stats);
  }
  return 0;
}
