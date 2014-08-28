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
// \brief Class to set up and build a masm_model given marked images.
// \author Tim Cootes


#include "masm_builder.h"
#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_exception.h>
#include <mbl/mbl_parse_colon_pairs_list.h>
#include <vul/vul_arg.h>
#include <vul/vul_string.h>
#include <vcl_sstream.h>
#include <vcl_fstream.h>
#include <vcl_algorithm.h>
#include <vsl/vsl_indent.h>
#include <mbl/mbl_stats_nd.h>

#include <msm/msm_shape_model_builder.h>
#include <msdi/msdi_marked_images.h>

//=======================================================================
// Dflt ctor
//=======================================================================

masm_builder::masm_builder()
  : verbose_(false),wt_for_fixed_(100)
{
}


//=======================================================================
// Destructor
//=======================================================================

masm_builder::~masm_builder()
{
}


//====================================================================


//: Gather samples for searchers using supplied data and shape_model
//  Builds all parts of model except the searchers.
void masm_builder::gather_samples(
                       msdi_marked_images& marked_images,
                       const msm_shape_model& shape_model,
                       masm_model& new_model)
{
  unsigned n_points = shape_model.size();
  msm_shape_instance shape_inst(shape_model);
  vcl_vector<mfpf_pose> frame_pose(n_points);
  
  // Set up object which defines directions at each point.
  masm_local_frame_maker frame_maker;
  frame_maker.set_from_curves(n_points,curves_);

  // Compute width of mean shape in shape model ref frame:
  vgl_box_2d<double> bounds = shape_model.mean_points().bounds();
  
  // Define step size so that frame_width steps across the mean shape in ref frame.
  frame_maker.set_ref_scale_step(bounds.width()/frame_width_);

  unsigned n_images = marked_images.size();
  finder_builders_.clear(shape_model.size(),n_images);

  marked_images.reset();
  for (unsigned im=0;im<n_images;++im,marked_images.next())
  {
    if (verbose_ && im%10==0) vcl_cout<<"."<<vcl_flush;
    if (verbose_ && n_images>=100 && im>0 && (im%(n_images/10)==0))
      vcl_cout<<(im+1)*100/n_images<<"%"<<vcl_flush;

    // Set up frame pose from the best fit transform
    // Currently assumes a similarity transformation.
    shape_inst.fit_to_points(marked_images.points());
    frame_maker.create_frames(shape_inst,frame_pose);
    
    finder_builders_.add_sample(marked_images.image_pyr(),frame_pose);
  }
  if (verbose_) vcl_cout<<"\nBuilding searchers..."<<vcl_endl;

  new_model.set(shape_model,frame_maker);
}

//: Builds searchers and sets in given model
void masm_builder::build_finders(masm_model& new_model)
{
  vcl_vector<masm_point_finder_model*> finders;

  finder_builders_.build(finders);
  new_model.take_finders(finders);
}

//: Build new model using supplied data and shape_model
void masm_builder::build_asm(
                       msdi_marked_images& marked_images,
                       const msm_shape_model& shape_model,
                       masm_model& new_model)
{
  gather_samples(marked_images,shape_model,new_model);
  build_finders(new_model);
  new_model.set_wt_for_fixed(wt_for_fixed_);
}

//: Build new_model from supplied data
void masm_builder::build(msdi_marked_images& marked_images,
                             masm_model& new_model)
{
  msm_shape_model shape_model;
  build_shape_model(marked_images,shape_model);
  
  vcl_vector<const masm_point_finder_model*> old_finders;
  finder_builders_.set_old_finders(old_finders);  // Force build from scratch

  build_asm(marked_images,shape_model,new_model);
}

/* Perhaps a version of this should be in msm somewhere? */
//: Function to create a shape model centred on particular examples.
//  Given a shape model defining some modes about a mean, this generates
//  a new model, using the same modes about a new mean.
//  An example use is to take the shape of one persons face and a general
// expression model, to generate a shape model estimating how that person
//  may change expression.
//
//  The supplied model is used to "neutralise" the supplied points
//  (ie to remove variation which can be explained by the model).
//  These are used to estimate a new mean for the model.
//  Thus new_model uses the same modes as model0, and only differs in the
//  mean and the default pose.
void masm_specialise_shape_model(const msm_shape_model& model0,
                                   const vcl_vector<msm_points>& points,
                                   msm_shape_model& new_model)
{
  msm_shape_instance sm_inst(model0);

  // Rather than attempting to generate the modes analytically from combining
  // covariance matricies, we will simply generate representative examples
  // centred on each points[i], and rebuild a shape model.  The approach avoids
  // problems which may arise if one tried to rotate tangent planes to allow
  // for different model means, and also allows the new model to have a
  // different form to the old (eg similarity vs affine model, for example).
  unsigned n_egs = points.size();

  vnl_vector<double> mean_offset(points[0].vector().size(),0.0);
  vnl_vector<double> default_pose;

  msm_points points_in_ref,points1,offset;
  vnl_vector<double> b=sm_inst.params();
  for (unsigned i=0;i<n_egs;++i)
  {
    sm_inst.fit_to_points(points[i]);
    if (i==0) default_pose=sm_inst.pose();

    // Compute the residual offset for points[i] which can't be explained by the model
    // This is the information which will be encoded in the new model.
    // If only a single example is supplied, then the offsets define the displacement
    // of the mean.
    vnl_vector<double> inv_pose = model0.aligner().inverse(sm_inst.pose());
    model0.aligner().apply_transform(points[i],inv_pose,points_in_ref);
    mean_offset += points_in_ref.vector()-sm_inst.model_points().vector();
  }
  mean_offset/=n_egs;

  msm_points new_mean = model0.mean_points();
  new_mean.vector() += mean_offset;

  new_model.set(new_mean,model0.modes(),model0.mode_var(),
                default_pose,model0.aligner(),model0.param_limiter());
}


//: Build new_model from supplied data and an existing model.
//  Gathers samples from supplied marked data, and optionally combines
//  with information from old_model to generate the new model.
void masm_builder::rebuild(msdi_marked_images& marked_images,
                     const masm_model& old_model,
                     masm_model& new_model)
{
  msm_shape_model shape_model;
  if (rebuild_replaces_mean_shape_)
  {
    replace_mean_shape(marked_images,old_model,shape_model);
  }
  else
    shape_model = old_model.shape_model();
  
  unsigned n_pts=shape_model.size();
  vcl_vector<const masm_point_finder_model*> old_finders(n_pts);
  for (unsigned i=0;i<n_pts;++i) old_finders[i]=&old_model.finder(i);
  
  finder_builders_.set_old_finders(old_finders);

  build_asm(marked_images,shape_model,new_model);
}

//====================================================================

//: Get all shapes, applying points_maker to each example
void masm_builder::get_shapes(msdi_marked_images& marked_images,
                  vcl_vector<msm_points>& shapes)
{
  unsigned n=marked_images.size();
  shapes.resize(n);
  msm_points no_guide_points;
  marked_images.reset();
  for (unsigned i=0;i<n;++i,marked_images.next())
  {
    shapes[i]=marked_images.points();
  }
}

//: Build the shape model component
void masm_builder::build_shape_model(
                         msdi_marked_images& marked_images,
                         msm_shape_model& shape_model)
{
  // ==== Build the shape model ====
  msm_shape_model_builder shape_builder;
  shape_builder.set_aligner(*aligner_);
  shape_builder.set_param_limiter(*param_limiter_);
  shape_builder.set_mode_choice(0,max_modes_,var_prop_);

  // === Load in all the shapes ===
  vcl_vector<msm_points> shapes;
  get_shapes(marked_images,shapes);

  shape_builder.build_model(shapes,shape_model);
}

//: Generate shape model by replacing mean in old shape model
void masm_builder::replace_mean_shape(
                         msdi_marked_images& marked_images,
                         const masm_model& old_model,
                         msm_shape_model& shape_model)
{
  vcl_vector<msm_points> shapes;
  get_shapes(marked_images,shapes);

  masm_specialise_shape_model(old_model.shape_model(),
                              shapes, shape_model);
}


void masm_builder::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,short(1));  // Version number
  vsl_b_write(bfs,aligner_);
  vsl_b_write(bfs,max_modes_);
  vsl_b_write(bfs,var_prop_);
  vsl_b_write(bfs,param_limiter_);
  vsl_b_write(bfs,frame_width_);
  vsl_b_write(bfs,finder_builders_);
  vsl_b_write(bfs,rebuild_replaces_mean_shape_);
  vsl_b_write(bfs,curves_);
  vsl_b_write(bfs,wt_for_fixed_);
  vsl_b_write(bfs,verbose_);
}

void masm_builder::b_read(vsl_b_istream& bfs)
{
  if (!bfs) return;
  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      vsl_b_read(bfs,aligner_);
      vsl_b_read(bfs,max_modes_);
      vsl_b_read(bfs,var_prop_);
      vsl_b_read(bfs,param_limiter_);
      vsl_b_read(bfs,frame_width_);
      vsl_b_read(bfs,finder_builders_);
      vsl_b_read(bfs,rebuild_replaces_mean_shape_);
      vsl_b_read(bfs,curves_);
      vsl_b_read(bfs,wt_for_fixed_); 
      vsl_b_read(bfs,verbose_);
      break;
    default:
      vcl_cerr << "I/O ERROR: vsl_b_read(vsl_b_istream&) \n";
      vcl_cerr << "           Unknown version number "<< version << vcl_endl;
      bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
      return;
  }
}

inline bool get_bool(mbl_read_props_type& props,
                     const vcl_string& s1,
                     const vcl_string& s2)
{
  return vul_string_to_bool(props.get_optional_property(s1,s2));
}

//: Initialise from a text stream.
//  Use model_dir as base directory when loading from files.
void masm_builder::config_from_stream(vcl_istream &is,
                                  const vcl_string& model_dir)
{
  mbl_read_props_type props = mbl_read_props_ws(is);

  max_modes_=vul_string_atoi(props.get_optional_property("max_modes","99"));
  var_prop_=vul_string_atof(props.get_optional_property("var_prop","0.95"));
  wt_for_fixed_=vul_string_atof(props.get_optional_property("wt_for_fixed","100"));

  frame_width_=vul_string_atoi(props.get_optional_property("frame_width","50"));

  verbose_=vul_string_to_bool(props.get_optional_property("verbose","false"));

  rebuild_replaces_mean_shape_ = 
        get_bool(props,"rebuild_replaces_mean_shape","false");

  {
    vcl_string aligner_str 
       = props.get_required_property("aligner");
    vcl_stringstream ss(aligner_str);
    aligner_ = *msm_aligner::create_from_stream(ss);
  }
  {
    vcl_string param_limiter_str 
       = props.get_optional_property("param_limiter",
                                     "msm_no_limiter { }");
    vcl_stringstream ss(param_limiter_str);
    param_limiter_ = *msm_param_limiter::create_from_stream(ss);
  }
  
  {
    vcl_string builders_str = props.get_required_property("finder_builders");
    vcl_stringstream ss(builders_str);
    finder_builders_.config_from_stream(ss);
  }

  vcl_string curves_path = props.get_optional_property("curves_path","-");
  if (curves_path!="-") curves_path = model_dir+curves_path;
  else curves_=msm_curves();

  if (curves_path!="-" && !curves_.read_text_file(curves_path))
  {
    vcl_string error_str="Failed to load curves from "+curves_path;
    throw mbl_exception_parse_error(error_str);
  }

  mbl_read_props_look_for_unused_props(
        "masm_builder::config_from_stream",
         props,   mbl_read_props_type());

}

//=======================================================================
// Method: print
//=======================================================================

void masm_builder::print_summary(vcl_ostream& os) const
{
  os<<" {\n";
  vsl_indent_inc(os);
  if (aligner_.isDefined())
    os<<vsl_indent()<<"aligner: "<<aligner_<<vcl_endl;
  if (param_limiter_.isDefined())
    os<<vsl_indent()<<"param_limiter: "<<param_limiter_<<vcl_endl;

  os<<vsl_indent()<<"max_modes: "<<max_modes_<<vcl_endl;
  os<<vsl_indent()<<"var_prop: "<<var_prop_<<vcl_endl;
  os<<vsl_indent()<<"frame_width: "<<frame_width_<<vcl_endl;
  os<<vsl_indent()<<"finder_builders: "<<finder_builders_<<vcl_endl;
  os<<vsl_indent()<<"wt_for_fixed: "<<wt_for_fixed_<<vcl_endl;

  vsl_indent_dec(os);
  os<<vsl_indent()<<"}"<<vcl_endl;

}

