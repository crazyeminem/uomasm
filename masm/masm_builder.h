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

#ifndef masm_builder_h_
#define masm_builder_h_

//:
// \file
// \brief Class to set up and build a masm_model given marked images.
// Used by the tool masm_build_asm
// \author Tim Cootes

/*
Example parameter file information:
<START FILE>
//: Aligner for shape model
aligner: msm_similarity_aligner

// Maximum number of shape modes
max_modes: 5

// Proportion of shape variation to explain
var_prop: 0.95

// Object to apply limits to shape parameters
param_limiter: msm_ellipsoid_limiter { accept_prop: 0.98 }

// Sampling step size is set so that there are frame_width such steps across the mean shape.
frame_width: 50

//: Weight to use for fixed points in ASM fitting
wt_for_fixed: 100

curves_path: face_front.crvs

// Define the type of builder for the local models
finder_builders: {
  default_builder: masm_ncc_finder_builder { ni: 7 nj: 3 search_ni: 5 search_nj: 3 }
  override_params: {
    params: { index: 0 data: { ni: 7 nj: 13 } }
    params: { index: { 2 5 : 7 } data: { ni: 9 nj: 11 } }
  }
}

//: When true, during rebuild() change the mean shape
//  Retain old sampler and shape modes.  If false, shape
//  and sampler are identical to old_model.
rebuild_replaces_mean_shape: true

<END FILE>
*/

#include <masm/masm_model.h>
#include <masm/masm_finder_set_builder.h>
#include <mbl/mbl_cloneable_ptr.h>
#include <msm/msm_shape_model.h>
#include <msm/msm_curve.h>

class msdi_marked_images;

//: Class to set up and build a masm_model given marked images.
//  The build() and rebuild() functions are passed sets of images
//  and associated points, which are used to construct the model.
class masm_builder
{
private:
  // For shape model:

  //: Aligner for shape model
  mbl_cloneable_ptr<msm_aligner> aligner_;

  //: Maximum number of shape modes
  unsigned max_modes_;

  //: Proportion of shape variation to explain
  double var_prop_;

  //: Object to apply limits to shape parameters
  mbl_cloneable_ptr<msm_param_limiter> param_limiter_;

  // Sampling step size is set so that there are frame_width such steps across the mean shape.
  unsigned frame_width_;

  //: Connectivity for the points
  msm_curves curves_;
  
  //: Weight to use for fixed points
  double wt_for_fixed_;

  bool verbose_;
  
  //: Object to build local models
  masm_finder_set_builder finder_builders_;

  //: When true, during rebuild() change the mean shape
  //  Retain old sampler and shape modes.  If false, shape
  //  and sampler are identical to old_model.
  bool rebuild_replaces_mean_shape_;

  //: Get all shapes, applying points_maker to each example
  void get_shapes(msdi_marked_images& marked_images,
                  vcl_vector<msm_points>& shapes);

  //: Generate shape model by replacing mean in old shape model
  void replace_mean_shape(msdi_marked_images& marked_images,
                          const masm_model& old_model,
                          msm_shape_model& shape_model);

public:
  //: Dflt ctor
  masm_builder();

  //: Destructor
  virtual ~masm_builder();

  masm_finder_set_builder& finder_builders() 
  { return finder_builders_; }


  //: Build the shape model component
  void build_shape_model(msdi_marked_images& marked_images,
                         msm_shape_model& shape_model);

  //: Gather samples for finders using supplied data and shape_model
  //  Builds all parts of model except the finders.
  void gather_samples(msdi_marked_images& marked_images,
                       const msm_shape_model& shape_model,
                       masm_model& new_model);

  //: Builds finders and sets in given model
  void build_finders(masm_model& new_model);

  //: Build new model using supplied data and shape_model
  //  Builds the searcher components and sets the ranges.
  void build_asm(msdi_marked_images& marked_images,
                       const msm_shape_model& shape_model,
                       masm_model& new_model);

  //: Build new model using supplied data
  void build(msdi_marked_images& marked_images,
                 masm_model& new_model);

  //: Build new_model from supplied data and an existing model.
  //  Gathers samples from supplied marked data, and optionally combines
  //  with information from old_model to generate the new model.
  virtual void rebuild(msdi_marked_images& marked_images,
                       const masm_model& old_model,
                       masm_model& new_model);

  //: Print class to os
  virtual void print_summary(vcl_ostream& os) const;

  //: Save class to binary file stream
  virtual void b_write(vsl_b_ostream& bfs) const;

  //: Load class from binary file stream
  virtual void b_read(vsl_b_istream& bfs);

  //: Initialise from a text stream.
  //  Use model_dir as base directory when loading from files.
  virtual void config_from_stream(vcl_istream &is, 
                                  const vcl_string& model_dir);
};


#endif


