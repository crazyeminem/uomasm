/* 
* Copyright 2013-2014 The University of Manchester
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
// \brief Search with a ASM searcher and edit results.
// \author Tim Cootes

#include "qasm_searcher_widget.h"
#include <QMessageBox>
#include <vil/vil_convert.h>
#include <vil/vil_save.h>
#include <qvcr/qvcr_image_viewer.h>

qasm_searcher_widget::qasm_searcher_widget(QWidget *parent)
  : qmsm_shape_editor_widget(parent)
{
  search_but_ = new QPushButton("Search",this);
  search_but_->setToolTip("Search with current model\nFixed points unchanged.");
  connect(search_but_,SIGNAL(pressed()), SLOT(search()));

  reset_but_ = new QPushButton("Reset",this);
  reset_but_->setToolTip("Reset to mean shape");
  connect(reset_but_,SIGNAL(pressed()), SLOT(reset_shape()));

  layout_box1_->addWidget(search_but_);
  layout_box1_->addWidget(reset_but_);
  layout_box1_->addStretch();  // Eats any spare space

  search_but_->setEnabled(false);
  reset_but_->setEnabled(false);  
}

//: Destructor
qasm_searcher_widget::~qasm_searcher_widget()
{
}

//: Define model to use (ref retained)
void qasm_searcher_widget::set_model(const masm_model_series& model)
{
  searcher_.set_asms(model);
  set_shape_model(model.last_asm().shape_model());
  
  points_ = searcher_.points();

  if (points_.size()==0)
  {
    set_points(points_);
  }
  else
  {
    if (points_.size()!=points().size())
    {
      QMessageBox::information(this,
            tr("New Model Loaded\n"),
            tr("Number of points is changing."));
      set_points(points_);
    }
  }
  search_but_->setEnabled(true);
  reset_but_->setEnabled(true);
}

//: Define image to use (Takes a deep copy)
void qasm_searcher_widget::set_image(const vimt_image_2d_of<vxl_byte>& image)
{
  qmsm_shape_editor_widget::set_image(image);

  vimt_gaussian_pyramid_builder_2d<vxl_byte> pyr_builder;

  if (image.image().nplanes()==1)
  {
    pyr_builder.build(image_pyr_,image);
  }
  else
  {
    vimt_image_2d_of<vxl_byte> grey_image;
    vil_convert_planes_to_grey(image.image(),grey_image.image());
    grey_image.set_world2im(image.world2im());
    pyr_builder.build(image_pyr_,grey_image);
  }
}

//: Perform search from current position
void qasm_searcher_widget::search()
{
  if (!searcher_.got_model()) return;

  points_ = points();
  searcher_(0).fit_to_points(points_);
  
  if (state().n_fixed()>0)
  {
    searcher_.search_with_fixed(image_pyr_,points_,state().fixed_pts);
    
    // Copy the points which are not fixed:
    for (unsigned i=0;i<points_.size();++i)
      if (!state().fixed_pts[i])
      {
        vgl_point_2d<double> p = searcher_.points()[i];
        points_.set_point(i,p.x(),p.y());
      }
  }
  else
  {
    searcher_.search(image_pyr_);
    points_ = searcher_.points();
  }
  
  set_points(points_);
}

//: Return shape to model mean
void qasm_searcher_widget::reset_shape()
{
  if (!searcher_.got_model()) return;
  state().clear_fixed_pts();
  points_=points();
  searcher_.fit_to_points(points_);
  searcher_(0).set_to_mean();
  points_ = searcher_(0).points();
  points_.get_points(state().pts);
  set_graphics_to_state();
  record_state();
}
