#ifndef qasm_searcher_widget_h_
#define qasm_searcher_widget_h_

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
// \brief Search with a hclm searcher and edit results.
// \author Tim Cootes

#include <qmsm/qmsm_shape_editor_widget.h>
#include <masm/masm_series_searcher.h>
#include <masm/masm_model_series.h>
#include <vimt/vimt_image_pyramid.h>
#include <vimt/vimt_gaussian_pyramid_builder_2d.h>

class qvcr_image_viewer;

class qasm_searcher_widget : public qmsm_shape_editor_widget
{
  Q_OBJECT

protected:
  QPushButton *search_but_;
  QPushButton *reset_but_;

  masm_series_searcher searcher_;
  
  // Current point positions (including fixed points)
  msm_points points_;

  //: Pyramid built from current image
  vimt_image_pyramid image_pyr_;

  //: Define image to use (Takes a deep copy)
  virtual void set_image(const vimt_image_2d_of<vxl_byte>&);
  
public:
  qasm_searcher_widget(QWidget *parent=0);

  //: Destructor
  virtual ~qasm_searcher_widget();

  //: Define model to use (ref retained)
  void set_model(const masm_model_series& model);

public slots:

  //: Perform search from current position
  void search();

  //: Return shape to model mean
  void reset_shape();
};

#endif
