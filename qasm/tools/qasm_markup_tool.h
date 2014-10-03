#ifndef qasm_markup_tool_h_
#define qasm_markup_tool_h_

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
// \brief Tool to search and annotate images by using an ASM.
// \author Tim Cootes

/*
// Example of image list file:
curves_path: /home/bim/images/faces/surrey/models/face_front.crvs
image_dir: /home/bim/images/faces/surrey/images/session_1/
points_dir: /home/bim/images/faces/surrey/points/session_1/

load_points: false

images:
{
000_1_1.pts : 000_1_1.jpg
001_1_1.pts : 001_1_1.jpg
001_1_2.pts : 001_1_2.jpg
002_1_1.pts : 002_1_1.jpg
003_1_1.pts : 003_1_1.jpg
003_1_2.pts : 003_1_2.jpg
}
*/

#include <QMainWindow>
#include <QMenu>
#include <QAction>
#include <QLabel>
#include <qasm/qasm_searcher_widget.h>
#include <msm/msm_shape_model.h>
#include <msm/msm_curve.h>

class qasm_markup_tool : public QMainWindow
{
  Q_OBJECT

protected:
  qasm_searcher_widget searcher_widget_;
  
  //: Loaded model points (for reseting shape)
  msm_points model_points;

  //: Path to curves file (optional)
  vcl_string curves_path_;

  //: Directory containing images
  vcl_string image_dir_;

  //: Directory containing points
  vcl_string points_dir_;

  //: List of image names
  vcl_vector<vcl_string> image_names_;

  //: List of points file names
  vcl_vector<vcl_string> points_names_;

  //: Description of connectivity
  msm_curves curves_;

  //: Index of current image
  unsigned index_;

  bool parse_image_list(vcl_string path);

  //: Load points specified in image list
  bool load_points_;

  void set_image();

  bool image_list_active;

public: 
  qasm_markup_tool();
  ~qasm_markup_tool();

  qasm_searcher_widget& searcher_widget() 
  { return searcher_widget_; }

  //: Return true if never to offer to save points when changing images
  bool never_save_points_automatically() const;

public slots:
  void load_image();
  void load_image(const QString& filename);

  //: Load in model
  void load_model();
  void load_model(const QString& filename);

  //: Load image list
  void load_image_list();
  void load_image_list(const QString& filename);

  void first_image();
  void backward_10images();
  void previous_image();
  void next_image();
  void forward_10images();  
  void last_image();

  void save_points_as();
  void save_points();
  void save_points(const QString& path);
  signed save_points_automatically();

  //: Display help information
  void show_help();

  //: Reset shape to loaded mean model
  void reset_shape();

  void set_image_list_active(bool value);

private:

  void create_actions();
  void create_menus();

  QMenu *file_menu_;
  QMenu *view_menu_;
  QMenu *help_menu_;

  QAction *load_model_act_;
  QAction *load_image_list_act_;
  QAction *load_image_act_;
  QAction *load_points_act_;
  QAction *load_curves_act_;
  QAction *save_points_as_act_;
  QAction *save_points_act_;

  QAction *first_act_;
  QAction *backward10_act_;
  QAction *prev_act_;
  QAction *next_act_;
  QAction *forward10_act_;
  QAction *last_act_;
  QAction *always_auto_save_pts_;

  QAction *reset_shape_act_;
  QAction *clear_selected_points_act_;
  QAction *fix_selected_points_act_;
  QAction *graph_dialog_act_;
  QAction *help_act_;

  QLabel *status_;

  masm_model_series asm_model_;
};

#endif
