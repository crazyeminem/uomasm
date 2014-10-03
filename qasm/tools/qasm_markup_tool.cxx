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
// \brief Tool to search and annotate images by using a ASM.
// \author Tim Cootes

#include <QApplication>
#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>

#include <QFile>
#include <QDir>
#include <QDateTime>
#include <QTextStream>

#include <vul/vul_file.h>
#include <vul/vul_arg.h>
#include <vul/vul_string.h>
#include <vsl/vsl_quick_file.h>

#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_exception.h>
#include <mbl/mbl_parse_colon_pairs_list.h>

#include "qasm_markup_tool.h"
#include <msm/msm_add_all_loaders.h>
#include <masm/masm_add_all_loaders.h>

qasm_markup_tool::qasm_markup_tool()
  : searcher_widget_(this)
{
  setCentralWidget(&searcher_widget_);
  create_actions();
  create_menus();

  status_=new QLabel;
  statusBar()->addPermanentWidget(status_);
  status_->setText(tr("Default 128x128"));

  set_image_list_active(false);
}


qasm_markup_tool::~qasm_markup_tool()
{
}

void qasm_markup_tool::create_actions()
{
  load_model_act_ = new QAction(tr("Load AS&M"), this);
  load_model_act_->setShortcut(tr("Ctrl+M"));
  load_model_act_->setStatusTip(tr("Load a ASM"));
  connect(load_model_act_, SIGNAL(triggered()), this, SLOT(load_model()));

  load_image_act_ = new QAction(tr("&Open image"), this);
  load_image_act_->setShortcut(tr("Ctrl+O"));
  load_image_act_->setStatusTip(tr("Open an image"));
  connect(load_image_act_, SIGNAL(triggered()), this, SLOT(load_image()));

  load_image_list_act_ = new QAction(tr("&Load image list"), this);
  load_image_list_act_->setShortcut(tr("Ctrl+L"));
  load_image_list_act_->setStatusTip(tr("Load in list of images"));
  connect(load_image_list_act_, SIGNAL(triggered()), 
          this, SLOT(load_image_list()));

  load_points_act_ = new QAction(tr("Load poin&ts"), this);
  load_points_act_->setShortcut(tr("Ctrl+T"));
  load_points_act_->setStatusTip(tr("Load points"));
  connect(load_points_act_, SIGNAL(triggered()),
          &searcher_widget_, SLOT(load_points()));

  load_curves_act_ = new QAction(tr("Load cu&rves"), this);
  load_curves_act_->setShortcut(tr("Ctrl+R"));
  load_curves_act_->setStatusTip(tr("Load curves"));
  connect(load_curves_act_, SIGNAL(triggered()),
          &searcher_widget_, SLOT(load_curves()));

  save_points_as_act_ = new QAction(tr("Save &points as..."), this);
  save_points_as_act_->setShortcut(tr("Ctrl+P"));
  save_points_as_act_->setStatusTip(tr("Open file dialog to save points"));
  connect(save_points_as_act_, SIGNAL(triggered()), 
          this, SLOT(save_points_as()));

  save_points_act_ = new QAction(tr("Save point&s"), this);
  save_points_act_->setShortcut(tr("Ctrl+S"));
  save_points_act_->setStatusTip(tr("Save points as specified in image list"));
  connect(save_points_act_, SIGNAL(triggered()), 
          this, SLOT(save_points()));

  first_act_ = new QAction(tr("&First"), this);
  first_act_->setShortcut(tr("F"));
  first_act_->setStatusTip(tr("First: Move to first image"));
  connect(first_act_, SIGNAL(triggered()), this, SLOT(first_image()));

  backward10_act_ = new QAction(tr("10 images back"), this);
  backward10_act_->setShortcut(QKeySequence(Qt::CTRL+Qt::Key_Left));
  backward10_act_->setStatusTip(tr("-10: Move 10 images back"));
  connect(backward10_act_, SIGNAL(triggered()), this, SLOT(backward_10images()));

  prev_act_ = new QAction(tr("Previous"), this);
  prev_act_->setShortcut(Qt::Key_Left);
  prev_act_->setStatusTip(tr("Previous: Move to previous image"));
  connect(prev_act_, SIGNAL(triggered()), this, SLOT(previous_image()));

  next_act_ = new QAction(tr("Next"), this);
  next_act_->setShortcut(Qt::Key_Right);
  next_act_->setStatusTip(tr("Next: Move to next image"));
  connect(next_act_, SIGNAL(triggered()), this, SLOT(next_image()));

  forward10_act_ = new QAction(tr("10 images forward"), this);
  forward10_act_->setShortcut(QKeySequence(Qt::CTRL+Qt::Key_Right));
  forward10_act_->setStatusTip(tr("+10: Move 10 images forward"));
  connect(forward10_act_, SIGNAL(triggered()), this, SLOT(forward_10images()));

  last_act_ = new QAction(tr("La&st"), this);
  last_act_->setShortcut(tr("L"));
  last_act_->setStatusTip(tr("Last: Move to last image"));
  connect(last_act_, SIGNAL(triggered()), this, SLOT(last_image()));

  always_auto_save_pts_ = new QAction(tr("Always ask to save pts"), this);
  always_auto_save_pts_->setStatusTip(tr("Always give user option of saving points when\nmoving to a new image.\nIf not true, then never prompt user (and don't save the points)."));
  always_auto_save_pts_->setCheckable(true);
  always_auto_save_pts_->setChecked(true);

  reset_shape_act_ = new QAction(tr("Reset s&hape"), this);
  reset_shape_act_->setShortcut(tr("Ctrl+H"));
  reset_shape_act_->setStatusTip(tr("Reset shape to mean shape and pose"));
  connect(reset_shape_act_, SIGNAL(triggered()), 
	  this, SLOT(reset_shape()));

  graph_dialog_act_ = new QAction(tr("&Graph props"), this);
  graph_dialog_act_->setShortcut(tr("Ctrl+G"));
  graph_dialog_act_->setStatusTip(tr("Edit colours etc"));
  connect(graph_dialog_act_, SIGNAL(triggered()), 
          &searcher_widget_, SLOT(show_graph_dialog()));

  clear_selected_points_act_ = new QAction(tr("Cle&ar selected points"), this);
  clear_selected_points_act_->setShortcut(tr("Ctrl+A"));
  clear_selected_points_act_->setStatusTip(tr("Clear selected points"));
  connect(clear_selected_points_act_, SIGNAL(triggered()), 
          &searcher_widget_, SLOT(clear_selected_points()));

  fix_selected_points_act_ = new QAction(tr("Fi&x selected points"), this);
  fix_selected_points_act_->setShortcut(tr("Ctrl+X"));
  fix_selected_points_act_->setStatusTip(tr("Fix selected points"));
  connect(fix_selected_points_act_, SIGNAL(triggered()), 
           &searcher_widget_, SLOT(fix_selected_points()));

  help_act_ = new QAction(tr("&Help"), this);
  help_act_->setShortcut(tr("Ctrl+H"));
  help_act_->setStatusTip(tr("Help: Show help information"));
  connect(help_act_, SIGNAL(triggered()), this, SLOT(show_help()));
}


void qasm_markup_tool::create_menus()
{
  file_menu_ = menuBar()->addMenu(tr("&File"));
  file_menu_->addAction(load_model_act_);
  file_menu_->addAction(load_image_list_act_);
  file_menu_->addAction(load_image_act_);
  file_menu_->addAction(load_points_act_);
  file_menu_->addAction(load_curves_act_);
  file_menu_->addAction(save_points_as_act_);
  file_menu_->addAction(save_points_act_);

  view_menu_ = menuBar()->addMenu(tr("&View"));
  view_menu_->addAction(first_act_);
  view_menu_->addAction(backward10_act_);
  view_menu_->addAction(prev_act_);
  view_menu_->addAction(next_act_);
  view_menu_->addAction(forward10_act_);
  view_menu_->addAction(last_act_);
  view_menu_->addAction(reset_shape_act_);
  view_menu_->addAction(searcher_widget_.centre_shape_act());
  view_menu_->addAction(graph_dialog_act_);
  view_menu_->addAction(clear_selected_points_act_);
  view_menu_->addAction(fix_selected_points_act_);
  view_menu_->addAction(always_auto_save_pts_);

  help_menu_ = menuBar()->addMenu(tr("&Help"));
  help_menu_->addAction(help_act_);
}

bool qasm_markup_tool::parse_image_list(vcl_string path)
{
  vcl_ifstream ifs(path.c_str());
  if (!ifs)
  {
    vcl_string error_msg = "Failed to open file: "+path;
    QMessageBox::warning(this,
        tr("qasm_markup_tool"),
        tr(error_msg.c_str()));

    return false;
  }

  try
  {
    mbl_read_props_type props = mbl_read_props_ws(ifs);

    curves_path_=props.get_optional_property("curves_path","");
    image_dir_=props.get_optional_property("image_dir","./");
    points_dir_=props.get_optional_property("points_dir","./");

    load_points_ = vul_string_to_bool(props.get_optional_property("load_points","false"));

    mbl_parse_colon_pairs_list(props.get_required_property("images"),
                               points_names_,image_names_);
  }
  catch (mbl_exception_parse_error& e)
  {
    vcl_string error_msg = "Failed to parse file: "+path;
    QMessageBox::warning(this,
        tr("qasm_markup_tool"),
        tr(error_msg.c_str())
        +tr(e.what()));

    return false;
  }

  return true;
}

void qasm_markup_tool::load_image()
{
  QString filename = QFileDialog::getOpenFileName(this,
                    "Open Image",
                     QString(searcher_widget_.image_dir().c_str()));
  if (!filename.isEmpty())
    load_image(filename);
}

void qasm_markup_tool::load_image(const QString& path)
{
  if (!searcher_widget_.load_image(path.toStdString())) return;

  vcl_string name = vul_file::strip_directory((path.toStdString()));
  vcl_stringstream ss;
  ss<<name<<" "<<searcher_widget_.image().image().ni()
               <<"x"<<searcher_widget_.image().image().nj();
  status_->setText(QString(ss.str().c_str()));

  set_image_list_active(false);
}

void qasm_markup_tool::load_model()
{
  QString filename = QFileDialog::getOpenFileName(this);
  if (!filename.isEmpty())
    load_model(filename);
}

void qasm_markup_tool::load_model(const QString& path)
{
  if (!vsl_quick_file_load(asm_model_,path.toStdString()))
  {
    QMessageBox::information(this,
                 tr("qasm_markup_tool"),
                 tr("Failed to load in ASM sequence model.\n"));
    return;
  }

  searcher_widget_.set_model(asm_model_);
  model_points = searcher_widget_.points();
}

void qasm_markup_tool::load_image_list()
{
  QString filename = QFileDialog::getOpenFileName(this);
  if (!filename.isEmpty())
    load_image_list(filename);
}

void qasm_markup_tool::load_image_list(const QString& path)
{
  if (!parse_image_list(path.toStdString())) return;

  if (curves_path_!="")
  {
    if (!curves_.read_text_file(curves_path_))
    {
      QMessageBox::warning(this,
          tr("qasm_markup_tool"),
          tr("Failed to load curves from ")+curves_path_.c_str());
    }
  }

  set_image_list_active(true);
  first_image();
}

void qasm_markup_tool::first_image()
{
  if (!image_list_active) return;
  if (!save_points_automatically()) return;

  index_=0;
  set_image();
}

void qasm_markup_tool::backward_10images()
{
  if (!image_list_active) return;
  if (!save_points_automatically()) return;

  if ((index_-10)<0) index_=0;
  else index_-=10;

  set_image();
}

void qasm_markup_tool::previous_image()
{
  if (!image_list_active) return;
  if (!save_points_automatically()) return;

  if (index_==0) return;
  index_--;
  set_image();
}

void qasm_markup_tool::next_image()
{
  if (!image_list_active) return;
  if (!save_points_automatically()) return;

  if (index_+1>=image_names_.size()) return;
  index_++;
  set_image();
}

void qasm_markup_tool::forward_10images()
{
  if (!image_list_active) return;
  if (!save_points_automatically()) return;

  if ((index_+10)>=image_names_.size())
    index_=image_names_.size()-1;
  else index_+=10;

  set_image();
}

void qasm_markup_tool::last_image()
{
  if (!image_list_active) return;
  if (!save_points_automatically()) return;

  index_=image_names_.size()-1;
  set_image();
}

void qasm_markup_tool::set_image()
{
  if (index_>=image_names_.size()) return;
  vcl_string im_path = image_dir_+"/"+image_names_[index_];

  load_image(QString(im_path.c_str()));
  set_image_list_active(true);
 
  if (load_points_) {
    vcl_string pts_path = points_dir_+"/"+points_names_[index_];
    searcher_widget().load_points(QString(pts_path.c_str()));
    searcher_widget().zoom_to_points();
  }
}

//: Reset shape to loaded mean model
void qasm_markup_tool::reset_shape()
{
  searcher_widget_.set_points(model_points);
  searcher_widget_.clear_fixed_pts();
}

//: Display help information
void qasm_markup_tool::show_help()
{
  QMessageBox::information(this,
   tr("qasm_markup_tool"),
    tr("Tool to annotate an image with points.\n")
   +tr("Includes various ways of moving the points around.\n")
   +tr("Fixed points are shown in red.\n\n")
   +tr("Modes available:\n")
   +tr("Drag:      Drag points or viewport\n")
   +tr("Select:    Select groups of points to move around.\n")
   +tr("DragShape: Use shape model to warp shapes based on\n")
   +tr("           current position of fixed points.\n")
   +tr("Shortcuts:\n")
   +tr("  F    : First image\n")
   +tr("  Left : Previous image\n")
   +tr("  Right: Next image\n")
   +tr("  L: Last image\n"));
}

void qasm_markup_tool::save_points_as()
{
  QString image_dir;

  if (!image_list_active)  
    image_dir = QString((points_dir_).c_str());
  else
    image_dir = QString((points_dir_+"/"+image_names_[index_]+".pts").c_str());


  QString filename = QFileDialog::getSaveFileName(this, "Save Points", image_dir,"*.pts");

  if (!filename.isEmpty())
    save_points(filename);
}

void qasm_markup_tool::save_points(const QString& path)
{
  if (!searcher_widget().points().write_text_file(path.toStdString()))
    QMessageBox::warning(this,tr("Save Points"),
                 tr("Failed to save points to\n")+path); 
  else
      vcl_cout<<"Save points to "<<path.toStdString()<<vcl_endl;
}

void qasm_markup_tool::save_points()
{
  if (index_>=image_names_.size()) return;

  QString filename = QString((points_dir_+"/"+points_names_[index_]).c_str());
  QString abspath  = (QFileInfo(filename).absoluteDir()).absolutePath();

  if (vul_file_exists(filename.toStdString())){
    int r = QMessageBox::question(this,tr("Save Points"),
                 tr("Points file (")+filename+tr(") already exists. Replace? \n"),
                 QMessageBox::Yes|QMessageBox::Escape, QMessageBox::No|QMessageBox::Default);

    if(r == QMessageBox::No) 
	return;
  }

  if (!filename.isEmpty()){
    if (!QDir((abspath)).exists()){ 
      QDir().mkdir((abspath));
      vcl_cout<<"Create directory "<<abspath.toStdString()<<vcl_endl;
    }
    save_points(filename);
  }
}

//: Return true if never to offer to save points when changing images
bool qasm_markup_tool::never_save_points_automatically() const
{
  return !always_auto_save_pts_->isChecked();
}

signed qasm_markup_tool::save_points_automatically()
{
  if (never_save_points_automatically()) return 1;

  if (index_>=image_names_.size()) return -1;

  QString filename = QString((points_dir_+"/"+points_names_[index_]).c_str());

  int r = QMessageBox::question(this,tr("Save Points"),
               tr("Save points to\n")+filename+tr("? \n"),
               QMessageBox::Cancel|QMessageBox::Yes|QMessageBox::No, QMessageBox::No);
  if(r == QMessageBox::Cancel) return 0; 
  if(r == QMessageBox::Yes) save_points(); 

  return 1;
}

void qasm_markup_tool::set_image_list_active(bool value)
{
  image_list_active = value;
  save_points_act_->setEnabled(value);    
 
  QApplication::processEvents();
}

int main( int argc, char **argv )
{
  QApplication app( argc, argv );

  msm_add_all_loaders();
  masm_add_all_loaders();

  vul_arg<vcl_string> image_path("-i","Image path");
  vul_arg<vcl_string> points_path("-p","Points path");
  vul_arg<vcl_string> curves_path("-c","Curves path");
  vul_arg<vcl_string> model_path("-m","ASM path");
  vul_arg<vcl_string> image_list_path("-l","Image list path");

  vul_arg_parse(argc,argv);

  qasm_markup_tool tool;
  tool.show();

  if (image_path()!="")
    tool.load_image(QString(image_path().c_str()));
  if (points_path()!="")
    tool.searcher_widget().load_points(QString(points_path().c_str()));
  if (model_path()!="")
    tool.load_model(QString(model_path().c_str()));
  if (curves_path()!="")
    tool.searcher_widget().load_curves(QString(curves_path().c_str()));
  if (image_list_path()!="")
    tool.load_image_list(QString(image_list_path().c_str()));

  return app.exec();
}
