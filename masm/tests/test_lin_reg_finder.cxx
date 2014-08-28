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
// This is mul/mfpf/tests/test_lin_reg_finder.cxx
#include <testlib/testlib_test.h>
//:
// \file
// \author Tim Cootes
// \brief test masm_lin_reg

#include <vcl_iostream.h>
#include <vcl_sstream.h>
#include <vsl/vsl_binary_loader.h>
#include <masm/masm_bldr_add_all_loaders.h>
#include <masm/masm_lin_reg_finder.h>
#include <masm/masm_lin_reg_builder.h>
#include <vil/vil_bilin_interp.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_vector_2d.h>
#include <msm/msm_wt_mat_2d.h>
#include <vpdfl/vpdfl_axis_gaussian_builder.h>
#include <vpdfl/vpdfl_add_all_binary_loaders.h>

//=======================================================================

void test_lin_reg_search(masm_point_finder_builder& b)
{
  vcl_cout<<"Testing building and search."<<vcl_endl;

  masm_point_finder_model* pf_model = b.new_finder();

  // Create a test image
  vimt_image_2d_of<float> image(20,20);
  image.image().fill(0);
  
  // Create a square in the centre
  for (unsigned j=9;j<14;++j)
    for (unsigned i=9;i<14;++i)
      image.image()(i,j)=99;

  vgl_point_2d<double> p0(9.5,9.5), p1(8.0,8.5), p2(10.1,11.1);
  vgl_vector_2d<double> u(1,0);

  b.clear(1);
  b.add_sample(image,mfpf_pose(p0,u));
  b.build(*pf_model);

  vcl_cout<<"Built model: "<<pf_model<<vcl_endl;
  
  masm_point_finder* pf = pf_model->create_finder();
  

  vgl_point_2d<double> new_p;
  vgl_vector_2d<double> new_u;

  TEST_NEAR("Evaluate at true point",pf->fit_at_point(image,mfpf_pose(p0,u)),0,0.05);
  TEST_NEAR("Evaluate at true point + (0.5,0)",
            pf->fit_at_point(image,mfpf_pose(p0.x()+0.5,p0.y(),1,0)),0.5,0.1);
  TEST_NEAR("Evaluate at true point + (1,0)",pf->fit_at_point(image,mfpf_pose(p0.x()+1,p0.y(),1,0)),1,0.1);
  TEST_NEAR("Evaluate at true point + (2,0)",pf->fit_at_point(image,mfpf_pose(p0.x()+2,p0.y(),1,0)),2,0.3);
  TEST_NEAR("Evaluate at true point + (0,1)",pf->fit_at_point(image,mfpf_pose(p0.x(),p0.y()+1,1,0)),1,0.1);

  vcl_cout<<"True point: "<<p0<<vcl_endl;
  double fit;
  pf->set_search_area(0,0);
  pf->search(image,mfpf_pose(p0,u),new_p,fit);
  vcl_cout<<"Found point: "<<new_p<<vcl_endl;
  TEST_NEAR("Correct location (1)",(new_p-p0).length(),0.0,0.1);

  pf->set_search_area(2,2);
  pf->search(image,mfpf_pose(p1,u),new_p,fit);
  vcl_cout<<"From p1 found point: "<<new_p<<vcl_endl;

  TEST_NEAR("Correct location (2)",(new_p-p0).length(),0.0,0.5);

  pf->search(image,mfpf_pose(p2,u),new_p,fit);
  vcl_cout<<"From p2 found point: "<<new_p<<vcl_endl;

  TEST_NEAR("Correct location (3)",(new_p-p0).length(),0.0,0.6);
  
  delete pf;
  delete pf_model;
}

void test_lin_reg_search2()
{
  masm_lin_reg_builder lin_reg_builder;
  masm_lin_reg_model lin_reg_model;
  lin_reg_builder.set_as_box(5,5);
  lin_reg_builder.set_max_displacement(2.5,2.5);
  lin_reg_builder.set_n_random(199);
  
  // Create a test image
  vimt_image_2d_of<float> image(25,25);
  image.image().fill(0);
  
  // Create a square in the centre
  for (unsigned j=8;j<=17;++j)
    for (unsigned i=8;i<=17;++i)
      image.image()(i,j)=99;

  vgl_point_2d<double> p0(7.5,12.5), p1(6.5,12.5), p2(12.5,6.5);
  vgl_vector_2d<double> u(1,0),v(0,1);

  lin_reg_builder.clear(1);
  lin_reg_builder.add_sample(image,mfpf_pose(p0,u));
  lin_reg_builder.build(lin_reg_model);

  vcl_cout<<"Built model: "<<lin_reg_model<<vcl_endl;
  
  masm_point_finder* pf = lin_reg_model.create_finder();
  
  vgl_point_2d<double> new_p;
  msm_wt_mat_2d wt_mat;
  
  pf->search2(image,mfpf_pose(p1,u),new_p,wt_mat);
  vcl_cout<<"Found pt: "<<new_p<<" wt_mat: "<<wt_mat<<vcl_endl;
  TEST_NEAR("x-edge found",new_p.x(),7.5,0.1);
  TEST("wt_mat aligned with j axis",wt_mat.m11()>wt_mat.m22(),true);
  
  pf->search2(image,mfpf_pose(p2,v),new_p,wt_mat);
  vcl_cout<<"Found pt: "<<new_p<<" wt_mat: "<<wt_mat<<vcl_endl;
  TEST_NEAR("y-edge found",new_p.y(),7.5,0.1);
  TEST("wt_mat aligned with i axis",wt_mat.m11()<wt_mat.m22(),true);
  
  delete pf;
}

void test_lin_reg_finder()
{
  vcl_cout << "**************************\n"
           << " Testing masm_lin_reg_finder\n"
           << "**************************\n";

  masm_bldr_add_all_loaders();
  vpdfl_add_all_binary_loaders();
  
  masm_lin_reg_builder lin_reg_builder;
  lin_reg_builder.set_as_box(9,9);
  lin_reg_builder.set_max_displacement(2.5,2.5);
  lin_reg_builder.set_n_random(199);
  test_lin_reg_search(lin_reg_builder);

  // -------------------------------------------
  //  Test configuring from stream
  // -------------------------------------------
  {
    vcl_istringstream ss(
          "masm_lin_reg_builder\n"
          "{\n"
          "  shape: box { ni: 9 nj: 9 } \n"
          "  norm: linear n_random: 50 max_di: 2.7 max_dj: 1.9 \n"
          "  search_ni: 17 search_nj: 15\n"
          "}\n");

    vcl_auto_ptr<masm_point_finder_builder>
            pf = masm_point_finder_builder::create_from_stream(ss);

    TEST("Correct Point Finder Builder", pf->is_a(),"masm_lin_reg_builder");
    if (pf->is_a()=="masm_lin_reg_builder")
    {
      masm_lin_reg_builder &a_pf = static_cast<masm_lin_reg_builder&>(*pf);
      vcl_cout<<a_pf<<vcl_endl;
      
      TEST("search_ni configured",a_pf.search_ni(),17);
      TEST("search_nj configured",a_pf.search_nj(),15);
    }
  }

  {
    // Test builder returns correct type of object
    masm_lin_reg_builder b;
    masm_point_finder_model* pf_model = b.new_finder();
    TEST("Builder: Correct Finder Model",pf_model->is_a(),"masm_lin_reg_model");
    delete pf_model;
  }

  {
    masm_lin_reg_model lin_reg_model;
    
// Really should set it up...
    
    // Test binary load and save
    masm_point_finder_model * base_ptr = &lin_reg_model;

    vsl_b_ofstream bfs_out("test_lin_reg.bvl.tmp");
    TEST ("Created test_lin_reg.bvl.tmp for writing", (!bfs_out), false);
    vsl_b_write(bfs_out, lin_reg_model);
    vsl_b_write(bfs_out, base_ptr);
    bfs_out.close();

    masm_lin_reg_model lin_reg_in;
    masm_point_finder_model *base_ptr_in = 0;

    vsl_b_ifstream bfs_in("test_lin_reg.bvl.tmp");
    TEST ("Opened test_lin_reg.bvl.tmp for reading", (!bfs_in), false);
    vsl_b_read(bfs_in, lin_reg_in);
    vsl_b_read(bfs_in, base_ptr_in);
    TEST ("Finished reading file successfully", (!bfs_in), false);
    bfs_in.close();
    vcl_cout<<lin_reg_model<<vcl_endl
            <<lin_reg_in<<vcl_endl;
//    TEST("Loaded==Saved",lin_reg_in,lin_reg_model);
    TEST("Load lin_reg by base ptr (type)",
         base_ptr_in->is_a()==lin_reg_model.is_a(),true);

    delete base_ptr_in;
  }
  
  test_lin_reg_search2();

  vsl_delete_all_loaders();
}

TESTMAIN(test_lin_reg_finder);
