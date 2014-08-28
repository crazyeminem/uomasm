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
// This is mul/mfpf/tests/test_region_pdf.cxx
#include <testlib/testlib_test.h>
//:
// \file
// \author Tim Cootes
// \brief test masm_region_pdf

#include <vcl_iostream.h>
#include <vcl_sstream.h>
#include <vsl/vsl_binary_loader.h>
#include <masm/masm_bldr_add_all_loaders.h>
#include <masm/masm_region_pdf_finder.h>
#include <masm/masm_region_pdf_builder.h>
#include <vil/vil_bilin_interp.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_vector_2d.h>
#include <msm/msm_wt_mat_2d.h>
#include <vpdfl/vpdfl_axis_gaussian_builder.h>
#include <vpdfl/vpdfl_add_all_binary_loaders.h>

//=======================================================================

void test_region_pdf_search(masm_point_finder_builder& b)
{
  vcl_cout<<"Testing building and search."<<vcl_endl;

  masm_point_finder_model* pf_model = b.new_finder();

  // Create a test image
  vimt_image_2d_of<float> image(20,20);
  image.image().fill(0);
  
  // Create a square in the centre
  for (unsigned j=9;j<12;++j)
    for (unsigned i=9;i<12;++i)
      image.image()(i,j)=99;

  vgl_point_2d<double> p0(9.5,9.5), p1(7.5,8.5), p2(10.7,11.3), p3(7.8,8.2);
  vgl_vector_2d<double> u(1,0);

  b.clear(1);
  b.add_sample(image,mfpf_pose(p0,u));
  b.build(*pf_model);

  vcl_cout<<"Built model: "<<pf_model<<vcl_endl;
  
  masm_point_finder* pf = pf_model->create_finder();
  

  vgl_point_2d<double> new_p;
  vgl_vector_2d<double> new_u;

  TEST_NEAR("Evaluate at true point",pf->fit_at_point(image,mfpf_pose(p0,u)),-149.72,0.01);

  double fit;
  pf->set_search_area(0,0);
  pf->search(image,mfpf_pose(p0,u),new_p,fit);
  vcl_cout<<"Found point: "<<new_p<<vcl_endl;
  TEST_NEAR("Search with one point",fit,-149.72,0.01);
  TEST_NEAR("Correct location",(new_p-p0).length(),0.0,1e-6);

  pf->set_search_area(5,5);
  pf->search(image,mfpf_pose(p1,u),new_p,fit);
  vcl_cout<<"Found point: "<<new_p<<vcl_endl;

  TEST_NEAR("Correct location",(new_p-p0).length(),0.0,1e-6);

  pf->search(image,mfpf_pose(p2,u),new_p,fit);
  vcl_cout<<"Found point: "<<new_p<<vcl_endl;

  TEST_NEAR("Correct location",(new_p-p0).length(),0.0,0.5);

  vcl_cout<<"Without parabolic optimisation: "<<vcl_endl;
  pf->search(image,mfpf_pose(p3,u),new_p,fit);
  vcl_cout<<"Found point: "<<new_p<<vcl_endl;
  
  delete pf;
  delete pf_model;
}

void test_region_pdf_search2()
{
  masm_region_pdf_builder region_pdf_builder;
  masm_region_pdf_model region_pdf_model;
  vpdfl_axis_gaussian_builder axis_gauss_builder;
  region_pdf_builder.set_as_box(5,5,axis_gauss_builder);
  
  // Create a test image
  vimt_image_2d_of<float> image(25,25);
  image.image().fill(0);
  
  // Create a square in the centre
  for (unsigned j=8;j<=17;++j)
    for (unsigned i=8;i<=17;++i)
      image.image()(i,j)=99;

  vgl_point_2d<double> p0(7.5,12.5), p1(6.5,12.5), p2(12.5,6.5);
  vgl_vector_2d<double> u(1,0),v(0,1);

  region_pdf_builder.clear(1);
  region_pdf_builder.add_sample(image,mfpf_pose(p0,u));
  region_pdf_builder.build(region_pdf_model);

  vcl_cout<<"Built model: "<<region_pdf_model<<vcl_endl;
  
  masm_point_finder* pf = region_pdf_model.create_finder();
  
  vgl_point_2d<double> new_p;
  msm_wt_mat_2d wt_mat;
  
  pf->search2(image,mfpf_pose(p1,u),new_p,wt_mat);
  vcl_cout<<"Found pt: "<<new_p<<" wt_mat: "<<wt_mat<<vcl_endl;
  TEST_NEAR("x-edge found",new_p.x(),7.5,1e-3);
  TEST("wt_mat aligned with j axis",wt_mat.m11()>wt_mat.m22(),true);
  
  pf->search2(image,mfpf_pose(p2,v),new_p,wt_mat);
  vcl_cout<<"Found pt: "<<new_p<<" wt_mat: "<<wt_mat<<vcl_endl;
  TEST_NEAR("y-edge found",new_p.y(),7.5,1e-3);
  TEST("wt_mat aligned with i axis",wt_mat.m11()<wt_mat.m22(),true);
  
  delete pf;
}

void test_region_pdf_finder()
{
  vcl_cout << "**************************\n"
           << " Testing masm_region_pdf_finder\n"
           << "**************************\n";

  masm_bldr_add_all_loaders();
  vpdfl_add_all_binary_loaders();
  
  masm_region_pdf_builder region_pdf_builder;
  vpdfl_axis_gaussian_builder axis_gauss_builder;
  region_pdf_builder.set_as_box(5,5,axis_gauss_builder);
  test_region_pdf_search(region_pdf_builder);

  // -------------------------------------------
  //  Test configuring from stream
  // -------------------------------------------
  {
    vcl_istringstream ss(
          "masm_region_pdf_builder\n"
          "{\n"
          "  shape: box { ni: 9 nj: 9 } \n"
          "  norm: linear pdf_builder: vpdfl_axis_gaussian_builder { } \n"
          "  search_ni: 17 search_nj: 15\n"
          "}\n");

    vcl_auto_ptr<masm_point_finder_builder>
            pf = masm_point_finder_builder::create_from_stream(ss);

    TEST("Correct Point Finder Builder", pf->is_a(),"masm_region_pdf_builder");
    if (pf->is_a()=="masm_region_pdf_builder")
    {
      masm_region_pdf_builder &a_pf = static_cast<masm_region_pdf_builder&>(*pf);
      vcl_cout<<a_pf<<vcl_endl;
      
      TEST("search_ni configured",a_pf.search_ni(),17);
      TEST("search_nj configured",a_pf.search_nj(),15);
//      TEST("ni configured",a_pf.ni(),4);
//      TEST("nj configured",a_pf.nj(),5);

    }
  }

  {
    // Test builder returns correct type of object
    masm_region_pdf_builder b;
    masm_point_finder_model* pf_model = b.new_finder();
    TEST("Builder: Correct Finder Model",pf_model->is_a(),"masm_region_pdf_model");
    delete pf_model;
  }

  {
    masm_region_pdf_model region_pdf_model;
    
// Really should set it up...
    
    // Test binary load and save
    masm_point_finder_model * base_ptr = &region_pdf_model;

    vsl_b_ofstream bfs_out("test_region_pdf.bvl.tmp");
    TEST ("Created test_region_pdf.bvl.tmp for writing", (!bfs_out), false);
    vsl_b_write(bfs_out, region_pdf_model);
    vsl_b_write(bfs_out, base_ptr);
    bfs_out.close();

    masm_region_pdf_model region_pdf_in;
    masm_point_finder_model *base_ptr_in = 0;

    vsl_b_ifstream bfs_in("test_region_pdf.bvl.tmp");
    TEST ("Opened test_region_pdf.bvl.tmp for reading", (!bfs_in), false);
    vsl_b_read(bfs_in, region_pdf_in);
    vsl_b_read(bfs_in, base_ptr_in);
    TEST ("Finished reading file successfully", (!bfs_in), false);
    bfs_in.close();
    vcl_cout<<region_pdf_model<<vcl_endl
            <<region_pdf_in<<vcl_endl;
//    TEST("Loaded==Saved",region_pdf_in,region_pdf_model);
    TEST("Load region_pdf by base ptr (type)",
         base_ptr_in->is_a()==region_pdf_model.is_a(),true);

    delete base_ptr_in;
  }
  
  test_region_pdf_search2();

  vsl_delete_all_loaders();
}

TESTMAIN(test_region_pdf_finder);
