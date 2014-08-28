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
// \brief Tool to combine files into a masm_model_series
// Assumes that the individual ASM files have be built in advance.
// This loads them all in and generates a single masm_model_series.
// \author Tim Cootes

#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_exception.h>
#include <mbl/mbl_parse_string_list.h>
#include <vul/vul_arg.h>
#include <vul/vul_string.h>
#include <vcl_sstream.h>
#include <vcl_fstream.h>
#include <vcl_string.h>
#include <vcl_vector.h>

#include <masm/masm_model_series.h>

#include <masm/masm_add_all_loaders.h>
#include <vsl/vsl_quick_file.h>

/*
Parameter file format:
<START FILE>
// Directory for the models
model_dir: /home/data/masm/models/

// List of built models
model_files: {
  model1.masm
  model2.masm
  model3.masm
}
<END FILE>
*/

void print_usage()
{
  vcl_cout << "masm_make_asm_series -p param_file -o asm_seq.bfs\n"
   << "Assumes that the individual ASM have been built. \n"
   << "This loads them all in and generates a single masm_model_series.\n"
   << vcl_endl;

  vul_arg_display_usage_and_exit();
}

//: Structure to hold parameters (mostly text)
struct tool_params
{
  //: Model directory
  vcl_string model_dir;

  //: List of AAM filenames
  vcl_vector<vcl_string> asm_name;

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

  model_dir=props.get_optional_property("model_dir","./");
  if (model_dir=="-") model_dir="";

  vcl_string model_files_str = props.get_required_property("model_files");
  mbl_parse_string_list(model_files_str,asm_name);

  try 
  {
    mbl_read_props_look_for_unused_props(
      "::read_from_file", props, mbl_read_props_type());
  }
  catch (mbl_exception_unused_props& e)
  {
    throw (mbl_exception_parse_error(e.what()));
  }
}

int main(int argc, char** argv)
{
  vul_arg<vcl_string> param_path("-p","Parameter filename");
  vul_arg<vcl_string> output_path("-o","Path for model","asm_seq.bfs");
  vul_arg_parse(argc,argv);

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

  masm_model_series asm_sequence;

  unsigned n_models = params.asm_name.size();
  asm_sequence.resize(n_models);
  
  vcl_cout<<"Loading in "<<n_models<<" ASMs."<<vcl_endl;

  for (unsigned i=0;i<n_models;++i)
  {
    vcl_string asm_path=params.model_dir+params.asm_name[i];
    if (!vsl_quick_file_load(asm_sequence(i),asm_path))
    {
      return 1;
    }
  }

  vcl_cout<<"ASM Sequence: "<<asm_sequence<<vcl_endl;

  if (vsl_quick_file_save(asm_sequence,output_path()))
  {
    vcl_cout<<"Saved ASM sequence to "<<output_path()<<vcl_endl;
  }
  else
  {
    vcl_cerr<<"Failed to save to "<<output_path()<<vcl_endl;
    return 3;
  }

  return 0;
}
