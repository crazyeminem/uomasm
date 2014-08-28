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
#include <mbl/mbl_cloneables_factory.h>

#include <msm/msm_add_all_loaders.h>
#include <vpdfl/vpdfl_add_all_binary_loaders.h>

#include <masm/masm_ncc_finder_model.h>
#include <masm/masm_region_pdf_model.h>
#include <masm/masm_lin_reg_model.h>
#include <masm/masm_grad_corr_model.h>

//: Add all binary loaders and factory objects for masm library
void masm_add_all_loaders()
{
  msm_add_all_loaders();
  vpdfl_add_all_binary_loaders();

  vsl_add_to_binary_loader(masm_ncc_finder_model());
  vsl_add_to_binary_loader(masm_region_pdf_model());
  vsl_add_to_binary_loader(masm_lin_reg_model());
  vsl_add_to_binary_loader(masm_grad_corr_model());
}
