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
#include <testlib/testlib_register.h>

DECLARE( test_ncc_finder );
DECLARE( test_region_pdf_finder );
DECLARE( test_lin_reg_finder );
DECLARE( test_grad_corr_finder );

void register_tests()
{
  REGISTER( test_ncc_finder );
  REGISTER( test_region_pdf_finder );
  REGISTER( test_lin_reg_finder );
  REGISTER( test_grad_corr_finder );
}

DEFINE_MAIN;
