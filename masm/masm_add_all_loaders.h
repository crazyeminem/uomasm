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

#ifndef masm_add_all_loaders_h_
#define masm_add_all_loaders_h_

//: Add all binary loaders and factory objects available in masm
//  Necessary for binary loading of derived classes by
//  base class pointer.  One instance of each possible
//  derived class must be added to the binary loader.
void masm_add_all_loaders();

#endif // masm_add_all_loaders_h_
