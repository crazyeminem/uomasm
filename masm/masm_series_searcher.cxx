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
// \author Tim Cootes
// \brief Holds a set of ASM objects, to be applied sequentially.

#include <vcl_cstdlib.h>

#include "masm_series_searcher.h"

#include <vil/vil_math.h>

#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>
#include <vsl/vsl_vector_io.h>
#include <vcl_algorithm.h>

#include <vcl_ctime.h>
#include <vil/vil_save.h>
#include <vil/vil_convert.h>

//=======================================================================
// Dflt ctor
//=======================================================================
masm_series_searcher::masm_series_searcher()
  : asm_seq_(0)
{
}

//: Set AAM objects to use (pointer retained)
//  asm_seq must remain in scope.
masm_series_searcher::masm_series_searcher(const masm_model_series& asm_seq)
{
  set_asms(asm_seq);
}

//: Set maximum number of iterations per model
void masm_series_searcher::set_max_its(unsigned n_its) 
{ 
  for (unsigned i=0;i<size();++i) 
    searcher_[i].set_max_its(n_its); 
}

//=======================================================================
// Destructor
//=======================================================================
masm_series_searcher::~masm_series_searcher()
{
}

//: Partially set up the model (predictors are empty).
//  Takes copies of each object.
void masm_series_searcher::set_asms(const masm_model_series& asm_seq)
{
  asm_seq_=&asm_seq;

  searcher_.resize(asm_seq.size());
  for (unsigned i=0;i<asm_seq.size();++i)
    searcher_[i].set_asm(asm_seq(i));
}

//: Search from current position (sm_inst(0).points())
//  Initialise sm_inst(0) before calling this.
//  It applies a sequence of AAM update steps.
//  Final result is in sm_inst(n-1).points(), where n=asm().size().
void masm_series_searcher::search(const vimt_image_pyramid& im_pyr)
{
  searcher_[0].search(im_pyr);
  for (unsigned i=1;i<searcher_.size();++i)
  {
    searcher_[i].fit_to_points(searcher_[i-1].points());
    searcher_[i].search(im_pyr);
  }
}

//: Perform search, allowing fixed points
//  If fixed[i]==true, then don't search for point i, assume it to be at fixed_points[i]
//  Note that on completion points()[i] will be the best shape model approximation to
//  fixed_points[i], but won't necessarily be exactly that value.
//  If use_wt_mat_in_search_ then takes covariance of local matches into account.
void masm_series_searcher::search_with_fixed(const vimt_image_pyramid& image_pyr,
                         const msm_points& fixed_points,
                         const vcl_vector<bool>& fixed)
{
  searcher_[0].search_with_fixed(image_pyr,fixed_points,fixed);
  for (unsigned i=1;i<searcher_.size();++i)
  {
    searcher_[i].fit_to_points(searcher_[i-1].points());
    searcher_[i].search_with_fixed(image_pyr,fixed_points,fixed);
  }
}



//: Print class to os
void masm_series_searcher::print_summary(vcl_ostream& os) const
{
  os<<"n_models: "<<searcher_.size();
}

vcl_ostream& operator<<(vcl_ostream& os,
                        const masm_series_searcher& b)
{
  vsl_indent_inc(os);
  b.print_summary(os);
  vsl_indent_dec(os);
  return os;
}


