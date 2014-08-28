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

#ifndef masm_series_searcher_h_
#define masm_series_searcher_h_

//:
// \file
// \author Tim Cootes
// \brief Searches an image using a series of masm_searchers

#include <masm/masm_model_series.h>
#include <masm/masm_searcher.h>

//: Searches an image using a sequence of masm_asms.
//  This assumes that all the shape models represent the same set of points, so that one can initialise
//  one from the previous by just fitting it to the points.
class masm_series_searcher 
{
protected:
  //: Set of individual ASMs (external)
  const masm_model_series *asm_seq_;

  //: Set of searchers, one per model
  vcl_vector<masm_searcher> searcher_;

public:
  //: Dflt ctor
  masm_series_searcher();

  //: Set AAM objects to use (pointer retained)
  //  asm_seq must remain in scope.
  masm_series_searcher(const masm_model_series& asm_seq);

  //: Destructor
  virtual ~masm_series_searcher();

  //: Return true if a model has been set.
  bool got_model() const { return asm_seq_!=0; }
  
  //: Set AAM objects to use (pointer retained)
  //  asm_seq must remain in scope.
  void set_asms(const masm_model_series& asm_seq);


  //: Number of shape models in searcher
  unsigned size() { return searcher_.size(); }

  //: Set maximum number of iterations per model
  void set_max_its(unsigned n_its);

  //: Fit first searcher in sequence to given points
  //  Utility function to initialise the search
  void fit_to_points(const msm_points& points)
  { searcher_[0].fit_to_points(points); }

  //: Access to AAMs
  const masm_model_series& asm_seq() const
  { assert(asm_seq_!=0); return *asm_seq_; }

  //: Access to last AAM in set
  const masm_model& last_asm() const
  { assert(asm_seq_!=0); return asm_seq_->last_asm(); }

  //: Access to searcher i
  const masm_searcher& operator()(unsigned i) const
  { assert(i<searcher_.size()); return searcher_[i]; }

  //: Access to searcher i - use with care.
  masm_searcher& operator()(unsigned i)
  { assert(i<searcher_.size()); return searcher_[i]; }

  //: Access to last shape instance
  masm_searcher& last_searcher()
  { assert(searcher_.size()>0); return searcher_[searcher_.size()-1]; }
  
  const msm_points& points() { return last_searcher().points(); }

  //: Search from current position (searcher(0).sm_inst().points())
  //  Initialise first one before calling this.
  //  It applies a sequence of ASM update steps.
  //  Final result is in last_searcher().points()
  void search(const vimt_image_pyramid& im_pyr);

  //: Perform search, allowing fixed points
  //  If fixed[i]==true, then don't search for point i, assume it to be at fixed_points[i]
  //  Note that on completion points()[i] will be the best shape model approximation to
  //  fixed_points[i], but won't necessarily be exactly that value.
  //  If use_wt_mat_in_search_ then takes covariance of local matches into account.
  void search_with_fixed(const vimt_image_pyramid& image_pyr,
                         const msm_points& fixed_points,
                         const vcl_vector<bool>& fixed);

  //: Print class to os
  virtual void print_summary(vcl_ostream& os) const;
};

//: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,
                        const masm_series_searcher& b);

#endif // masm_series_searcher_h_


