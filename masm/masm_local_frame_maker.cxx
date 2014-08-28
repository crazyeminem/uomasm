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
// \brief Local frame defined as normal to curve at point

#include "masm_local_frame_maker.h"
#include <msm/msm_shape_model.h>
#include <vsl/vsl_binary_loader.h>
#include <mbl/mbl_read_props.h>
#include <mbl/mbl_parse_block.h>
#include <mbl/mbl_exception.h>
#include <vsl/vsl_vector_io.h>

//=======================================================================

//: Define step size between samples in reference frame
void masm_local_frame_maker::set_ref_scale_step(double s)
{
  ref_scale_step_ = s;
}

//: Set up definitions of directions from the curves.
void masm_local_frame_maker::set_from_curves(unsigned n_points, 
                                           const msm_curves& curves)
{
  end0_.resize(n_points,0);
  end1_.resize(n_points,0);

  for (unsigned j=0;j<curves.size();++j)
  {
    const msm_curve& curve=curves[j];
    unsigned nc=curve.size();
    // Tangent at point defined by previous and next points
    for (unsigned i=0;i<nc;++i)
    {
      if (defined(curve[i])) continue;
      end0_[curve[i]]=curve[(i+nc-1)%nc];
      end1_[curve[i]]=curve[(i+nc+1)%nc];
    }
    // For open curves, first point tangent is p0-p1,
    // last is p[nc-2],p[nc-1]
    if (curve.open())
    {
      if (defined(curve[0]))
        end0_[curve[0]]=curve[0];  // First point is p[0]-p[1]
      if (defined(curve[0]))
        end1_[curve[nc-1]]=curve[nc-1];
    }
  }
}

//: Frame in ref. is aligned with x axis.
//  For each point in instance.points(), generate one local frame.
//  Generally origin will be the point itself.
void masm_local_frame_maker::create_default_frames(
                             msm_shape_instance& instance,
                             vcl_vector<mfpf_pose>& frame_pose) const
{
  const msm_points& points = instance.points();
  msm_points offsets=instance.model_points();
  offsets.translate_by(ref_scale_step_,0.0);
  instance.model().aligner().apply_transform(offsets,instance.pose(),
                                             offsets);
  frame_pose.resize(offsets.size());
  for (unsigned i=0;i<offsets.size();++i)
    frame_pose[i]=mfpf_pose(points[i],offsets[i]-points[i]);
}



//: Create a local frame for each point, defined using an mfpf_pose
//  For each point in instance.points(), generate one local frame.
//  Generally origin will be the point itself.
void masm_local_frame_maker::create_frames(
                             msm_shape_instance& instance,
                             vcl_vector<mfpf_pose>& frame_pose) const
{
  if (end0_.size()==0)
  {
    create_default_frames(instance,frame_pose);
    return;
  }

  const msm_points& points = instance.points();
  const msm_points& ref_points = instance.model_points();

  unsigned n_points=points.size();
  msm_points offsets(n_points);
  for (unsigned i=0;i<n_points;++i)
  {
    vgl_point_2d<double> p=ref_points[i];
    if (end0_[i]==end1_[i])
    {
      offsets.set_point(i,p.x()+ref_scale_step_,p.y());
    }
    else
    {
      vgl_vector_2d<double> t = ref_points[end0_[i]]
                                  - ref_points[end1_[i]];
      double L=t.length();
      if (L<=1e-8)
        offsets.set_point(i,p.x()+ref_scale_step_,p.y());
      else
      {
        t*=ref_scale_step_/L;
        // Normal is (-t.y(),t.x())
        offsets.set_point(i,p.x()-t.y(),p.y()+t.x());
      }
    }
  }
  instance.model().aligner().apply_transform(offsets,instance.pose(),
                                             offsets);
  frame_pose.resize(offsets.size());
  for (unsigned i=0;i<offsets.size();++i)
    frame_pose[i]=mfpf_pose(points[i],offsets[i]-points[i]);
}

//=======================================================================

void masm_local_frame_maker::print_summary(vcl_ostream& os) const
{
  os<<"rel_scale_step: "<<ref_scale_step_<<" n_points: "<<end0_.size();
}

const static short version_no = 1;

//: Save class to binary file stream
void masm_local_frame_maker::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,version_no);
  vsl_b_write(bfs,ref_scale_step_);
  vsl_b_write(bfs,end0_);
  vsl_b_write(bfs,end1_);
}


//: Load class from binary file stream
void masm_local_frame_maker::b_read(vsl_b_istream& bfs)
{
  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      vsl_b_read(bfs,ref_scale_step_);
      vsl_b_read(bfs,end0_);
      vsl_b_read(bfs,end1_);
      break;
    default:
      vcl_cerr << "masm_local_frame_maker::b_read() :\n"
               << "Unexpected version number " << version << vcl_endl;
      bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
      return;
  }
}


//: Binary file stream output operator for class reference
void vsl_b_write(vsl_b_ostream& bfs, const masm_local_frame_maker& b)
{ b.b_write(bfs); }

//: Binary file stream input operator for class reference
void vsl_b_read(vsl_b_istream& bfs, masm_local_frame_maker& b)
{ b.b_read(bfs); }

//: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const masm_local_frame_maker& b)
{
  b.print_summary(os);
  return os;
}


