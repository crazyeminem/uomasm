# isbe_apm/masm/CMakeLists.txt
#
#   Tim Cootes
#

doxygen_add_library(uomasm/masm
  DEPENDS contrib/mul/msm contrib/mul/mfpf
  PACKAGE uomasm
  DESCRIPTION "Manchester's ASM Library"
)

# Data for the masm library
SET(masm_sources
  masm_add_all_loaders.h       masm_add_all_loaders.cxx
  masm_local_frame_maker.h     masm_local_frame_maker.cxx

  masm_model.h                 masm_model.cxx
  masm_searcher.h              masm_searcher.cxx

  masm_model_series.h          masm_model_series.cxx
  masm_series_searcher.h       masm_series_searcher.cxx

  masm_point_finder.h          masm_point_finder.cxx
  masm_point_finder_model.h    masm_point_finder_model.cxx

  masm_ncc_finder_model.h      masm_ncc_finder_model.cxx
  masm_ncc_finder.h            masm_ncc_finder.cxx

  masm_region_pdf_model.h      masm_region_pdf_model.cxx
  masm_region_pdf_finder.h     masm_region_pdf_finder.cxx

  masm_lin_reg_model.h         masm_lin_reg_model.cxx
  masm_lin_reg_finder.h        masm_lin_reg_finder.cxx

  masm_grad_corr_model.h       masm_grad_corr_model.cxx
  masm_grad_corr_finder.h      masm_grad_corr_finder.cxx
)

ADD_LIBRARY(masm ${masm_sources} )
TARGET_LINK_LIBRARIES(masm msm mfpf)

# Data for the masm_bldr library, containing the classes to build the models.
SET(masm_bldr_sources
  masm_finder_set_builder.h     masm_finder_set_builder.cxx
  masm_builder.h                masm_builder.cxx

  masm_bldr_add_all_loaders.h   masm_bldr_add_all_loaders.cxx
  masm_point_finder_builder.h   masm_point_finder_builder.cxx
  masm_ncc_finder_builder.h     masm_ncc_finder_builder.cxx
  masm_region_pdf_builder.h     masm_region_pdf_builder.cxx
  masm_lin_reg_builder.h        masm_lin_reg_builder.cxx
  masm_grad_corr_builder.h      masm_grad_corr_builder.cxx
)

ADD_LIBRARY(masm_bldr ${masm_bldr_sources} )
TARGET_LINK_LIBRARIES(masm_bldr masm msdi)


SUBDIRS(tools)

IF( BUILD_TESTING)
  SUBDIRS(tests)
ENDIF( BUILD_TESTING)
