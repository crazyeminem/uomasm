#
# ./masm/tools/CMakeLists.txt
#
#  Tim Cootes
#

LINK_LIBRARIES(masm msm_utils)

ADD_EXECUTABLE(masm_build_asm masm_build_asm.cxx)
TARGET_LINK_LIBRARIES(masm_build_asm masm_bldr)

ADD_EXECUTABLE(masm_draw_mean_model  masm_draw_mean_model.cxx)
ADD_EXECUTABLE(masm_search_image     masm_search_image.cxx)
ADD_EXECUTABLE(masm_make_asm_series  masm_make_asm_series.cxx)
ADD_EXECUTABLE(masm_series_search  masm_series_search.cxx)
ADD_EXECUTABLE(masm_test_search      masm_test_search.cxx)
