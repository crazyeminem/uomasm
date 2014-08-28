# isbe_apm/masm/tests/CMakeLists.txt

ADD_EXECUTABLE( masm_test_all
  test_driver.cxx
  test_ncc_finder.cxx
  test_region_pdf_finder.cxx
  test_lin_reg_finder.cxx
  test_grad_corr_finder.cxx
  )

TARGET_LINK_LIBRARIES( masm_test_all masm masm_bldr vil_algo testlib )

ADD_TEST( masm_test_ncc_finder ${EXECUTABLE_OUTPUT_PATH}/masm_test_all test_ncc_finder)
ADD_TEST( masm_test_region_pdf_finder ${EXECUTABLE_OUTPUT_PATH}/masm_test_all test_region_pdf_finder)
ADD_TEST( masm_test_lin_reg_finder ${EXECUTABLE_OUTPUT_PATH}/masm_test_all test_lin_reg_finder)
ADD_TEST( masm_test_grad_corr_finder ${EXECUTABLE_OUTPUT_PATH}/masm_test_all test_grad_corr_finder)

#ADD_EXECUTABLE( masm_test_include test_include.cxx )
#TARGET_LINK_LIBRARIES( masm_test_include masm masm_bldr)
