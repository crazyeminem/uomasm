#
# ./uomasm/qasm/CMakeLists.txt
#
# Tim Cootes
#

doxygen_add_library(uomasm/qasm
  DEPENDS uomasm/masm uomqvxl/qmsm
  PACKAGE uomasm
  DESCRIPTION "Library of Widgets for Displaying and Manipulating Active Shape Models"
  )

SET(qasm_sources
  qasm_searcher_widget.h            qasm_searcher_widget.cxx
)

SET(qasm_moc_classes
  qasm_searcher_widget.h
)

QT_WRAP_CPP(qasm qasm_moc_files ${qasm_moc_classes})

ADD_LIBRARY(qasm ${qasm_sources} ${qasm_moc_files} )

TARGET_LINK_LIBRARIES(qasm masm qmsm)

SUBDIRS(tools)


