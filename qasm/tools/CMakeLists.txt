#
# ./uomasm/qasm/tools/CMakeLists.txt
#
#   Tim Cootes
#

LINK_LIBRARIES(qasm) 


SET(qasm_markup_tool_moc_classes qasm_markup_tool.h )
QT_WRAP_CPP(qasm_markup_tool qasm_markup_tool_moc_files
            ${qasm_markup_tool_moc_classes})
ADD_EXECUTABLE( qasm_markup_tool qasm_markup_tool.cxx
                ${qasm_markup_tool_moc_files} )

