
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set("solo-collisions_INCLUDE_DIRS" "/usr/local/include")
set("SOLO_COLLISIONS_INCLUDE_DIRS" "/usr/local/include")
set("solo-collisions_DOXYGENDOCDIR" "${PACKAGE_PREFIX_DIR}/share/doc/solo-collisions/doxygen-html")
set("SOLO_COLLISIONS_DOXYGENDOCDIR" "${PACKAGE_PREFIX_DIR}/share/doc/solo-collisions/doxygen-html")
set("solo-collisions_DEPENDENCIES" "pinocchio;CppAD;CppADCodeGen;Eigen3")
set("solo-collisions_PKG_CONFIG_DEPENDENCIES" "")

# Find absolute library paths for all _PKG_CONFIG_LIBS as CMake expects full paths, while pkg-config does not.
set(_PACKAGE_CONFIG_LIBRARIES "")
set(_PKG_CONFIG_LIBDIR "/opt/openrobots/lib")
set(_PKG_CONFIG_LIBS_LIST "")
if(_PKG_CONFIG_LIBS_LIST)
  string(FIND ${_PKG_CONFIG_LIBS_LIST} ", " _is_comma_space)
  while(_is_comma_space GREATER -1)
    string(REPLACE ", " "," _PKG_CONFIG_LIBS_LIST "${_PKG_CONFIG_LIBS_LIST}")
    string(FIND ${_PKG_CONFIG_LIBS_LIST} ", " _is_comma_space)
  endwhile()
  string(REPLACE " " ";" _PKG_CONFIG_LIBS_LIST "${_PKG_CONFIG_LIBS_LIST}")
  set(LIBDIR_HINTS ${_PKG_CONFIG_LIBDIR})
  foreach(component ${_PKG_CONFIG_LIBS_LIST})
    string(STRIP ${component} component)
    # If the component is a link directory ("-L/full/path"), append to LIBDIR_HINTS.
    string(FIND ${component} "-L" _is_library_dir)
    if(${_is_library_dir} EQUAL 0)
      string(REPLACE "-L" "" lib_path ${component})
      list(APPEND LIBDIR_HINTS "${lib_path}")
      continue()
    endif()
    # If the component is a library name
    string(FIND ${component} "-l" _is_library_name)
    if(${_is_library_name} EQUAL 0)
      string(REPLACE "-l" "" lib ${component})
      find_library(abs_lib_${lib} ${lib} HINTS ${LIBDIR_HINTS})
      if(NOT abs_lib_${lib})
        message(STATUS "${lib} searched on ${_LIBDIR_HINTS} not FOUND.")
      else()
        message(STATUS "${lib} searched on ${_LIBDIR_HINTS} FOUND. ${lib} at ${abs_lib_${lib}}")
        list(APPEND _PACKAGE_CONFIG_LIBRARIES "${abs_lib_${lib}}")
      endif()
      unset(abs_lib_${lib} CACHE)
      continue()
    endif()
    # If the component contains a collection of additional arguments
    string(FIND ${component} "," _is_collection)
    if(${_is_collection} GREATER -1)
      string(REPLACE "," ";" component_list "${component}")
      list(GET component_list -1 lib_info)
      set(options ${component})
      list(REMOVE_AT options -1)
      string(FIND ${lib_info} "-l" _is_library_name)
      if(${_is_library_name} GREATER -1)
        string(REPLACE "-l" "" lib ${lib_info})
        find_library(abs_lib_${lib} ${lib} HINTS ${LIBDIR_HINTS})
        if(NOT abs_lib_${lib})
          message(STATUS "${lib} searched on ${_LIBDIR_HINTS} not FOUND.")
        else()
          message(STATUS "${lib} searched on ${_LIBDIR_HINTS} FOUND. ${lib} at ${abs_lib_${lib}}")
          list(APPEND _PACKAGE_CONFIG_LIBRARIES "${abs_lib_${lib}}")
        endif()
        unset(abs_lib_${lib} CACHE)
        continue()
      else() # This is an absolute lib
        list(APPEND _PACKAGE_CONFIG_LIBRARIES "${component}")
      endif()
      continue()
    endif()
    # Else, this is just an absolute lib
    if(EXISTS "${component}")
      list(APPEND _PACKAGE_CONFIG_LIBRARIES "${component}")
    endif()
  endforeach()
endif(_PKG_CONFIG_LIBS_LIST)

set("solo-collisions_LIBRARIES" ${_PACKAGE_CONFIG_LIBRARIES})
set("SOLO_COLLISIONS_LIBRARIES" ${_PACKAGE_CONFIG_LIBRARIES})

include(CMakeFindDependencyMacro)
find_package(pinocchio REQUIRED)
find_package(CppAD REQUIRED)
find_package(CppADCodeGen REQUIRED)
find_package(Eigen3 REQUIRED)

IF(COMMAND ADD_REQUIRED_DEPENDENCY)
  FOREACH(pkgcfg_dep ${solo-collisions_PKG_CONFIG_DEPENDENCIES})
    ADD_REQUIRED_DEPENDENCY(${pkgcfg_dep})
  ENDFOREACH()
ENDIF(COMMAND ADD_REQUIRED_DEPENDENCY)

include("${CMAKE_CURRENT_LIST_DIR}/solo-collisionsTargets.cmake")
check_required_components("solo-collisions")


