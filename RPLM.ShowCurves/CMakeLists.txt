if (DEFINED ENV{sCMakeVersion})
    cmake_minimum_required(VERSION $ENV{sCMakeVersion})
else()
    cmake_minimum_required(VERSION 3.16.0)
endif()

project(RPLM.CAD.ConjugationCurves LANGUAGES C CXX)

find_path(RPLM_CMAKE
    NAMES 
    "CMakeCompilerFlags.txt"
    PATHS
    # For RPLM.<Project name>/<Main|Dev>
    "${CMAKE_CURRENT_LIST_DIR}/../../RPLM.Common/CMake"
    # For RPLM.ALL
    "${CMAKE_CURRENT_LIST_DIR}/../CMake"
    # For RPLM.<Project name>/FeatureBranches/<Feature name>
    "${CMAKE_CURRENT_LIST_DIR}/../../../RPLM.Common/CMake"
    #For ~rplm.tp~ model
    "${CMAKE_CURRENT_LIST_DIR}/../../RPLM.BuildSystem/CMake"
    DOC "Find CMake folder"
    NO_DEFAULT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_CMAKE_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH)

if(NOT DEFINED RPLM_CMAKE)
	message(FATAL_ERROR "CMake folder is not found")
endif()

list(APPEND CMAKE_MODULE_PATH ${RPLM_CMAKE}/Includes)

include(RPLMCommon)
include(FindRPLM)
include(RPLMMiscMacros)

OPTION(RPLM_FOLDER_GROUP "Create folder structure." ON)

###QT paths
GET_QT_PREFIX_PATH(QT_PATH)

SET(CMAKE_PREFIX_PATH ${QT_PATH})

# Указывается директория с исходными файлами проекта
RPLM_ADD_SUBDIRECTORY(src)