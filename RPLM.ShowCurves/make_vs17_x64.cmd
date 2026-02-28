@echo off

IF "%1" == "-h" (
	echo Options
	echo    "-h"          = Help
	echo    "-g"          = Download files from TFS server
	echo    "-p"          = Only generate project
	echo    "-c"          = Clear previous build results. Rebuld all. Optional
	echo    Selected configuration or all configurations, if not specified
	echo        "-r"      = Release
	echo        "-d"      = Debug
	echo        "-rwdi"   = Release With Debug Info
	echo    "-i"          = Install
	goto:EOF
)

set /a getfiles = 0
set /a rebuild  = 0
set /a release = 0
set /a debug = 0
set /a releasedebug = 0
set /a installresults = 0
set /a onlygenerateproject = 0

IF "%1" == "-g" set /a getfiles = 1
IF "%1" == "-c" set /a rebuild = 1
IF "%1" == "-i" set /a installresults = 1
IF "%1" == "-r" set /a release = 1
IF "%1" == "-d" set /a debug = 1
IF "%1" == "-rwdi" set /a releasedebug = 1
IF "%1" == "-p" set /a onlygenerateproject = 1

IF "%2" == "-g" set /a getfiles = 1
IF "%2" == "-c" set /a rebuild = 1
IF "%2" == "-i" set /a installresults = 1
IF "%2" == "-r" set /a release = 1
IF "%2" == "-d" set /a debug = 1
IF "%2" == "-rwdi" set /a releasedebug = 1
IF "%2" == "-p" set /a onlygenerateproject = 1

IF "%3" == "-g" set /a getfiles = 1
IF "%3" == "-c" set /a rebuild = 1
IF "%3" == "-i" set /a installresults = 1
IF "%3" == "-r" set /a release = 1
IF "%3" == "-d" set /a debug = 1
IF "%3" == "-rwdi" set /a releasedebug = 1
IF "%3" == "-p" set /a onlygenerateproject = 1

IF "%4" == "-g" set /a getfiles = 1
IF "%4" == "-c" set /a rebuild = 1
IF "%4" == "-i" set /a installresults = 1
IF "%4" == "-r" set /a release = 1
IF "%4" == "-d" set /a debug = 1
IF "%4" == "-rwdi" set /a releasedebug = 1
IF "%4" == "-p" set /a onlygenerateproject = 1

IF "%5" == "-g" set /a getfiles = 1
IF "%5" == "-c" set /a rebuild = 1
IF "%5" == "-i" set /a installresults = 1
IF "%5" == "-r" set /a release = 1
IF "%5" == "-d" set /a debug = 1
IF "%5" == "-rwdi" set /a releasedebug = 1
IF "%5" == "-p" set /a onlygenerateproject = 1

IF %getfiles% == 1 (
echo Download files from TFS server
"D:\Program_Files\VS2019\Common7\IDE\CommonExtensions\Microsoft\TeamFoundation\Team Explorer\tf" get %CD% /recursive
)

SET cmake_path="%cd%/../CMake/"
IF EXIST %cmake_path% (
	SET build_path="%cd%/../../builds/vs17/x64/RPLM.CAD.ConjugationCurves"
	SET install_path="%cd%/../../install/vs17/x64"
) ELSE (
	SET build_path="%cd%/../../../builds/vs17/x64/RPLM.CAD.ConjugationCurves"
	SET install_path="%cd%/../../../install/vs17/x64"
)

set config_types=Debug;Release;RelWithDebInfo

IF %onlygenerateproject% == 1 (
	echo Generate project
	cmake -S"." -G"Visual Studio 15 2017" -B%build_path% -A"x64" -DCMAKE_INSTALL_PREFIX=%install_path% -DCMAKE_CONFIGURATION_TYPES="%config_types%"
	goto:EOF
)

IF %rebuild% == 1 (
	IF EXIST %build_path% (
		echo Clear previous build results. Rebuld all
	rd /S /Q %build_path%
	)
)

echo Generate project
cmake -S"." -G"Visual Studio 15 2017" -B%build_path% -A"x64" -DCMAKE_INSTALL_PREFIX=%install_path% -DCMAKE_CONFIGURATION_TYPES="%config_types%"

IF %release% == 1 goto build_relase
IF %debug% == 1 goto build_debug
IF %releasedebug% == 1 goto build_release_with_debug_info

goto build_all

:build_relase
echo Build Release
cmake --build %build_path% --config Release
IF %installresults% == 1 (
echo Install Release
cmake  -DCMAKE_INSTALL_PREFIX=%install_path% -DCMAKE_INSTALL_CONFIG_NAME=Release -P %build_path%/cmake_install.cmake
)
goto:EOF

:build_debug
echo Build Debug
cmake --build %build_path% --config Debug
IF %installresults% == 1 (
echo Install Debug
cmake  -DCMAKE_INSTALL_PREFIX=%install_path% -DCMAKE_INSTALL_CONFIG_NAME=Debug -P %build_path%/cmake_install.cmake
)
goto:EOF

:build_release_with_debug_info
echo Build RelWithDebInfo
cmake --build %build_path% --config RelWithDebInfo
IF %installresults% == 1 (
echo Install RelWithDebInfo
cmake  -DCMAKE_INSTALL_PREFIX=%install_path% -DCMAKE_INSTALL_CONFIG_NAME=RelWithDebInfo -P %build_path%/cmake_install.cmake
)
goto:EOF

:build_all
echo Build All
cmake --build %build_path% --config Release
IF %installresults% == 1 (
echo Install Release
cmake  -DCMAKE_INSTALL_PREFIX=%install_path% -DCMAKE_INSTALL_CONFIG_NAME=Release -P %build_path%/cmake_install.cmake
)
cmake --build %build_path% --config Debug
IF %installresults% == 1 (
echo Install Debug
cmake  -DCMAKE_INSTALL_PREFIX=%install_path% -DCMAKE_INSTALL_CONFIG_NAME=Debug -P %build_path%/cmake_install.cmake
)
cmake --build %build_path% --config RelWithDebInfo
IF %installresults% == 1 (
echo Install RelWithDebInfo
cmake  -DCMAKE_INSTALL_PREFIX=%install_path% -DCMAKE_INSTALL_CONFIG_NAME=RelWithDebInfo -P %build_path%/cmake_install.cmake
)
goto:EOF
