@echo off
rem Build with the Ninja generator (output in build-ninja; the Visual Studio build in build is unaffected).
rem   tools\build.cmd [debug|release] [test]
rem Sets up the Visual C++ environment, configures build-ninja\ on first use, builds, and runs the tests with "test".
setlocal
set "CFG=%~1"
if "%CFG%"=="" set "CFG=debug"
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDIR=%%i"
if not defined VSDIR (
  echo Visual Studio with the C++ tools was not found.
  exit /b 1
)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
cd /d "%~dp0.."
if not exist build-ninja\build.ninja ( cmake --preset ninja || exit /b 1 )
cmake --build --preset ninja-%CFG% || exit /b 1
if /i "%~2"=="test" ctest --preset ninja-%CFG%
