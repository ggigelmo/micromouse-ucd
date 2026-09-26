@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /std:c++17 /EHsc /O2 /Fe:micromouse.exe Main.cpp API.cpp src\MazeMap.cpp src\FloodFill.cpp src\MouseAgent.cpp
