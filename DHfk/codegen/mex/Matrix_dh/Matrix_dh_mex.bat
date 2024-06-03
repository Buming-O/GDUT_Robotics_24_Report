@echo off
set MATLAB=D:\MATLAB_R2021b
call "D:\MATLAB_R2021b\sys\lcc64\lcc64\mex\lcc64opts.bat"
"D:\MATLAB_R2021b\toolbox\shared\coder\ninja\win64\ninja.exe" -t compdb cc cxx cudac > compile_commands.json
"D:\MATLAB_R2021b\toolbox\shared\coder\ninja\win64\ninja.exe" -v %*
