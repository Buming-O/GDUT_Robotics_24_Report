@echo off
set MATLAB=H:\MATLAB~1
call "H:\matlabR2021b\sys\lcc64\lcc64\mex\lcc64opts.bat"
"H:\matlabR2021b\toolbox\shared\coder\ninja\win64\ninja.exe" -t compdb cc cxx cudac > compile_commands.json
"H:\matlabR2021b\toolbox\shared\coder\ninja\win64\ninja.exe" -v %*
