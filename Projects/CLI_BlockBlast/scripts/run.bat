@echo off
echo Building the project...

:: Set the compiler (assuming you have MinGW or MSYS2 installed)
g++ -o build\CLI_Blaster.exe src\*.cpp

:: Check if the build was successful
if %ERRORLEVEL% NEQ 0 (
    echo Build failed!
    exit /b %ERRORLEVEL%
)

:: Run the executable
echo Running the program...
CLI_Blaster.exe

pause