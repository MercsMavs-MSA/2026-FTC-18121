@echo off
rem Laptop simulation of the Maverick robot code (no robot needed).
rem Put this "sim" folder in the project root, next to TeamCode (NOT inside TeamCode).
rem Run:   run_sim.bat               all scenarios
rem        run_sim.bat shoot fixed   only scenarios whose name contains a word
rem Needs a JDK. Uses JAVA_HOME, then Android Studio's bundled JDK, then whatever is on PATH.
setlocal enabledelayedexpansion
cd /d "%~dp0"
set "MAV=%~dp0..\TeamCode\src\main\java\org\firstinspires\ftc\Maverick"
set "JDK="
if defined JAVA_HOME if exist "%JAVA_HOME%\bin\javac.exe" set "JDK=%JAVA_HOME%\bin\"
if not defined JDK if exist "%ProgramFiles%\Android\Android Studio\jbr\bin\javac.exe" set "JDK=%ProgramFiles%\Android\Android Studio\jbr\bin\"
if exist build rmdir /s /q build
mkdir build
for /r fakesdk %%f in (*.java) do (set "p=%%~ff" & echo "!p:\=/!">> build\sources.txt)
for /r src %%f in (*.java) do (set "p=%%~ff" & echo "!p:\=/!">> build\sources.txt)
for /r "%MAV%\Subsystems" %%f in (*.java) do (set "p=%%~ff" & echo "!p:\=/!">> build\sources.txt)
set "p=%MAV%\Robot.java"
echo "!p:\=/!">> build\sources.txt
"%JDK%javac" -nowarn -encoding UTF-8 -d build @build\sources.txt
if errorlevel 1 (
  echo Compile failed.
  pause
  exit /b 1
)
"%JDK%java" -cp build sim.SimRunner %*
pause
