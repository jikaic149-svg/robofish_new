@echo off
setlocal
if exist "D:\python\3.13\python.exe" (
  "D:\python\3.13\python.exe" "%~dp0install.py"
) else (
  py -3 "%~dp0install.py"
)
if errorlevel 1 pause
