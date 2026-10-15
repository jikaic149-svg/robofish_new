@echo off
setlocal
rem ---------------------------------------------------------------------------
rem  rlyoulink host UI launcher
rem  1) already running -> just open the browser
rem  2) not running      -> start the local service with pythonw (no console)
rem  3) python missing   -> show a dialog instead of failing silently
rem  ASCII only on purpose: cmd.exe reads .cmd as ANSI, so non-ASCII comment and
rem  dialog text here would be mis-decoded on some machines.
rem  Optional args are forwarded to launch.py, e.g. --no-browser
rem ---------------------------------------------------------------------------

set "HERE=%~dp0"
set "LAUNCH=%HERE%launch.py"
set "PYW="

rem -- try in order: py launcher, PATH, common install roots ------------------
where pyw.exe >nul 2>nul && set "PYW=pyw.exe"
if not defined PYW (
  where pythonw.exe >nul 2>nul && set "PYW=pythonw.exe"
)
if not defined PYW (
  for %%D in (D C) do (
    if not defined PYW if exist "%%D:\python\3.13\pythonw.exe" set "PYW=%%D:\python\3.13\pythonw.exe"
  )
)
if not defined PYW (
  for %%D in (D C) do (
    if not defined PYW if exist "%%D:\Python313\pythonw.exe" set "PYW=%%D:\Python313\pythonw.exe"
  )
)

if not defined PYW (
  mshta "javascript:alert('Python not found. Install Python 3 and enable Add python.exe to PATH, then run this file again.');close()"
  exit /b 1
)

start "" "%PYW%" "%LAUNCH%" %*
exit /b 0
