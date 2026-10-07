@echo off
setlocal
cd /d "%~dp0"
if exist "%LocalAppData%\Python\pythoncore-3.14-64\python.exe" (
    "%LocalAppData%\Python\pythoncore-3.14-64\python.exe" "%~dp0simulate.py" %*
) else (
    py -3 "%~dp0simulate.py" %*
)
if errorlevel 1 pause
