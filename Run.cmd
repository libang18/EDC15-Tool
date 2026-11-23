@echo off
where pythonw.exe >nul 2>nul
if errorlevel 1 (
    echo Python with Tkinter was not found in PATH.
    pause
    exit /b 1
)
start "EDC15 Tool" pythonw.exe -B "%~dp0app.py"
