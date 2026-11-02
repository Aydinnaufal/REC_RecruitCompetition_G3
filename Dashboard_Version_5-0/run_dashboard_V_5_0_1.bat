@echo off
cd /d "%~dp0"
if exist "..\.venv\Scripts\python.exe" (set "PY=..\.venv\Scripts\python.exe") else (set "PY=python")
echo Using %PY%
"%PY%" -m pip install -q -r requirements_V_5_0.txt
"%PY%" controller_dashboard_V_5_0_1.py
pause
