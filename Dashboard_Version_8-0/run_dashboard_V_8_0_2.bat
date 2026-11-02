@echo off
cd /d "%~dp0"
set PY=python
if exist "..\.venv\Scripts\python.exe" set PY="..\.venv\Scripts\python.exe"
%PY% -m pip install -q -r requirements_V_8_0.txt
%PY% controller_dashboard_V_8_0_2.py
pause
