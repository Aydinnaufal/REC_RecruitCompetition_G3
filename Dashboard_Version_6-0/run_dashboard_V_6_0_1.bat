@echo off
cd /d "%~dp0"
set PY=python
if exist "..\.venv\Scripts\python.exe" set PY="..\.venv\Scripts\python.exe"
%PY% -m pip install -q -r requirements_V_6_0.txt
%PY% controller_dashboard_V_6_0_1.py
pause
