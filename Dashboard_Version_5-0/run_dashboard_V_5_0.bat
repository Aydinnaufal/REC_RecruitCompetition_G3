@echo off
cd /d "%~dp0"
rem Uses the .venv in the folder above when there is one (the Programming folder), otherwise the python on your PATH.
if exist "..\.venv\Scripts\python.exe" (set "PY=..\.venv\Scripts\python.exe") else (set "PY=python")
echo Using %PY%
"%PY%" -m pip install -q -r requirements_V_5_0.txt
"%PY%" controller_dashboard_V_5_0.py
pause
