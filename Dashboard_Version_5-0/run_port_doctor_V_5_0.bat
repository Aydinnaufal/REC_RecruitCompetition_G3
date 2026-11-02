@echo off
cd /d "%~dp0"
if exist "..\.venv\Scripts\python.exe" (set "PY=..\.venv\Scripts\python.exe") else (set "PY=python")
echo Using %PY%
"%PY%" -m pip install -q pyserial bleak
"%PY%" port_doctor_V_5_0.py --ble
pause
