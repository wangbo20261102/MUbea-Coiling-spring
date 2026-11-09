@echo off
chcp 65001 >nul
set "ZONE_PYTHON=%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
if exist "%ZONE_PYTHON%" (
    "%ZONE_PYTHON%" -X utf8 "%~dp0test_zone_live.py" --ip 192.168.1.10
) else (
    python -X utf8 "%~dp0test_zone_live.py" --ip 192.168.1.10
)
echo.
pause
