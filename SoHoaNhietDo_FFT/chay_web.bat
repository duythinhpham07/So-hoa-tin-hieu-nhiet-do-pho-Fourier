@echo off
title Pho Fourier - ESP32
cd /d "%~dp0"
echo Dang khoi dong web server...
start "" "http://localhost:8000"
python server.py 8000
pause
