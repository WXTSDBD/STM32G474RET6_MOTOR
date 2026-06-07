@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0fix_keil_ac6_freertos.ps1"
exit /b %ERRORLEVEL%
