@echo off
chcp 65001 >nul
title 修复Cursor找不到工程文件（修复版）

:: 【核心修复】保存脚本原始所在的工程目录
set "original_dir=%~dp0"
set "original_dir=%original_dir:~0,-1%" :: 去掉末尾的\，避免路径错误

:: 自动申请管理员权限
fltmc >nul 2>&1 || (
    echo 正在请求管理员权限...
    powershell -Command "Start-Process cmd -ArgumentList '/c ""%~f0""' -Verb RunAs"
    exit /b
)

:: 【核心修复】提权后强制切回脚本所在的工程目录
cd /d "%original_dir%"
echo ========================================
echo 当前工作目录：%cd%
echo ========================================
echo.
echo 正在修复 Cursor 无法读取的工程文件...
echo.

:: 1. 仅在工程目录移除只读/隐藏/系统属性
echo [1/2] 移除文件属性...
attrib -r -h -s /s /d *

:: 2. 给当前用户赋予工程目录完全控制权限
echo [2/2] 修复文件夹权限...
icacls . /grant %username%:F /T /C /Q

echo.
echo ✅ 修复完成！
echo 请重新用 Cursor 打开工程根目录：%original_dir%
echo.
pause