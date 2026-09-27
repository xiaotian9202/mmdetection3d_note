@echo off
chcp 65001 >nul
set "PATH=%LOCALAPPDATA%\Programs\MinGit\cmd;%LOCALAPPDATA%\Programs\MinGit\mingw64\bin;%PATH%"

echo ===================================================
echo [1/3] 检查 Git 环境...
git --version
if errorlevel 1 (
    echo [错误] 找不到 Git，请确认 MinGit 是否已下载。
    pause
    exit /b 1
)

echo.
echo [2/3] 检查远程仓库配置...
git remote -v

echo.
echo [3/3] 正在推送到 GitHub (https://github.com/xiaotian9202/mmdetection3d_note.git)...
echo 提示: 如果弹出浏览器登录窗口，请点击授权登录您的 Google/GitHub 账号。
echo.

git push -u origin main

if errorlevel 1 (
    echo.
    echo ===================================================
    echo [推送失败排查指引]:
    echo 1. 确认是否已在 GitHub 创建该仓库:
    echo    请访问 https://github.com/new 创建名为 mmdetection3d_note 的仓库
    echo.
    echo 2. 如果提示认证失败 (Authentication failed):
    echo    可在网页生成 Token (https://github.com/settings/tokens)
    echo    然后执行: git remote set-url origin https://xiaotian9202:TOKEN@github.com/xiaotian9202/mmdetection3d_note.git
    echo ===================================================
) else (
    echo.
    echo [成功] 代码已成功推送到 https://github.com/xiaotian9202/mmdetection3d_note !
)

pause
