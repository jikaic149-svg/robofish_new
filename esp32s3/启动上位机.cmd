@echo off
rem 根目录入口：转发到 AmphiLink_UI\start.cmd
rem 与 AmphiLink_UI\start.cmd 是同一个流程，保留此文件只是为了在项目根目录能直接双击。
call "%~dp0AmphiLink_UI\start.cmd" %*
