<#
.SYNOPSIS
    在桌面创建 rlyoulink 上位机的启动快捷方式。

.DESCRIPTION
    快捷方式指向 start.cmd，双击即后台启动本地服务并自动打开浏览器，
    不使用控制台窗口。

.PARAMETER StartMenu
    改为安装到当前用户的开始菜单，而不是桌面。

.PARAMETER Remove
    删除已安装的快捷方式。

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File install-shortcut.ps1
    powershell -ExecutionPolicy Bypass -File install-shortcut.ps1 -StartMenu
    powershell -ExecutionPolicy Bypass -File install-shortcut.ps1 -Remove
#>
[CmdletBinding()]
param(
    [switch]$StartMenu,
    [switch]$Remove
)

$ErrorActionPreference = 'Stop'

$ui       = Split-Path -Parent $MyInvocation.MyCommand.Path
$target   = Join-Path $ui 'start.cmd'
$iconPath = Join-Path $ui 'rlyoulink.ico'
$name     = '机器鱼工作站.lnk'

if (-not (Test-Path -LiteralPath $target)) {
    throw "找不到启动入口：$target"
}

$folder = if ($StartMenu) {
    Join-Path ([Environment]::GetFolderPath('Programs')) 'rlyoulink'
} else {
    [Environment]::GetFolderPath('Desktop')
}
if (-not (Test-Path -LiteralPath $folder)) {
    New-Item -ItemType Directory -Path $folder -Force | Out-Null
}
$linkPath = Join-Path $folder $name

if ($Remove) {
    if (Test-Path -LiteralPath $linkPath) {
        Remove-Item -LiteralPath $linkPath -Force
        Write-Host "已删除：$linkPath"
    } else {
        Write-Host "未安装，无需删除：$linkPath"
    }
    return
}

# 图标按品牌标记现场生成，避免把二进制资源提交进仓库。
if (-not (Test-Path -LiteralPath $iconPath)) {
    $python = (Get-Command python -ErrorAction SilentlyContinue).Source
    if (-not $python) { $python = (Get-Command pythonw -ErrorAction SilentlyContinue).Source }
    if (-not $python) { $python = (Get-Command py -ErrorAction SilentlyContinue).Source }
    if ($python) {
        & $python (Join-Path $ui 'make_icon.py') $iconPath | Out-Null
    } else {
        Write-Warning '未找到 Python，改用默认图标。'
    }
}

$shell = New-Object -ComObject WScript.Shell
$link = $shell.CreateShortcut($linkPath)
$link.TargetPath       = $target
$link.WorkingDirectory = $ui
$link.Description      = '机器鱼工作站：控制、视觉、数据、调试与固件维护'
$link.WindowStyle      = 7          # 最小化，start.cmd 本身不显示窗口
if (Test-Path -LiteralPath $iconPath) {
    $link.IconLocation = "$iconPath,0"
}
$link.Save()

Write-Host "已创建快捷方式："
Write-Host "  $linkPath"
Write-Host "  -> $target"
Write-Host ''
Write-Host '以后双击该图标即可：服务在后台启动，浏览器自动打开。'
