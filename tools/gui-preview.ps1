param([string]$Output = '.\build\gui-preview')
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path $PSScriptRoot -Parent
$taskOutput = if ([System.IO.Path]::IsPathRooted($Output)) {
    [System.IO.Path]::GetFullPath($Output)
} else { [System.IO.Path]::GetFullPath((Join-Path $taskRoot $Output)) }
New-Item -ItemType Directory -Force -Path $taskOutput | Out-Null
$taskCompiler = Get-Command gcc -ErrorAction SilentlyContinue
if ($taskCompiler) { $taskCompilerPath = $taskCompiler.Source }
elseif (Test-Path 'C:\MinGW\bin\gcc.exe') { $taskCompilerPath = 'C:\MinGW\bin\gcc.exe' }
else { throw 'A host GCC compiler is required for the PBM preview exporter.' }
$taskSources = @('tools/gui-preview.c','firmware/gui/gui_canvas.c','firmware/gui/gui_font_assets.c',
    'firmware/gui/gui_plot.c','firmware/gui/gui_scene.c','firmware/gui/gui_menu.c','firmware/gui/gui_dashboard.c','firmware/gui/gui_tuning.c',
    'firmware/services/parameters/settings_record.c')
$taskSources += Get-ChildItem (Join-Path $taskRoot 'firmware/third_party/u8g2/csrc') -Filter '*.c' |
    ForEach-Object { $_.FullName }
$taskFiles = $taskSources | ForEach-Object {
    if ([System.IO.Path]::IsPathRooted($_)) { $_ } else { Join-Path $taskRoot $_ }
}
$taskExe = Join-Path $taskOutput 'gui-preview.exe'
& $taskCompilerPath -std=c11 -Os -flto -ffunction-sections -fdata-sections -Wall -Wextra `
    -I (Join-Path $taskRoot 'firmware/gui') -I (Join-Path $taskRoot 'firmware/third_party/u8g2/csrc') `
    -I (Join-Path $taskRoot 'firmware/services') -I (Join-Path $taskRoot 'firmware/services/parameters') `
    -I (Join-Path $taskRoot 'firmware/algorithms/calibration') `
    @taskFiles '-Wl,--gc-sections' -lm -o $taskExe
if ($LASTEXITCODE -ne 0) { throw 'Preview exporter build failed.' }
& $taskExe $taskOutput
if ($LASTEXITCODE -ne 0) { throw 'Preview export failed.' }
Write-Host "PBM previews: $taskOutput"
