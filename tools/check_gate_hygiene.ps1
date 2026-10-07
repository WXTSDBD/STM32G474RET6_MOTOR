# GATE / 条件编译卫生扫描（只报告，不改文件）
# 规格见 docs/规划_架构改进_2026-10-07.md §5.3 B3
#
# 用法：
#   .\tools\check_gate_hygiene.ps1
#   .\tools\check_gate_hygiene.ps1 -Root D:\stm32\STM32G474RET6_MOTOR

param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Stop"
Set-Location $Root

$scanDirs = @("motor", "config", "cal", "bringup", "board", "platform", "debug")
$excludeName = @("motor_params_m1.h") # 密度基线单独报，不进「编号宏名」误报堆

Write-Host "=== GATE hygiene @ $Root ==="
Write-Host ""

# 1) #if / #elif 与数字字面量比较（真 GATE 残留）
Write-Host "-- #if/#elif == <number> --"
$gateHits = @()
foreach ($d in $scanDirs) {
    $p = Join-Path $Root $d
    if (-not (Test-Path $p)) { continue }
    Get-ChildItem -Path $p -Recurse -Include *.c,*.h -File | ForEach-Object {
        $lines = Get-Content -LiteralPath $_.FullName -Encoding UTF8
        for ($i = 0; $i -lt $lines.Count; $i++) {
            if ($lines[$i] -match '^\s*#\s*(if|elif)\b.*==\s*\d+') {
                $gateHits += [pscustomobject]@{
                    File = $_.FullName.Substring($Root.Length + 1)
                    Line = $i + 1
                    Text = $lines[$i].Trim()
                }
            }
        }
    }
}
if ($gateHits.Count -eq 0) {
    Write-Host "  (none)"
} else {
    $gateHits | ForEach-Object { Write-Host ("  {0}:{1}: {2}" -f $_.File, $_.Line, $_.Text) }
}
Write-Host ("  count={0}" -f $gateHits.Count)
Write-Host ""

# 2) 编号型宏名：M1_*_GATE / GATE_<digits>
Write-Host "-- numbered-style macros (M1_*GATE* / GATE_<n>) --"
$macroHits = @()
foreach ($d in $scanDirs) {
    $p = Join-Path $Root $d
    if (-not (Test-Path $p)) { continue }
    Get-ChildItem -Path $p -Recurse -Include *.c,*.h -File | ForEach-Object {
        if ($excludeName -contains $_.Name) { return }
        $lines = Get-Content -LiteralPath $_.FullName -Encoding UTF8
        for ($i = 0; $i -lt $lines.Count; $i++) {
            if ($lines[$i] -match '#\s*define\s+(M1_\w*GATE\w*|GATE_\d+)\b') {
                $macroHits += [pscustomobject]@{
                    File = $_.FullName.Substring($Root.Length + 1)
                    Line = $i + 1
                    Text = $lines[$i].Trim()
                }
            }
        }
    }
}
if ($macroHits.Count -eq 0) {
    Write-Host "  (none outside motor_params_m1.h)"
} else {
    $macroHits | Select-Object -First 40 | ForEach-Object {
        Write-Host ("  {0}:{1}: {2}" -f $_.File, $_.Line, $_.Text)
    }
    if ($macroHits.Count -gt 40) {
        Write-Host ("  ... +{0} more" -f ($macroHits.Count - 40))
    }
}
Write-Host ("  count={0}" -f $macroHits.Count)
Write-Host ""

# 3) motor_params_m1.h 条件编译密度
$mp = Join-Path $Root "config\motor_params_m1.h"
Write-Host "-- conditional density: config/motor_params_m1.h --"
if (Test-Path $mp) {
    $lines = Get-Content -LiteralPath $mp -Encoding UTF8
    $total = $lines.Count
    $cond = 0
    foreach ($ln in $lines) {
        if ($ln -match '^\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b') {
            $cond++
        }
    }
    $pct = if ($total -gt 0) { [math]::Round(100.0 * $cond / $total, 1) } else { 0 }
    Write-Host ("  lines={0}  preprocessor_dir={1}  density={2}%  (baseline 20.5%)" -f $total, $cond, $pct)
} else {
    Write-Host "  MISSING"
}

Write-Host ""
Write-Host "done (report only)."
exit 0
