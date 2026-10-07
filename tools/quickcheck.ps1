#requires -Version 5.1
<#
  quickcheck.ps1 —— 改完代码后 1~2 秒知道有没有编译错

  为什么需要它：
    Cursor 改完文件，IDE 会立刻回传 linter 错误（协议里有 read_lints、
    were_all_new_linter_errors_resolved_by_this_edit 这些字段）。
    Cline 拿不到这些 —— 它改完是"瞎"的，只能跑一次 Keil 全量编译（1~2 分钟）。
    本脚本用 Keil 自带的 armclang 做单文件语法检查，约 1.8 秒/文件，
    判定与 Keil 一致（实测：不加额外标志时，与 Keil 的 0 Warning 结果对齐）。

  用法：
    .\tools\quickcheck.ps1                                    # 自动查 git 改动过的 .c
    .\tools\quickcheck.ps1 motor\motor_current.c motor\foc_svpwm.c
    .\tools\quickcheck.ps1 motor\motor_current.c -Strict       # -Wall，比 Keil 严

  退出码：0 = 干净；1 = 有 error 或 warning；2 = 环境问题

  ⚠ 三个已踩过的坑（改这个脚本时别踩回去）：

    1. 本文件必须存成 UTF-8 **带 BOM**。否则 Windows PowerShell 5.1 按 ANSI 读，
       中文变乱码、引号错位导致解析失败。

    2. `Set-Location` / `Push-Location` **不会**改变 .NET 进程的 CurrentDirectory，
       而原生 exe 继承的是后者。所以 IncludePath 必须转成**绝对路径**，
       不能直接用工程里的 `../config` 相对写法。

    3. 给参数手动加引号（`'"' + $p + '"'`）在 PS 5.1 下**不会**被剥掉，
       clang 会把引号当成文件名的一部分 → "no such file or directory"。
       直接传裸路径。路径含空格时 PS 5.1 有已知问题，脚本会提示。

    4. `$xml.Project...TargetArmAds.Cads.VariousControls.IncludePath` 的属性导航
       在本工程取到空值 —— 改用正则从 <TargetArmAds> 段里直接抓。
#>
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Files,

    [switch]$Strict
)

$ErrorActionPreference = 'Continue'

$armclang = 'D:\keil5\ARM\ARMCLANG\bin\armclang.exe'
if (-not (Test-Path -LiteralPath $armclang)) {
    Write-Host "[quickcheck] 找不到 armclang: $armclang" -ForegroundColor Red
    exit 2
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$uvprojx  = Join-Path $repoRoot 'MDK-ARM\STM32G474RET6_MOTOR.uvprojx'
if (-not (Test-Path -LiteralPath $uvprojx)) {
    Write-Host "[quickcheck] 找不到 Keil 工程: $uvprojx" -ForegroundColor Red
    exit 2
}
$mdkDir = Split-Path -Parent $uvprojx

# --- 从 Keil 工程读真实的 IncludePath / Define（不猜） ---
$raw = Get-Content -LiteralPath $uvprojx -Raw -Encoding UTF8
$ads = [regex]::Match($raw, '(?s)<TargetArmAds>.*?</TargetArmAds>').Value
if (-not $ads) {
    Write-Host "[quickcheck] .uvprojx 里找不到 <TargetArmAds> 段" -ForegroundColor Red
    exit 2
}
$defines = @(([regex]::Match($ads, '<Define>(.*?)</Define>')).Groups[1].Value -split ',' |
             Where-Object { $_ })
$rawInc  = ([regex]::Match($ads, '<IncludePath>(.*?)</IncludePath>')).Groups[1].Value
# 关键：转绝对路径（见文件头第 2 条坑）
$includes = @($rawInc -split ';' | Where-Object { $_ } | ForEach-Object {
    [IO.Path]::GetFullPath((Join-Path $mdkDir $_))
})
if ($includes.Count -eq 0) {
    Write-Host "[quickcheck] 没解析出 IncludePath" -ForegroundColor Red
    exit 2
}

$baseArgs = @(
    '--target=arm-arm-none-eabi',
    '-mcpu=cortex-m4',
    '-mfpu=fpv4-sp-d16',
    '-mfloat-abi=hard',
    '-c',
    '-std=gnu11',
    '-fsyntax-only'
)
if ($Strict) { $baseArgs += '-Wall' }
foreach ($d in $defines)  { $baseArgs += ('-D' + $d) }
foreach ($i in $includes) { $baseArgs += ('-I' + $i) }

# 让原生 exe 的 CWD 也落在 MDK-ARM（双保险）
[Environment]::CurrentDirectory = $mdkDir

# --- 没给文件就自动取 git 改动的 .c ---
if (-not $Files -or $Files.Count -eq 0) {
    Push-Location $repoRoot
    try     { $rawGit = & git status --porcelain 2>$null }
    finally { Pop-Location }
    $Files = @($rawGit | ForEach-Object { ($_ -replace '^...', '').Trim('"') } |
               Where-Object { $_ -match '\.c$' })
    if ($Files.Count -eq 0) {
        Write-Host "[quickcheck] 没有 git 改动过的 .c。"
        Write-Host "             用法: .\tools\quickcheck.ps1 文件1 文件2 ..."
        exit 0
    }
    Write-Host "[quickcheck] 自动选中 $($Files.Count) 个改动文件"
}

$stdoutTmp = [IO.Path]::GetTempFileName()
$stderrTmp = [IO.Path]::GetTempFileName()

$totErr = 0; $totWarn = 0; $failed = 0; $checked = 0
$sw = [Diagnostics.Stopwatch]::StartNew()

foreach ($f in $Files) {
    $full = $f
    if (-not [IO.Path]::IsPathRooted($full)) { $full = Join-Path $repoRoot $f }
    if (-not (Test-Path -LiteralPath $full)) {
        Write-Host ("[SKIP] {0}  (文件不存在)" -f $f) -ForegroundColor DarkGray
        continue
    }
    if ($full -match '\s') {
        Write-Host ("[SKIP] {0}  (路径含空格，PS 5.1 传参有已知问题)" -f $f) -ForegroundColor Yellow
        continue
    }
    $checked++

    $a = $baseArgs + @($full)
    if ([IO.Path]::GetExtension($full) -eq '.h') {
        $a = $baseArgs + @('-x','c-header',$full)
    }

    & $armclang @a 1>$stdoutTmp 2>$stderrTmp

    $txt = ''
    $h1 = Get-Content -LiteralPath $stdoutTmp -Raw -ErrorAction SilentlyContinue
    $h2 = Get-Content -LiteralPath $stderrTmp -Raw -ErrorAction SilentlyContinue
    if ($h1) { $txt += $h1 }
    if ($h2) { $txt += $h2 }

    $e = [regex]::Matches($txt, '(?m)^.*\berror:.*$')
    $w = [regex]::Matches($txt, '(?m)^.*\bwarning:.*$')
    $totErr += $e.Count; $totWarn += $w.Count

    $rel = $f
    if ([IO.Path]::IsPathRooted($rel) -and $rel.ToLower().StartsWith($repoRoot.ToLower())) {
        $rel = $rel.Substring($repoRoot.Length).TrimStart('\', '/')
    }
    $rel = $rel -replace '/', '\'

    if ($e.Count -gt 0) {
        Write-Host ("[ERR ] {0}  ({1} error, {2} warning)" -f $rel, $e.Count, $w.Count) -ForegroundColor Red
        $failed++
    } elseif ($w.Count -gt 0) {
        Write-Host ("[WARN] {0}  ({1} warning)" -f $rel, $w.Count) -ForegroundColor Yellow
        $failed++
    } else {
        Write-Host ("[ OK ] {0}" -f $rel) -ForegroundColor Green
    }

    if ($e.Count -gt 0 -or $w.Count -gt 0) {
        foreach ($m in @($e) + @($w)) {
            $line = $m.Value.Trim()
            $line = $line -replace [regex]::Escape($repoRoot + '\'), ''
            $line = $line -replace '^armclang\.exe\s*:\s*', ''
            Write-Host ("       " + $line)
        }
    }
}

Remove-Item -LiteralPath $stdoutTmp, $stderrTmp -Force -ErrorAction SilentlyContinue
$sw.Stop()

$color = 'Green'
if ($failed -gt 0) { $color = 'Red' }
Write-Host ''
Write-Host ("[quickcheck] 检查 {0} 个文件 / error {1} / warning {2} / {3}s" -f `
    $checked, $totErr, $totWarn, [math]::Round($sw.Elapsed.TotalSeconds, 1)) -ForegroundColor $color

if ($failed -gt 0) {
    Write-Host "            -> 先修干净再说话。这只是单文件检查，交付前仍须跑完整 Keil（见 .clinerules/10-交付前编译.md）"
    exit 1
}
Write-Host "            -> 通过。注意：只查了语法/类型，不代替完整 Keil 编译。"
exit 0