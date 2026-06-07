# Run after CubeMX Generate: Keil AC6 (ArmClang) + FreeRTOS fixes.
# - Restore GCC port files
# - Patch uvprojx (RVDS -> GCC, strip UTF-8 BOM)
# - Patch FreeRTOSConfig.h USER CODE (SystemCoreClock, configENABLE_FPU)

$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$uvprojx = Join-Path $root 'MDK-ARM\STM32G474RET6_MOTOR.uvprojx'
$freertosConfig = Join-Path $root 'Core\Inc\FreeRTOSConfig.h'
$gccDir = Join-Path $root 'Middlewares\Third_Party\FreeRTOS\Source\portable\GCC\ARM_CM4F'
$bundledGccDir = Join-Path $root 'tools\freertos_port\GCC\ARM_CM4F'
$fwGccDir = Join-Path $env:USERPROFILE 'STM32Cube\Repository\STM32Cube_FW_G4_V1.6.2\Middlewares\Third_Party\FreeRTOS\Source\portable\GCC\ARM_CM4F'

function Ensure-GccPortFiles {
    $srcDir = $null
    if ((Test-Path (Join-Path $bundledGccDir 'port.c')) -and (Test-Path (Join-Path $bundledGccDir 'portmacro.h'))) {
        $srcDir = $bundledGccDir
    }
    elseif ((Test-Path (Join-Path $fwGccDir 'port.c')) -and (Test-Path (Join-Path $fwGccDir 'portmacro.h'))) {
        $srcDir = $fwGccDir
    }
    else {
        Write-Error "GCC port missing. Expected: $bundledGccDir"
    }

    New-Item -ItemType Directory -Force -Path $gccDir | Out-Null
    Copy-Item (Join-Path $srcDir 'port.c') -Destination $gccDir -Force
    Copy-Item (Join-Path $srcDir 'portmacro.h') -Destination $gccDir -Force
}

function Patch-Uvprojx {
    if (-not (Test-Path $uvprojx)) {
        Write-Error "uvprojx not found: $uvprojx"
    }

    $bytes = [System.IO.File]::ReadAllBytes($uvprojx)
    $hadBom = ($bytes.Length -ge 3) -and ($bytes[0] -eq 0xEF) -and ($bytes[1] -eq 0xBB) -and ($bytes[2] -eq 0xBF)

    $utf8NoBom = New-Object System.Text.UTF8Encoding $false
    $content = $utf8NoBom.GetString($bytes)
    if ($hadBom) {
        $content = $content.TrimStart([char]0xFEFF)
    }
    $original = $content

    $content = $content.Replace('portable/RVDS/ARM_CM4F', 'portable/GCC/ARM_CM4F')
    $content = $content.Replace('portable\RVDS\ARM_CM4F', 'portable\GCC\ARM_CM4F')
    $content = $content.Replace(',__CC_ARM', '')
    $content = $content.Replace('__CC_ARM,', '')

    if (($content -ne $original) -or $hadBom) {
        [System.IO.File]::WriteAllText($uvprojx, $content, $utf8NoBom)
        if ($content -ne $original) {
            Write-Host "Patched: $uvprojx (RVDS -> GCC)"
        }
        if ($hadBom) {
            Write-Host "Removed UTF-8 BOM from: $uvprojx"
        }
    }
    else {
        Write-Host "uvprojx already uses GCC port."
    }
}

function Write-Utf8NoBom {
    param(
        [string]$Path,
        [string]$Content
    )
    $utf8NoBom = New-Object System.Text.UTF8Encoding $false
    [System.IO.File]::WriteAllText($Path, $Content, $utf8NoBom)
}

function Patch-FreeRTOSConfig {
    if (-not (Test-Path $freertosConfig)) {
        Write-Warning "FreeRTOSConfig.h not found, skipped: $freertosConfig"
        return
    }

    $utf8NoBom = New-Object System.Text.UTF8Encoding $false
    $content = $utf8NoBom.GetString([System.IO.File]::ReadAllBytes($freertosConfig))
    $original = $content
    $changed = $false

    if ($content -match '(?s)/\*\s*USER CODE BEGIN Includes\s*\*/(.*?)/\*\s*USER CODE END Includes\s*\*/') {
        if ($Matches[1] -notmatch 'extern\s+uint32_t\s+SystemCoreClock') {
            $ac6Includes = @"
#include <stdint.h>
extern uint32_t SystemCoreClock;

"@
            $content = $content -replace '(?s)(/\*\s*USER CODE BEGIN Includes\s*\*/\r?\n)', "`$1$ac6Includes"
            $changed = $true
            Write-Host "Patched: FreeRTOSConfig.h (USER CODE Includes: SystemCoreClock)"
        }
    }

    if ($content -match '(?s)/\*\s*USER CODE BEGIN Defines\s*\*/(.*?)/\*\s*USER CODE END Defines\s*\*/') {
        $needsFpu = $Matches[1] -notmatch '#\s*define\s+configENABLE_FPU\s+1\b'
    }
    else {
        $needsFpu = $true
    }
    if ($needsFpu) {
        $fpuDefines = @"
#undef configENABLE_FPU
#define configENABLE_FPU                         1

"@
        $content = $content -replace '(?s)(/\*\s*USER CODE BEGIN Defines\s*\*/\r?\n)', "`$1$fpuDefines"
        $changed = $true
        Write-Host "Patched: FreeRTOSConfig.h (USER CODE Defines: configENABLE_FPU)"
    }

    if (-not ($content -match '(?s)/\*\s*USER CODE BEGIN Includes\s*\*/')) {
        Write-Warning 'FreeRTOSConfig.h: USER CODE BEGIN Includes not found, skipped SystemCoreClock patch.'
    }

    if ($changed) {
        Write-Utf8NoBom -Path $freertosConfig -Content $content
    }
    else {
        Write-Host "FreeRTOSConfig.h already configured for AC6."
    }
}

Ensure-GccPortFiles
Patch-Uvprojx
Patch-FreeRTOSConfig
Write-Host 'OK: Keil AC6 FreeRTOS fix applied.'
