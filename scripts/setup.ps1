[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$toolsDir = Join-Path $projectRoot ".tools"
$archive = Join-Path $toolsDir "winlibs.7z"
$toolBin = Join-Path $toolsDir "mingw64\bin"
$toolUrl = "https://github.com/brechtsanders/winlibs_mingw/releases/download/16.1.0posix-14.0.0-ucrt-r2/winlibs-x86_64-posix-seh-gcc-16.1.0-mingw-w64ucrt-14.0.0-r2.7z"
$toolSha256 = "62fb8588d2deee7d662dbcbd386702adbf19643764c971c38aa4839472eee232"
$extractor = Join-Path $toolsDir "7zr.exe"
$extractorUrl = "https://www.7-zip.org/a/7zr.exe"
$extractorSha256 = "56b8cc9f4971cef253644fafe54063ed7fdca551d4dee0f8c6baa81b855acd72"

New-Item -ItemType Directory -Path $toolsDir -Force | Out-Null
if (-not (Test-Path (Join-Path $toolBin "g++.exe"))) {
    if (-not (Test-Path $archive)) {
        Write-Host "Downloading portable GCC/CMake/Ninja toolchain..."
        & curl.exe -L --fail --retry 3 --progress-bar -o $archive $toolUrl
        if ($LASTEXITCODE -ne 0) { throw "Toolchain download failed" }
    }
    $actualHash = (Get-FileHash -Algorithm SHA256 $archive).Hash.ToLowerInvariant()
    if ($actualHash -ne $toolSha256) {
        throw "Toolchain SHA-256 mismatch. Expected $toolSha256, got $actualHash"
    }
    if (-not (Test-Path $extractor)) {
        Write-Host "Downloading the official standalone 7-Zip extractor..."
        & curl.exe -L --fail --retry 3 --progress-bar -o $extractor $extractorUrl
        if ($LASTEXITCODE -ne 0) { throw "7-Zip extractor download failed" }
    }
    $actualExtractorHash = (Get-FileHash -Algorithm SHA256 $extractor).Hash.ToLowerInvariant()
    if ($actualExtractorHash -ne $extractorSha256) {
        throw "7-Zip SHA-256 mismatch. Expected $extractorSha256, got $actualExtractorHash"
    }
    Write-Host "Extracting portable toolchain..."
    $outputOption = "-o$toolsDir"
    & $extractor x $archive $outputOption -y
    if ($LASTEXITCODE -ne 0) { throw "Could not extract the WinLibs .7z archive" }
}

$env:Path = "$toolBin;$env:Path"
$cmake = Join-Path $toolBin "cmake.exe"
$compiler = Join-Path $toolBin "g++.exe"
$pythonLauncher = Get-Command py.exe -ErrorAction SilentlyContinue
$venvDir = Join-Path $projectRoot ".venv"
$venvPython = Join-Path $venvDir "Scripts\python.exe"

if (-not (Test-Path $venvPython)) {
    Write-Host "Creating Python 3.11 virtual environment..."
    if ($pythonLauncher) {
        & $pythonLauncher.Source -3.11 -m venv $venvDir
    } else {
        & python.exe -m venv $venvDir
    }
    if ($LASTEXITCODE -ne 0) { throw "Could not create Python virtual environment" }
}

Write-Host "Python demo uses only the standard library; no packages to download."

$buildDir = Join-Path $projectRoot "build-x4"
Write-Host "Configuring the official YDLIDAR SDK as a native DLL..."
$configureArgs = @(
    "-S", $projectRoot,
    "-B", $buildDir,
    "-G", "Ninja",
    "-DCMAKE_BUILD_TYPE=Release",
    "-DCMAKE_C_COMPILER=$(Join-Path $toolBin 'gcc.exe')",
    "-DCMAKE_CXX_COMPILER=$compiler",
    "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
    "-DBUILD_SHARED_LIBS=ON",
    "-DBUILD_EXAMPLES=OFF",
    "-DBUILD_TEST=OFF",
    "-DBUILD_CSHARP=OFF",
    "-DBUILD_SDK_INSTALL=OFF",
    "-DCMAKE_DISABLE_FIND_PACKAGE_SWIG=TRUE",
    "-DCMAKE_DISABLE_FIND_PACKAGE_PythonInterp=TRUE",
    "-DCMAKE_DISABLE_FIND_PACKAGE_PythonLibs=TRUE",
    "-DCMAKE_DISABLE_FIND_PACKAGE_GTest=TRUE"
)
& $cmake @configureArgs
if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed" }

Write-Host "Building ydlidar_sdk.dll..."
& $cmake --build $buildDir --target ydlidar_sdk --parallel
if ($LASTEXITCODE -ne 0) { throw "Native SDK build failed" }

foreach ($runtimeName in @("libgcc_s_seh-1.dll", "libstdc++-6.dll", "libwinpthread-1.dll")) {
    # WinLibs keeps its runtime DLLs beside g++.exe; -print-file-name only
    # searches linker libraries and returns the bare name for these files.
    $runtimePath = Join-Path $toolBin $runtimeName
    if (Test-Path $runtimePath) {
        Copy-Item -Force $runtimePath $buildDir
    } else {
        throw "Required compiler runtime not found: $runtimePath"
    }
}

$sdkDll = Join-Path $buildDir "ydlidar_sdk.dll"
if (-not (Test-Path $sdkDll)) { throw "Build completed but $sdkDll was not produced" }

Write-Host "Running offline ABI and scan simulation tests..."
Push-Location $projectRoot
try {
    & $venvPython -m unittest demo.test_x4_driver
    if ($LASTEXITCODE -ne 0) { throw "Offline tests failed" }
} finally {
    Pop-Location
}

Write-Host ""
Write-Host "Setup complete. Start offline: .\scripts\run_demo.ps1 -Simulate"
Write-Host "Start hardware:              .\scripts\run_demo.ps1 -Port COM4"
