$ErrorActionPreference = 'Stop'
$compiler = Get-Command gcc -ErrorAction SilentlyContinue
if (-not $compiler) {
    $compiler = Get-Command clang -ErrorAction SilentlyContinue
}
if (-not $compiler) {
    throw 'A host gcc or clang compiler is required to run the Yaw LQR tests.'
}

$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$exe = Join-Path $env:TEMP "yaw_lqr_controller_test_$PID.exe"
try {
    & $compiler.Source -std=c17 -Wall -Wextra -Werror `
        -I (Join-Path $root 'application\gimbal') `
        (Join-Path $root 'application\gimbal\yaw_lqr_eso_controller.c') `
        (Join-Path $PSScriptRoot 'yaw_lqr_controller_test.c') `
        -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw 'Yaw LQR test compilation failed.' }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw 'Yaw LQR tests failed.' }
} finally {
    Remove-Item -LiteralPath $exe -ErrorAction SilentlyContinue
}
