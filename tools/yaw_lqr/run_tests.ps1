$ErrorActionPreference = 'Stop'
$compiler = Get-Command gcc -ErrorAction SilentlyContinue
if (-not $compiler) {
    $compiler = Get-Command clang -ErrorAction SilentlyContinue
}
if (-not $compiler) {
    throw 'A host gcc or clang compiler is required to run the Yaw LQR tests.'
}

$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$controllerExe = Join-Path $env:TEMP "yaw_lqr_controller_test_$PID.exe"
$identificationExe = Join-Path $env:TEMP "yaw_identification_test_test_$PID.exe"
try {
    & $compiler.Source -std=c17 -Wall -Wextra -Werror `
        -I (Join-Path $root 'application\gimbal') `
        (Join-Path $root 'application\gimbal\yaw_lqr_eso_controller.c') `
        (Join-Path $PSScriptRoot 'yaw_lqr_controller_test.c') `
        -lm -o $controllerExe
    if ($LASTEXITCODE -ne 0) { throw 'Yaw LQR test compilation failed.' }
    & $controllerExe
    if ($LASTEXITCODE -ne 0) { throw 'Yaw LQR tests failed.' }

    & $compiler.Source -std=c17 -Wall -Wextra -Werror `
        -I (Join-Path $root 'application\test') `
        (Join-Path $root 'application\test\yaw_identification_test.c') `
        (Join-Path $PSScriptRoot 'yaw_identification_test_test.c') `
        -lm -o $identificationExe
    if ($LASTEXITCODE -ne 0) { throw 'Yaw identification test compilation failed.' }
    & $identificationExe
    if ($LASTEXITCODE -ne 0) { throw 'Yaw identification tests failed.' }
} finally {
    Remove-Item -LiteralPath $controllerExe -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $identificationExe -ErrorAction SilentlyContinue
}
