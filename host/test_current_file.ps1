param(
    [Parameter(Mandatory = $true)]
    [string]$InputFile,
    [Parameter(Mandatory = $true)]
    [ValidateSet('Normal run', 'Host diameter sweep', 'MCU diameter sweep')]
    [string]$Mode,
    [Parameter(Mandatory = $true)]
    [string]$RunnerPath
)

$ErrorActionPreference = 'Stop'
if (!(Test-Path -LiteralPath $InputFile -PathType Leaf)) {
    throw "Input file does not exist: $InputFile"
}
$inputPath = (Resolve-Path -LiteralPath $InputFile).Path
$extension = [System.IO.Path]::GetExtension($inputPath)
if ($extension -notin @('.nc', '.ngc')) {
    throw 'Open an .nc or .ngc file in the editor before running this task.'
}
if (!(Test-Path -LiteralPath $RunnerPath -PathType Leaf)) {
    throw "Runner not found: $RunnerPath. Build cc_runner first."
}
$workspace = Split-Path -Parent $PSScriptRoot
$outputFolder = Join-Path $workspace ('output\single-file\' + [System.IO.Path]::GetFileName($inputPath))
New-Item -ItemType Directory -Path $outputFolder -Force | Out-Null
Write-Host "Testing: $inputPath"
Write-Host "Mode: $Mode"
Write-Host "Output: $outputFolder"
if ($Mode -eq 'Normal run') {
    & $RunnerPath $inputPath $outputFolder
} else {
    $engine = 'host'
    if ($Mode -eq 'MCU diameter sweep') {
        $engine = 'mcu'
    }
    & $RunnerPath --sweep $engine $inputPath $outputFolder
}
$result = $LASTEXITCODE
if ($result -ne 0) {
    Write-Error "Test runner failed with exit code $result. Review the diagnostics above."
}
exit $result
