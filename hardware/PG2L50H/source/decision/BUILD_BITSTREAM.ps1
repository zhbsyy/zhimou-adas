$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$pds = 'C:/pango/PDS_2022.2-SP6.4/bin/pds_shell.exe'
if (-not (Test-Path -LiteralPath $pds)) { throw 'Install PDS 2022.2-SP6.4 or update this tool path.' }
$job = Start-Process -FilePath $pds -ArgumentList @('-project','project.pds','-run','gen_bit_stream') -WindowStyle Hidden -PassThru -RedirectStandardOutput 'build_stdout.log' -RedirectStandardError 'build_stderr.log'
$job.WaitForExit()
if ($job.ExitCode -ne 0) { throw "PDS failed: $($job.ExitCode)" }
Write-Host 'Review generated bitstream/timing. Frozen PROGRAM_THIS_BITSTREAM artifact was not overwritten.'
