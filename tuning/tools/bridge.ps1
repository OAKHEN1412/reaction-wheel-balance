# Serial bridge for hardware tuning sessions.
#
# Keeps ONE long-lived connection to the board open, because re-opening the
# port resets the Arduino (DTR) and wipes the run you are in the middle of.
# Everything the board prints is appended to a log file; commands are taken
# from a file so another process (an agent, a script, you with a text editor)
# can drive the board without touching the port.
#
#   .\bridge.ps1 -Port COM8 -Log run.log -Cmd cmd.txt
#
# To send a command:  Set-Content cmd.txt "jump 550 450 8 55"
# Each write to the command file is sent once; the file is emptied after.
#
# ALWAYS switch the 12V supply OFF before flashing or resetting the board:
# PWM is inverted, so a floating PWM pin runs the motor at full speed.

param(
  [string]$Port = "COM8",
  [int]$Baud = 115200,
  [string]$Log = "bridge.log",
  [string]$Cmd = "bridge-cmd.txt"
)

$ErrorActionPreference = "Stop"

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$sp.ReadTimeout = 200
$sp.WriteTimeout = 1000
$sp.NewLine = "`n"
# Leave DTR/RTS alone: asserting DTR pulses reset on CH340/FTDI boards.
$sp.DtrEnable = $false
$sp.RtsEnable = $false
$sp.Open()

if (-not (Test-Path $Cmd)) { New-Item -ItemType File -Path $Cmd | Out-Null }
"[bridge] opened $Port at $Baud, log=$Log, cmd=$Cmd" | Tee-Object -FilePath $Log -Append

$lastCmdWrite = (Get-Item $Cmd).LastWriteTimeUtc

try {
  while ($true) {
    # Drain whatever the board has said since the last pass.
    try {
      $chunk = $sp.ReadExisting()
      if ($chunk) { [System.IO.File]::AppendAllText($Log, $chunk) }
    } catch [TimeoutException] { }

    # A changed command file means there is something to send.
    $stamp = (Get-Item $Cmd).LastWriteTimeUtc
    if ($stamp -ne $lastCmdWrite) {
      $lastCmdWrite = $stamp
      $lines = @(Get-Content $Cmd -ErrorAction SilentlyContinue | Where-Object { $_.Trim() -ne "" })
      foreach ($line in $lines) {
        $sp.WriteLine($line.Trim())
        [System.IO.File]::AppendAllText($Log, "[bridge] > $($line.Trim())`n")
        Start-Sleep -Milliseconds 120
      }
      if ($lines.Count -gt 0) {
        Set-Content -Path $Cmd -Value "" -Encoding ascii
        $lastCmdWrite = (Get-Item $Cmd).LastWriteTimeUtc
      }
    }

    Start-Sleep -Milliseconds 40
  }
}
finally {
  if ($sp.IsOpen) { $sp.Close() }
  "[bridge] closed $Port" | Add-Content -Path $Log
}
