#!/bin/bash
# bead.init's USB serial port from WSL. The port stays on the Windows side,
# so it's opened through Windows PowerShell.
#
#   serial.sh read [SECONDS]   print what the module sends, 0: until Ctrl-C
#   serial.sh send STRING      send commands: b, 1-4, c
#
# COM_PORT selects the port (default COM4).

# Report PowerShell's exit status, not tr's.
set -o pipefail

port=${COM_PORT:-COM4}
open="\$p = New-Object System.IO.Ports.SerialPort $port,115200
\$p.DtrEnable = \$true
try { \$p.Open() } catch { [Console]::Error.WriteLine('$port not available'); exit 1 }"

case "${1:-}" in
  read)
    seconds=${2:-0}
    powershell.exe -NoProfile -Command "$open
      \$end = if ($seconds -gt 0) { (Get-Date).AddSeconds($seconds) } else { [DateTime]::MaxValue }
      while ((Get-Date) -lt \$end) {
        \$s = \$p.ReadExisting()
        if (\$s) { [Console]::Out.Write(\$s); [Console]::Out.Flush() }
        Start-Sleep -Milliseconds 100
      }
      \$p.Close()" | tr -d '\r'
    ;;
  send)
    # The module may reset right away ('b'), so a failing Close() is fine.
    powershell.exe -NoProfile -Command "$open
      \$p.Write('${2:-}')
      Start-Sleep -Milliseconds 300
      try { \$p.Close() } catch {}" | tr -d '\r'
    ;;
  *)
    echo "usage: $0 read [SECONDS] | send STRING" >&2
    exit 2
    ;;
esac
