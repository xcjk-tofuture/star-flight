param(
    [string]$Port = 'COM7',
    [int]$Baud = 115200,
    [switch]$ShowTelemetry,
    [string]$ReplayFile
)

$ErrorActionPreference = 'Stop'
$script:Pending = New-Object 'System.Collections.Generic.List[byte]'
$script:LogBytes = New-Object 'System.Collections.Generic.List[byte]'
$script:TelemetryCount = 0
$script:LogFrameCount = 0
$script:CrcErrors = 0
$script:LastLogSequence = $null
$script:ExpectedLogSequence = $null
$script:DiscardLogTail = $false
$script:LogSequenceGaps = 0
$script:LogDuplicates = 0
$script:Clock = [System.Diagnostics.Stopwatch]::StartNew()
$script:LastTelemetryMs = -1000

function Read-U16([byte[]]$Bytes, [int]$Offset) {
    return (([int]$Bytes[$Offset]) -bor (([int]$Bytes[$Offset + 1]) -shl 8))
}

function Get-FrameCrc([byte[]]$Bytes, [int]$Offset, [int]$Count) {
    $crc = 0xffff
    for ($i = $Offset; $i -lt $Offset + $Count; $i++) {
        $crc = $crc -bxor (([int]$Bytes[$i]) -shl 8)
        for ($bit = 0; $bit -lt 8; $bit++) {
            if ($crc -band 0x8000) {
                $crc = (($crc -shl 1) -bxor 0x1021) -band 0xffff
            } else {
                $crc = ($crc -shl 1) -band 0xffff
            }
        }
    }
    return $crc
}

function Show-LogLines {
    while ($script:LogBytes.Contains([byte]10)) {
        $end = $script:LogBytes.IndexOf([byte]10) + 1
        $lineBytes = $script:LogBytes.GetRange(0, $end).ToArray()
        $script:LogBytes.RemoveRange(0, $end)
        $line = [System.Text.Encoding]::UTF8.GetString($lineBytes).TrimEnd([char[]]"`r`n")
        if ($line.Length) {
            $stamp = [DateTimeOffset]::Now.ToString('yyyy-MM-dd HH:mm:ss.fff zzz')
            Write-Host ("$stamp [LOG] $line") -ForegroundColor Yellow
        }
    }
    # Bound host memory if a producer sends a very long line without a newline.
    if ($script:LogBytes.Count -gt 4096) {
        $stamp = [DateTimeOffset]::Now.ToString('yyyy-MM-dd HH:mm:ss.fff zzz')
        Write-Host ("$stamp [LOG] " + [System.Text.Encoding]::UTF8.GetString($script:LogBytes.ToArray()))
        $script:LogBytes.Clear()
    }
}

function Handle-Frame([byte[]]$Frame) {
    $command = Read-U16 $Frame 6
    $length = Read-U16 $Frame 8
    $sequence = Read-U16 $Frame 4
    if ($command -eq 0x20f0 -and $Frame[3] -eq 2) {
        $script:LogFrameCount++
        # Retried frames keep their sequence. Ignore a duplicate before joining text.
        if ($sequence -ne 0 -and $null -ne $script:LastLogSequence -and
            $sequence -eq $script:LastLogSequence) {
            $script:LogDuplicates++
            return
        }
        if ($null -ne $script:ExpectedLogSequence -and $sequence -ne $script:ExpectedLogSequence) {
            $script:LogBytes.Clear()
            if ($sequence -eq 0) {
                # A new boot begins at sequence zero and carries a fresh text prefix.
                $script:DiscardLogTail = $false
            } else {
                $script:LogSequenceGaps++
                $script:DiscardLogTail = $true
                $stamp = [DateTimeOffset]::Now.ToString('yyyy-MM-dd HH:mm:ss.fff zzz')
                Write-Host ("$stamp [WARN] LOG sequence gap: expected=$($script:ExpectedLogSequence) received=$sequence; incomplete line discarded") -ForegroundColor Red
            }
        }
        $script:LastLogSequence = $sequence
        $script:ExpectedLogSequence = ($sequence + 1) -band 0xffff
        if ($length) {
            [byte[]]$payload = $Frame[10..(9 + $length)]
            if ($script:DiscardLogTail) {
                $newline = [Array]::IndexOf($payload, [byte]10)
                if ($newline -lt 0) { return }
                $script:DiscardLogTail = $false
                if ($newline + 1 -ge $payload.Length) { return }
                $payload = [byte[]]$payload[($newline + 1)..($payload.Length - 1)]
            }
            $script:LogBytes.AddRange($payload)
            Show-LogLines
        }
    } elseif (($command -eq 4 -or $command -eq 0x2000) -and $length -eq 14) {
        $script:TelemetryCount++
        if ($ShowTelemetry -and $script:Clock.ElapsedMilliseconds - $script:LastTelemetryMs -ge 1000) {
            $roll = [System.BitConverter]::ToSingle($Frame, 12)
            $pitch = [System.BitConverter]::ToSingle($Frame, 16)
            $yaw = [System.BitConverter]::ToSingle($Frame, 20)
            $stamp = [DateTimeOffset]::Now.ToString('yyyy-MM-dd HH:mm:ss.fff zzz')
            Write-Host ('{0} [TEL] seq={1} state={2} roll={3:F3} pitch={4:F3} yaw={5:F3} rad' -f
                $stamp, $sequence, $Frame[11], $roll, $pitch, $yaw) -ForegroundColor Cyan
            $script:LastTelemetryMs = $script:Clock.ElapsedMilliseconds
        }
    }
}

function Feed-Bytes([byte[]]$Bytes) {
    $script:Pending.AddRange($Bytes)
    while ($script:Pending.Count -ge 2) {
        if ($script:Pending[0] -ne 0xa5 -or $script:Pending[1] -ne 0x5a) {
            $script:Pending.RemoveAt(0)
            continue
        }
        if ($script:Pending.Count -lt 10) { break }
        $length = ([int]$script:Pending[8]) -bor (([int]$script:Pending[9]) -shl 8)
        if ($script:Pending[2] -ne 1 -or $script:Pending[3] -gt 2 -or $length -gt 128) {
            $script:Pending.RemoveAt(0)
            continue
        }
        $total = 12 + $length
        if ($script:Pending.Count -lt $total) { break }
        [byte[]]$frame = $script:Pending.GetRange(0, $total).ToArray()
        if ((Get-FrameCrc $frame 2 ($total - 4)) -ne (Read-U16 $frame ($total - 2))) {
            $script:CrcErrors++
            $script:Pending.RemoveAt(0)
            continue
        }
        $script:Pending.RemoveRange(0, $total)
        Handle-Frame $frame
    }
}

$serial = $null
try {
    if ($ReplayFile) {
        [byte[]]$recording = [System.IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $ReplayFile))
        # Replay fragmented reads too, rather than requiring frame boundaries.
        for ($offset = 0; $offset -lt $recording.Length; $offset += 17) {
            $last = [Math]::Min($offset + 16, $recording.Length - 1)
            Feed-Bytes ([byte[]]$recording[$offset..$last])
        }
    } else {
        $serial = New-Object System.IO.Ports.SerialPort
        $serial.PortName = $Port
        $serial.BaudRate = $Baud
        $serial.DataBits = 8
        $serial.Parity = [System.IO.Ports.Parity]::None
        $serial.StopBits = [System.IO.Ports.StopBits]::One
        $serial.DtrEnable = $false
        $serial.RtsEnable = $false
        $serial.Open()
        Write-Host ("Connected to $Port at $Baud 8N1. Ctrl+C to stop.")
        while ($true) {
            $count = $serial.BytesToRead
            if ($count) {
                $buffer = New-Object byte[] $count
                $received = $serial.Read($buffer, 0, $count)
                if ($received) { Feed-Bytes ([byte[]]$buffer[0..($received - 1)]) }
            } else {
                Start-Sleep -Milliseconds 10
            }
        }
    }
} finally {
    if ($serial) {
        if ($serial.IsOpen) { $serial.Close() }
        $serial.Dispose()
    }
    if ($script:LogBytes.Count) {
        $stamp = [DateTimeOffset]::Now.ToString('yyyy-MM-dd HH:mm:ss.fff zzz')
        Write-Host ($stamp + ' [LOG partial] ' + [System.Text.Encoding]::UTF8.GetString($script:LogBytes.ToArray()))
    }
    Write-Host ('Frames: telemetry={0} log_frames={1} crc_errors={2} log_gaps={3} log_duplicates={4}' -f
        $script:TelemetryCount, $script:LogFrameCount, $script:CrcErrors,
        $script:LogSequenceGaps, $script:LogDuplicates)
}
