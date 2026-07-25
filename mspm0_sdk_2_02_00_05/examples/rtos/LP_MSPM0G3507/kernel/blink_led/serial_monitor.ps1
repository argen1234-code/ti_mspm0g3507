$port = New-Object System.IO.Ports.SerialPort COM17, 115200, None, 8, One
$port.Open()
$sw = New-Object System.IO.StreamWriter "$PSScriptRoot\serial_log.txt", $true
$sw.AutoFlush = $true
$end = (Get-Date).AddSeconds(60)
while ((Get-Date) -lt $end) {
    try {
        $line = $port.ReadLine()
        $sw.WriteLine($line)
    } catch {}
}
$port.Close()
$sw.Close()
