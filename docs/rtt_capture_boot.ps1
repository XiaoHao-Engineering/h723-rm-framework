# 连上 RTT -> 通过 OpenOCD telnet 复位 -> 从头抓启动日志
$ErrorActionPreference = "Stop"

$rtt = New-Object System.Net.Sockets.TcpClient
$rtt.Connect("127.0.0.1", 9090)
$rs = $rtt.GetStream()
$rs.ReadTimeout = 500

Start-Sleep -Milliseconds 300

$tel = New-Object System.Net.Sockets.TcpClient
$tel.Connect("127.0.0.1", 4444)
$ts = $tel.GetStream()
$cmd = [System.Text.Encoding]::ASCII.GetBytes("reset run`n")
$ts.Write($cmd, 0, $cmd.Length)
$ts.Flush()
Start-Sleep -Milliseconds 400
$tel.Close()

$buf = New-Object byte[] 65536
$sb = New-Object System.Text.StringBuilder
$deadline = (Get-Date).AddSeconds(3)
while ((Get-Date) -lt $deadline) {
    try {
        $n = $rs.Read($buf, 0, $buf.Length)
        if ($n -le 0) { break }
        [void]$sb.Append([System.Text.Encoding]::UTF8.GetString($buf, 0, $n))
    } catch { }
}
$rtt.Close()

$txt = $sb.ToString() -replace "`e\[[0-9;]*m", ""
$out = Join-Path $PSScriptRoot "_rtt_boot.log"
$txt | Out-File $out -Encoding utf8
"bytes=$($txt.Length) -> $out"
