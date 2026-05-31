param(
    [Parameter(Mandatory = $true)]
    [string]$Source,
    [Parameter(Mandatory = $true)]
    [string]$Target,
    [switch]$Once
)

function Get-Hash([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path)) {
        return $null
    }
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash
}

function Sync-Settings {
    Copy-Item -LiteralPath $Source -Destination $Target -Force
    Write-Host ("Synced to: {0}" -f $Target)
}

$sourceHash = Get-Hash $Source
if (-not $sourceHash) {
    Write-Error ("Source not found: {0}" -f $Source)
    exit 1
}

Sync-Settings
if ($Once) {
    exit 0
}

$targetDir = Split-Path -Path $Target -Parent
$targetFile = Split-Path -Path $Target -Leaf
if (-not (Test-Path -LiteralPath $targetDir)) {
    New-Item -ItemType Directory -Path $targetDir | Out-Null
}

$watcher = New-Object System.IO.FileSystemWatcher
$watcher.Path = $targetDir
$watcher.Filter = $targetFile
$watcher.NotifyFilter = [System.IO.NotifyFilters]::LastWrite -bor [System.IO.NotifyFilters]::Size -bor [System.IO.NotifyFilters]::FileName
$watcher.EnableRaisingEvents = $true

Register-ObjectEvent -InputObject $watcher -EventName Changed -Action {
    Start-Sleep -Milliseconds 200
    $srcHash = Get-Hash $using:Source
    $dstHash = Get-Hash $using:Target
    if ($srcHash -and $dstHash -and ($srcHash -ne $dstHash)) {
        Copy-Item -LiteralPath $using:Source -Destination $using:Target -Force
        Write-Host ("Re-synced at {0}" -f (Get-Date))
    }
} | Out-Null

Register-ObjectEvent -InputObject $watcher -EventName Created -Action {
    Start-Sleep -Milliseconds 200
    Copy-Item -LiteralPath $using:Source -Destination $using:Target -Force
    Write-Host ("Re-synced at {0}" -f (Get-Date))
} | Out-Null

while ($true) {
    Start-Sleep -Seconds 1
}
