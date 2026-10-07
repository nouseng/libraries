<#
.SYNOPSIS
Remove OpenModelica build/simulation artifacts from the repo root and build/.

Dry-run by default: lists what would go and the total size.
Pass -Delete to actually remove the enumerated files.

Never removes: git-tracked files, anything under a dot-directory
(.vscode, .git, ...), anything under build/*-fmu (exported FMUs) or
build/notebook-* (pip-installed Python envs), simulation results, metadata,
or authored scripts and models. Only ignored files with allowlisted compiler
extensions are candidates. Inaccessible directories are left alone.
#>
param([switch]$Delete)

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path

$exts = '.bat', '.bin', '.c', '.h', '.o', '.obj', '.exe', '.libs', '.makefile', '.log'

$ignored = @(& git -C $root ls-files --others --ignored --exclude-standard 2>$null)
if ($LASTEXITCODE -ne 0) { throw 'Cannot enumerate ignored files' }

$checkedDirectories = @{}
$victims = @(foreach ($relativePath in $ignored) {
    if (($relativePath.Contains('/') -and $relativePath -notlike 'build/*') -or
        $relativePath -match '(^|/)\.' -or
        $relativePath -like 'build/*-fmu/*' -or
        $relativePath -like 'build/notebook-*' -or
        [IO.Path]::GetExtension($relativePath).ToLowerInvariant() -notin $exts) {
        continue
    }
    $path = [IO.Path]::GetFullPath((Join-Path $root $relativePath))
    if (-not $path.StartsWith($root + [IO.Path]::DirectorySeparatorChar,
        [StringComparison]::OrdinalIgnoreCase)) { throw "Path outside repository: $path" }
    $file = Get-Item -LiteralPath $path -Force
    if ($file.Attributes -band [IO.FileAttributes]::ReparsePoint) { continue }
    $directory = $file.Directory
    $linked = $false
    while ($directory.FullName -ne $root) {
        if (-not $checkedDirectories.ContainsKey($directory.FullName)) {
            $checkedDirectories[$directory.FullName] = [bool](
                $directory.Attributes -band [IO.FileAttributes]::ReparsePoint)
        }
        if ($checkedDirectories[$directory.FullName]) { $linked = $true; break }
        $directory = $directory.Parent
    }
    if (-not $linked) { $file }
})

if ($victims.Count -eq 0) {
    'No compiler artifacts to remove.'
    return
}

$sum = $victims | Measure-Object Length -Sum
$victims | Group-Object Extension | Sort-Object Count -Descending |
    Select-Object Count, Name | Format-Table -AutoSize
"{0} files, {1:N1} MB" -f $sum.Count, ($sum.Sum / 1MB)

if ($Delete) {
    foreach ($file in $victims) { Remove-Item -LiteralPath $file.FullName -Force }
    "Deleted $($victims.Count) compiler artifacts."
} else {
    "Dry run. Re-run with -Delete to remove."
}
