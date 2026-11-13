# wiring_diagram.html 을 GitHub Pages용 독립 페이지(docs/index.html)로 만든다.
# 원본은 doctype/head 없이 작성되어 있으므로 앞뒤만 감싼다.
$root = Split-Path -Parent $PSScriptRoot
$src  = Join-Path $PSScriptRoot "wiring_diagram.html"
$dst  = Join-Path $root "docs\index.html"

$head = @"
<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">

"@

New-Item -ItemType Directory -Force (Split-Path $dst) | Out-Null
$body = [IO.File]::ReadAllText($src, [Text.Encoding]::UTF8)
[IO.File]::WriteAllText($dst, $head + $body + "`n</html>`n", (New-Object Text.UTF8Encoding $false))
Write-Output "built $dst"
