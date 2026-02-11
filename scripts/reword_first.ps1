$path = $args[0]
(Get-Content $path) -replace '^pick ', 'reword ' | Set-Content $path
