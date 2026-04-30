$domain='midominio.com'
$expected=@('elsa.ns.cloudflare.com','margo.ns.cloudflare.com')
$ns = (Resolve-DnsName -Type NS $domain -ErrorAction SilentlyContinue | Select-Object -ExpandProperty NameHost | ForEach-Object { $_.TrimEnd('.').ToLower() })
if(-not $ns){
  Write-Output "NS_CHECK: sin respuesta"
  exit 2
}
$nsSorted = $ns | Sort-Object
$expSorted = $expected | Sort-Object
$ok = (@($nsSorted) -join ',') -eq (@($expSorted) -join ',')
Write-Output ("NS_CHECK: " + ($nsSorted -join ', '))
if($ok){
  Write-Output "NS_STATUS: OK_CLOUDFLARE"
  exit 0
}
Write-Output "NS_STATUS: PENDING_OR_OLD"
exit 1
