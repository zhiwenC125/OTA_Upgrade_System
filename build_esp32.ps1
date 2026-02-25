$env:IDF_PATH = "E:\esp\v5.5.2\esp-idf"
$oldPath = $env:PATH
$env:PATH = "E:\esp\Espressif\python_env\idf5.5_py3.11_env\Scripts;E:\esp\Espressif\tools\idf-git\2.44.0\cmd;" + $oldPath

Write-Host "IDF_PATH: $env:IDF_PATH"
Write-Host "Python: $(& 'E:\esp\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe' --version)"

Set-Location "E:\IoT2\ESP_Firmware"
Write-Host "CWD: $(Get-Location)"

$process = Start-Process -FilePath "E:\esp\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe" `
    -ArgumentList "E:\esp\v5.5.2\esp-idf\tools\idf.py", "build" `
    -WorkingDirectory "E:\IoT2\ESP_Firmware" `
    -NoNewWindow -PassThru -Wait

Write-Host "Exit code: $($process.ExitCode)"
