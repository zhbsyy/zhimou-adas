# Offline debug build using the SDK tools already installed on computer A.
# Android Studio/Gradle can also build this project with the installed API 33.
$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$sdk = 'D:\Android_Studio_Rabbit\SDK_box'
$tools = Join-Path $sdk 'build-tools\36.0.0'
$androidJar = Join-Path $sdk 'platforms\android-33\android.jar'
$jdk = 'D:\Android Studio\jbr\bin'
$env:JAVA_HOME = 'D:\Android Studio\jbr'
$work = Join-Path $project '.build'
$dist = Join-Path $project 'dist'
New-Item -ItemType Directory -Path $work,$dist -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $work 'classes'),(Join-Path $work 'dex') -Force | Out-Null

$manifest = Get-Content (Join-Path $project 'app\src\main\AndroidManifest.xml') -Raw -Encoding UTF8
$manifest = $manifest.Replace('<manifest xmlns:android="http://schemas.android.com/apk/res/android">',
    '<manifest xmlns:android="http://schemas.android.com/apk/res/android" package="cn.embedded.amapgateway">')
$tempManifest = Join-Path $work 'AndroidManifest.xml'
Set-Content -LiteralPath $tempManifest -Value $manifest -Encoding UTF8

$sources = Get-ChildItem -LiteralPath (Join-Path $project 'app\src\main\java') -Filter '*.java' -Recurse | ForEach-Object FullName
& (Join-Path $jdk 'javac.exe') --release 8 -encoding UTF-8 -classpath $androidJar -d (Join-Path $work 'classes') $sources
if ($LASTEXITCODE -ne 0) { throw 'javac failed' }
$classJar = Join-Path $work 'classes.jar'
Remove-Item -LiteralPath $classJar -Force -ErrorAction SilentlyContinue
Add-Type -AssemblyName System.IO.Compression.FileSystem
[System.IO.Compression.ZipFile]::CreateFromDirectory((Join-Path $work 'classes'), $classJar)
& (Join-Path $tools 'd8.bat') --min-api 23 --lib $androidJar --output (Join-Path $work 'dex') $classJar
if ($LASTEXITCODE -ne 0) { throw 'd8 failed' }

$base = Join-Path $work 'base.apk'
Remove-Item -LiteralPath $base -Force -ErrorAction SilentlyContinue
& (Join-Path $tools 'aapt2.exe') link -o $base --manifest $tempManifest -I $androidJar `
    -A (Join-Path $project 'app\src\main\assets') --min-sdk-version 23 --target-sdk-version 28 `
    --version-code 2 --version-name '1.1' --debug-mode
if ($LASTEXITCODE -ne 0) { throw 'aapt2 failed' }

$archive = [System.IO.Compression.ZipFile]::Open($base, [System.IO.Compression.ZipArchiveMode]::Update)
try {
    [System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile($archive,
        (Join-Path $work 'dex\classes.dex'), 'classes.dex') | Out-Null
} finally { $archive.Dispose() }

$aligned = Join-Path $work 'aligned.apk'
Remove-Item -LiteralPath $aligned -Force -ErrorAction SilentlyContinue
& (Join-Path $tools 'zipalign.exe') -p -f 4 $base $aligned
if ($LASTEXITCODE -ne 0) { throw 'zipalign failed' }

$keystore = Join-Path $work 'debug.keystore'
if (-not (Test-Path -LiteralPath $keystore)) {
    & (Join-Path $jdk 'keytool.exe') -genkeypair -alias androiddebugkey -keyalg RSA -keysize 2048 `
        -validity 10000 -keystore $keystore -storepass android -keypass android `
        -dname 'CN=Android Debug,O=Android,C=US' -noprompt
    if ($LASTEXITCODE -ne 0) { throw 'keytool failed' }
}
$apk = Join-Path $dist 'Cockpit_Android_Emulator_debug.apk'
& (Join-Path $tools 'apksigner.bat') sign --ks $keystore --ks-pass pass:android `
    --key-pass pass:android --out $apk $aligned
if ($LASTEXITCODE -ne 0) { throw 'apksigner failed' }
& (Join-Path $tools 'apksigner.bat') verify --verbose $apk
if ($LASTEXITCODE -ne 0) { throw 'APK verification failed' }
Write-Host "APK ready: $apk"
