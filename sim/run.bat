@echo off
rem Starts the sim and opens http://localhost:8080. Ctrl+C (or close this window) to quit.
rem Gradle needs Java 17+, Android Studio ships one, so use it if it's there.
if exist "C:\Program Files\Android\Android Studio\jbr\bin\java.exe" set "JAVA_HOME=C:\Program Files\Android\Android Studio\jbr"
cd /d "%~dp0"
call ..\gradlew.bat -p . run --console=plain
pause
