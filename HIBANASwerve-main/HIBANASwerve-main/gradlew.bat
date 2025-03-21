@rem
@rem Copyright 2015 the original author or authors.
@rem
@rem Licensed under the Apache License, Version 2.0 (the "License");
@rem you may not use this file except in compliance with the License.
@rem You may obtain a copy of the License at
@rem
@rem      https://www.apache.org/licenses/LICENSE-2.0
@rem
@rem Unless required by applicable law or agreed to in writing, software
@rem distributed under the License is distributed on an "AS IS" BASIS,
@rem WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
@rem See the License for the specific language governing permissions and
@rem limitations under the License.
@rem
@rem 著作権 2015 オリジナルの著者または著者。
@rem
@rem Apache License, Version 2.0 (以下「ライセンス」) に基づいてライセンスされています。
@rem このファイルは、ライセンスに従って使用する必要があります。
@rem ライセンスのコピーは以下から入手できます。
@rem
@rem      https://www.apache.org/licenses/LICENSE-2.0
@rem
@rem 適用される法律で要求されない限り、または書面で合意されない限り、
@rem ライセンスに基づいて配布されるソフトウェアは「現状のまま」提供され、
@rem 明示または黙示を問わず、いかなる種類の保証もありません。
@rem ライセンスで規定されている権利と制限については、ライセンスを参照してください。
@rem

@if "%DEBUG%"=="" @echo off
@rem ##########################################################################
@rem
@rem  Gradle startup script for Windows
@rem
@rem ##########################################################################
@rem
@rem  Windows用Gradleスタートアップスクリプト
@rem
@rem ##########################################################################

@rem Set local scope for the variables with windows NT shell
@rem Windows NTシェルで変数のローカルスコープを設定
if "%OS%"=="Windows_NT" setlocal

set DIRNAME=%~dp0
if "%DIRNAME%"=="" set DIRNAME=.
@rem This is normally unused
@rem これは通常使用されません
set APP_BASE_NAME=%~n0
set APP_HOME=%DIRNAME%

@rem Resolve any "." and ".." in APP_HOME to make it shorter.
@rem APP_HOMEの中の"."と".."を解決して短くします。
for %%i in ("%APP_HOME%") do set APP_HOME=%%~fi

@rem Add default JVM options here. You can also use JAVA_OPTS and GRADLE_OPTS to pass JVM options to this script.
@rem ここにデフォルトのJVMオプションを追加します。このスクリプトにJVMオプションを渡すためにJAVA_OPTSとGRADLE_OPTSも使用できます。
set DEFAULT_JVM_OPTS="-Xmx64m" "-Xms64m"

@rem Find java.exe
@rem java.exeを探します
if defined JAVA_HOME goto findJavaFromJavaHome

set JAVA_EXE=java.exe
%JAVA_EXE% -version >NUL 2>&1
if %ERRORLEVEL% equ 0 goto execute

echo.
echo ERROR: JAVA_HOME is not set and no 'java' command could be found in your PATH.
echo.
echo Please set the JAVA_HOME variable in your environment to match the
echo location of your Java installation.
echo.
echo エラー: JAVA_HOMEが設定されておらず、PATHに'java'コマンドが見つかりません。
echo.
echo 環境変数JAVA_HOMEをJavaインストールの場所に設定してください。

goto fail

:findJavaFromJavaHome
set JAVA_HOME=%JAVA_HOME:"=%
set JAVA_EXE=%JAVA_HOME%/bin/java.exe

if exist "%JAVA_EXE%" goto execute

echo.
echo ERROR: JAVA_HOME is set to an invalid directory: %JAVA_HOME%
echo.
echo Please set the JAVA_HOME variable in your environment to match the
echo location of your Java installation.
echo.
echo エラー: JAVA_HOMEが無効なディレクトリに設定されています: %JAVA_HOME%
echo.
echo 環境変数JAVA_HOMEをJavaインストールの場所に設定してください。

goto fail

:execute
@rem Setup the command line
@rem コマンドラインを設定

set CLASSPATH=%APP_HOME%\gradle\wrapper\gradle-wrapper.jar

@rem Execute Gradle
@rem Gradleを実行
"%JAVA_EXE%" %DEFAULT_JVM_OPTS% %JAVA_OPTS% %GRADLE_OPTS% "-Dorg.gradle.appname=%APP_BASE_NAME%" -classpath "%CLASSPATH%" org.gradle.wrapper.GradleWrapperMain %*

:end
@rem End local scope for the variables with windows NT shell
@rem Windows NTシェルで変数のローカルスコープを終了
if %ERRORLEVEL% equ 0 goto mainEnd

:fail
rem Set variable GRADLE_EXIT_CONSOLE if you need the _script_ return code instead of
rem the _cmd.exe /c_ return code!
rem スクリプトの戻りコードが必要な場合は、変数GRADLE_EXIT_CONSOLEを設定してください。
set EXIT_CODE=%ERRORLEVEL%
if %EXIT_CODE% equ 0 set EXIT_CODE=1
if not ""=="%GRADLE_EXIT_CONSOLE%" exit %EXIT_CODE%
exit /b %EXIT_CODE%

:mainEnd
if "%OS%"=="Windows_NT" endlocal

:omega
