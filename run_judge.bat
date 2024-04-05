cd .\judge

@REM set /a seed=%RANDOM%
@REM set /a seed=21507
set seed=0
set time=0

.\SemiFinalJudge.exe -m .\maps\map1.txt ..\build\main.exe -f %time% -s %seed%

.\SemiFinalJudge.exe -m .\maps\map2.txt ..\build\main.exe -f %time% -s %seed%

.\SemiFinalJudge.exe -m .\maps\map3.txt ..\build\main.exe -f %time% -s %seed%
