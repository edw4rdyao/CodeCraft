cd .\judge

@REM set /a seed=%RANDOM%
@REM set /a seed=21507
set seed=0
set time=0

@REM .\SemiFinalJudge.exe -m .\maps\map1.txt ..\build\main.exe -f %time% -s %seed% -d output_map1.txt

@REM .\SemiFinalJudge.exe -m .\maps\map2.txt ..\build\main.exe -f %time% -s %seed% -d output_map2.txt

@REM .\SemiFinalJudge.exe -m .\maps\map3.txt ..\build\main.exe -f %time% -s %seed% -d output_map3.txt

.\SemiFinalJudge.exe -m .\maps\map5.txt ..\build\main.exe -f %time% -s %seed% -d output_map2.txt

.\SemiFinalJudge.exe -m .\maps\map6.txt ..\build\main.exe -f %time% -s %seed% -d output_map3.txt
