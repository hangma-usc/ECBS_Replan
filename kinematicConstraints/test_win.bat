C:\ECBS_Master\x64\Release\ECBS.exe -m C:\ECBS_Master\discreteHolonomic\example1.map -a C:\ECBS_Master\discreteHolonomic\example1.agents -w C:\ECBS_Master\discreteHolonomic\example1.hwy -o test.json
C:\ECBS_Master\x64\Release\stp.exe -a examples\agents2.json -s test.json --delta 0.4 -o schedule.json
dot stp.dot -Tpng -o stp.png