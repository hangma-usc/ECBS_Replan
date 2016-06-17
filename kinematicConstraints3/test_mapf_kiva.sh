cd ../discreteHolonomic/build
make -j8
./discreteHolonomic -m ../kiva_0.map -a ../kiva100_1.agents -w ../kiva_0_1.hwy -g 1.5 -f 1.5 -o test.json
cd ../../kinematicConstraints/build
make -j8
cd ..
build/kinematicConstraints -a examples/agents100.json -s ../discreteHolonomic/build/test.json --delta 0.4

cd ../visualizer
./vis.sh ../discreteHolonomic/example1.map ../kinematicConstraints/schedule.json 0.4 30
cd ../kinematicConstraints
cp ../visualizer/output.mp4 .
