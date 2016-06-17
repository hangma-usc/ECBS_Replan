cd ../discreteHolonomic/build
make -j8
./discreteHolonomic -m ../example1.map -a ../example1.agents -w ../example1.hwy -o test.json
cd ../../kinematicConstraints/build
make -j8
cd ..
build/kinematicConstraints -a examples/agents2.json -s ../discreteHolonomic/build/test.json --delta 0.4

cd ../visualizer
./vis.sh ../discreteHolonomic/example1.map ../kinematicConstraints/schedule.json 0.4 30
cd ../kinematicConstraints
cp ../visualizer/output.mp4 .
