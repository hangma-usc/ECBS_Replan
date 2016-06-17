cd ../discreteHolonomic/build
make -j8
./discreteHolonomic -m ../corridor2.map -a ../corridor2.agents -w ../corridor2.hwy -g 1.5 -f 3.0 -o test.json
cd ../../kinematicConstraints/build
make -j8
cd ..
build/kinematicConstraints -a examples/agents100.json -s ../discreteHolonomic/build/test.json --delta 0.4 -o schedule.json

cd ../visualizer
./vis.sh ../discreteHolonomic/example1.map ../kinematicConstraints/schedule.json 0.4 30
cd ../kinematicConstraints
cp ../visualizer/output.mp4 .
