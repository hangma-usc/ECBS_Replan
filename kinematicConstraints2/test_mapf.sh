cd ../discreteHolonomic/build
make -j8
./discreteHolonomic -m ../example1.map -a ../example1.agents -w ../example1.hwy -o test.json
cd ../../kinematicConstraints2/build
make -j8
cd ..
build/kinematicConstraints2 -a examples/agents2.json -s ../discreteHolonomic/build/test.json --delta 0.25
dot -Kneato -Tsvg -o stp.svg stp.dot

cd ../visualizer
./vis.sh ../discreteHolonomic/example1.map ../kinematicConstraints2/schedule.json 0.4 30
cd ../kinematicConstraints2
cp ../visualizer/output.mp4 .
