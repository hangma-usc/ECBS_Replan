cd ../discreteHolonomic/build
make -j8
./discreteHolonomic -m ../kiva_0.map -a ../kiva100_1.agents -w ../kiva_0_1.hwy -g 1.5 -f 1.5 -o test.json
cd ../../kinematicConstraints2/build
make -j8
cd ..
build/kinematicConstraints2 -a ../kinematicConstraints/examples/agents100.json -s ../discreteHolonomic/build/test.json --delta 0.5
# dot -Kneato -Tsvg -o stp.svg stp.dot

cd ../visualizer
./vis.sh ../discreteHolonomic/kiva_0.map ../kinematicConstraints2/schedule.json 0.176 30
cd ../kinematicConstraints2
cp ../visualizer/output.mp4 .
