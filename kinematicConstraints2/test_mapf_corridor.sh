cd ../discreteHolonomic/build
make -j8
./discreteHolonomic -m ../corridor2.map -a ../corridor2.agents -w ../corridor2.hwy -g 1.5 -f 3.0 -o test.json
cd ../../kinematicConstraints2/build
make -j8
cd ..
build/kinematicConstraints2 -a ../kinematicConstraints/examples/agents100.json -s ../discreteHolonomic/build/test.json --delta 0.1
# dot -Kneato -Tsvg -o stp.svg stp.dot

cd ../visualizer
./vis.sh ../discreteHolonomic/corridor2.map ../kinematicConstraints2/schedule.json 0.176 30
cd ../kinematicConstraints2
cp ../visualizer/output.mp4 .
