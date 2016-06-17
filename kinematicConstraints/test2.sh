cd ../codeHang/build
make -j8
./driver -m ../examples/map-20x20 -a ../examples/test.agents -o test.json
cd ../../kinematicConstraints/build
make -j8
cd ..
build/kinematicConstraints -a examples/agents32.json -s ../codeHang/build/test.json --delta 0.4

cd ../visualizer
./visDiscrete.sh ../codeHang/examples/map-20x20 ../codeHang/build/test.json
./vis.sh ../codeHang/examples/map-20x20 ../kinematicConstraints/schedule.json 0.4 30
cd ../kinematicConstraints
cp ../visualizer/output.mp4 .
cp ../visualizer/iter.zip .


