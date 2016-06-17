MAP=~/projects/phd/mapf/codeHang/examples/mediummap3d.json
AGENTS=~/projects/phd/mapf/codeHang/examples/medium2_3d.agents
AGENT_KINEMATICS=~/projects/phd/mapf/kinematicConstraints/examples/agents32.json
CAMERA=0,11,-3.5,40,0,90,60

cd ../codeHang/build
make -j8
./driver -m $MAP -a $AGENTS -o test.json
cd ../../kinematicConstraints2/build
make -j8
cd ..
build/kinematicConstraints2 -a $AGENT_KINEMATICS -s ../codeHang/build/test.json --delta 0.7

cd ../visualizer3d
./visDiscrete.sh $MAP ../kinematicConstraints2/schedule.json
./vis.sh $MAP ../kinematicConstraints2/schedule.json $CAMERA
cd ../kinematicConstraints2
cp ../visualizer3d/output.mp4 .
cp ../visualizer3d/iter.zip .
