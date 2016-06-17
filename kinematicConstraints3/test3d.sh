MAP=~/projects/phd/mapf/codeHang/examples/mediummap3d.json
AGENTS=~/projects/phd/mapf/codeHang/examples/medium2_3d.agents
AGENT_KINEMATICS=~/projects/phd/mapf/kinematicConstraints/examples/agents32.json
CAMERA=0,11,-3.5,40,0,90,60

cd ../codeHang/build
make -j8
./driver -m $MAP -a $AGENTS -o test.json
cd ../../kinematicConstraints/build
make -j8
cd ..
build/kinematicConstraints -a $AGENT_KINEMATICS -s ../codeHang/build/test.json --delta 0.45

cd ../visualizer3d
./visDiscrete.sh $MAP ../kinematicConstraints/schedule.json
./vis.sh $MAP ../kinematicConstraints/schedule.json $CAMERA
cd ../kinematicConstraints
cp ../visualizer3d/output.mp4 .
cp ../visualizer3d/iter.zip .
