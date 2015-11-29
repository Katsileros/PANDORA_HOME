Training:
	./create_Features

Testing:
	./test_nearest_neighbors -k 4 -thresh 50 test/0_PFH.pcd
	
png2pcd:
	./png2pcd -format 0 -mode DEFAULT --intensity_type FLOAT driller/data/color0.jpg driller/depth/depth0.png ./testingPointCloud0.pcd
