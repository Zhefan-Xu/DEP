#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <DEP/env.h>
#include <iostream>
#include <fstream>

using namespace octomap;

using std::cout; using std::endl; using std::cerr;


AbstractOcTree* abtree;
OcTree* tree_ptr;
int tree_size;


void callback(const octomap_msgs::Octomap::ConstPtr& bmap){
	abtree = octomap_msgs::binaryMsgToMap(*bmap);
	tree_ptr = dynamic_cast<OcTree*>(abtree);
	tree_ptr->setResolution(RES);
	tree_size = tree_ptr->size();
	delete abtree;
}


void print_tree_size(){
	// ros::Rate loop_rate(0.25);
	ros::Rate loop_rate(0.10);
	std::ofstream data;
	data.open("/home/zhefan/Desktop/Experiment_Data/apartment/DEP/apartment_data6.txt");
	while (ros::ok()){
		cout << "tree size is: " << tree_size <<endl;
		data << tree_size << "\n";
		ros::spinOnce();
		loop_rate.sleep();
	}

}


int main(int argc, char** argv){
	ros::init(argc, argv, "tree_size_monitor");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("octomap_binary", 100, callback);
	// ros::spin();
	print_tree_size();
	return 0;
}