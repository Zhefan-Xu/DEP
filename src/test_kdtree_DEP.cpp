#include <ros/ros.h>
#include <iostream>
#include <DEP/kdtree.h>
#include <DEP/utils.h>
using std::cout; using std::endl;

void testNode(){
	cout << "================Test Node===================" << endl;
	point3d p1 (1, 2, 3);
	point3d p2 (2, 3, 4);
	point3d p3 (3, 4, 5);
	Node n1 (p1);
	Node n2 (p2);
	Node n3 (p2);
	cout << "Node attributes: " << endl;
	// print_point3d(n1.p);
	// cout << "yaw: " << n1.yaw << endl;
	// cout << "info gain: " <<  n1.ig << endl;
	print_node(n1);
	cout << "============================================" << endl;
	return;
}

void test_kdtree_insert(){
	cout << "============Test KDTree Insert==============" << endl;
	point3d p1 (1, 2, 3);
	point3d p2 (2, 3, 4);
	point3d p3 (3, 4, 5);
	point3d p4 (5, 2, 1);
	point3d p5 (10, 0, 10);
	Node n1 (p1);
	Node n2 (p2);
	Node n3 (p3);
	Node n4 (p4);
	Node n5 (p5);

	//Initiaize KDTree;
	KDTree t1; 
	t1.insert(&n2);
	t1.insert(&n1);
	t1.insert(&n3);
	t1.insert(&n4);
	t1.insert(&n5);
	cout << "size of kdtree: " << t1.getSize() <<  endl;
	Node* root = t1.getRoot();
	print_node((*root));
	print_node(*(root->left));
	print_node(*(root->right));
	print_node(*(root->right->left));
	print_node(*(root->right->left->right));
	// print_node((*root));
	// print_node(*(root->left));
	// print_node(*(root->right));
	// print_node(*(root->right->left));
	cout << "============================================" << endl;
	return;
}

void test_kdtree_nn(){
	cout << "==========Test KDTree Nearest Neighbor=========" << endl;
	point3d p1 (1, 2, 3);
	point3d p2 (2, 3, 4);
	point3d p3 (3, 4, 5);
	point3d p4 (5, 2, 1);
	point3d p5 (10, 0, 10);

	Node n1 (p1);
	Node n2 (p2);
	Node n3 (p3);
	Node n4 (p4);
	Node n5 (p5);

	//Initiaize KDTree;
	KDTree t1; 
	t1.insert(&n2);
	t1.insert(&n1);
	t1.insert(&n3);
	t1.insert(&n4);
	t1.insert(&n5);
	// cout << "size of kdtree: " << t1.getSize() <<  endl;

	// Find nearest neighbor for n6
	point3d p6 (9, 0, 9); // nearest neighbor should be (10, 0, 10)
	Node n6 (p6);
	// Node* nn1 = t1.nearestNeighbor(&n6);
	// print_node(*nn1);

	// Find nearest neighbor for n7
	point3d p7 (2.1, 2, 3); // nearest neighbor should be (1, 2, 3)
	Node n7 (p7);
	// Node* nn2 = t1.nearestNeighbor(&n7);
	// print_node(*nn2);

	// Find nearest neighbor for n9
	point3d p8 (6, 3, 4);
	point3d pa (6, 1, 0);
	point3d pb (3, 3, 0);
	point3d pc (9, 3, 0);
	point3d p9 (9, 3, 2); // nearest neighbor should be (9, 3, 0)
	Node n8 (p8);
	Node na (pa);
	Node nb (pb);
	Node nc (pc);
	Node n9 (p9);
	t1.insert(&na);
	t1.insert(&nb);
	t1.insert(&nc);
	t1.insert(&n9);
	Node* nn3 = t1.nearestNeighbor(&n9);
	print_node(*nn3);

	// Test Clear Tree:
	t1.clear();
	// cout << "size of tree after clear: " << t1.getSize() << endl;
	// cout << "root: " << t1.getRoot() << endl;
	cout << "===============================================" << endl;
}

void test_kdtree_knn(){
	cout << "==========Test KDTree K Nearest Neighbor=========" << endl;
	point3d p1 (1, 2, 3);
	point3d p2 (2, 3, 4);
	point3d p3 (3, 4, 5);
	point3d p4 (5, 2, 1);
	point3d p5 (10, 0, 10);

	Node n1 (p1);
	Node n2 (p2);
	Node n3 (p3);
	Node n4 (p4);
	Node n5 (p5);

	//Initiaize KDTree;
	KDTree t1; 
	t1.insert(&n2);
	t1.insert(&n1);
	t1.insert(&n3);
	t1.insert(&n4);
	t1.insert(&n5);
	// cout << "size of kdtree: " << t1.getSize() <<  endl;

	// Find nearest neighbor for n6
	point3d p6 (9, 0, 9); // nearest neighbor should be (10, 0, 10)
	Node n6 (p6);
	// Node* nn1 = t1.nearestNeighbor(&n6);
	// print_node(*nn1);

	// Find nearest neighbor for n7
	point3d p7 (2.1, 2, 3); // nearest neighbor should be (1, 2, 3)
	Node n7 (p7);
	// Node* nn2 = t1.nearestNeighbor(&n7);
	// print_node(*nn2);

	// Find nearest neighbor for n9
	point3d p8 (6, 3, 4);
	point3d pa (6, 1, 0);
	point3d pb (3, 3, 0);
	point3d pc (9, 3, 0);
	point3d p9 (10, 0, 10.1); // nearest neighbor should be (9, 3, 0)
	Node n8 (p8);
	Node na (pa);
	Node nb (pb);
	Node nc (pc);
	Node n9 (p9);
	t1.insert(&na);
	t1.insert(&nb);
	t1.insert(&nc);
	// t1.insert(&n9);
	std::vector<Node*> knn;
	knn = t1.kNearestNeighbor(&n9, 7);
	print_node_vector(knn);

	// Test Clear Tree:
	t1.clear();
	// cout << "size of tree after clear: " << t1.getSize() << endl;
	// cout << "root: " << t1.getRoot() << endl;
	cout << "===============================================" << endl;
}


// Test
int main(int argc, char** argv){
	cout << "Test DEP kdtree" << endl;
	// testNode();
	// test_kdtree_insert();
	// test_kdtree_nn();
	test_kdtree_knn();
	return 0;
}