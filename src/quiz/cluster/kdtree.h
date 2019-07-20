/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	void insertHelper(Node* &node, size_t depth, std::vector<float>& point, int& id)
	{
		if(node == NULL)
		{
			node = new Node(point, id);
		}
		else{
			size_t currDim = depth % 2;
			if(point[currDim] < node->point[currDim]){
				insertHelper(node->left, depth+1, point, id);
			}
			else{
				insertHelper(node->right, depth+1, point, id);
			}
		}
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0, point, id);
	}

	inline float computeDistance(const std::vector<float>& a, const std::vector<float>& b){
		float distance = 0.0;
		for(size_t i=0; i<a.size(); i++){
			distance += (a[i]-b[i]) * (a[i]-b[i]);
		}
		return sqrt(distance);;
	}
	void searchHelper(Node* &node, size_t depth, std::vector<float>& target, std::vector<int>& innerIndice, float distanceTol)
	{
		if(node == NULL)
			return;
		size_t currDim = depth % 2;
		if(fabs(node->point[0] - target[0])< distanceTol 
			&& fabs(target[1] - node->point[1]) < distanceTol){
			if(computeDistance(target, node->point) <= distanceTol)
				innerIndice.push_back(node->id);
		}

		if((target[currDim]  - distanceTol) < node->point[currDim])
			searchHelper(node->left, depth + 1, target, innerIndice, distanceTol);
			
		
		if((target[currDim]  + distanceTol) > node->point[currDim])
			searchHelper(node->right, depth + 1, target, innerIndice, distanceTol);
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{

		std::vector<int> ids;
		searchHelper(root, 0, target, ids, distanceTol);
		return ids;
	}
	

};




