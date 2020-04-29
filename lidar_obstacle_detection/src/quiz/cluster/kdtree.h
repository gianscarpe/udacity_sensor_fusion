/* Gianluca Scarpellini */
// Quiz on implementing kd tree
#ifndef KDTREE_H
#define KDTREE_H

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


	void insertHelper(Node* &node, uint depth, std::vector<float> point, int id)
	{
		if (node == NULL)
			node = new Node(point, id);
		else
		{
			uint ui = depth % 2;
			if (point[ui] < node->point[ui]) 
				insertHelper(node->left, depth+1, point, id);
			else
				insertHelper(node->right, depth+1, point, id);
		}

	}


	void insert(std::vector<float> point, int id)
	{

		insertHelper(root, 0, point, id);
	}



	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node!=NULL)
		{
			float n_x = node->point[0];
			float n_y = node->point[1];
			float t_x = target[0];
			float t_y = target[1];
			if ((n_x >= t_x - distanceTol) && (n_x <= t_x + distanceTol) && (n_y >= t_y - distanceTol) && (n_y <= t_y + distanceTol))
			{
				float x_diff = n_x - t_x;
				float y_diff = n_y - t_y;
				float distance = sqrt(x_diff*x_diff + y_diff*y_diff);
				if (distance < distanceTol)
					ids.push_back(node->id);
			}

			// recursive iterate
			int di = depth % 2;
			if (target[di] - distanceTol  < node->point[di])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if (target[di] + distanceTol  > node->point[di])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}


	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	

};

#endif



