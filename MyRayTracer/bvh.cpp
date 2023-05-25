#include "rayAccelerator.h"
#include "macros.h"
#include <algorithm>

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
			//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {

	BVHNode *root = new BVHNode();
	
	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	AABB world_bbox = AABB(min, max);
	
	for (Object* obj : objs) {
		AABB bbox = obj->GetBoundingBox();
		world_bbox.extend(bbox);
		objects.push_back(obj);
	}
	world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
	world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
	root->setAABB(world_bbox);
	nodes.push_back(root);
	build_recursive(0, objects.size(), root); // -> root node takes all the 
}

int BVH::getLongestAxis(BVHNode* node, float& midPoint) {
	int axisIndex; 
	float axisDist;
	AABB box = node->getAABB();

	if ((box.max.x - box.min.x) < (box.max.y - box.min.y)) {
		axisIndex = 1;
		axisDist = box.max.y - box.min.y;
		midPoint = (box.max.y + box.min.y) / 2;
	}
	else {
		axisIndex = 0;
		axisDist = box.max.x - box.min.x;
		midPoint = (box.max.x + box.min.x) / 2;
	}
	if ((box.max.z - box.min.z) > axisDist) {
		axisIndex = 2;
		axisDist = box.max.z - box.min.z;
		midPoint = (box.max.z + box.min.z) / 2;
	}

	return axisIndex;
}

int BVH::medianSplit(int left_index, int right_index, int dimension) {
	int numObjs = right_index - left_index;
	int halfObjects = floor(numObjs / 2.f);
	return left_index + halfObjects;
}

void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {
	//PUT YOUR CODE HERE
	//right_index, left_index and split_index refer to the indices in the objects vector
	// do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	// node.index can have a index of objects vector or a index of nodes vector

	Comparator cmp;
	AABB boxLeft, boxRight;
	float midPoint;
	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX);
	Vector max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	int split_index;
	bool initialized = false;
	
	if ((right_index - left_index) <= Threshold) //check if number of objects in the box is not less than threshold
		node->makeLeaf(left_index, right_index - left_index);
	else {

		cmp.dimension = getLongestAxis(node, midPoint);
		std::sort(objects.begin() + left_index, objects.begin() + right_index, cmp);

		if (objects[0]->GetBoundingBox().centroid().getAxisValue(cmp.dimension) > midPoint) { //check if left Node is empty
			split_index = medianSplit(left_index, right_index, cmp.dimension);
			initialized = true;
		}
		else {
			for (int i = left_index; i < right_index; i++) {
				if (objects[i]->GetBoundingBox().centroid().getAxisValue(cmp.dimension) > midPoint) {
					split_index = i;
					initialized = true;
					break;
				}
				else
					continue;
			}
		}

		if (!initialized) //check if right Node is empty
			split_index = medianSplit(left_index, right_index, cmp.dimension);

		node->makeNode(nodes.size());

		//Left Node
		BVHNode* leftNode = new BVHNode();
		boxLeft = AABB(min, max);
		for (int i = left_index; i < split_index; i++) {
			AABB bboxL = objects[i]->GetBoundingBox();
			boxLeft.extend(bboxL);
		}
		leftNode->setAABB(boxLeft);

		//Right Node
		BVHNode* rightNode = new BVHNode();
		boxRight = AABB(min, max);
		for (int i = split_index; i < right_index; i++) {
			AABB bboxR = objects[i]->GetBoundingBox();
			boxRight.extend(bboxR);
		}
		rightNode->setAABB(boxRight);

		//node->makeNode(nodes.size());
		nodes.push_back(leftNode);
		nodes.push_back(rightNode);

		build_recursive(left_index, split_index, leftNode);
		build_recursive(split_index, right_index, rightNode);
	}
		
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
			float tmin = FLT_MAX;  //contains the closest primitive intersection
			bool hit = false;
			float t0, t1, t2;

			BVHNode* currentNode = nodes[0];

			//PUT YOUR CODE HERE
			int leftIndex, rightIndex;
			AABB leftBox, rightBox;
			
			if (!currentNode->getAABB().intercepts(ray, t0))
				return false;

			while (true){
				if (!currentNode->isLeaf()) {
					leftIndex = currentNode->getIndex();
					rightIndex = (currentNode->getIndex() + 1);
					leftBox = nodes[leftIndex]->getAABB();
					rightBox = nodes[rightIndex]->getAABB();

					bool leftIntercepts = leftBox.intercepts(ray, t1);
					bool rightIntercepts = rightBox.intercepts(ray, t2);

					if (leftBox.isInside(ray.origin)) t1 = 0;
					if (rightBox.isInside(ray.origin)) t2 = 0;

					if (leftIntercepts && rightIntercepts) {
						//check if r.origin is inside AABB

						StackItem stackItem1(nodes[leftIndex], t1);
						StackItem stackItem2(nodes[rightIndex], t2);

						if (t2 <= t1) {
							hit_stack.push(stackItem1);
							currentNode = nodes[rightIndex];
						}
						else {
							hit_stack.push(stackItem2);
							currentNode = nodes[leftIndex];
						}
						continue;
					}
					else if (leftIntercepts) {
						currentNode = nodes[leftIndex];
						continue;
					}
					else if (rightIntercepts) {
						currentNode = nodes[rightIndex];
						continue;
					}
				}
				else { //Leaf
					int numObjs = currentNode->getNObjs();
					int obj1Index = currentNode->getIndex(); //objs vector
					for (int i = obj1Index; i < obj1Index + numObjs; i++) {
						if (objects[i]->intercepts(ray, t1) && t1 < tmin) {
							tmin = t1;
							*hit_obj = objects[i];
							hit = true;
						}
					}
				}
				bool stackChange = false;

				while (!hit_stack.empty()) {
					StackItem topNode = hit_stack.top();
					hit_stack.pop();
					if (topNode.t < tmin) {
						currentNode = topNode.ptr;
						//tmin = topNode.t;
						stackChange = true;
						break;
					}
				}

				if (hit_stack.empty()) {
					if (hit && !stackChange) {
						hit_point = ray.origin + ray.direction * tmin;
						return true;
					}
					else if (stackChange) 
						continue;
					else
						return false;
				}
			}
	}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
			float tmp, t1, t2;

			double length = ray.direction.length(); //distance between light and intersection point
			ray.direction.normalize();

			BVHNode* currentNode = nodes[0];
			int leftIndex, rightIndex;
			AABB leftBox, rightBox;

			if (!currentNode->getAABB().intercepts(ray, tmp))
				return false;

			while (true){
				if (!currentNode->isLeaf()) {
					leftIndex = currentNode->getIndex();
					rightIndex = (currentNode->getIndex() + 1);
					leftBox = nodes[leftIndex]->getAABB();
					rightBox = nodes[rightIndex]->getAABB();

					if (leftBox.intercepts(ray, t1) && rightBox.intercepts(ray, t2)) {
						StackItem stackItem1(nodes[leftIndex], t1);
						StackItem stackItem2(nodes[rightIndex], t2);
						hit_stack.push(stackItem2);
						currentNode = nodes[leftIndex];
						continue;
					}
					else if (leftBox.intercepts(ray, t1)) {
						currentNode = nodes[leftIndex];
						continue;
					}
					else if (rightBox.intercepts(ray, t2)) {
						currentNode = nodes[rightIndex];
						continue;
					}
				}
				else { //Leaf
					int numObjs = currentNode->getNObjs();
					int obj1Index = currentNode->getIndex(); //objs vector
					for (int i = obj1Index; i < obj1Index + numObjs; i++) {
						if (objects[i]->intercepts(ray, t1))
							return true;
					}
				}

				if (hit_stack.empty())
					return false;

				StackItem topNode = hit_stack.top();
				currentNode = topNode.ptr;
				hit_stack.pop();
			}
	}		
