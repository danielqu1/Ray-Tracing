#pragma once

#ifndef __KDTREE_H__
#define __KDTREE_H__

// Note: you can put kd-tree here

#include <vector>
#include <glm/vec3.hpp>
#include "ray.h"
#include "scene.h"
#include "bbox.h"
#include <iostream>

using namespace std;

class SplitNode;
class LeafNode;

// plane struct
struct Plane{
    int axis; //0 = x, 1 = y, 2 = z
    double position;
    int leftCount; 
    int rightCount;
    double leftBBoxArea;
    double rightBBoxArea;
    BoundingBox leftBBox; 
    BoundingBox rightBBox; 
};

// parent node class
class Node {
public:
    virtual bool findIntersection(ray& r, isect& i, double& t_min, double& t_max) = 0;
};

// split point in the kdtree
class SplitNode : public Node {
public:
    int axis;
    Node* left;
    Node* right;
    double pos;
    BoundingBox bbox;

    SplitNode(int a, int p, BoundingBox b, Node* l, Node* r) : axis(a), pos(p), bbox(b), left(l), right(r) {}

    // check if ray intersects this node's bbox, return intersect if true
    bool findIntersection(ray& r, isect& i, double& t_min, double& t_max) {
        if (!bbox.intersect(r, t_min, t_max))
            return false;
        left->findIntersection(r, i, t_min, t_max);
        right->findIntersection(r, i, t_min, t_max);
            return i.getT() != 1000.0;

        // find max and min on axis
        double pos_tmin = r.at(t_min)[axis];
        double pos_tmax = r.at(t_max)[axis];
        double pos_min = min(pos_tmin, pos_tmax);
        double pos_max = max(pos_tmin, pos_tmax);

        // handle near parellel
        if (r.getDirection()[axis] < RAY_EPSILON && r.getDirection()[axis] > -RAY_EPSILON){
            pos_min+= 1e-6;
            pos_max+= 1e-6;
        }

        if(pos > pos_min && pos > pos_max){
            // check if hits only left
            left->findIntersection(r, i, t_min, t_max);
            return i.getT() != 1000.0;
        } else if(pos < pos_min && pos < pos_max){
            // check if hits only right
            right->findIntersection(r, i, t_min, t_max);
            return i.getT() != 1000.0;
        } else {
            // hits both
            left->findIntersection(r, i, t_min, t_max);
            right->findIntersection(r, i, t_min, t_max);
            return i.getT() != 1000.0;
        }

        return i.getT() != 1000.0;
    }

    ~SplitNode() {
        delete right;
        delete left;
    }

};

// Leaf node of kdtree
class LeafNode : public Node
{
public:

    std::vector<Geometry*> objList;
    LeafNode(std::vector<Geometry*> _obj) : objList(_obj) {}

    // checks for intersection for all objects at this node
    bool findIntersection(ray& r, isect& i, double& t_min, double& t_max){
        // loop through all objects
        for(const auto& obj : objList) {
			isect cur;
			if( obj->intersect(r, cur) ) {
				if(i.getT() == 1000.0 || (cur.getT() < i.getT())) {
					i = cur;
				}
			}
		}

        return i.getT() != 1000.0;
    }
    
    ~LeafNode() {};
};

template<typename T>
class KdTree
{ 
public:
    Node* root = nullptr;

    KdTree() {}

    ~KdTree() {
        delete root;
    }

    void buildTree(std::vector<Geometry*> objList, BoundingBox bbox, int depthLimit, int leafSize) {
        root = buildTreeHelper(objList, bbox, depthLimit, leafSize, 0);
    }

    bool intersect(ray& r, isect& i, double& t_min, double& t_max){
        return root->findIntersection(r, i, t_min, t_max);
    }

private:

    // recursively builds the tree
    Node* buildTreeHelper(std::vector<Geometry*> objList, BoundingBox bbox, int depthLimit, int leafSize, int depth) {
        // base case
        if (objList.size() <= leafSize || ++depth == depthLimit) { 
            return new LeafNode(objList);
        }

        std::vector<Geometry*> leftList;
        std::vector<Geometry*> rightList;
        Plane bestPlane = findBestPlane(objList, bbox);

        // loop through objects and place on a side
        for(const auto& obj : objList) {
            double min = obj->getBoundingBox().getMin()[bestPlane.axis];
            double max = obj->getBoundingBox().getMax()[bestPlane.axis];

            if (min < bestPlane.position) {
                leftList.emplace_back(obj);
            }
            if (max > bestPlane.position) {
                rightList.emplace_back(obj);
            } 
            if (bestPlane.position == max && bestPlane.position == min && obj->getNormal()[bestPlane.axis] < 0) {
                leftList.emplace_back(obj);
            } else if (bestPlane.position == max && bestPlane.position == min && obj->getNormal()[bestPlane.axis] >= 0) {
                rightList.emplace_back(obj); 
            }
        }

        // see if split is useless
        if (rightList.empty() || leftList.empty()) {
            return new LeafNode(objList);
        }
        // o/w return a new split node
        else return new SplitNode(bestPlane.axis, bestPlane.position, bbox,
            buildTreeHelper(leftList, bestPlane.leftBBox, depthLimit, leafSize, depth),
            buildTreeHelper(rightList, bestPlane.rightBBox, depthLimit, leafSize, depth)); 
    }

    // searches for the best plane in objList
    Plane findBestPlane(std::vector<Geometry*> objList, BoundingBox bbox){
        std::vector<Plane> planeList;
        Plane bestPlane;
        Plane plane;

        // make all planes
        for (int axis = 0 ; axis < 3; axis++) {
            for(const auto& obj : objList) {
                Plane p1;
                Plane p2;
             
                p1.position = obj->getBoundingBox().getMin()[axis];
                p1.axis = axis;
                p1.leftBBox = BoundingBox(bbox.getMin(), bbox.getMax());
                p1.leftBBox.setMax(axis, p1.position);
                p1.rightBBox = BoundingBox(bbox.getMin(), bbox.getMax());
                p1.rightBBox.setMin(axis, p1.position);


                p2.position = obj->getBoundingBox().getMax()[axis];
                p2.axis = axis;
                p2.leftBBox = BoundingBox(bbox.getMin(), bbox.getMax());
                p2.leftBBox.setMax(axis, p2.position);
                p2.rightBBox = BoundingBox(bbox.getMin(), bbox.getMax());
                p2.rightBBox.setMin(axis, p2.position);

                planeList.push_back(p1);
                planeList.push_back(p2);
            }
        }    
    
        // pick the best plane
        double minS = 1e100;      
        for (std::vector<Plane>::iterator q = planeList.begin(); q!= planeList.end(); ++q) {

            plane = *q;
            tie(plane.leftCount, plane.rightCount) = countP(objList, plane);
            plane.leftBBoxArea = plane.leftBBox.area();
            plane.rightBBoxArea = plane.rightBBox.area();
            double s = (plane.leftCount * plane.leftBBoxArea + plane.rightCount
                         * plane.rightBBoxArea)/bbox.area(); 

            
            if (s < minS){
                minS = s;
                bestPlane = plane;
            }
        }       
        return bestPlane;
    }

    // counts the number of objects on the left and right side of the plane
    std::pair<int, int> countP(std::vector<Geometry*> objList, Plane& plane){
        int countL = 0;
        int countR = 0;
        for(const auto& obj : objList) {
            double min = obj->getBoundingBox().getMin()[plane.axis];
            double max = obj->getBoundingBox().getMax()[plane.axis];
    
            if(min <  plane.position) countL++;
            if(max >  plane.position) countR++;
        }
        
        return make_pair(countL, countR);
    }
};

#endif // __KDTREE_H__
