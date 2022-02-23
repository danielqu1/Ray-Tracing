#pragma once

// Note: you can put kd-tree here

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "bbox.h"
#include "camera.h"
#include "material.h"
#include "ray.h"

#include <glm/geometric.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/matrix.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>


class SplitNode;
class LeafNode;

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

class Node {
public:
    virtual bool findIntersection(ray& r, isect& i, double t_min, double t_max) = 0;
};

class SplitNode : public Node {
public:
    int axis;
    Node* left;
    Node* right;
    double pos;
    BoundingBox bbox;

    SplitNode(int a, int p, BoundingBox b, Node* l, Node* r) : axis(a), pos(p), bbox(b), left(l), right(r) {}

    bool findIntersection(ray& r, isect& i, double t_min, double t_max) {
        bbox.intersect(r, t_min, t_max);
        double pos_min = r.at(t_min)[axis];
        double pos_max = r.at(t_max)[axis];


        if (r.getDirection()[axis] < RAY_EPSILON && r.getDirection()[axis] > -RAY_EPSILON){
            pos_min+= 1e-6;
            pos_max+= 1e-6;
        }

        if(pos > pos_min && pos > pos_max){
            if(left->findIntersection(r, i, t_min, t_max))
                return true;
        } else if(pos < pos_min && pos < pos_max){
            if(right->findIntersection(r, i, t_min, t_max))
                 return true;
        } else {
            if (left->findIntersection(r, i, t_min, t_max)) return true;
            if (right->findIntersection(r, i, t_min, t_max)) return true;
        }
        return false;

    }
    ~SplitNode() {
        delete right;
        delete left;
    };

};

class LeafNode : public Node
{
public:

    std::vector<Geometry*> objList;
    LeafNode(std::vector<Geometry*> _obj) : objList(_obj) {}

    bool findIntersection(ray& r, isect& i, double t_min, double t_max){
        bool found = false;
        i.setT(1e13);

        for(std::vector<Geometry*>::iterator t = objList.begin(); t != objList.end(); ++t){

            Geometry* obj = *t;
            obj->getBoundingBox().intersect(r, t_min, t_max);
            isect curr;

            if(obj->intersect(r, curr) && curr.getT() < i.getT() && curr.getT() >= t_min && curr.getT() <= t_max){
                i = curr;
                found = true;
            }
        }
        if (found) return true;
        return false;   
    }
    
    ~LeafNode() {};
};

template<typename T>
class KdTree
{ 
public:
    int depth;
    Node* root;

    KdTree() : depth(0) {}
    KdTree(std::vector<Geometry*>& objList, BoundingBox bbox, int depthLimit, int leafSize) {
        depth = 0;
        root = buildTree(objList, bbox, depthLimit, leafSize);
    }
    ~KdTree() {
        delete root;
    };

    Node* buildTree(std::vector<Geometry*> objList, BoundingBox bbox, int depthLimit, int leafSize) {
        if (objList.size() <= leafSize || ++depth == depthLimit) { 
            return new LeafNode(objList);   
        }
        std::vector<Geometry*> leftList;
        std::vector<Geometry*> rightList;
        Plane bestPlane = findBestPlane(objList, bbox);

        for (std::vector<Geometry*>::iterator t = objList.begin(); t != objList.end(); ++t){
            Geometry* obj = *t; 
            double min = obj->getBoundingBox().getMin()[bestPlane.axis];
            double max = obj->getBoundingBox().getMax()[bestPlane.axis];

            if (min < bestPlane.position) {
                    leftList.push_back(obj);
            }
            if (max > bestPlane.position) {
                rightList.push_back(obj);
            } 
            if (bestPlane.position == max && bestPlane.position == min && length(obj->getNormal() < 0) {
                leftList.push_back(obj);
            } else if (bestPlane.position == max && bestPlane.position == min && length(obj->getNormal() >= 0) {
                rightList.push_back(obj); 
            }
        }

        if (rightList.empty() || leftList.empty()) 
            return new LeafNode(objList);
        
        else return new SplitNode(bestPlane.axis, bestPlane.position, bbox,
                buildTree(leftList, bestPlane.leftBBox, depth, leafSize),
                buildTree(rightList, bestPlane.rightBBox, depth, leafSize)); 
    }

    Plane findBestPlane(std::vector<Geometry*> objList, BoundingBox bbox){
        std::vector<Plane> planeList;
        Plane bestPlane;
        Plane plane;

        for (int axis = 0 ; axis < 3; axis++) {
            for (std::vector<Geometry*>::iterator t = objList.begin(); 
                t != objList.end(); ++t){

                Geometry* obj = *t;   
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
    
        double minS = 1e100;      
        for (std::vector<Plane>::iterator q = planeList.begin(); q!= planeList.end(); ++q) {

            plane = *q;
            plane.leftCount = countP(objList, plane, true);
            plane.leftBBoxArea = plane.leftBBox.area();
            plane.rightCount = countP(objList, plane, false);
            plane.rightBBoxArea = plane.rightBBox.area();
            double s = (plane.leftCount * plane.leftBBoxArea + plane.rightCount
                         * plane.rightBBoxArea)/bbox.area(); 

            
            if (s < minSam){
                minSam = s;
                bestPlane = plane;
            }
        }       
        return bestPlane;
    }
    int countP(std::vector<Geometry*> objList, Plane& plane, bool left){

        int countL = 0;
        int countR = 0;
        for (std::vector<Geometry*>::iterator t = objList.begin(); t != objList.end(); ++t){

            Geometry* obj = *t;
            double min = obj->getBoundingBox().getMin()[plane.axis];
            double max = obj->getBoundingBox().getMax()[plane.axis];
    
            if(min <  plane.position) countL++;
            if(max >  plane.position) countR++;

        }
        if (left) {
            return countL;
        }
        return countR;
        
    }
};