#pragma once

#ifndef __KDTREE_H__
#define __KDTREE_H__

// Note: you can put kd-tree here

#include <vector>
#include <glm/vec3.hpp>
#include "ray.h"
#include "scene.h"
#include "bbox.h"

class Geometry;
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

    bool findIntersection(ray& r, isect& i, double t_min, double t_max);

    ~SplitNode();
};

class LeafNode : public Node
{
public:

    std::vector<std::unique_ptr<Geometry>> objList;
    LeafNode(std::vector<std::unique_ptr<Geometry>> _obj) : objList(std::move(_obj)) {}

    bool findIntersection(ray& r, isect& i, double t_min, double t_max);
    
    ~LeafNode();
};

// template<typename T>
class KdTree
{ 
public:
    Node* root;

    KdTree() {
        // root = new LeafNode(std::vector<std::unique_ptr<Geometry>>());
    }
    // KdTree(std::vector<std::unique_ptr<Geometry>> objList, BoundingBox bbox, int depthLimit, int leafSize);

    ~KdTree();

    void buildTree(std::vector<std::unique_ptr<Geometry>> objList, BoundingBox bbox, int depthLimit, int leafSize);
    Node* buildTreeHelper(std::vector<std::unique_ptr<Geometry>> objList, BoundingBox bbox, int depthLimit, int leafSize, int depth);
    // bool findIntersection(ray& r, isect& i, double t_min, double t_max);

    Plane findBestPlane(std::vector<std::unique_ptr<Geometry>> const& objList, BoundingBox bbox);
    int countP(std::vector<std::unique_ptr<Geometry>> const& objList, Plane& plane, bool left);
};

#endif 