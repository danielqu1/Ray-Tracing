#include "kdTree.h"
#include "scene.h"
#include <iostream>

using namespace std;

bool SplitNode::findIntersection(ray& r, isect& i, double t_min, double t_max) {
    // std::cout<<"at splitnode" << std::endl;

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

SplitNode::~SplitNode() {
    delete right;
    delete left;
}

bool LeafNode::findIntersection(ray& r, isect& i, double t_min, double t_max){
    // std::cout << "at leaf node" << std::endl;

    bool found = false;
    i.setT(1e13);

    for(const auto& obj : objList) {

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

LeafNode::~LeafNode() {
    for(const auto& obj : objList) delete &obj;
}

// KdTree::KdTree(std::vector<std::unique_ptr<Geometry>> objList, BoundingBox bbox, int depthLimit, int leafSize) {
//     depth = 0;
//     root = buildTree(objList, bbox, depthLimit, leafSize);
// }

KdTree::~KdTree() {
    delete root;
}

void KdTree::buildTree(std::vector<std::unique_ptr<Geometry>> objList, BoundingBox bbox, int depthLimit, int leafSize) {
    root = buildTreeHelper(std::move(objList), bbox, depthLimit, leafSize, 0);
}

Node* KdTree::buildTreeHelper(std::vector<std::unique_ptr<Geometry>> objList, BoundingBox bbox, int depthLimit, int leafSize, int depth) {
    std::cout << "buildTree depth: " << depth << std::endl;

    if (objList.size() <= leafSize || depth == depthLimit) {
        return new LeafNode(std::move(objList));
    }

    std::vector<std::unique_ptr<Geometry>> leftList;
    std::vector<std::unique_ptr<Geometry>> rightList;
    Plane bestPlane = findBestPlane(objList, bbox);

    // for(const auto& obj : objList) {
    //     double min = obj->getBoundingBox().getMin()[bestPlane.axis];
    //     double max = obj->getBoundingBox().getMax()[bestPlane.axis];

    //     if (min < bestPlane.position) {
    //         leftList.emplace_back(std::move(obj));
    //     }
    //     if (max > bestPlane.position) {
    //         rightList.emplace_back(std::move(obj));
    //     } 
    //     if (bestPlane.position == max && bestPlane.position == min && length(obj->getNormal()) < 0) {
    //         leftList.emplace_back(std::move(obj));
    //     } else if (bestPlane.position == max && bestPlane.position == min && length(obj->getNormal()) >= 0) {
    //         rightList.emplace_back(std::move(obj)); 
    //     }
    // }

    std::vector<std::unique_ptr<Geometry>>::iterator it = objList.begin();
    for( ; it != objList.end(); ) {
        double min = (*it)->getBoundingBox().getMin()[bestPlane.axis];
        double max = (*it)->getBoundingBox().getMax()[bestPlane.axis];

        if (min < bestPlane.position) {
            leftList.emplace_back(std::move(*it));
        }
        if (max > bestPlane.position) {
            rightList.emplace_back(std::move(*it));
        } 
        if (bestPlane.position == max && bestPlane.position == min && length((*it)->getNormal()) < 0) {
            leftList.emplace_back(std::move(*it));
        } else if (bestPlane.position == max && bestPlane.position == min && length((*it)->getNormal()) >= 0) {
            rightList.emplace_back(std::move(*it)); 
        }

        it = objList.erase(it);
    }

    if (rightList.empty()) {
        std::cout << "right list empty" << std::endl;
        return new LeafNode(std::move(leftList));
    } else if (leftList.empty()) {
        std::cout << "left list empty" << std::endl;
        return new LeafNode(std::move(rightList));
    }
    
    else return new SplitNode(bestPlane.axis, bestPlane.position, bbox,
        buildTreeHelper(std::move(leftList), bestPlane.leftBBox, depth, leafSize, depth + 1),
        buildTreeHelper(std::move(rightList), bestPlane.rightBBox, depth, leafSize, depth + 1)); 

    // return new LeafNode(std::move(objList));
}

// bool KdTree::findIntersection(ray& r, isect& i, double t_min, double t_max) {
//     return root->findIntersection(ray& r, isect& i, double t_min, double t_max);
// }

Plane KdTree::findBestPlane(std::vector<std::unique_ptr<Geometry>> const& objList, BoundingBox bbox){
    std::vector<Plane> planeList;
    Plane bestPlane;
    Plane plane;

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

    double minS = 1e100;      
    for (std::vector<Plane>::iterator q = planeList.begin(); q!= planeList.end(); ++q) {

        plane = *q;
        plane.leftCount = countP(objList, plane, true);
        plane.leftBBoxArea = plane.leftBBox.area();
        plane.rightCount = countP(objList, plane, false);
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

int KdTree::countP(std::vector<std::unique_ptr<Geometry>> const& objList, Plane& plane, bool left) {
    int countL = 0;
    int countR = 0;
    for(const auto& obj : objList) {
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