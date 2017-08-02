#include "typeRegistrar.h"

using namespace registrar;

node::node(size_t iId, std::string iName) :
    id(iId),
    name(iName),
    parentnode(this) {

};

node::node(size_t iId, std::string iName, node *iParent) :
    id(iId),
    name(iName),
    parentnode(iParent) {

};

node::~node() {};

std::vector<node *> &node::get_childs() {
    return childs;
}

void node::addChild(node *iChild) {
    childs.push_back(iChild);
}

size_t &node::get_Id() {
    return id;
};

node *registrar::findNodeById(node *startNode, size_t id) {
    if (!startNode) {
        std::queue<node *> queue;
        queue.push(startNode);
        do {
            if (queue.front()->get_Id() != id) {
                auto &childs = queue.front()->get_childs();
                for (auto &it : childs) {
                    queue.push(it);
                }
                queue.pop();
            } else {
                return queue.front();
            }
        } while (queue.size() > 0);
        std::cout << "Class not found!\n";
        return nullptr;
    } else {
        std::cout << "Start node is nullptr!\n";
        return nullptr;
    }
}

bool registrar::isChild(size_t iIdBase, size_t iIdDerived) {
    node *nodeA = findNodeById(baseNode, iIdBase);
    if (nodeA) {
        node *nodeB = findNodeById(nodeA, iIdDerived);
        if (nodeB) {
            return true;
        }
    }
    return false;
}