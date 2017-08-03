#include "typeRegistrar.h"

using namespace registrar;

node *registrar::n::baseNode;

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

size_t &node::get_id() {
    return id;
};

std::string &node::get_name() {
    return name;
}

node *registrar::n::findNodeById(node *startNode, size_t id) {
    if (startNode) {
        std::queue<node *> queue;
        queue.push(startNode);
        do {
            if (queue.front()->get_id() != id) {
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

bool registrar::n::isChild(size_t iIdBase, size_t iIdDerived) {
    node *nodeA = findNodeById(baseNode, iIdBase);
    if (nodeA) {
        node *nodeB = findNodeById(nodeA, iIdDerived);
        if (nodeB) {
            return true;
        }
    }
    return false;
}

bool registrar::n::isChildSet(std::vector<size_t> *iList, size_t iIdDerived) {
    bool found = false;
    auto it = iList->begin();
    do {
        if (isChild(*it, iIdDerived)) {
            found = true;
        } else {
            it++;
        }
    } while (!found && it != iList->end());
    return found;
}