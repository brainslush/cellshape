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

std::vector<node *> &node::get_children() {
    return children;
}

void node::addChild(node *iChild) {
    children.push_back(iChild);
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
                auto &children = queue.front()->get_children();
                for (auto &it : children) {
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

void registrar::n::registerType(const std::type_info &idA, const std::type_info &idB) {

    node *parentNode = findNodeById(registrar::n::baseNode, idA.hash_code());
    if (parentNode) {
        node *newChild = new node(idB.hash_code(), idB.name(), parentNode);
        parentNode->addChild(newChild);
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

std::set<node*> registrar::n::obtainChildrenNodes(size_t iId) {
    auto startnode = findNodeById(baseNode, iId);
    if (startnode) {
        std::set<node *> children;
        std::queue<node *> queue;
        queue.push(startnode);
        do {
            children.insert(queue.front());
            auto &childNodes = queue.front()->get_children();
            queue.pop();
            for (auto &it : childNodes) {
                queue.push(it);
            }
        } while (queue.size() > 0);
        return children;
    }
    return {startnode};
};

std::set<size_t> registrar::n::obtainChildren(size_t iId) {
    auto startnode = findNodeById(baseNode, iId);
    if (startnode) {
        std::set<size_t> children;
        std::queue<node *> queue;
        queue.push(startnode);
        do {
            children.insert(queue.front()->get_id());
            auto &childNodes = queue.front()->get_children();
            queue.pop();
            for (auto &it : childNodes) {
                queue.push(it);
            }
        } while (queue.size() > 0);
        return children;
    }
    return {iId};
}