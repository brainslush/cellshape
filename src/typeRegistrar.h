//
// Created by siegbahn on 31.07.17.
//

#include <vector>
#include <string>
#include <queue>
#include <iostream>
#include <typeinfo>

#ifndef SRC_TYPEREGISTRAR_H_
#define SRC_TYPEREGISTRAR_H_

namespace registrar {

    class node {
    public:

        node(size_t iId, std::string iName);

        node(size_t iId, std::string iName, node *iParent);

        ~node();

        std::vector<node *> &get_childs();

        void addChild(node *iChild);

        size_t &get_Id();

    protected:
        size_t id;
        std::string name;
        node *parentnode;
        std::vector<node *> childs;
    };

    node *baseNode = nullptr;

    node *findNodeById(node *startNode, size_t id);

    template<typename Base>
    void registerBase() {
        if (baseNode) {
            auto &type = typeid(Base);
            baseNode = new node(type.hash_code(), type.name());
        } else {
            std::cout << "A base class does already exist!\n";
        }
    }

    template<typename Base, typename Derived>
    void registerType() {
        auto idBase = typeid(Base).hash_code();

        node *parentNode = findNodeById(baseNode, idBase);
        if (parentNode) {
            auto &type = typeid(Derived);
            node *newChild = new node(type.hash_code(), type.name(), parentNode);
            parentNode->addChild(newChild);
        }
    }

    bool isChild(size_t iIdBase, size_t iIdDerived);

    template<typename Derived>
    static bool isChild(size_t iIdBase) {
        return isChild(iIdBase, typeid(Derived).hash_code());
    }

    bool isChildSet(std::vector<size_t> *iList, size_t iIdDerived) {
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

    template<typename Derived>
    bool isChildSet(std::vector<size_t> *iList) {
        return isChildSet(iList, typeid(Derived).hash_code());
    }
}

#endif //SRC_TYPEREGISTRAR_H_
