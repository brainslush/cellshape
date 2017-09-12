//
// Created by siegbahn on 31.07.17.
//

#include <set>
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

        std::vector<node *> &get_children();

        void addChild(node *iChild);

        size_t &get_id();

        std::string &get_name();

    protected:
        size_t id;
        std::string name;
        node *parentnode;
        std::vector<node *> children;
    };

    class n {
    public:
        static node *baseNode;

        static node *findNodeById(node *startNode, size_t id);

        static void registerBase(const std::type_info &id) {
            if (!registrar::n::baseNode) {
                baseNode = new node(id.hash_code(), id.name());
            } else {
                std::cout << "A base class does already exist! [" << baseNode->get_name() << ", " << baseNode->get_id() << "]\n";
            }
        }

        /*
        template<typename Base>
        static void registerBase() {
            if (!registrar::n::baseNode) {
                auto &type = typeid(Base);
                baseNode = new node(type.hash_code(), type.name());
            } else {
                std::cout << "A base class does already exist! [" << baseNode->get_name() << ", " << baseNode->get_id() << "]\n";
            }
        }
        */

        static void registerType(const std::type_info &idA, const std::type_info &idB);

        /*
        template<typename Base, typename Derived>
        static void registerType() {
            Base tempBase();
            Derived tempDerived();
            auto idBase = typeid(tempBase).hash_code();

            node *parentNode = findNodeById(registrar::n::baseNode, idBase);
            if (parentNode) {
                node *newChild = new node(typeid(Derived).hash_code(), typeid(Derived).name(), parentNode);
                parentNode->addChild(newChild);
            }
        }
        */

        static bool isChild(size_t iIdBase, size_t iIdDerived);

        template<typename Derived>
        static bool isChild(size_t iIdBase) {
            return isChild(iIdBase, typeid(Derived).hash_code());
        }

        static bool isChildSet(std::vector<size_t> *iList, size_t iIdDerived);

        template<typename Derived>
        static bool isChildSet(std::vector<size_t> *iList) {
            return isChildSet(iList, typeid(Derived).hash_code());
        }

        static std::set<size_t> obtainChildren(size_t iId);
    };
}

#endif //SRC_TYPEREGISTRAR_H_
