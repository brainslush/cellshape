//
// Created by siegbahn on 31.07.17.
//

#include <set>
#include <string>

#ifndef SRC_TYPEREGISTRAR_H_
#define SRC_TYPEREGISTRAR_H_

namespace registrar {

    template<typename T>
    class node {
    public:

        node(size_t iId) :
                id(typeid(T).hash_code()),
                name(typeid(T).name()) {
            parent = this;
        };

        node(node *iParent) :
                id(typeid(T).hash_code()),
                name(typeid(T).name()),
                parentnode(iParent){

        };

        ~node() {};

        std::vector &get_childs() {
            return childs;
        }

        void addChild(node *iChild) {
            childs.insert(iChild);
        };

        size_t &id();
    protected:
        size_t id;
        string name;
        node *parentnode;
        std::vector<node *> childs;
    };

    class n {
    public:

        template<typename Base>
        static void registerBase() {
            if (baseN) {
                baseNode = new node<Base>();
            } else {
                cout << "A base class does already exist!\n";
            }
        }

        template<typename Base, typename Derived>
        static void registerType() {
            size_t idBase = typeid(Base).hash_code();
            size_t idDerived = typeid(Derived).hash_code();

            node *parenNode = findNodeNyId(baseNode, idBase);
            if (parentNode) {
                node *newChild = new node<Derived>(queue.first());
                queue.first()->addChild(newChild);
                found = true;
            }
        }

        static bool isChild (size_t iIdBase, size_t iIdDerived) {
            node *nodeA = findNodeById(baseNode, iIdBase);
            if (nodeA) {
                node *nodeB = findNodeById(nodeA, iIdDerived);
                if (nodeB) {
                    return true
                }
            }
            return false;
        }

        template<typename Derived>
        static bool isChild (size_t iIdBase) {
            size_t idDerived = typeid(Derived);

        }

    protected:
        static node *findNodeById (node *startNode, size_t id) {
            if (!startNode) {
                bool found = false;
                std::queue<node *> queue;
                queue.push_back(startNode);
                do {
                    if (queue.first()->id() != id) {
                        auto &childs = queue.first()->get_childs();
                        for (auto &it : childs) {
                            queue.push_back(it);
                        }
                        queue.pop_front();
                    } else {
                        return queue.first();
                    }
                } while (!found || queue.size() > 0);
                if (!found) {
                    cout << "Class not found!\n";
                    return nullptr;
                }
            } else {
                cout << "Start node is nullptr!\n";
                return nullptr;
            }
        }

        static node *baseNode = nullptr;
    }
}

#endif //SRC_TYPEREGISTRAR_H_
