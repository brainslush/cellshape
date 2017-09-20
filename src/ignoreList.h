//
// Created by siegbahn on 02.08.17.
//

#include <unordered_map>
#include <typeinfo>
#include <set>
#include "typeRegistrar.h"

#ifndef SRC_IGNORELIST_H_
#define SRC_IGNORELIST_H_

namespace ignore {

    class n {
    public:
        static std::unordered_map<std::size_t, registrar::node *> idMap;
        static std::unordered_map<registrar::node*, std::set<registrar::node *>> list;

        static std::set<registrar::node *> *obtainIgnoreEntries(registrar::node *iId);

        static void listRules();

        static void addRule(const std::type_info &typeA, const std::type_info &typeB) {
            // get ids
            auto idA = typeA.hash_code();
            auto idB = typeB.hash_code();
            // get all children ids
            auto childrenNodesA = registrar::n::obtainChildrenNodes(idA);
            auto childrenNodesB = registrar::n::obtainChildrenNodes(idB);
            // run through all children and add the rule
            for (auto itA : childrenNodesA) {
                auto find = list.find(itA);
                if (find != list.end()) {
                    // append existing entry
                    for (auto &itB : childrenNodesB) {
                        find->second.insert(itB);
                    }
                } else {
                    // add new entry
                    idMap.insert({itA->get_id(),itA});
                    list.insert({itA, childrenNodesB});
                }
            }

            if (idA != idB) {
                for (auto &itB : childrenNodesB) {
                    auto find = list.find(itB);
                    if (find != list.end()) {
                        // append existing entry
                        for (auto &itA : childrenNodesA) {
                            find->second.insert(itA);
                        }
                    } else {
                        // add new entry
                        idMap.insert({itB->get_id(),itB});
                        list.insert({itB, childrenNodesA});
                    }
                }
            }
        };

        static bool isIgnored(const size_t &idA, const size_t &idB);
    };
}

#endif //SRC_IGNORELIST_H