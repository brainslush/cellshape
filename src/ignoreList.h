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
        static std::unordered_map<size_t, std::set<size_t >> list;

        static std::set<size_t> *obtainIgnoreEntries(size_t iId);

        template<typename T>
        static std::vector<size_t> *obtainIgnoreEntries() {
            return obtainIgnoreEntries(typeid(T).hash_code());
        };

        template<typename A, typename B>
        static void addRule() {
            auto idA = typeid(A).hash_code();
            auto idB = typeid(B).hash_code();
            // get the ids of all childs of the id
            auto childrenIdsA = registrar::n::obtainChildren(idA);
            auto childrenIdsB = registrar::n::obtainChildren(idB);

            for (auto &itA : childrenIdsA) {
                auto find = list.find(itA);
                if (find != list.end()) {
                    // append existing entry
                    for (auto &itB : childrenIdsB) {
                        find->second.insert(itB);
                    }
                } else {
                    // add new entry
                    list.insert({idA, childrenIdsB});
                }
            }

            if (idA != idB) {
                for (auto &itB : childrenIdsB) {
                    auto find = list.find(itB);
                    if (find != list.end()) {
                        // append existing entry
                        for (auto &itA : childrenIdsA) {
                            find->second.insert(itA);
                        }
                    } else {
                        // add new entry
                        list.insert({idB, childrenIdsA});
                    }
                }
            }
        };

        static bool isIgnored(const size_t &idA, const size_t &idB);
    };
}

#endif //SRC_IGNORELIST_H
