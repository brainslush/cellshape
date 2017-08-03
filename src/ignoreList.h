//
// Created by siegbahn on 02.08.17.
//

#include <unordered_map>
#include <typeinfo>
#include <vector>
#include "typeRegistrar.h"

#ifndef SRC_IGNORELIST_H_
#define SRC_IGNORELIST_H_

namespace ignore {

    class n {
    public:
        static std::unordered_map<size_t, std::vector<size_t >> list;

        static std::vector<size_t> *obtainIgnoreEntries(size_t iId);

        template<typename T>
        static std::vector<size_t> *obtainIgnoreEntries() {
            return obtainIgnoreEntries(typeid(T).hash_code());
        };

        template<typename A, typename B>
        static void addRule() {
            auto idA = typeid(A).hash_code();
            auto idB = typeid(B).hash_code();

            auto find = list.find(idA);
            if (find != list.end()) {
                // append existing entry
                find->second.push_back(idB);
            } else {
                // add new entry
                list.insert({idA, {idB}});
            }
            if (idA != idB) {
                find = list.find(idB);
                if (find != list.end()) {
                    // append existing entry
                    find->second.push_back(idA);
                } else {
                    // add new entry
                    list.insert({idB, {idA}});
                }
            }
        };

        static bool isIgnored(const size_t &idA, const size_t &idB);
    };
}

#endif //SRC_IGNORELIST_H
