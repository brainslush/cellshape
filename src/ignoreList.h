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

    std::unordered_map<size_t, std::vector<size_t >> list;

    std::vector<size_t> *obtainIgnoreEntries(size_t iId) {
        auto search = list.find(iId);
        if (search != list.end()) {
            return &(search->second);
        } else {
            return nullptr;
        }
    }

    template<typename T>
    std::vector<size_t> *obtainIgnoreEntries() {
        return obtainIgnoreEntries(typeid(T).hash_code());
    }

    template<typename A, typename B>
    void addRule() {
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

    bool isIgnored(const type_info &oA, const type_info &oB) {
        auto idA = oA.hash_code();
        auto idB = oB.hash_code();
        auto entries = obtainIgnoreEntries(idA);
        if (entries) {
            return registrar::isChildSet(entries, idB);
        } else {
            return false;
        }
    };
}

#endif //SRC_IGNORELIST_H
