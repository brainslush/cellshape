#include "ignoreList.h"

using namespace ignore;

std::unordered_map<size_t, std::set<size_t >> n::list = {};

std::set<size_t> *n::obtainIgnoreEntries(size_t iId) {
    auto search = list.find(iId);
    if (search != list.end()) {
        return &(search->second);
    } else {
        return nullptr;
    }
}

bool n::isIgnored(const size_t &idA, const size_t &idB) {
    //auto idA = oA.hash_code();
    //auto idB = oB.hash_code();
    auto entries = ignore::n::obtainIgnoreEntries(idA);
    if (entries) {
        //return registrar::n::isChildSet(entries, idB);
        return entries->find(idB) != entries->end();
    }
    return false;
};