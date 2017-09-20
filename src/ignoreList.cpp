#include "ignoreList.h"

using namespace ignore;

std::unordered_map<std::size_t, registrar::node *> n::idMap = {};
std::unordered_map<registrar::node *, std::set<registrar::node *>> n::list = {};

std::set<registrar::node *> *n::obtainIgnoreEntries(registrar::node *iId) {
    auto search = list.find(iId);
    if (search != list.end()) {
        return &(search->second);
    }
    return nullptr;
}

bool n::isIgnored(const size_t &idA, const size_t &idB) {
    auto node = idMap.find(idA);
    if (node != idMap.end()) {
        auto entries = ignore::n::obtainIgnoreEntries(node->second);
        if (entries) {
            node = idMap.find(idB);
            return entries->find(node->second) != entries->end();
        }
        return false;
    }
    return false;
}

void n::listRules() {
    for (auto &it : list) {
        std::cout << it.first << "\n";
        for (auto &lit2 : it.second) {
            std::cout << "|--" << lit2 << "\n";
        }
    }
};