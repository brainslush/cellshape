//
// Created by siegbahn on 31.07.17.
//

#include <set>

#ifndef SRC_TYPEREGISTRAR_H_
#define SRC_TYPEREGISTRAR_H_

namespace registrar {
    class node {
    public:

    protected:
        node *parent;
        std::set<node *> childs;
    };
    class n {
    public:
        template<typename Base>
        static void register_base() {

        }
        template<typename Base, typename Derived>
        static void register_type() {

        }

    protected:
        static ;
    };


}



#endif //SRC_TYPEREGISTRAR_H_
