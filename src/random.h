#pragma once

#include <set>
#include <string>
#include <fstream>
#include <boost/variant.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_smallint.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/bernoulli_distribution.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/lognormal_distribution.hpp>
#include <boost/random/exponential_distribution.hpp>

#ifndef SRC_RANDOM_H_
#define SRC_RANDOM_H_

class random_base {
public:
    random_base();

    virtual ~random_base();
};

class random_dist : public random_base {
public:
    random_dist();

    void create(
            boost::random::mt19937 *iGen,
            std::string iType,
            double iA);

    /*
     * boost/random/uniform_01.hpp
     */
    void create(
            boost::random::mt19937 *iGen,
            std::string iType);

    template<typename T, typename U>
    void create(
            boost::random::mt19937 *iGen,
            const std::string &iType,
            T iA,
            U iB
    ) {
        gen = iGen;
        type = 0;
        if (iType == "uniform_smallint") {
            type = 10;
            dist = new boost::random::uniform_smallint<>(iA, iB);
        } else if (iType == "uniform_real_distribution") {
            type = 20;
            dist = new boost::random::uniform_real_distribution<>(iA, iB);
        } else if (iType == "normal_distribution") {
            type = 21;
            dist = new boost::random::normal_distribution<>(iA, iB);
        } else if (iType == "lognormal_distribution") {
            type = 22;
            dist = new boost::random::lognormal_distribution<>(iA, iB);
        }
    }

    template<typename T>
    T draw() {
        switch (type) {
            case 10:
                return (*boost::get<boost::random::uniform_smallint<> *>(dist))(*gen);
            case 20:
                return (*boost::get<boost::random::uniform_real_distribution<> *>(dist))(*gen);
            case 21:
                return (*boost::get<boost::random::normal_distribution<> *>(dist))(*gen);
            case 22:
                return (*boost::get<boost::random::lognormal_distribution<> *>(dist))(*gen);
            case 30:
                return (*boost::get<boost::random::bernoulli_distribution<> *>(dist))(*gen);
            case 31:
                return (*boost::get<boost::random::exponential_distribution<> *>(dist))(*gen);
            case 40:
                return (*boost::get<boost::random::uniform_01<> *>(dist))(*gen);
            default: {
                return 0;
            }
        }
    }

protected:
    unsigned type;
    boost::random::mt19937 *gen;
    boost::variant<
            boost::random::uniform_smallint<> *,
            boost::random::uniform_01<> *,
            boost::random::uniform_real_distribution<> *,
            boost::random::bernoulli_distribution<> *,
            boost::random::normal_distribution<> *,
            boost::random::lognormal_distribution<> *,
            boost::random::exponential_distribution<> *
    > dist;
};

class random_container : public random_base {
public:
    random_container();

    virtual ~random_container();

    uint32_t &get_seed();

    void set_seed();

    void set_seed(uint32_t iSeed);

    template<class ... A>
    random_dist *register_random(
            std::string iType,
            A... args
    ) {
        auto *temp = new random_dist();
        temp->create(&gen, iType, args...);
        distributions.insert(temp);
        return temp;
    };

    void unregister_random(random_dist *iDist);

protected:
    uint32_t seed;

    unsigned long long get_uptime();

    boost::random::mt19937 gen;
    std::set<random_dist *> distributions;
};

#endif
