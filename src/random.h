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

    ~random_base();
};

class random_dist : public random_base {
public:
    /*
     * boost/random/uniform_smallint.hpp
     */
    random_dist(
            boost::random::mt19937 *iGen,
            std::string iType,
            long long iA,
            long long iB);

    /*
     * boost/random/uniform_real_distribution.hpp
     * boost/random/normal_distribution.hpp
     * boost/random/lognormal_distribution.hpp
     */
    random_dist(
            boost::random::mt19937 *iGen,
            std::string iType,
            double iA,
            double iB);

    /*
     * boost/random/bernoulli_distribution.hpp
     * boost/random/exponential_distribution.hpp
     */
    random_dist(
            boost::random::mt19937 *iGen,
            std::string iType,
            double iA);

    /*
     * boost/random/uniform_01.hpp
     */
    random_dist(
            boost::random::mt19937 *iGen,
            std::string iType);

    template<typename T>
    T draw() {
        switch (type) {
            case 10:
                return (*boost::get<boost::random::uniform_smallint<> *>(dist))(*gen);
                break;
            case 20:
                return (*boost::get<boost::random::uniform_real_distribution<> *>(dist))(*gen);
                break;
            case 21:
                return (*boost::get<boost::random::normal_distribution<> *>(dist))(*gen);
                break;
            case 22:
                return (*boost::get<boost::random::lognormal_distribution<> *>(dist))(*gen);
                break;
            case 30:
                return (*boost::get<boost::random::bernoulli_distribution<> *>(dist))(*gen);
                break;
            case 31:
                return (*boost::get<boost::random::exponential_distribution<> *>(dist))(*gen);
                break;
            case 40:
                return (*boost::get<boost::random::uniform_01<> *>(dist))(*gen);
                break;
        }
        return 0;
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

    unsigned long long &get_seed();

    void set_seed();

    void set_seed(unsigned long long iSeed);

    template<class ... A>
    random_dist *register_random(
            std::string iType,
            A... args
    ) {
        random_dist *temp = new random_dist(&gen, iType, args...);
        distributions.insert(temp);
        return temp;
    };

    void unregister_random(random_dist *iDist);

protected:
    unsigned long long seed;

    unsigned long long get_uptime();

    boost::random::mt19937 gen;
    std::set<random_dist *> distributions;
};

#endif
