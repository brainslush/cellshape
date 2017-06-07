#include "random.h"

/***************************
* random_base for std containers
***************************/
random_base::random_base() {}

random_base::~random_base() {}

/***************************
 * random_dist
 ***************************/
random_dist::random_dist(
        boost::random::mt19937 *iGen,
        std::string iType,
        long long iA,
        long long iB) {
    gen = iGen;
    type = 0;
    if (iType == "uniform_smallint") {
        type = 10;
        dist = new boost::random::uniform_smallint<>(iA, iB);
    }
}

random_dist::random_dist(
        boost::random::mt19937 *iGen,
        std::string iType,
        double iA,
        double iB) {
    gen = iGen;
    type = 0;
    if (iType == "uniform_real_distribution") {
        type = 20;
        dist = new boost::random::uniform_real_distribution<>(iA, iB);
    }
    if (iType == "normal_distribution") {
        type = 21;
        dist = new boost::random::normal_distribution<>(iA, iB);
    }
    if (iType == "lognormal_distribution") {
        type = 22;
        dist = new boost::random::lognormal_distribution<>(iA, iB);
    }
}

random_dist::random_dist(
        boost::random::mt19937 *iGen,
        std::string iType,
        double iA) {
    gen = iGen;
    type = 0;
    if (iType == "bernoulli_distribution") {
        type = 30;
        dist = new boost::random::bernoulli_distribution<>(iA);
    }
    if (iType == "exponential_distribution") {
        type = 31;
        dist = new boost::random::exponential_distribution<>(iA);
    }
}

random_dist::random_dist(
        boost::random::mt19937 *iGen,
        std::string iType) {
    gen = iGen;
    type = 0;
    if (iType == "uniform_01") {
        type = 40;
        dist = new boost::random::uniform_01<>();
    }
}

/***************************
 * random_container
 ***************************/

random_container::random_container() {
    seed = gen.default_seed;
}

random_container::~random_container() {
    for (auto &it : distributions) {
        delete it;
    }
}

unsigned long long &random_container::get_seed() {
    return seed;
}

void random_container::set_seed() {
    seed = get_uptime();
}

void random_container::set_seed(unsigned long long iSeed) {
    seed = iSeed;
    gen.seed(seed);
}

void random_container::unregister_random(random_dist *iDist) {
    delete iDist;
    distributions.erase(iDist);
    iDist = NULL;
}

unsigned long long random_container::get_uptime() {
#if defined(BOOST_WINDOWS)
    return GetTickCount64();
#elif defined(__linux__) || defined(__linux) || defined(linux)
    unsigned long long uptime;
    double uptime_seconds;
    if (std::ifstream("/proc/uptime", std::ios::in) >> uptime_seconds) {
        uptime = static_cast<unsigned long long>(uptime_seconds) * 1000ULL;
    }
    return uptime;
#elif defined(macintosh) || defined(__APPLE__) || defined(__APPLE_CC__)
    /* still missing */
    return 0;
#elif (defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__) || defined(__DragonFly__)) && defined(CLOCK_UPTIME)
    /* still missing */
    return 0;
#else
    return 0;
#endif
}
