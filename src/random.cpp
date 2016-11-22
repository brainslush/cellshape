#include "random.h"

boost::random::mt19937 RandomGen;

/***************************
 * random_base
 ***************************/
random_base::random_base(
		boost::random::mt19937* iGen,
		std::string iType,
		long long iA,
		long long iB)
{
	gen = iGen;
	type = 0;
	if (iType == "uniform_smallint") {
		type = 10;
		dist = new boost::random::uniform_smallint<> (iA,iB);
	}
}
random_base::random_base(
		boost::random::mt19937* iGen,
		std::string iType,
		double iA,
		double iB)
{
	gen = iGen;
	type = 0;
	if (iType == "uniform_real_distribution") {
		type = 20;
		dist = new boost::random::uniform_real_distribution<> (iA,iB);
	}
	if (iType == "normal_distribution") {
		type = 21;
		dist = new boost::random::normal_distribution<> (iA,iB);
	}
	if (iType == "lognormal_distribution") {
		type = 22;
		dist = new boost::random::lognormal_distribution<> (iA,iB);
	}
}
random_base::random_base(
		boost::random::mt19937* iGen,
		std::string iType,
		double iA)
{
	gen = iGen;
	type = 0;
	if (iType == "bernoulli_distribution") {
		type = 30;
		dist = new boost::random::bernoulli_distribution<> (iA);
	}
	if (iType == "exponential_distribution") {
		type = 31;
		dist = new boost::random::exponential_distribution<> (iA);
	}
}
random_base::random_base(
		boost::random::mt19937* iGen,
		std::string iType)
{
	gen = iGen;
	type = 0;
	if (iType == "uniform_01") {
		type = 40;
		dist = new boost::random::uniform_01<>();
	}
}

template<class T> T random_base::draw() {
	switch(type) {
		case 10:
			return (*boost::get<boost::random::uniform_smallint<>*>(dist))(*gen);
		break;
		case 20:
			return (*boost::get<boost::random::uniform_real_distribution<>*>(dist))(*gen);
		break;
		case 21:
			return (*boost::get<boost::random::normal_distribution<>*>(dist))(*gen);
		break;
		case 22:
			return (*boost::get<boost::random::lognormal_distribution<>*>(dist))(*gen);
		break;
		case 30:
			return (*boost::get<boost::random::bernoulli_distribution<>*>(dist))(*gen);
		break;
		case 31:
			return (*boost::get<boost::random::exponential_distribution<>*>(dist))(*gen);
		break;
		case 32:
			return (*boost::get<boost::random::uniform_01<>*>(dist))(*gen);
		break;
	}
}

/***************************
 * random_container
 ***************************/

random_container::random_container() {
	seed = gen.default_seed;
}
random_container::~random_container() {
	for (auto& it : distributions) {
		delete it;
	}
}
unsigned long long& random_container::get_seed() {
	return seed;
}
void random_container::set_seed() {

}
void random_container::set_seed(unsigned long long iSeed) {
	seed = iSeed;
	gen.seed(seed);
}
template<typename... A> random_base* random_container::register_random(
		std::string iType,
		A... args
) {
	random_base* temp = new random_base(&gen,iType,args...);
	distributions.insert(temp);
	return temp;
}
void random_container::unregister_random(random_base* iDist) {
	delete iDist;
	distributions.erase(iDist);
}
