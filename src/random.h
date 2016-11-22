#include "extIncludes.h"

#ifndef __H_CLASSES_RANDOM
#define __H_CLASSES_RANDOM

class random_base {
	public:
		/*
		 * boost/random/uniform_smallint.hpp
		 */
		random_base(
				boost::random::mt19937* iGen,
				std::string iType,
				long long iA,
				long long iB);
		/*
		 * boost/random/uniform_real_distribution.hpp
		 * boost/random/normal_distribution.hpp
		 * boost/random/lognormal_distribution.hpp
		 */
		random_base(
				boost::random::mt19937* iGen,
				std::string iType,
				double iA,
				double iB);
		/*
		 * boost/random/bernoulli_distribution.hpp
		 * boost/random/exponential_distribution.hpp
		 */
		random_base(
				boost::random::mt19937* iGen,
				std::string iType,
				double iA);
		/*
		 * boost/random/uniform_01.hpp
		 */
		random_base(
				boost::random::mt19937* iGen,
				std::string iType);
		template<class T> T draw ();
	protected:
		unsigned type;
		boost::random::mt19937* gen;
		boost::variant<
			boost::random::uniform_smallint<>*,
			boost::random::uniform_01<>*,
			boost::random::uniform_real_distribution<>*,
			boost::random::bernoulli_distribution<>*,
			boost::random::normal_distribution<>*,
			boost::random::lognormal_distribution<>*,
			boost::random::exponential_distribution<>*
		> dist;
};

class random_container {
	public:
		random_container();
		~random_container();
		unsigned long long& get_seed();
		void set_seed();
		void set_seed(unsigned long long iSeed);
		template<typename... A > random_base* register_random(
				std::string iType,
				A... args
		);
		void unregister_random(random_base* iDist);
	protected:
		unsigned long long seed;
		boost::random::mt19937 gen;
		std::set<random_base*> distributions;
};

#endif
