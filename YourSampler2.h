#ifndef _YOURSAMPLER2_H_
#define _YOURSAMPLER2_H_

#include <rl/plan/Sampler.h>
#include <random>

namespace rl
{
    namespace plan
    {
        /**
         * Uniform random sampling strategy.
         */
        class YourSampler2 : public Sampler
        {
        public:
            YourSampler2();

            virtual ~YourSampler2();

            ::rl::math::Vector generate();

            virtual void seed(const ::std::mt19937::result_type &value);

        protected:
            ::std::uniform_real_distribution<::rl::math::Real>::result_type rand();

            ::std::uniform_real_distribution<::rl::math::Real> randDistribution;

            ::std::mt19937 randEngine;

        private:
        };
    }
}

#endif // _YOURSAMPLER2_H_
