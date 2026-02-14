#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_


#include <rl/plan/Sampler.h>
#include <random>

namespace rl
{
    namespace plan
    {
        enum class DistributionType
        {
            UNIFORM,
            NORMAL
        };

        /**
         * Random sampling strategy with configurable distribution.
         */
        class YourSampler : public Sampler
        {
        public:
            YourSampler(DistributionType distType = DistributionType::NORMAL);

            virtual ~YourSampler();

            ::rl::math::Vector generate();

            virtual void seed(const ::std::mt19937::result_type& value);

        protected:
            ::std::uniform_real_distribution< ::rl::math::Real>::result_type rand();

            DistributionType distributionType;
            
            ::std::uniform_real_distribution< ::rl::math::Real> randDistribution;
            ::std::normal_distribution< ::rl::math::Real> normalDistribution;

            ::std::mt19937 randEngine;

        private:

        };
    }
}


#endif // _YOURSAMPLER_H_
