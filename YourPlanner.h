#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "RrtConConBase.h"
#include "YourSampler.h"

using namespace ::rl::plan;

/**
*	The implementation of your planner.
*	modify any of the existing methods to improve planning performance.
*/



class YourPlanner : public RrtConConBase
{
public:
  YourPlanner(DistributionType distType = DistributionType::NORMAL);

  virtual ~YourPlanner();
  
  DistributionType getDistributionType() const { return distributionType; }

  virtual ::std::string getName() const;

  bool solve();


  // Extension toggle flags (set before calling solve())
  bool useDynamicDomain;   // Extension 1: dynamic-domain rejection sampling
  bool useWeightedMetric;  // Extension 2: weighted joint-space distance metric
  bool useGoalBias;        // Extension 3: bidirectional goal-biased sampling

  // Dynamic-domain state (Extension 1)
  bool hasBoundaryNodes;
  ::rl::math::Vector bbMin;
  ::rl::math::Vector bbMax;
  ::rl::math::Real boundaryRadius;

  void expandBoundingBox(const ::rl::math::Vector& q);
  void markBoundary(Tree& tree, const Vertex& v);

  // Weighted metric state (Extension 2)
  ::rl::math::Vector weights;

  DistributionType distributionType;

protected:
  Vertex addVertex(Tree& tree, const ::rl::plan::VectorPtr& q);
  void choose(::rl::math::Vector& chosen, const Tree& tree);
  RrtConConBase::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);
  Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen) override;
  ::rl::math::Real weightedDistance(const ::rl::math::Vector& a, const ::rl::math::Vector& b) const;
private:

};

#endif // _YOUR_PLANNER_H_
