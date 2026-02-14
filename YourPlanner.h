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

  // epsilon and delta are defined in the base class RrtConConBase
  bool hasBoundaryNodes;
  ::rl::math::Vector bbMin;          // single shared bbox (not array)
  ::rl::math::Vector bbMax;
  ::rl::math::Real boundaryRadius;
  DistributionType distributionType;

  void expandBoundingBox(const ::rl::math::Vector& q);
  void markBoundary(Tree& tree, const Vertex& v);

protected:
  Vertex addVertex(Tree& tree, const ::rl::plan::VectorPtr& q);
  void choose(::rl::math::Vector& chosen);
  void chooseFromBoundingBox(::rl::math::Vector& chosen, int treeIdx);
  void expandBoundingBox(int treeIdx, const ::rl::math::Vector& q);
  RrtConConBase::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

private:

};

#endif // _YOUR_PLANNER_H_
