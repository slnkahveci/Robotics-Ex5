#include "YourPlanner2.h"
#include <rl/plan/SimpleModel.h>

YourPlanner2::YourPlanner2(DistributionType distType) : RrtConConBase()
{
}

YourPlanner2::~YourPlanner2()
{
}

::std::string
YourPlanner2::getName() const
{
  return "YourPlanner2";
}

void YourPlanner2::choose(::rl::math::Vector &chosen)
{
  // your modifications here
  RrtConConBase::choose(chosen);
}

RrtConConBase::Vertex
YourPlanner2::connect(Tree &tree, const Neighbor &nearest, const ::rl::math::Vector &chosen)
{
  // your modifications here
  return RrtConConBase::connect(tree, nearest, chosen);
}

RrtConConBase::Vertex
YourPlanner2::extend(Tree &tree, const Neighbor &nearest, const ::rl::math::Vector &chosen)
{
  // your modifications here
  return RrtConConBase::extend(tree, nearest, chosen);
}

bool YourPlanner2::solve()
{
  // your modifications here
  return RrtConConBase::solve();
}
