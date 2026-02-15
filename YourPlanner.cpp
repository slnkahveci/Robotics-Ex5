#include "YourPlanner.h"
#include <cmath>
#include <rl/plan/Sampler.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Viewer.h>

YourPlanner::YourPlanner(DistributionType distType) :
  RrtConConBase(),
  distributionType(distType),
  useDynamicDomain(true),
  useWeightedMetric(true),
  useGoalBias(true),
  hasBoundaryNodes(false)
{
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Your Planner";
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  // --- Extension 3: Goal-biased sampling ---
  // With 5% probability, sample the opposite tree's root to encourage convergence
  if (useGoalBias && (rand() % 20) == 0)
  {
    if (currentTree == &this->tree[0])
      chosen = *this->goal;   // extending from start -> bias towards goal
    else
      chosen = *this->start;  // extending from goal  -> bias towards start
    return;
  }

  // --- Extension 1: Dynamic-domain bounding-box sampling ---
  if (useDynamicDomain && hasBoundaryNodes)
  {
    for (int i = 0; i < chosen.size(); ++i)
    {
      ::rl::math::Real t = static_cast<::rl::math::Real>(rand()) / RAND_MAX;
      chosen[i] = bbMin[i] + t * (bbMax[i] - bbMin[i]);
    }
  }
  else
  {
    // Baseline: uniform random sample
    chosen = this->sampler->generate();
  }
}

void
YourPlanner::expandBoundingBox(const ::rl::math::Vector& q)
{
  if (!hasBoundaryNodes)
  {
    bbMin.resize(q.size());
    bbMax.resize(q.size());
    for (int j = 0; j < q.size(); ++j)
    {
      bbMin[j] = q[j] - boundaryRadius;
      bbMax[j] = q[j] + boundaryRadius;
    }
    hasBoundaryNodes = true;
  }
  else
  {
    for (int j = 0; j < q.size(); ++j)
    {
      bbMin[j] = std::min(bbMin[j], q[j] - boundaryRadius);
      bbMax[j] = std::max(bbMax[j], q[j] + boundaryRadius);
    }
  }
}

RrtConConBase::Vertex
YourPlanner::addVertex(Tree& tree, const ::rl::plan::VectorPtr& q)
{
  Vertex v = ::boost::add_vertex(tree);
  tree[v].index = ::boost::num_vertices(tree) - 1;
  tree[v].q = q;
  tree[v].radius = std::numeric_limits<::rl::math::Real>::infinity(); // Line 8: non-boundary

  if (NULL != this->viewer)
  {
    this->viewer->drawConfigurationVertex(*tree[v].q);
  }

  return v;
}

void
YourPlanner::markBoundary(Tree& tree, const Vertex& v)
{
  // Line 12: mark node as boundary and expand sampling bbox
  tree[v].radius = boundaryRadius;
  expandBoundingBox(*tree[v].q);
}

::rl::math::Real
YourPlanner::weightedDistance(const ::rl::math::Vector& a, const ::rl::math::Vector& b) const
{
  ::rl::math::Real sum = 0.0;
  for (int i = 0; i < a.size(); ++i)
  {
    ::rl::math::Real diff = a[i] - b[i];
    sum += weights[i] * diff * diff;
  }
  return std::sqrt(sum);
}

RrtConConBase::Neighbor
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{
  Neighbor p(Vertex(), (::std::numeric_limits<::rl::math::Real>::max)());

  if (useWeightedMetric)
  {
    // --- Extension 2: Weighted distance metric ---
    ::rl::math::Real bestWeightedDist = (::std::numeric_limits<::rl::math::Real>::max)();

    for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
    {
      ::rl::math::Real wd = weightedDistance(chosen, *tree[*i.first].q);
      if (wd < bestWeightedDist)
      {
        p.first = *i.first;
        bestWeightedDist = wd;
        // Store actual model distance for geometric operations
        p.second = this->model->distance(chosen, *tree[*i.first].q);
      }
    }
  }
  else
  {
    // Baseline: model's transformed distance
    for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
    {
      ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);
      if (d < p.second)
      {
        p.first = *i.first;
        p.second = d;
      }
    }
    p.second = this->model->inverseOfTransformedDistance(p.second);
  }

  return p;
}

RrtConConBase::Vertex
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = distance;
  bool reached = false;

  if (step <= this->delta)
  {
    reached = true;
  }
  else
  {
    step = this->delta;
  }

  ::rl::plan::VectorPtr last = ::std::make_shared<::rl::math::Vector>(this->model->getDof());
  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);
  this->model->setPosition(*last);
  this->model->updateFrames();

  if (this->model->isColliding())
  {
    // --- Extension 1: mark boundary on collision ---
    if (useDynamicDomain)
      markBoundary(tree, nearest.first);
    return NULL;
  }

  ::rl::math::Vector next(this->model->getDof());

  while (!reached)
  {
    distance = this->model->distance(*last, chosen);
    step = distance;

    if (step <= this->delta)
    {
      reached = true;
    }
    else
    {
      step = this->delta;
    }

    this->model->interpolate(*last, chosen, step / distance, next);
    this->model->setPosition(next);
    this->model->updateFrames();

    if (this->model->isColliding())
    {
      // --- Extension 1: mark boundary on collision ---
      if (useDynamicDomain)
        markBoundary(tree, nearest.first);
      break;
    }

    *last = next;
  }

  Vertex connected = this->addVertex(tree, last);
  this->addEdge(nearest.first, connected, tree);
  return connected;
}

bool
YourPlanner::solve()
{
  // --- Extension 1: Dynamic-domain init ---
  if (useDynamicDomain)
  {
    boundaryRadius = 15.0 * this->delta;    // adjust this param
    hasBoundaryNodes = false;
  }

  // --- Extension 2: Weighted metric init ---
  if (useWeightedMetric)
  {
    std::size_t dof = this->model->getDof();
    weights.resize(dof);
    for (std::size_t i = 0; i < dof; ++i)
    {
      // Linearly decreasing weights from base to end-effector
      weights[i] = static_cast<::rl::math::Real>(dof - i) / dof;
    }
  }

  this->time = ::std::chrono::steady_clock::now();
  this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared<::rl::math::Vector>(*this->start));
  this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared<::rl::math::Vector>(*this->goal));

  Tree* a = &this->tree[0];
  Tree* b = &this->tree[1];

  ::rl::math::Vector chosen(this->model->getDof());

  while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
  {
    for (::std::size_t j = 0; j < 2; ++j)
    {
      Neighbor aNearest;

      if (useDynamicDomain)
      {
        // --- Extension 1: Rejection sampling from dynamic domain ---
        int attempts = 0;
        do
        {
          currentTree = a; 
          this->choose(chosen);
          aNearest = this->nearest(*a, chosen);
          ++attempts;
        }
        while ((*a)[aNearest.first].radius != std::numeric_limits<::rl::math::Real>::infinity()
               && aNearest.second > (*a)[aNearest.first].radius
               && attempts < 30); // adjust this param
      }
      else
      {
        // Baseline: single sample, no rejection
        currentTree = a;
        this->choose(chosen);
        aNearest = this->nearest(*a, chosen);
      }

      Vertex aConnected = this->connect(*a, aNearest, chosen);

      if (NULL != aConnected)
      {
        Neighbor bNearest = this->nearest(*b, *(*a)[aConnected].q);
        Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q);

        if (NULL != bConnected)
        {
          if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q))
          {
            this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
            this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
            return true;
          }
        }
      }

      using ::std::swap;
      swap(a, b);
    }
  }

  return false;
}