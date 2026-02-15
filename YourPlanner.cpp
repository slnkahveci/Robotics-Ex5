#include "YourPlanner.h"
#include <cmath>
#include <rl/plan/Sampler.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Viewer.h>

YourPlanner::YourPlanner(DistributionType distType) :
  RrtConConBase(),
  distributionType(distType),
  hasBoundaryNodes(false)
{
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "YourPlanner";
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  // with %5 probability, directly sample the goal to encourage faster convergence
  if ((rand() % 20) == 0)
  {
    chosen = *this->goal;
    return;
  }

  if (hasBoundaryNodes)
  {
    // Sample within bounding box of all boundary domains (both trees)
    for (int i = 0; i < chosen.size(); ++i)
    {
      ::rl::math::Real t = static_cast<::rl::math::Real>(rand()) / RAND_MAX;
      chosen[i] = bbMin[i] + t * (bbMax[i] - bbMin[i]);
    }
  }
  else
  {
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
  tree[v].radius = std::numeric_limits<::rl::math::Real>::infinity(); // default non-boundary
  tree[v].failCount = 0; // for lk-RRT

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
  // Compute k = max(1, ceil(num_vertices / 100))
  std::size_t numVerts = ::boost::num_vertices(tree);
  std::size_t k = std::max((std::size_t)1, 
                  (std::size_t)std::ceil(numVerts / 100.0));

  // Collect all non-exhausted nodes with their weighted distances
  std::vector<std::pair<Vertex, ::rl::math::Real>> candidates;

  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
    // l-RRT: skip exhausted nodes
    if (tree[*i.first].failCount >= exhaustionLimit)
      continue;

    ::rl::math::Real weightedDist = weightedDistance(chosen, *tree[*i.first].q);
    candidates.push_back({*i.first, weightedDist});
  }

  // Fallback: if all nodes exhausted, use them anyway
  if (candidates.empty())
  {
    for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
    {
      ::rl::math::Real weightedDist = weightedDistance(chosen, *tree[*i.first].q);
      candidates.push_back({*i.first, weightedDist});
    }
  }

  // Partial sort to get k smallest
  k = std::min(k, candidates.size());
  std::partial_sort(candidates.begin(), candidates.begin() + k, candidates.end(),
    [](const std::pair<Vertex, ::rl::math::Real>& a, const std::pair<Vertex, ::rl::math::Real>& b) { return a.second < b.second; });

  // k-RRT: pick randomly among k nearest
  std::size_t idx = rand() % k;
  Vertex chosen_v = candidates[idx].first;

  // Return with ACTUAL model distance (not weighted)
  ::rl::math::Real actualDist = this->model->distance(chosen, *tree[chosen_v].q);
  return Neighbor(chosen_v, actualDist);
}

RrtConConBase::Vertex
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = distance;
  bool reached = false;

  // Safety check: if distance is too small, we've essentially reached the target
  if (distance < 1e-6)
  {
    return NULL;
  }

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
    markBoundary(tree, nearest.first);
    tree[nearest.first].failCount++; // Increment failure count for lk-RRT
    return NULL;
  }

  ::rl::math::Vector next(this->model->getDof());

  while (!reached)
  {
    distance = this->model->distance(*last, chosen);
    step = distance;

    // Safety check: if distance is too small, we've reached the target
    if (distance < 1e-6)
    {
      break;
    }

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
      markBoundary(tree, nearest.first);
      tree[nearest.first].failCount++; // Increment failure count for lk-RRT
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
  // Paper: R = 10 * epsilon (interpolation step)
  boundaryRadius = 10.0 * this->delta;
  hasBoundaryNodes = false;

  exhaustionLimit = 10; // l: max consecutive failures before removal

  // Initialize weighted metric: base joints (larger workspace impact) get higher weight
  std::size_t dof = this->model->getDof();
  weights.resize(dof);
  for (std::size_t i = 0; i < dof; ++i)
  {
    // Linearly decreasing weights from base to end-effector
    weights[i] = static_cast<::rl::math::Real>(dof - i) / dof;
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
      // Lines 3-6: Rejection sampling from dynamic domain
      Neighbor aNearest;
      int attempts = 0;
      do
      {
        this->choose(chosen);
        aNearest = this->nearest(*a, chosen);
        ++attempts;
      }
      while ((*a)[aNearest.first].radius != std::numeric_limits<::rl::math::Real>::infinity() 
             && aNearest.second < (*a)[aNearest.first].radius 
             && attempts < 100);

      // Line 7: CONNECT
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