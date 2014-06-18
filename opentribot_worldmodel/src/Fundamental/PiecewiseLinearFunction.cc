#include "PiecewiseLinearFunction.h"

using namespace Tribots;
using namespace std;

PiecewiseLinearFunction::PiecewiseLinearFunction()
{
}

PiecewiseLinearFunction::~PiecewiseLinearFunction()
{}

void
PiecewiseLinearFunction::addVertex(double x, double y)
{
  Vec vertex(x,y);
  if (vertices.size() == 0) {         // first vertex -> simple
    vertices.push_back(vertex);
    return;
  }
  if (vertices[vertices.size()-1].x < vertex.x) { // it's the biggest x
    vertices.push_back(vertex);
  }
  
  for (vector<Vec>::iterator i = vertices.begin(); i != vertices.end(); i++) {
    if (i->x == vertex.x) {
      *i = vertex; // replace
      break;
    }
    else if (i->x > vertex.x) {
      vertices.insert(i, vertex);
    }
  }
}

double
PiecewiseLinearFunction::getValue(double x)
{
  if (vertices.size() == 0) {
    return x;
  }

  double sign = 1.;
  if (x < 0) {
   sign = -1;
   x = -x;
  }
  
  Vec first  = Vec(0.,0.); 
  Vec second = Vec(0.,0.);
  unsigned int pos = 0;
  
  do {                      // find correct vertices
    first = second;
    second = vertices[pos];
  } while (!(first.x <= x && x < second.x) && ++pos < vertices.size());
  
  if (second.x <= x) {      // x is on or "behind" the last vertex
    return (second.y + (x-second.x)) * sign;
  }
  // assert: first.x <= x < second.x
  return 
    (first.y +              // y of first vertex + 
     ((second.y - first.y) / (second.x - first.x)) *  // dy / dx (gradient) * 
     (x - first.x)) *       // dx'
     sign;                  // Vorzeichen
}
