
#include "TeammateLocation.h"
#include "../Fundamental/geometry.h"
#include <cmath>

using namespace Tribots;
using namespace std;

const double TeammateOccupancyGrid::inner_radius = 800;
const double TeammateOccupancyGrid::outer_radius = 1500;
const unsigned int TeammateOccupancyGrid::num_cells = 16;

namespace {

  const double cell_rad = static_cast<double>(4.0*M_PI/TeammateOccupancyGrid::num_cells);

  /* Zellennummer im inneren Ring, der zu einem Winkel gehoert */
  inline unsigned int get_cell (const Angle& a) {
    return static_cast<unsigned int>(floor(a.get_rad()/cell_rad));
  }

}


TeammateOccupancyGrid::TeammateOccupancyGrid () throw () {
  for (unsigned int i=0; i<num_cells; i++)
    cells[i]=false;
}
TeammateOccupancyGrid::TeammateOccupancyGrid (const TeammateOccupancyGrid& og) throw () {
  for (unsigned int i=0; i<num_cells; i++)
    cells[i]=og.cells[i];
}
const TeammateOccupancyGrid& TeammateOccupancyGrid::operator= (const TeammateOccupancyGrid& og) throw () {
  for (unsigned int i=0; i<num_cells; i++)
    cells[i]=og.cells[i];
  return (*this);
}

unsigned short int TeammateOccupancyGrid::occupied (Angle a) const throw () {
  unsigned int index=get_cell(a);
  return (cells[index] ? 1 : 0)+(cells[index+8] ? 2 : 0);
}

unsigned short int TeammateOccupancyGrid::occupied (Angle a1, Angle a2) const throw () {
  Angle a = a1;
  while (a.in_between (a1, a2)) {
    if (occupied(a)) return 1;
    a+=Angle::rad_angle (cell_rad);
  }
  return 0;
}

void TeammateOccupancyGrid::visualize (std::ostream& logout, Vec robot) throw (std::bad_alloc) {
  logout << "% black thin solid circle" << robot << inner_radius << " circle" << robot << outer_radius;
  for (unsigned int i=0; i<4; i++) {
    Vec endp1 = robot+outer_radius*Vec::unit_vector (Angle::deg_angle(i*45));
    Vec endp2 = robot+outer_radius*Vec::unit_vector (Angle::deg_angle(i*45+180));
    logout << " line" << endp1 << endp2;
  }
  logout << "thick";
  for (unsigned int i=0; i<8; i++) {
    Vec p1 = robot+0.5*inner_radius*Vec::unit_vector (Angle::deg_angle(i*45+22.5));
    Vec p2 = robot+0.5*(inner_radius+outer_radius)*Vec::unit_vector (Angle::deg_angle(i*45+22.5));
    if (cells[i]) logout << " circle " << p1 << "50";
    if (cells[i+8]) logout << " circle " << p2 << "50";
  }
  logout << '\n';
}

TeammateOccupancyGrid::TeammateOccupancyGrid (Vec robot, const ObstacleLocation& oloc) throw () {
  for (unsigned int i=0; i<num_cells; i++)
    cells[i]=false;
  vector<ObstacleDescriptor>::const_iterator it = oloc.begin();
  vector<ObstacleDescriptor>::const_iterator itend = oloc.end();
  vector<LineSegment> inner_segment_lines (8);  // enthaelt die Segmentbegrenzungen auf dem inneren Ring
  vector<LineSegment> outer_segment_lines (8);  // enthaelt die Segmentbegrenzungen auf dem aeusseren Ring
  for (unsigned int i=0; i<8; i++) {
    Vec unit = Vec::unit_vector (Angle::rad_angle (0.25*M_PI*i));
    inner_segment_lines [i].p1=robot;
    inner_segment_lines [i].p2=outer_segment_lines [i].p1=robot+inner_radius*unit;
    outer_segment_lines [i].p2=robot+outer_radius*unit;
  }
  while (it<itend) {
    Vec robotobs = it->pos-robot;
    double r = robotobs.length();
    if (r<outer_radius && r!=0) {
      Angle a = robotobs.angle();
      unsigned int segment_index = static_cast<unsigned int>(floor(4.0*a.get_rad()/M_PI));
      if (r<inner_radius)
        cells[segment_index]=true;
      cells[segment_index+8]=true;
      Vec ortho (robotobs.normalize().rotate_quarter());
      LineSegment obstacle (robot+robotobs+0.5*it->width*ortho, robot+robotobs-0.5*it->width*ortho);
      for (unsigned int j=1; j<=4; j++) {
        bool intersection_found=false;
        if (intersect (obstacle, inner_segment_lines[(segment_index+j)%8]).size()>0) {
          // Schnittpunkt des Hindernisses mit der Segmentgrenze des inneren Ring
          intersection_found=true;
          cells[(segment_index+j)%8]=cells[(segment_index+j)%8+8]=true;
        } else if (intersect (obstacle, outer_segment_lines[(segment_index+j)%8]).size()>0) {
          // Schnittpunkt des Hindernisses mit der Segmentgrenze des aeusseren Ring
          intersection_found=true;
          cells[(segment_index+j)%8+8]=true;
        }
        if (!intersection_found) break;
      }
      for (int j=1; j<=4; j++) {
        bool intersection_found=false;
        if (intersect (obstacle, inner_segment_lines[(segment_index+9-j)%8]).size()>0) {
          // Schnittpunkt des Hindernisses mit der Segmentgrenze des inneren Ring
          intersection_found=true;
          cells[(segment_index+8-j)%8]=cells[(segment_index+8-j)%8+8]=true;
        } else if (intersect (obstacle, outer_segment_lines[(segment_index+9-j)%8]).size()>0) {
          // Schnittpunkt des Hindernisses mit der Segmentgrenze des aeusseren Ring
          intersection_found=true;
          cells[(segment_index+8-j)%8+8]=true;
        }
        if (!intersection_found) break;
      }
    }
    it++;
  }
}
