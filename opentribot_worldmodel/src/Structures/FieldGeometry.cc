
#include "FieldGeometry.h"
#include "../Fundamental/stringconvert.h"
#include <cmath>
#include <sstream>

using namespace std;
using namespace Tribots;

Tribots::FieldGeometry::FieldGeometry () throw () :
    clipping_geometry(NULL) {;}

Tribots::FieldGeometry::~FieldGeometry () throw () {
   if (clipping_geometry)
     delete clipping_geometry;
}

FieldGeometry::FieldGeometry (const FieldGeometry& fg) throw () :
    clipping_geometry(NULL) {
  operator= (fg);
}

const FieldGeometry& FieldGeometry::operator= (const FieldGeometry& fg) throw () {
  if (clipping_geometry) {
    delete clipping_geometry;
    clipping_geometry=NULL;
  }
  field_length=fg.field_length;
  field_width=fg.field_width;
  side_band_width=fg.side_band_width;
  goal_band_width=fg.goal_band_width;
  goal_area_length=fg.goal_area_length;
  goal_area_width=fg.goal_area_width;
  penalty_area_length=fg.penalty_area_length;
  penalty_area_width=fg.penalty_area_width;
  center_circle_radius=fg.center_circle_radius;
  corner_arc_radius=fg.corner_arc_radius;
  penalty_marker_distance=fg.penalty_marker_distance;
  line_thickness=fg.line_thickness;
  border_line_thickness=fg.border_line_thickness;
  goal_width=fg.goal_width;
  goal_length=fg.goal_length;
  goal_height=fg.goal_height;
  pole_height=fg.pole_height;
  pole_diameter=fg.pole_diameter;
  pole_position_offset_x=fg.pole_position_offset_x;
  pole_position_offset_y=fg.pole_position_offset_y;
  ball_diameter=fg.ball_diameter;
  lineset_type=fg.lineset_type;
  clipping_offset=fg.clipping_offset;
  clipping_angle=fg.clipping_angle;
  clipping_geometry=(fg.clipping_geometry ? new FieldGeometry (*fg.clipping_geometry) : NULL);
  return *this;
}

Tribots::FieldGeometry::FieldGeometry (const ConfigReader& vread) throw (InvalidConfigurationException) :
    clipping_geometry(NULL) {
  if (vread.get ("FieldGeometry::field_length", field_length)<=0 || field_length<0)
    throw Tribots::InvalidConfigurationException ("field_length");
  if (vread.get ("FieldGeometry::field_width", field_width)<=0 || field_width<0)
    throw Tribots::InvalidConfigurationException ("field_width");
  if (vread.get ("FieldGeometry::side_band_width", side_band_width)<=0 || side_band_width<0)
    throw Tribots::InvalidConfigurationException ("side_band_width");
  if (vread.get ("FieldGeometry::goal_band_width", goal_band_width)<=0 || goal_band_width<0)
    throw Tribots::InvalidConfigurationException ("goal_band_width");
  if (vread.get ("FieldGeometry::goal_area_length", goal_area_length)<=0 || goal_area_length<0)
    throw Tribots::InvalidConfigurationException ("goal_area_length");
  if (vread.get ("FieldGeometry::goal_area_width", goal_area_width)<=0 || goal_area_width<0)
    throw Tribots::InvalidConfigurationException ("goal_area_width");
  if (vread.get ("FieldGeometry::penalty_area_length", penalty_area_length)<=0 || penalty_area_length<0)
    throw Tribots::InvalidConfigurationException ("penalty_area_length");
  if (vread.get ("FieldGeometry::penalty_area_width", penalty_area_width)<=0 || penalty_area_width<0)
    throw Tribots::InvalidConfigurationException ("penalty_area_width");
  if (vread.get ("FieldGeometry::center_circle_radius", center_circle_radius)<=0 || center_circle_radius<0)
    throw Tribots::InvalidConfigurationException ("center_circle_radius");
  if (vread.get ("FieldGeometry::corner_arc_radius", corner_arc_radius)<=0 || corner_arc_radius<0)
    throw Tribots::InvalidConfigurationException ("corner_arc_radius");
  if (vread.get ("FieldGeometry::penalty_marker_distance", penalty_marker_distance)<=0 || penalty_marker_distance<0)
    throw Tribots::InvalidConfigurationException ("penalty_marker_distance");
  if (vread.get ("FieldGeometry::line_thickness", line_thickness)<=0 || line_thickness<0)
    throw Tribots::InvalidConfigurationException ("line_thickness");
  if (vread.get ("FieldGeometry::border_line_thickness", border_line_thickness)<=0 || border_line_thickness<0)
    throw Tribots::InvalidConfigurationException ("border_line_thickness");
  if (vread.get ("FieldGeometry::goal_width", goal_width)<=0 || goal_width<0)
    throw Tribots::InvalidConfigurationException ("goal_width");
  if (vread.get ("FieldGeometry::goal_length", goal_length)<=0 || goal_length<0)
    throw Tribots::InvalidConfigurationException ("goal_length");
  if (vread.get ("FieldGeometry::goal_height", goal_height)<=0 || goal_height<0)
    throw Tribots::InvalidConfigurationException ("goal_height");
  if (vread.get ("FieldGeometry::pole_height", pole_height)<=0)
    throw Tribots::InvalidConfigurationException ("pole_height");
  if (vread.get ("FieldGeometry::pole_diameter", pole_diameter)<=0)
    throw Tribots::InvalidConfigurationException ("pole_diameter");
  if (vread.get ("FieldGeometry::pole_position_offset_x", pole_position_offset_x)<=0)
    throw Tribots::InvalidConfigurationException ("pole_position_offset_x");
  if (vread.get ("FieldGeometry::pole_position_offset_y", pole_position_offset_y)<=0)
    throw Tribots::InvalidConfigurationException ("pole_position_offset_y");
  if (vread.get ("FieldGeometry::ball_diameter", ball_diameter)<=0 || ball_diameter<0)
    throw Tribots::InvalidConfigurationException ("ball_diameter");
  std::string lineset;
  lineset_type=linesetNormal;
  if (vread.get ("FieldGeometry::lineset_type", lineset)>0) {
    if (lineset == "center_circle")
      lineset_type=linesetNormal;
    else if (lineset == "center_triangle")
      lineset_type=linesetTriangle;
    else if (lineset == "center_triangle_inv")
      lineset_type=linesetTriangleInverted;
    else if (lineset == "without_center")
      lineset_type=linesetWithoutCenter;
    else if (lineset == "crosswise") {
      lineset_type=linesetCrosswise;
      clipping_angle = Angle::quarter;
    } else if (lineset == "clipping") {
      lineset_type=linesetCrosswise;
      clipping_angle = Angle::zero;
    } else {
      throw Tribots::InvalidConfigurationException ("lineset_type");
    }
    if (lineset_type==linesetCrosswise) {
      clipping_offset = Vec(0,0);
      vector<double> cw (2);
      if (vread.get ("FieldGeometry::clipping_offset", cw)<2)
        throw Tribots::InvalidConfigurationException ("clipping_offset");
      clipping_offset.x=cw[0];
      clipping_offset.y=cw[1];
      string clipping_filename;
      if (vread.get ("FieldGeometry::clipping_geometry", clipping_filename)<=0)
        throw Tribots::InvalidConfigurationException ("clipping_filename");
      ConfigReader cfg2;
      cfg2.append_from_file (clipping_filename.c_str());
      try{
        clipping_geometry = new FieldGeometry (cfg2);
      }catch(InvalidConfigurationException& e) {
        string msg = e.what();
        msg+=" in file ";
        msg+=clipping_filename;
        throw Tribots::InvalidConfigurationException (msg.c_str());
      }
    } else {
      throw Tribots::InvalidConfigurationException ("lineset_type");
    }
  }
}

bool Tribots::FieldGeometry::operator== (const FieldGeometry& fg) const throw () {
  if (std::abs(field_length-fg.field_length)>10) return false;
  if (std::abs(field_width-fg.field_width)>10) return false;
  if (std::abs(side_band_width-fg.side_band_width)>10) return false;
  if (std::abs(goal_band_width-fg.goal_band_width)>10) return false;
  if (std::abs(goal_area_length-fg.goal_area_length)>10) return false;
  if (std::abs(goal_area_width-fg.goal_area_width)>10) return false;
  if (std::abs(penalty_area_length-fg.penalty_area_length)>10) return false;
  if (std::abs(penalty_area_width-fg.penalty_area_width)>10) return false;
  if (std::abs(center_circle_radius-fg.center_circle_radius)>10) return false;
  if (std::abs(corner_arc_radius-fg.corner_arc_radius)>10) return false;
  if (std::abs(penalty_marker_distance-fg.penalty_marker_distance)>10) return false;
  if (std::abs(line_thickness-fg.line_thickness)>10) return false;
  if (std::abs(border_line_thickness-fg.border_line_thickness)>10) return false;
  if (std::abs(goal_width-fg.goal_width)>10) return false;
  if (std::abs(goal_length-fg.goal_length)>10) return false;
  if (std::abs(goal_height-fg.goal_height)>10) return false;
  if (std::abs(pole_diameter-fg.pole_diameter)>10) return false;
  if (std::abs(pole_height-fg.pole_height)>10) return false;
  if (std::abs(pole_position_offset_x-fg.pole_position_offset_x)>10) return false;
  if (std::abs(pole_position_offset_y-fg.pole_position_offset_y)>10) return false;
  if (std::abs(ball_diameter-fg.ball_diameter)>10) return false;
  if (lineset_type!=fg.lineset_type) return false;
  return true;
}

bool Tribots::FieldGeometry::operator!= (const FieldGeometry& fg) const throw () {
  return !operator== (fg);
}

void FieldGeometry::make_lineset (std::vector<LineSegment>& lines, std::vector<Arc>& arcs, bool trueLines) const throw (std::bad_alloc) {
  lines.clear();
  arcs.clear();
  if (lineset_type==linesetCrosswise && trueLines) {
    if (!clipping_geometry)
      return;
    clipping_geometry->make_lineset (lines, arcs, true);
    for (unsigned int i=0; i<lines.size(); i++) {
      lines[i].s_rotate(clipping_angle);
      lines[i].s_translate(clipping_offset);
    }
    for (unsigned int i=0; i<arcs.size(); i++) {
      arcs[i].s_rotate(clipping_angle);
      arcs[i].s_translate(clipping_offset);
    }
    return;
  } else if (!trueLines && lineset_type!=linesetCrosswise) {
    return;
  }
  double fwh = 0.5*field_width;
  double flh = 0.5*field_length;
  // Seitenlinien
  lines.push_back (LineSegment (Vec(-fwh, -flh), Vec(-fwh, flh)));
  lines.push_back (LineSegment (Vec(-fwh, flh), Vec(fwh, flh)));
  lines.push_back (LineSegment (Vec(fwh, flh), Vec(fwh, -flh)));
  lines.push_back (LineSegment (Vec(fwh, -flh), Vec(-fwh, -flh)));

  // Strafraeume
  if (penalty_area_length>0) {
    lines.push_back (LineSegment (Vec(-0.5*penalty_area_width, -flh), Vec(-0.5*penalty_area_width, -flh+penalty_area_length)));
    lines.push_back (LineSegment (Vec(0.5*penalty_area_width, -flh), Vec(0.5*penalty_area_width, -flh+penalty_area_length)));
    lines.push_back (LineSegment (Vec(-0.5*penalty_area_width, -flh+penalty_area_length), Vec(0.5*penalty_area_width, -flh+penalty_area_length)));
    lines.push_back (LineSegment (Vec(-0.5*penalty_area_width, flh), Vec(-0.5*penalty_area_width, flh-penalty_area_length)));
    lines.push_back (LineSegment (Vec(0.5*penalty_area_width, flh), Vec(0.5*penalty_area_width, flh-penalty_area_length)));
    lines.push_back (LineSegment (Vec(-0.5*penalty_area_width, flh-penalty_area_length), Vec(0.5*penalty_area_width, flh-penalty_area_length)));
  }

  // Torraeume
  if (goal_area_length>0) {
    lines.push_back (LineSegment (Vec(-0.5*goal_area_width, -flh), Vec(-0.5*goal_area_width, -flh+goal_area_length)));
    lines.push_back (LineSegment (Vec(0.5*goal_area_width, -flh), Vec(0.5*goal_area_width, -flh+goal_area_length)));
    lines.push_back (LineSegment (Vec(-0.5*goal_area_width, -flh+goal_area_length), Vec(0.5*goal_area_width, -flh+goal_area_length)));
    lines.push_back (LineSegment (Vec(-0.5*goal_area_width, flh), Vec(-0.5*goal_area_width, flh-goal_area_length)));
    lines.push_back (LineSegment (Vec(0.5*goal_area_width, flh), Vec(0.5*goal_area_width, flh-goal_area_length)));
    lines.push_back (LineSegment (Vec(-0.5*goal_area_width, flh-goal_area_length), Vec(0.5*goal_area_width, flh-goal_area_length)));
  }

  // Viertelkreise an den Ecken
  if (corner_arc_radius>0) {
    arcs.push_back (Arc (Vec(fwh,flh), corner_arc_radius, Angle::half, Angle::three_quarters));
    arcs.push_back (Arc (Vec(fwh,-flh), corner_arc_radius, Angle::quarter, Angle::half));
    arcs.push_back (Arc (Vec(-fwh,flh), corner_arc_radius, Angle::three_quarters, Angle::zero));
    arcs.push_back (Arc (Vec(-fwh,-flh), corner_arc_radius, Angle::zero, Angle::quarter));
  }

  // Strafstosspunkt (als zwei Kreuze)
  if (penalty_marker_distance>0) {
    double y = 0.5*field_length-penalty_marker_distance;
    double l = 50;
    lines.push_back (LineSegment (Vec (-l, y), Vec(l,y)));
    lines.push_back (LineSegment (Vec (0, y-l), Vec(0,y+l)));
    lines.push_back (LineSegment (Vec (-l, -y), Vec(l,-y)));
    lines.push_back (LineSegment (Vec (0, -y-l), Vec(0,-y+l)));
  }

  // Mittellinie und Mittelkreis
  if (lineset_type==linesetNormal || !trueLines) {
    lines.push_back (LineSegment (Vec(-fwh,0),Vec(fwh,0)));
    if (center_circle_radius>0) {
      arcs.push_back (Arc (Vec(0,0), center_circle_radius, Angle::zero, Angle::half));
      arcs.push_back (Arc (Vec(0,0), center_circle_radius, Angle::half, Angle::zero));
    }
  } else if (lineset_type==linesetTriangle || lineset_type==linesetTriangleInverted) {
    lines.push_back (LineSegment (Vec (-fwh,0), Vec(-430,0)));
    lines.push_back (LineSegment (Vec (fwh,0), Vec(430,0)));
    double dir = (lineset_type==linesetTriangle ? 1 : -1);
    // Dreieck zeichnen
    lines.push_back (LineSegment (Vec(0, dir*470), Vec(615, dir*-480)));
    lines.push_back (LineSegment (Vec(0, dir*470), Vec(-615, dir*-480)));
    lines.push_back (LineSegment (Vec(615, dir*-480), Vec(-615, dir*-480)));
    // T zeichnen
    lines.push_back (LineSegment (Vec(0, dir*300), Vec(0,dir*-270)));
    lines.push_back (LineSegment (Vec(325, dir*-270), Vec(-325, dir*-270)));
  } else if (lineset_type==linesetWithoutCenter) {
    // keine Mittellinien und Mittelkreis einzeichnen
  }

}

std::string FieldGeometry::serialize () const throw (std::bad_alloc) {
  stringstream inout;
  inout << field_length << ' '
            << field_width << ' '
            << side_band_width << ' '
            << goal_band_width << ' '
            << goal_area_length << ' '
            << goal_area_width << ' '
            << penalty_area_length << ' '
            << penalty_area_width << ' '
            << center_circle_radius << ' '
            << corner_arc_radius << ' '
            << penalty_marker_distance << ' '
            << line_thickness << ' '
            << border_line_thickness << ' '
            << goal_width << ' '
            << goal_length << ' '
            << goal_height << ' '
            << pole_height << ' '
            << pole_diameter << ' '
            << pole_position_offset_x << ' '
            << pole_position_offset_y << ' '
            << ball_diameter << ' '
            << lineset_type;
  if (lineset_type==linesetCrosswise) {
    inout << ' ' << clipping_offset.x << ' ' << clipping_offset.y << ' ' << clipping_angle.get_deg();
    if (clipping_geometry)
      inout << " > " << clipping_geometry->serialize() << endl;
    else
      inout << " |" << endl;
  }
  string result;
  getline (inout, result);
  return result;
}

bool FieldGeometry::deserialize (const std::string& s) throw () {
  FieldGeometry fg;
  std::vector<std::string> parts;
  Tribots::split_string (parts, s);
  if (parts.size()<22)
    return false;
  string2double (fg.field_length, parts[0]);
  string2double (fg.field_width, parts[1]);
  string2double (fg.side_band_width, parts[2]);
  string2double (fg.goal_band_width, parts[3]);
  string2double (fg.goal_area_length, parts[4]);
  string2double (fg.goal_area_width, parts[5]);
  string2double (fg.penalty_area_length, parts[6]);
  string2double (fg.penalty_area_width, parts[7]);
  string2double (fg.center_circle_radius, parts[8]);
  string2double (fg.corner_arc_radius, parts[9]);
  string2double (fg.penalty_marker_distance, parts[10]);
  string2double (fg.line_thickness, parts[11]);
  string2double (fg.border_line_thickness, parts[12]);
  string2double (fg.goal_width, parts[13]);
  string2double (fg.goal_length, parts[14]);
  string2double (fg.goal_height, parts[15]);
  string2double (fg.pole_height, parts[16]);
  string2double (fg.pole_diameter, parts[17]);
  string2double (fg.pole_position_offset_x, parts[18]);
  string2double (fg.pole_position_offset_y, parts[19]);
  string2double (fg.ball_diameter, parts[20]);
  int lineset=0;
  string2int (lineset, parts[21]);
  fg.lineset_type= FieldGeometry::LinesetType (lineset);
  if (fg.lineset_type==linesetCrosswise) {
    if (parts.size()<26)
      return false;
    string2double (fg.clipping_offset.x, parts[22]);
    string2double (fg.clipping_offset.y, parts[23]);
    double ang;
    string2double (ang, parts[24]);
    fg.clipping_angle.set_deg(ang);
    if (parts[25].length()==0)
      return false;
    if (parts[25][0]=='>') {
      string fgs = "";
      for (unsigned int i=26; i<parts.size(); i++)
        fgs+=parts[i]+" ";
      fg.clipping_geometry = new FieldGeometry;
      if (!fg.clipping_geometry->deserialize (fgs))
        return false;
    }
  }
  operator= (fg);
  return true;
}
