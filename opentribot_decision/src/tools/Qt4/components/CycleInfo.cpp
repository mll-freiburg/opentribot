
#include "CycleInfo.h"

TribotsTools::CycleInfo::CycleInfo () {;}

TribotsTools::CycleInfo::CycleInfo (const TribotsTools::CycleInfo& ci) : rloc_vis(0), robot_attr(0), bloc_vis(0), rloc_exec(0), bloc_exec(0), ppos(0) {
  cycle_num = ci.cycle_num;
  time_msec = ci.time_msec;
  rloc_vis = ci.rloc_vis;
  robot_attr = ci.robot_attr;
  bloc_vis = ci.bloc_vis;
  rloc_exec = ci.rloc_exec;
  bloc_exec = ci.bloc_exec;
  oloc = ci.oloc;
  vloc = ci.vloc;
  dv = ci.dv;
  gs = ci.gs;
  logmsg = ci.logmsg;
  paintmsg = ci.paintmsg;
  ppos = ci.ppos;
}

const TribotsTools::CycleInfo& TribotsTools::CycleInfo::operator= (const TribotsTools::CycleInfo& ci) {
  cycle_num = ci.cycle_num;
  time_msec = ci.time_msec;
  rloc_vis = ci.rloc_vis;
  robot_attr = ci.robot_attr;
  bloc_vis = ci.bloc_vis;
  rloc_exec = ci.rloc_exec;
  bloc_exec = ci.bloc_exec;
  oloc = ci.oloc;
  vloc = ci.vloc;
  dv = ci.dv;
  gs = ci.gs;
  logmsg = ci.logmsg;
  paintmsg = ci.paintmsg;
  ppos = ci.ppos;
  return (*this);
}
