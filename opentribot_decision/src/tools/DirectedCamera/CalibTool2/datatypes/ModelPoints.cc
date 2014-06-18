#include "ModelPoints.h"

ModelPoints::ModelPoints(double xm, double ym, bool clicked, double xw, double yw) 
{
	xModel = xm;
	yModel = ym;
	isClicked = clicked;
	xWorld = xw;
	yWorld = yw;
}

ModelPoints::~ModelPoints() 
{

}

double ModelPoints::getXModel() { return xModel; }
double ModelPoints::getYModel() { return yModel; }
bool ModelPoints::getClicked() { return isClicked; }
double ModelPoints::getXWorld() { return xWorld; }
double ModelPoints::getYWorld() { return yWorld; }

void ModelPoints::setXModel(double xm) { xModel = xm; }
void ModelPoints::setYModel(double ym) { yModel = ym; }
void ModelPoints::setClicked(bool clicked) { isClicked = clicked; }
void ModelPoints::setXWorld(double xw) { xWorld = xw; }
void ModelPoints::setYWorld(double yw) { yWorld = yw; }

std::ostream& operator << (std::ostream& os, ModelPoints& mp)
{
    os << "ModelPoint( ";
    os << mp.getXModel(); 
    os << ", ";
    os << mp.getYModel(); 
    os << ", ";  
    os << mp.getXWorld(); 
    os << ", ";
    os << mp.getYWorld(); 
    os << ", ";  
    os << mp.getClicked();
    os << " )" << std::endl;;    
    
    return os;
}