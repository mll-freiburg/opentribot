#include "Points.h"

Points::Points(double x, double y) 
{
	this->xPos = x;
	this->yPos = y;
}

Points::~Points() 
{

}

double Points::getX()
{
	return xPos;
}
double Points::getY()
{
	return yPos;
}
		
void Points::setX(double x)
{
	this->xPos = x;
}
void Points::setY(double y)
{
	this->yPos = y;
}
