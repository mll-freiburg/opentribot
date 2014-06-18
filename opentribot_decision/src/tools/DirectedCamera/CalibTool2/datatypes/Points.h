#ifndef POINTS_H_
#define POINTS_H_

class Points
{
	public:	
		Points(double x, double y);
		virtual ~Points();
		
		double getX();
		double getY();
		
		void setX(double x);
		void setY(double y);

	private: 
		double xPos;
		double yPos;
};

#endif /*POINTS_H_*/
