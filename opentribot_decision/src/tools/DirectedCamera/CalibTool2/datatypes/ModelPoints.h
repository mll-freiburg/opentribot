#ifndef _MODELPOINTS_H_
#define _MODELPOINTS_H_
#include <iostream>


/**  */
class ModelPoints 
{
	public:	
		ModelPoints(double xm, double ym, bool clicked, double xw, double yw);
		virtual ~ModelPoints();
		
		double getXModel();
		double getYModel();
		bool getClicked();
		double getXWorld();
		double getYWorld();
		
		void setXModel(double xm);
		void setYModel(double ym);
		void setClicked(bool clicked);
		void setXWorld(double xw);
		void setYWorld(double yw);
		
		friend std::ostream& operator << (std::ostream& os, ModelPoints&  mp);

	private: 
		double xModel;
		double yModel;
		bool isClicked;
		double xWorld;
		double yWorld;
};


#endif //_MODELPOINTS_H_

