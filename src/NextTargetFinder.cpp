#include "NextTargetFinder.h"

using namespace std;

#define PI 3.1415926535

typedef pair<FLOATTYPE, FLOATTYPE> Vector2d; 

NextTargetFinder::NextTargetFinder():
	TangencyFinder(),m_direction(CLOCKWISE), m_velocity(1.0), m_frequency(1.0)
{}

NextTargetFinder::NextTargetFinder(Point2d centre, FLOATTYPE R, Direction dir, FLOATTYPE vel, FLOATTYPE frequency):
	TangencyFinder(centre, R), m_direction(dir), m_velocity(vel), m_frequency(frequency)
{}

static bool IsZero(FLOATTYPE num)
{
    static const FLOATTYPE e = 1e-10;
    if (-e<num && num<e)
        return true;
    else
        return false;
}

static FLOATTYPE GetAngleWithXAxis(Point2d pt)
{
    if (IsZero(pt.second)  && pt.first>=0)
		return 0; 

    if (IsZero(pt.second) && pt.first<0)
		return 180; 

    if (IsZero(pt.first) && pt.second > 0)
		return 90; 
	
    if (IsZero(pt.first)  && pt.second < 0)
		return 270; 

    FLOATTYPE tangent = pt.second/pt.first;
	FLOATTYPE draftAngleRad = atan(tangent); 
	FLOATTYPE draftAngleC = draftAngleRad*180/PI; 

	if (pt.first>0 && pt.second > 0)
		return draftAngleC; 

	if (pt.first<0 && pt.second > 0)
		return 180 + draftAngleC; 

	if (pt.first<0 && pt.second < 0)
		return 180 + draftAngleC; 

	if (pt.first>0 && pt.second < 0)
		return 360 + draftAngleC; 

	return draftAngleC; // eliminate warning
}

static FLOATTYPE AngleDiff(FLOATTYPE angle1, FLOATTYPE angle2)
{
	FLOATTYPE res = angle1 - angle2; 
	if (res > 180.0)
	{
		return res - 360.0; 
	}

	if (res < -180.0)
	{
		return 360 + res; 
	}

	return res; 
}

Point2d NextTargetFinder::FindNextTarget(const Point2d& curPt)
{
	Point2d pt1, pt2; 
	Point2d centre = GetCentre(); 
	FLOATTYPE t_x0 = centre.first; 
	FLOATTYPE t_y0 = centre.second; 
	FLOATTYPE R = GetRadius(); 

	int res = FindTangency(curPt, pt1, pt2); 

	if (res == 1) // on the circle
	{
		FLOATTYPE t_x = pt1.first; 
		FLOATTYPE t_y = pt1.second; 

		FLOATTYPE A = t_x0 - t_x; 
		FLOATTYPE B = t_y0 - t_y; 
		FLOATTYPE C = POW2(t_x) + POW2(t_y) - t_x*t_x0 - t_y*t_y0; 

		Vector2d vec(C/A, -C/B); 

		// normalize the vector
		FLOATTYPE length = sqrt(POW2(vec.first) + POW2(vec.second)); 
		vec.first = vec.first/length; 
		vec.second = vec.second/length; 

		FLOATTYPE distance = m_velocity/m_frequency; 
		
		pt1.first = t_x + vec.first*distance; 
		pt1.second = t_y +vec.second*distance; 

		pt2.first = t_x - vec.first*distance; 
		pt2.second = t_y - vec.second*distance; 
	}
	else if (res == 0) // inside the circle
	{
		FLOATTYPE t_x = curPt.first; 
		FLOATTYPE t_y = curPt.second; 

        FLOATTYPE A = (t_x0 - t_x)/(t_y - t_y0);
        FLOATTYPE B = (POW2(t_x) - t_x0*t_x)/(t_y - t_y0) + t_y;

		FLOATTYPE t_a = POW2(A)+1; 
		FLOATTYPE t_b = 2*(A*B - t_x0 - A*t_y0); 
		FLOATTYPE t_c = POW2(t_x0) + POW2(t_y0) + POW2(B) - 2*t_y0*B - POW2(R); 

		FLOATTYPE sol1, sol2; 
		FormulaSolver::Solve(t_a, t_b, t_c, sol1, sol2); 

		pt1.first = sol1; 
		pt1.second = A*sol1 + B; 

		pt2.first = sol2; 
		pt2.second = A*sol2 + B; 
	}
	else if (res == 2)
	{
		// use pt1, pt2 directly
	}

	// choose the proper point according to the direction

	FLOATTYPE theta0 = GetAngleWithXAxis(Point2d(curPt.first-centre.first, curPt.second-centre.second)); 
	FLOATTYPE theta1 = GetAngleWithXAxis(Point2d(pt1.first-centre.first, pt1.second-centre.second)); 
	FLOATTYPE theta2 = GetAngleWithXAxis(Point2d(pt2.first-centre.first, pt2.second-centre.second)); 

	FLOATTYPE diff1 = AngleDiff(theta1, theta0); 
	FLOATTYPE diff2 = AngleDiff(theta2, theta0); 

	if (m_direction == COUNTERCLOCKWISE)
	{
		return diff1>0.0?pt1:pt2; 
	}
	else
	{
		return diff1<0.0?pt1:pt2; 
	}

}

