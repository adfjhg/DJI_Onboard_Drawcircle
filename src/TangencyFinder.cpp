#include "TangencyFinder.h"

Point2d TangencyFinder::s_invalidPoint = Point2d(NaN, NaN); 

using namespace std;

static bool IsZero(FLOATTYPE num)
{
	static const FLOATTYPE e = 1e-10; 
	if (-e<num && num<e)
		return true; 
	else 
		return false; 
}

int FormulaSolver::Solve(FLOATTYPE a, FLOATTYPE b, FLOATTYPE c, FLOATTYPE& sol1, FLOATTYPE& sol2)
{
	FLOATTYPE b2minus4ac = POW2(b)-4*a*c; 
	if (b2minus4ac<0.0)
	{
		sol1 = NaN; 
		sol2 = NaN; 
		return 0; 
	}

	if (IsZero(b2minus4ac))
	{
		sol1 = -b/(2*a); 
		sol2 = sol1; 
		return 1; 
	}

	sol1 = ( -b + sqrt(b2minus4ac))/(2*a); 
	sol2 = ( -b - sqrt(b2minus4ac))/(2*a);

	return 2; 
}

TangencyFinder::TangencyFinder():
	m_centre(0.0, 0.0), m_R(1.0)
{}

TangencyFinder::TangencyFinder(Point2d centre, FLOATTYPE R): 
	m_centre(centre), m_R(R)
{}

bool TangencyFinder::IsPointValid(const Point2d& point)
{
	if (point.first == NaN || point.second == NaN)
		return false; 
	
	return true; 
}

TangencyFinder::Relation TangencyFinder::CheckPointInsideCirlce(const Point2d& point)
{
	if (!IsPointValid(point))
		return INVALID; 

	FLOATTYPE diff = POW2(point.first - m_centre.first)
		+ POW2(point.second - m_centre.second)
		- POW2(m_R); 

	if (IsZero(diff))
		return ONCIRCLE; 
	else if (diff<0.0)
		return INSIDE; 
	else
		return OUTSIDE; 
}

int TangencyFinder::FindTangency(const Point2d& inputPoint, Point2d& outputPt1, Point2d& outputPt2)
{
	Relation relation = CheckPointInsideCirlce(inputPoint); 
	if ( relation == INSIDE )
	{
		outputPt1 = s_invalidPoint; 
		outputPt2 = s_invalidPoint; 
		return 0; 
	}
	else if ( relation == ONCIRCLE )
	{
		outputPt1 = inputPoint; 
		outputPt2 = inputPoint; 
		return 1; 
	}
	else if ( relation == OUTSIDE )
	{
		FLOATTYPE t_x = inputPoint.first; 
		FLOATTYPE t_y = inputPoint.second; 
		FLOATTYPE t_x0 = m_centre.first; 
		FLOATTYPE t_y0 = m_centre.second; 
		FLOATTYPE A = (t_x - t_x0) / (t_y0 - t_y); 
		FLOATTYPE B = (POW2(t_x0) + POW2(t_y0) - POW2(m_R) - t_x*t_x0 - t_y*t_y0)/(t_y0 - t_y);

		FLOATTYPE t_a = POW2(A)+1; 
		FLOATTYPE t_b = 2*(A*B - t_x0 - A*t_y0); 
		FLOATTYPE t_c = POW2(t_x0) + POW2(t_y0) -2*B*t_y0 - POW2(m_R) + POW2(B); 

		FLOATTYPE t_resX1 = 0, t_resX2 = 0; 

		FormulaSolver::Solve(t_a, t_b, t_c, t_resX1, t_resX2); 

		FLOATTYPE t_resY1 = A*t_resX1 + B; 
		outputPt1 = Point2d(t_resX1, t_resY1); 

		FLOATTYPE t_resY2 = A*t_resX2 + B; 
		outputPt2 = Point2d(t_resX2, t_resY2); 
		
		return 2; 
	}

	return -1; 
}