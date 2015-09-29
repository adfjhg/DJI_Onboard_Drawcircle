#ifndef TANGENCY_FINDER_H
#define TANGENCY_FINDER_H

#include <utility>
#include <cmath>

#define FLOATTYPE double
#define NaN std::sqrt(-1.0)
#define POW2(x) std::pow(x, 2)

class FormulaSolver
{
public:
	static int Solve(FLOATTYPE a, FLOATTYPE b, FLOATTYPE c, FLOATTYPE& sol1, FLOATTYPE& sol2); 
};

typedef std::pair<FLOATTYPE, FLOATTYPE> Point2d; 

class TangencyFinder
{
public:
	enum Relation
	{
		INSIDE, 
		ONCIRCLE, 
		OUTSIDE, 
		INVALID
	};
	TangencyFinder(); 
	TangencyFinder(Point2d centre, FLOATTYPE R); 

	void SetRadius(FLOATTYPE r) { m_R = r; }
	FLOATTYPE GetRadius() const { return m_R; } 

	void SetCentre(FLOATTYPE x, FLOATTYPE y) { m_centre = Point2d(x, y); } 
	void SetCentre(const Point2d& centre) { m_centre = centre; } 
	Point2d GetCentre() const { return m_centre; } 

	// return the num of tangency points
	int FindTangency(const Point2d& inputPt, Point2d& outputPt1, Point2d& outputPt2); 

	static bool IsPointValid(const Point2d& point); 

	static Point2d s_invalidPoint; 

protected:
	Relation CheckPointInsideCirlce(const Point2d& point); 

private:
	Point2d m_centre; 
	FLOATTYPE m_R; 

};

#endif
