#ifndef NEXT_TARGET_FINDER
#define NEXT_TARGET_FINDER

#include "TangencyFinder.h"

class NextTargetFinder: public TangencyFinder
{
public:
	enum Direction
	{
		CLOCKWISE, 
		COUNTERCLOCKWISE
	};
	NextTargetFinder(); 
	NextTargetFinder(Point2d centre, FLOATTYPE R, Direction dir, FLOATTYPE vel, FLOATTYPE frequency); 

	void SetDirection(Direction dir) { m_direction = dir; }
	Direction GetDirection() const { return m_direction; }

	void SetVelocity(FLOATTYPE vel) { m_velocity = vel; }
	FLOATTYPE GetVelocity() const { return m_velocity; }

	void SetFrequency(FLOATTYPE frequency) { m_frequency = frequency; }
	FLOATTYPE GetFrequency() const { return m_frequency; }

	Point2d FindNextTarget(const Point2d& curPt); 

private:
	Direction m_direction; 
	FLOATTYPE m_velocity; 
	FLOATTYPE m_frequency; 
};

#endif