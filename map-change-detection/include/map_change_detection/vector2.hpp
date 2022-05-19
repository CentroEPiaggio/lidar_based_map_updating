#pragma once

#include <math.h>
#include <stdexcept>
#include <boost/type_index.hpp>

// Represents a vector (x, y) in 2-dimensional space.
template<typename T>
class Vector2
{
	public:
	// Default constructor
	Vector2() {
		values[0] = 0;
		values[1] = 0;
	}

	// Constructor initializing vector location
	Vector2(T x, T y) {
		values[0] = x;
		values[1] = y;
	}

	inline const T& getX() const    {   return values[0];   }
	inline const T& getY() const    {   return values[1];   }

	inline void setX(const T& x)    {   values[0] = x;      }
	inline void setY(const T& y)    {   values[1] = y;      }

	inline Vector2<T> scale(const T& lambda) const	{	return typename Vector2<T>::Vector2((values[0] * lambda), (values[1] * lambda));	}

	static inline bool equal(const Vector2<T>& vec1, const Vector2<T>& vec2)				{	return ((vec1.getX() == vec2.getX()) && (vec1.getY() == vec2.getY()));					}
	static inline Vector2<T> sum(const Vector2<T>& v1, const Vector2<T>& v2)			{	return typename Vector2<T>::Vector2((v1.getX() + v2.getX()), (v1.getY() + v2.getY()));	}
	static inline Vector2<T> subtract(const Vector2<T>& v1, const Vector2<T>& v2)	{	return typename Vector2<T>::Vector2((v1.getX() - v2.getX()), (v1.getY() - v2.getY()));	}
	static inline double distance(const Vector2<T>& v1, const Vector2<T>& v2) {
		throw std::invalid_argument( std::string("Can't compute distance between non double Vector2. Given vectors are of type ") + std::string(boost::typeindex::type_id<T>().pretty_name()) );
	}

	static inline double squaredDistance(const Vector2<T>& v1, const Vector2<T>& v2) {
		throw std::invalid_argument( std::string("Can't compute distance between non double Vector2. Given vectors are of type ") + std::string(boost::typeindex::type_id<T>().pretty_name()) );
	}

	private:
		T values[2];
};


template <>
inline double Vector2<double>::distance(const Vector2<double>& v1, const Vector2<double>& v2) {
	double	dx, dy;
	dx = v1.getX() - v2.getX();
	dy = v1.getY() - v2.getY();

	return std::sqrt(dx * dx + dy * dy);
}

template <>
inline double Vector2<double>::squaredDistance(const Vector2<double>& v1, const Vector2<double>& v2) {
	double	dx, dy;
	dx = v1.getX() - v2.getX();
	dy = v1.getY() - v2.getY();

	return (dx * dx + dy * dy);
}
