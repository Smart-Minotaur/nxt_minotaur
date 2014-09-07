#include <cmath>
#include <sstream>
#include "minotaur_common/Vector2.hpp"

namespace minotaur
{
	void Vector2::set(const float p_x, const float p_y)
	{
		x = p_x;
		y = p_y;
	}
	
	float Vector2::lengthSQ() const
	{
		return x * x + y * y;
	}
	
	float Vector2::length() const
	{
		return sqrt(lengthSQ());
	}
	
	Vector2 Vector2::perpendicular() const
	{
		return Vector2(-y, x);
	}
	
	Vector2& Vector2::operator+=(Vector2 const& p_vec)
	{
		x += p_vec.x;
		y += p_vec.y;
		
		return *this;
	}
	
	Vector2& Vector2::operator-=(Vector2 const& p_vec)
	{
		x -= p_vec.x;
		y -= p_vec.y;
		
		return *this;
	}
	
	Vector2& Vector2::operator*=(const float p_factor)
	{
		x *= p_factor;
		y *= p_factor;
		
		return *this;
	}
	
	Vector2& Vector2::operator/=(const float p_divisor)
	{
		x /= p_divisor;
		y /= p_divisor;
		
		return *this;
	}
	
	std::string Vector2::str() const
	{
		std::stringstream ss;
		ss.precision(2);
		ss << "(" << x << ";" << y << ")";
		return ss.str();
	}
	
	const Vector2 operator+(Vector2 const& p_vec1, Vector2 const& p_vec2)
	{
		Vector2 result(p_vec1);
		result += p_vec2;
		
		return result;
	}
	
	const Vector2 operator-(Vector2 const& p_vec1, Vector2 const& p_vec2)
	{
		Vector2 result(p_vec1);
		result -= p_vec2;
		
		return result;
	}
	
	const Vector2 operator*(Vector2 const& p_vec, const float p_factor)
	{
		Vector2 result(p_vec);
		result *= p_factor;
		
		return result;
	}
	
	const Vector2 operator*(const float p_factor, Vector2 const& p_vec)
	{
		return p_vec * p_factor;
	}
	
	const Vector2 operator/(Vector2 const& p_vec, const float p_divisor)
	{
		Vector2 result(p_vec);
		result /= p_divisor;
		
		return result;
	}
	
	bool operator==(Vector2 const& p_vec1, Vector2 const& p_vec2)
	{
		return p_vec1.x == p_vec2.x && p_vec1.y == p_vec2.y;
	}
	
	bool operator!=(Vector2 const& p_vec1, Vector2 const& p_vec2)
	{
		return !(p_vec1 == p_vec2);
	}
}

