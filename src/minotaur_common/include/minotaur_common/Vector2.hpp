#ifndef MINOTAUR_VECTOR2_HPP
#define MINOTAUR_VECTOR2_HPP

#include <string>

namespace minotaur
{
	/* This class implements a 2D vector. It provides mathematical operators for adding,
	 * subtracting and scaling. */
	class Vector2
	{
	public:
		float x;
		float y;
	
		Vector2(): x(0), y(0) { }
		Vector2(float p_x, float p_y): x(p_x), y(p_y) { }
		~Vector2() { }
		
		void set(const float p_x, const float p_y);
		float lengthSQ() const;
		float length() const;
		Vector2 perpendicular() const;
		
		Vector2& operator+=(Vector2 const& p_vec);
		Vector2& operator-=(Vector2 const& p_vec);
		Vector2& operator*=(const float p_factor);
		Vector2& operator/=(const float p_divisor);
		
		std::string str() const;
	};
	
	const Vector2 operator+(Vector2 const& p_vec1, Vector2 const& p_vec2);
	const Vector2 operator-(Vector2 const& p_vec1, Vector2 const& p_vec2);
	const Vector2 operator*(Vector2 const& p_vec, const float p_factor);
	const Vector2 operator*(const float p_factor, Vector2 const& p_vec);
	const Vector2 operator/(Vector2 const& p_vec, const float p_divisor);
	bool operator==(Vector2 const& p_vec1, Vector2 const& p_vec2);
	bool operator!=(Vector2 const& p_vec1, Vector2 const& p_vec2);
}

#endif
