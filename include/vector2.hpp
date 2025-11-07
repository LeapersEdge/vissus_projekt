#pragma once
#include <cmath>

struct Vector2 
{
    float x = 0.0f;
    float y = 0.0f;

    //------------------------------------------------
    // OPERATOR OVERLOADS 

    Vector2& operator=(const Vector2& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        return *this;    
    }
    Vector2 operator-() const
    { 
        return {-x, -y}; 
    }
    Vector2& operator+=(const Vector2& rhs) 
    { 
        x += rhs.x; 
        y += rhs.y; 
        return *this; 
    }
    Vector2& operator-=(const Vector2& rhs) 
    {
        x -= rhs.x; 
        y -= rhs.y; 
        return *this;
    }
    Vector2& operator*=(float rhs)
    {
        x *= rhs; 
        y *= rhs; 
        return *this;
    }
    Vector2& operator/=(float rhs)
    {
        x /= rhs; 
        y /= rhs; 
        return *this;
    }

    Vector2 operator+(const Vector2& rhs) const
    {
        Vector2 vec = *this;
        vec.x += rhs.x; 
        vec.y += rhs.y; 
        return vec;
    }
    Vector2 operator-(const Vector2& rhs) const
    {
        Vector2 vec = *this;
        vec.x -= rhs.x; 
        vec.y -= rhs.y; 
        return vec;
    }
    Vector2 operator*(float rhs) const
    {
        Vector2 vec = *this;
        vec.x *= rhs; 
        vec.y *= rhs; 
        return vec;
    }
    Vector2 operator/(float rhs) const
    {
        Vector2 vec = *this;
        vec.x /= rhs; 
        vec.y /= rhs; 
        return vec;
    }
    bool operator==(const Vector2& rhs) const
    {
        return x == rhs.x && y == rhs.y;
    }
    bool operator!=(const Vector2& rhs) const 
    {
        return !(*this == rhs);
    }

    //------------------------------------------------
    // FUNCTIONS

    float length_squared() 
    { 
        return x*x + y*y; 
    }
    float length() 
    { 
        return std::sqrt(x*x + y*y); 
    }
    static Vector2 zero() 
    { 
        return {0.0f, 0.0f}; 
    }
};
