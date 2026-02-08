#include "utils.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>

float squared_euclidan_norm(Vector2 vec)
{
    return vec.x * vec.x + vec.y * vec.y;
}

float euclidean_norm(Vector2 vec)
{
    return std::sqrt(vec.x * vec.x + vec.y * vec.y);
}

float p_controller_update(float reference, float state, float kp)
{
    return (reference - state)*kp;
}

// --------------------------------------------------------------
// FUNCTION DEFFINITIONS

bool string_contains(std::string string, std::string substring)
{
    if (string.find(substring) != std::string::npos) 
        return true;
    return false;
}

uint parse_number_from_string(std::string string)
{
    std::string num = "";
    for (const char c : string)
    {
        if (c >= '0' && c <= '9')
            num += c;
    }

    if (num != "")
        return std::stoul(num);
    return 0;
}
