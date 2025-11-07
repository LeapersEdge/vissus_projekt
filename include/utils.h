#pragma once

#include <string>
#include "vector2.hpp"
#include "defines.h"

struct Boit
{
    Msg_Odom odom;
    bool initialized = false;
    float last_rotation = 0.0f; 
    clock_t last_time = 0;
};

float squared_euclidan_norm(Vector2 vec);

float p_controller_update(float reference, float state, float kp);

bool string_contains(std::string string, std::string substring);

uint parse_number_from_string(std::string string);
