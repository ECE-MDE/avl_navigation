//==============================================================================
// Autonomous Vehicle Library
//
// Description: Enum defining the type of a state variable.
//==============================================================================

#ifndef STATE_TYPE_H
#define STATE_TYPE_H

//==============================================================================
//                              ENUM DEFINITION
//==============================================================================

// Enum defining possible state types. States can be a standard state, or an
//  angle in degrees or radians
typedef enum
{
    STATE_STANDARD,
    STATE_ANGLE_RAD,
    STATE_ANGLE_DEG
} StateType;

#endif // STATE_TYPE_H
