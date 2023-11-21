#ifndef STQUAD_ENUM_H_
#define STQUAD_ENUM_H_

enum Joint
{
    HR = 0,
    HP,
    KP,
    DoF = 3,
    NUM_JOINTS = 3
};

enum Leg_Num
{
    F1 = 0,
    //F2,
    //H1,
    //H2,
    Leg_N = 1
};

enum Pos
{
    X,
    Y,
    Z,
    RP
};

enum ControlMode
{
    INIT,
    HOMING,
    WALKING,
    WALKING2,
    WALKING3,
    TaskSpacePD,
    FINISH
};

enum TaskSpace
{
    MOVING1,
    MOVING2,
    MOVING3,
    MOVING4,
    FINISHTaskSpacePD
};

#endif