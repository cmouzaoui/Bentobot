#ifndef _threshold_h
#define _threshold_h
const int orange_min [3] = {11,26,233};
const int orange_max [3] = {30,217,255};
const int blue_min [3] = {97,54,89};
const int blue_max [3] = {127,180,189};

struct Threshold
{
    int min[3];
    int max[3];
};

const Threshold ORANGE1 = {.min = {9,26,228},
    .max = {42,244,255}};

const Threshold ORANGE2 = {.min = {11,231,36},
    .max ={23,255,255}};

#endif
