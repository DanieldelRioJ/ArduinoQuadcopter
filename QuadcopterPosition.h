#ifndef QuadcopterPosition_h
#define QuadcopterPosition_h

class QuadcopterPosition{
    public:
        float ypr[3];
        unsigned long updateTime;

        void setYPR(float ypr[3]);
};

#endif