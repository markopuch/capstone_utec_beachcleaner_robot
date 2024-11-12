#ifndef DEF_SONAR
#define DEF_SONAR

class HCSR04
{
  public:
    HCSR04();
    void init(int trigger, int echo);
    double distance(int timeout);

  private:
    void recordPulseLength();
    int trigger;
    int echo;
    volatile long startTimeUsec;
    volatile long endTimeUsec;
    double distanceMeters;
    long travelTimeUsec;
    long now;
};

#endif

