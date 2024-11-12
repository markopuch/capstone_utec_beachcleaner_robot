#include <iostream>
#include <wiringPi.h>
#include "libHCSR04/libHCSR04.h"

using namespace std;

int trigger = 1;
int echo = 5;

int main()
{

    if (wiringPiSetup() == -1)
        return -1;

    HCSR04 ultrasonic;
    ultrasonic.init(trigger, echo);

    while(1){
        cout << "Distance is " << ultrasonic.distance(1000000) << " cm." << endl;
    }
}
