
/*
 * PIDSetup()
 * Sets up PID controller
 */

void PIDSetup();
 
/* functionSetup()
 *  the setup for all functions
 */

void functionSetup();

/*
 * setSteering()
 * Sets the steering angle of the car
 * Input: float angle, the desired angle
 * Output: N/A
 */
void SetSteering(float angle);
/*
 * setSpeed()
 * Sets the desired speed of the car
 * Input: float mps, the desired speed
 * Output: N/A
 */
void SetSpeed(float mps);

/*
 * getSpeed();
 * Calculates the current speed of the car using measured frequency and radius
 * Input: N/A
 * Output: N/A
 */
float getSpeed();

/*
 * fetchFreq()
 * Gets the average of the last 3 frequency measurements
 * Input: sensorNo, chosing one of the equipped sensors
 * Output: Frequency
 */

int fetchFreq(int sensorNo);
