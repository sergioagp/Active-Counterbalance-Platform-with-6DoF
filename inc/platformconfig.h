/* The max e min pulse width from my Servo (TowerPro SG90) Datasheet */
#define MAX_SERVO_PULSE	1800
#define MIN_SERVO_PULSE 1200

/* Position of servos mounted in inverse direction */
#define INVERSE_SERVO_1 1
#define INVERSE_SERVO_2 3
#define INVERSE_SERVO_3 5

/* Multiplier used to convert radian to pulses in us */
#define SERVO_MULT ( 400.0f/(PI/4.0f))

/* The max e min range of Servos in radians */
#define SERVO_MIN -PI/6 // degToRad(-80)
#define SERVO_MAX PI/6 // degToRad(80)

/* Here you should put Your Platform Values in millimeters */
/* Here you put the length of your servos arm . */
#define LENGTH_SERVO_ARM 25
/* Here you put the length of your rods length. */
#define LENGTH_SERVO_LEG 120
/* Here you put the default Heigh of your platform. 
 * This value should be close to yours rods length.
*/
#define PLATFORM_HEIGHT_DEFAULT 115
/* Here you put the radius of the top of your platform. */
#define PLATFORM_TOP_RADIUS 56.0f
/* Here you put the radius of the base of your platform. */
#define PLATFORM_BASE_RADIUS 80.5f
/* Here you put the angle between two servos axis points */
#define THETA_P_ANGLE degToRad(23.2f)
/* Here you put the angle between two platform attachment points */
#define THETA_R_ANGLE degToRad(48.0f)
/* Here you dont need to change*/
#define THETA_ANGLE ((PI/3.0f - THETA_P_ANGLE) / 2.0f)
/* Here you put the pulses of each of yours servos respective to their position in horizontal */
#define SERVO_ZERO_POSITION DEG90_SVMOTOR, DEG90_SVMOTOR, DEG90_SVMOTOR - 30, DEG90_SVMOTOR, DEG90_SVMOTOR, DEG90_SVMOTOR + 50
/* Here you put the rotation of servo arms in respect to axis X */
//#define BETA_ANGLES -PI/3, 2*PI/3, PI, 0, PI/3, -2*PI/3
#define BETA_ANGLES PI / 2, -PI / 2, -PI / 6, 5 * PI / 6, -5 * PI / 6, PI / 6
