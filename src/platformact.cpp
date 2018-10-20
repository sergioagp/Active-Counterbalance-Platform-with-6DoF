#include <stm32f4xx.h>
#include "platformact.h"
#include "platformconfig.h"

static const float beta[NSERVOS] = {
	BETA_ANGLES
};

static const int servo_def_pulse[NSERVOS] = {
	SERVO_ZERO_POSITION
};

CPlatformAct::CPlatformAct(void)
	: m_translation(0,0, PLATFORM_HEIGHT_DEFAULT), 
	m_servomotor(CServoMotor::getInstance()) {
		  
	initJoints();
	
	// calculate lengths and angles
	calcLengths();
	calcAlpha();
	// calculate pulses
	calcServoPulse();
	// set new motors position
	//reset();
	setPosition();
	m_servomotor->start();
}
	
void CPlatformAct::reset() {
	for (int i = 0; i < m_servo.n_servos; ++i) {
		m_servo.pulse[i] = servo_def_pulse[i];
	}
}

void CPlatformAct::defaultPosition() {
	reset();
	setPosition();
}

void CPlatformAct::initJoints(void) {
	m_alpha[0] = 0;
	m_alpha[1] = 0;
	m_alpha[2] = 0;
	m_alpha[3] = 0;
	m_alpha[4] = 0;
	m_alpha[5] = 0;
	
	m_basejoint[0].x = -PLATFORM_BASE_RADIUS * std::cos((float) degToRad(30) - THETA_ANGLE);
	m_basejoint[1].x = -PLATFORM_BASE_RADIUS * std::cos((float) degToRad(30) - THETA_ANGLE);
	m_basejoint[2].x =  PLATFORM_BASE_RADIUS * std::sin((float) THETA_ANGLE);
	m_basejoint[3].x =  PLATFORM_BASE_RADIUS * std::cos((float) degToRad(30) + THETA_ANGLE);
	m_basejoint[4].x =  PLATFORM_BASE_RADIUS * std::cos((float) degToRad(30) + THETA_ANGLE);
	m_basejoint[5].x =  PLATFORM_BASE_RADIUS * std::sin((float) THETA_ANGLE);

	m_basejoint[0].y = -PLATFORM_BASE_RADIUS * std::sin((float) degToRad(30) - THETA_ANGLE);
	m_basejoint[1].y =  PLATFORM_BASE_RADIUS * std::sin((float) degToRad(30) - THETA_ANGLE);
	m_basejoint[2].y =  PLATFORM_BASE_RADIUS * std::cos((float) THETA_ANGLE);
	m_basejoint[3].y =  PLATFORM_BASE_RADIUS * std::sin((float) degToRad(30) + THETA_ANGLE);
	m_basejoint[4].y = -PLATFORM_BASE_RADIUS * std::sin((float) degToRad(30) + THETA_ANGLE);
	m_basejoint[5].y = -PLATFORM_BASE_RADIUS * std::cos((float) THETA_ANGLE);

	m_platformjoint[0].x = -PLATFORM_TOP_RADIUS * std::sin((float) degToRad(30) + THETA_R_ANGLE / 2);
	m_platformjoint[1].x = -PLATFORM_TOP_RADIUS * std::sin((float) degToRad(30) + THETA_R_ANGLE / 2);
	m_platformjoint[2].x = -PLATFORM_TOP_RADIUS * std::sin((float) degToRad(30) - THETA_R_ANGLE / 2);
	m_platformjoint[3].x =  PLATFORM_TOP_RADIUS * std::cos((float) THETA_R_ANGLE / 2);
	m_platformjoint[4].x =  PLATFORM_TOP_RADIUS * std::cos((float) THETA_R_ANGLE / 2);	
	m_platformjoint[5].x = -PLATFORM_TOP_RADIUS * std::sin((float) degToRad(30) - THETA_R_ANGLE / 2);

	m_platformjoint[0].y = -PLATFORM_TOP_RADIUS * std::cos((float) degToRad(30) + THETA_R_ANGLE / 2);
	m_platformjoint[1].y =  PLATFORM_TOP_RADIUS * std::cos((float) degToRad(30) + THETA_R_ANGLE / 2);
	m_platformjoint[2].y =  PLATFORM_TOP_RADIUS * std::cos((float) degToRad(30) - THETA_R_ANGLE / 2);
	m_platformjoint[3].y =  PLATFORM_TOP_RADIUS * std::sin((float) THETA_R_ANGLE / 2);
	m_platformjoint[4].y = -PLATFORM_TOP_RADIUS * std::sin((float) THETA_R_ANGLE / 2);
	m_platformjoint[5].y = -PLATFORM_TOP_RADIUS * std::cos((float) degToRad(30) - THETA_R_ANGLE / 2);
}

void CPlatformAct::setTranslation(const point_t pos) {
	m_translation.x = pos.x;
	m_translation.y = pos.y;
	m_translation.z = pos.z + PLATFORM_HEIGHT_DEFAULT;
}

void CPlatformAct::setRotation(const slope_t& rotation) {
	// set desired rotation
	m_rotation = rotation;
	// calculate lengths and angles
	calcLengths();
	calcAlpha();
	// calculate pulses
	calcServoPulse();
	// set new motors position
	setPosition();
}

void CPlatformAct::getRotationMatrix(float rotationMatrix[3][3]) {
	float roll = m_rotation.m_roll;
	float pitch = m_rotation.m_pitch;
	float yaw = m_rotation.m_yaw;

	rotationMatrix[0][0] =  std::cos((float) yaw) * std::cos((float) pitch);
	rotationMatrix[1][0] = -std::sin((float) yaw) * std::cos((float) roll) 
		+ std::cos((float) yaw) * std::sin((float) pitch) * std::sin((float) roll);
	rotationMatrix[2][0] =  std::sin((float) yaw) * std::sin((float) roll) 
		+ std::cos((float) yaw) * std::cos((float) roll)  * std::sin((float) pitch);
	
	rotationMatrix[0][1] = std::sin((float) yaw)  	* std::cos((float) pitch);
	rotationMatrix[1][1] = std::cos((float) yaw)  	* std::cos((float) roll) 
		+ std::sin((float) yaw) * std::sin((float) pitch) * std::sin((float) roll);
	rotationMatrix[2][1] = std::cos((float) pitch)	* std::sin((float) roll);
	
	rotationMatrix[0][2] = -std::sin((float) pitch);
	rotationMatrix[1][2] = -std::cos((float) yaw)   * std::sin((float) roll) 
		+ std::sin((float) yaw) * std::sin((float) pitch) * std::cos((float) roll);
	rotationMatrix[2][2] =  std::cos((float) pitch) * std::cos((float) roll);
}

/* This method is used for calculating
* the angle made by the servo motor and its leg
*/
void CPlatformAct::calcAlpha() {
	point_t basePoint, Li;
	float min, max, dist;

	for (int i = 0; i < NSERVOS; i++){		
		min = SERVO_MIN; 
		max = SERVO_MAX;
		for (int j = 0; j < 20; j++){
			basePoint.x = LENGTH_SERVO_ARM * std::cos((float) m_alpha[i]) * std::cos((float) beta[i]) + m_basejoint[i].x;
			basePoint.y = LENGTH_SERVO_ARM * std::cos((float) m_alpha[i]) * std::sin((float) beta[i]) + m_basejoint[i].y;
			basePoint.z = LENGTH_SERVO_ARM * std::sin((float) m_alpha[i]);

			Li.x = m_leglength[i].x - basePoint.x;
			Li.y = m_leglength[i].y - basePoint.y;
			Li.z = m_leglength[i].z - basePoint.z;

			dist = std::sqrt(Li.x * Li.x + Li.y * Li.y + Li.z * Li.z);

			if (std::abs(LENGTH_SERVO_LEG - dist) < 0.01f) {
				break;
			}
			
			if (dist < LENGTH_SERVO_LEG) {
				max = m_alpha[i];
			}
			else {
				min = m_alpha[i];
			}
			if (max == SERVO_MIN || min == SERVO_MAX) {
				break;
			}
			
			m_alpha[i] = min + (max - min) / 2;
		}
	}
}

void CPlatformAct::calcLengths() {
	float rotMatrix[3][3] = {0};
	getRotationMatrix(rotMatrix);

	for (int i = 0; i < 6; i++) {
		m_leglength[i].x = (rotMatrix[0][0] * m_platformjoint[i].x) 
			+ (rotMatrix[0][1] * m_platformjoint[i].y) 
			+ (rotMatrix[0][2] * m_platformjoint[i].z);
		m_leglength[i].y = (rotMatrix[1][0] * m_platformjoint[i].x) 
			+ (rotMatrix[1][1] * m_platformjoint[i].y)
			+ (rotMatrix[1][2] * m_platformjoint[i].z);
		m_leglength[i].z = (rotMatrix[2][0] * m_platformjoint[i].x) 
			+ (rotMatrix[2][1] * m_platformjoint[i].y) 
			+ (rotMatrix[2][2] * m_platformjoint[i].z);
		m_leglength[i].x += m_translation.x; 
		m_leglength[i].y += m_translation.y; 
		m_leglength[i].z += m_translation.z; 
	}
} 

void CPlatformAct::calcServoPulse() {
	for (int i = 0; i < 6; i++) {
		if (i == INVERSE_SERVO_1 || i == INVERSE_SERVO_2 || i == INVERSE_SERVO_3) {
			m_servo.pulse[i] = constrain<int>(servo_def_pulse[i] - (int) (m_alpha[i] *  SERVO_MULT), MIN_SERVO_PULSE, MAX_SERVO_PULSE);
		}
		else {
			m_servo.pulse[i] = constrain<int>(servo_def_pulse[i] + (int) (m_alpha[i] *  SERVO_MULT), MIN_SERVO_PULSE, MAX_SERVO_PULSE);
		}
	}
}

void CPlatformAct::setPosition() {
	m_servomotor->setPosition(m_servo);
}
