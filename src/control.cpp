#include "control.h"
#include "acc_gyro.h"
#include "touchscreen.h"

CControl::CControl(float maxerror, CPlatform* parent) 
	: m_parent(parent), m_maxerror(maxerror) {
    /* Set PID parameters */
    /* Set this for your needs */
    m_pid[0] = PID_PARAM_KP;        /* Proporcional */
    m_pid[1] = PID_PARAM_KI;        /* Integral */
    m_pid[2] = PID_PARAM_KD;        /* Derivative */
	// init mutexes
	m_mtx_kp = xSemaphoreCreateMutex();
	m_mtx_ki = xSemaphoreCreateMutex();
	m_mtx_kd = xSemaphoreCreateMutex();
}
	
void CControl::resetPosition() {
	m_motors.defaultPosition();
}
	
float CControl::getKp(void) { 
	float kp;
	if (xSemaphoreTake(m_mtx_kp, portMAX_DELAY) == pdTRUE) {
		kp = m_pid[0];
		xSemaphoreGive(m_mtx_kp);
	}
	return kp; 
}

void CControl::setKp(float gain) {
	if (xSemaphoreTake(m_mtx_kp, portMAX_DELAY) == pdTRUE) {
		m_pid[0] = gain;
		xSemaphoreGive(m_mtx_kp);
	}
}

float CControl::getKi(void) { 
	float ki;
	if (xSemaphoreTake(m_mtx_ki, portMAX_DELAY) == pdTRUE) {
		ki = m_pid[1];
		xSemaphoreGive(m_mtx_ki);
	}
	return ki; 
}

void CControl::setKi(float gain) {
	if (xSemaphoreTake(m_mtx_ki, portMAX_DELAY) == pdTRUE) {
		m_pid[1] = gain;
		xSemaphoreGive(m_mtx_ki);
	}
}

float CControl::getKd(void) { 
	float kd;
	if (xSemaphoreTake(m_mtx_kd, portMAX_DELAY) == pdTRUE) {
		kd = m_pid[2];
		xSemaphoreGive(m_mtx_kd);
	}
	return kd; 
}

void CControl::setKd(float gain) {
	if (xSemaphoreTake(m_mtx_kd, portMAX_DELAY) == pdTRUE) {
		m_pid[2] = gain;
		xSemaphoreGive(m_mtx_kd);
	}
}

inline arm_pid_instance_f32 CControl::pid_cast (float pid[3]) {
			/* ARM PID Instance, float_32 format */
		arm_pid_instance_f32 PID;
		/* Set PID parameters */
		/* Set this for your needs */
		PID.Kp = pid[0];        /* Proporcional */
		PID.Ki = pid[1];        /* Integral */
		PID.Kd = pid[2];        /* Derivative */		
		/* Initialize PID system, float32_t format */	
		arm_pid_init_f32(&PID, 1);

		setconstraint(PID);
		return PID;
}

void CControl::autotune(void) {
	float dp[3] = {1.0f, 1.0f, 1.0f};
	float rollerror, pitcherror;
	float sum, err, best_err;
	bool newerror = false;
	
	m_pid[0] = 0;
	m_pid[1] = 0;
	m_pid[2] = 0;
	
	sum = (dp[0]+dp[1]+dp[2]);
	
	while(sum > 0.00001f) {
			for (int i=0;i<3;i++) {
				m_pid[i] += dp[i];

				if(newerror == false) {
					getCurrent();
					newerror = true;
				}
				
				getError(&rollerror, &pitcherror);
				actPID(rollerror, pitcherror);

				getCurrent();
				err = getPitchError();
				newerror = true;
				
				if (err < best_err) {
					best_err = err;
					dp[i] *= 1.1f;
				} else {
					m_pid[i] -= 2*dp[i];
					
					getError(&rollerror, &pitcherror);
					actPID(rollerror, pitcherror);

					getCurrent();
					err = getPitchError();
					newerror = true;
					
					if (err < best_err) {
						best_err = err;
						dp[i] *= 1.1f;
					} else {
						m_pid[i] += dp[i];
						dp[i] *= 0.9f;
					}
				}
			}
	}
	
}

inline void  CControl::setconstraint(arm_pid_instance_f32 PID) {
	m_maxgain = arm_pid_f32(&PID, fabs(m_maxerror));
}

void CControl::setmaxerror(float error) {
	m_maxerror = error;
}

void CControl::actPID(float rollerror, float pitcherror) {
	float roll_gain, pitch_gain;
	//ROLL PID CALC
	arm_pid_instance_f32 PID = pid_cast (m_pid);
	
	roll_gain = arm_pid_f32(&PID, fabs(rollerror));
	roll_gain *= ((rollerror < 0.0f)? 1 : -1); //* constrain<float>(roll_gain, 0, m_maxgain);
	//PITCH PID CALC
	pitch_gain = arm_pid_f32(&PID, fabs(pitcherror));
	pitch_gain *= ((pitcherror < 0.0f)? 1 : -1); //* constrain<float>(pitch_gain, 0, m_maxgain);
	
	m_actslope.m_roll = roll_gain * SLOPE_MAX / m_maxgain;
	m_actslope.m_pitch = pitch_gain * SLOPE_MAX / m_maxgain;
	
	m_motors.setRotation(m_actslope);
}

CCounterBalance::CCounterBalance(CPlatform* parent)
	: CControl(400, parent), m_center(400, 240), m_coord(800, 480)
{
	m_mtx_center = xSemaphoreCreateMutex();
	// create task
	xTaskCreate(tCounterBalance, "tCounterBalance", configMINIMAL_STACK_SIZE, this, 7, &m_tcounterbalance);
}

CCounterBalance::~CCounterBalance(void) {
	vTaskDelete(m_tcounterbalance);
}

void CCounterBalance::setCoordCenter(coordinates_t coord) {
	xSemaphoreTake(m_mtx_center, portMAX_DELAY);
	
	m_center = coord;
	
	xSemaphoreGive(m_mtx_center);
	
	setmaxerror(constrain<std::uint16_t> (800U - coord.m_x, 0U, 800U));
}

void CCounterBalance::getCurrent(void) {
	static CTouchScreen* touchscreen = CTouchScreen::getInstance();
	
	touchscreen->getMQCoordinates(&m_coord);
}

void CCounterBalance::tCounterBalance(void* arg) {
	//BaseType_t xStatus;
	CCounterBalance* counterbalance = static_cast<CCounterBalance*>(arg);
	float rollerror, pitcherror;
//	counterbalance->autotune();
	for(;;) {
		counterbalance->getCurrent();
		counterbalance->getError(&rollerror, &pitcherror);
		
		if(rollerror || pitcherror) {
			counterbalance->actPID(rollerror, pitcherror);
		}
		else {
			//Case the ball is in center
		}
	}
}

void CCounterBalance::getError(float* pitch, float* roll) {
	xSemaphoreTake(m_mtx_center, portMAX_DELAY);
	coordinates_t error = m_coord - m_center;
	xSemaphoreGive(m_mtx_center);
	*pitch = (std::int16_t) error.m_x;
	*roll = (std::int16_t) error.m_y;
}

float CCounterBalance::getRollError(void) {
	xSemaphoreTake(m_mtx_center, portMAX_DELAY);
	float error =  m_coord.m_x - m_center.m_x;
	xSemaphoreGive(m_mtx_center);
	
	return error;
}

float CCounterBalance::getPitchError(void) {
	xSemaphoreTake(m_mtx_center, portMAX_DELAY);
	float error =  m_coord.m_y - m_center.m_y;
	xSemaphoreGive(m_mtx_center);

	return error;
}

CStabilizer::CStabilizer(CPlatform* parent)
	:  CControl(SLOPE_MAX, parent), m_resetslope(0,0) {
	// create task
	xTaskCreate(tStabilizer, "tStabilizer", configMINIMAL_STACK_SIZE, this, 7, &m_tstabilizer);	
}

CStabilizer::~CStabilizer(void) {
	vTaskDelete(m_tstabilizer);
}

void CStabilizer::getCurrent(void) {
	static CAccGyro* accgyro = CAccGyro::getInstance();
	
	accgyro->getMQSlope(&m_slope);
}

void CStabilizer::getError(float* pitch, float* roll) {
	slope_t error = m_slope - m_resetslope;
	*pitch = error.m_pitch;
	*roll = error.m_roll;
}

float CStabilizer::getRollError(void) {
	return m_slope.m_roll - m_resetslope.m_roll;
}

float CStabilizer::getPitchError(void) {	
	return m_slope.m_pitch - m_resetslope.m_pitch;
}

void CStabilizer::tStabilizer(void* arg) {
	CStabilizer* stabilizer = static_cast<CStabilizer*>(arg);
	float rollerror, pitcherror;
	
	stabilizer->autotune();
	
	for(;;) {
		stabilizer->getCurrent();
		stabilizer->getError(&rollerror, &pitcherror);
		
		if(rollerror || pitcherror) {
			stabilizer->actPID(rollerror, pitcherror);
		}
		else {
			//Case the ball is in center
		}
	}
}
