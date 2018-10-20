#include "acc_gyro.h"
#include <stm32f4xx.h>
/* Include ARM math */
#include <arm_math.h>
#include "platform.h"
float radToDeg(float radians) { return radians * 180.0f / PI; }
float degToRad(float degree) { return degree * PI / 180.0f; }
extern "C" void DMA1_Stream0_IRQHandler(void) {
	static MPU6050_t mpu6050_data;
	static BaseType_t xHigherPriorityTaskWoken;
	static CAccGyro* slope = CAccGyro::getInstance();
	
	if (DMA_GetFlagStatus(DMA1_Stream0,DMA_IT_TCIF0)) {
		/* Clear transmission complete flag */
		DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0);
		I2C_DMACmd(I2C1, DISABLE);
		/* Send I2C1 STOP Condition */
		I2C_GenerateSTOP(I2C1, ENABLE);
		/* Disable DMA channel*/
		DMA_Cmd(DMA1_Stream0, DISABLE);		
		
		mpu6050_data = slope->getRawMeasure();
		slope->sendRawMeasure(mpu6050_data, &xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

CAccGyro::CAccGyro(CPlatform* parent): m_parent(parent), m_i2c(I2C_ADDRESS) {
	
	m_mq_measure = xQueueCreate(10, sizeof(MPU6050_t));
	m_mq_slope = xQueueCreate(10, sizeof(MPU6050_t));
	m_mtx_slope = xSemaphoreCreateMutex();
	m_sem_startacquisition = xSemaphoreCreateBinary();
	
	if(setup()) {
		configRCC();
		configDMA();
		configNVIC();
		
		xTaskCreate(tSlopeMeasure, "tSlopeMeasure", configMINIMAL_STACK_SIZE, this, 6, &m_tslopemeasure);
	}
}

void CAccGyro::configRCC(void) {
	/* --------------------------- System Clocks Configuration -----------------*/

    /* DMA1 clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
}


void CAccGyro::configDMA(void) {
	DMA_InitTypeDef  DMA_InitStructure;

	DMA_DeInit(DMA1_Stream0); //reset DMA1 channe1 to default values;

	DMA_InitStructure.DMA_BufferSize = BUFFSIZE;	//number of bytes to be transfered
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	//Location assigned to peripheral register will be source
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&buffer; //variable to store data
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//destination memory data size = 8bit
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//automatic memory increment enable for memory
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	//setting normal mode (non circular)
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(I2C1->DR); //address of data reading register of I2C1
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//source peripheral data size = 8bit
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //automatic memory increment disable for peripheral
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;	//medium priority
	
	DMA_Init(DMA1_Stream0, &DMA_InitStructure);

}

void CAccGyro::configNVIC(void) {
	NVIC_InitTypeDef NVIC_InitStruct;
	 
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x06;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x06;
	NVIC_Init(&NVIC_InitStruct);
	DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
}

bool CAccGyro::setup(void) {
	std::uint8_t temp;
	
	/* Wakeup MPU6050 */
	if(!m_i2c.write(PWR_MGMT_1, 0x00)) {
		return false;
	}
	
	/* Config accelerometer */
	temp = m_i2c.read(ACCEL_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)Accelerometer_4G;
	if(!m_i2c.write(ACCEL_CONFIG, temp)) {
		return false;
	}
	
	/* Config gyroscope */
	temp = m_i2c.read(GYRO_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)Gyroscope_250s;
	if(!m_i2c.write(GYRO_CONFIG, temp)) {
		return false;
	}
	
	return true;
}

bool CAccGyro::check(void){

	if(m_i2c.read(WHO_AM_I) == I_AM)
		return true;

	return false;
}

void CAccGyro::setlowpower(void) {
	//Cycle mode (when in sleep mode, ZMOT_DUR_step=~10s! however, not relevant)
	m_i2c.write(PWR_MGMT_2, 0x47);//5Hz, Gyro Disabled
	m_i2c.write(PWR_MGMT_1, 0x28);//Enter Cycle Mode
}

bool CAccGyro::readAll() {
	
		/* Disable DMA channel*/
	DMA_Cmd(DMA1_Stream0, DISABLE);
	/* Set current data number again to 15 for MPu6050, only possible after disabling the DMA channel */
	DMA_SetCurrDataCounter(DMA1_Stream0, 14);
	if(!m_i2c.start(TransmitterMode))
		return false;
	if(!m_i2c.write(ACCEL_XOUT_H))
		return false;
	m_i2c.stop();
	
	I2C_AcknowledgeConfig(I2C1, ENABLE);	
	/* Enable DMA NACK automatic generation */
	I2C_DMALastTransferCmd(I2C1, ENABLE);					//Note this one, very important

	if(!m_i2c.start(ReceiverMode))
		return false;
	/* Start DMA to receive data from I2C */
	DMA_Cmd(DMA1_Stream0, ENABLE);
	I2C_DMACmd(I2C1, ENABLE);
	
	return true;
}

inline MPU6050_t CAccGyro::getRawMeasure(void) {
	MPU6050_t mpu6050_data;
	
	mpu6050_data.Accelerometer_X = ((short)buffer[0] << 8) + buffer[1];		// Accelerometer value X axis */
	mpu6050_data.Accelerometer_Y = ((short)buffer[2] << 8) + buffer[3];		// Accelerometer value Y axis */
	mpu6050_data.Accelerometer_Z = ((short)buffer[4] << 8) + buffer[5]; 	// Accelerometer value Z axis */
	mpu6050_data.Temperature = ((short)buffer[6] << 8) + buffer[7];
	//mpu6050_data.Temperature = (float) ((mpu6050_data.Temperature / 340U) + 36.53f);
	mpu6050_data.Gyroscope_X = ((short)buffer[8] << 8) + buffer[9];     	// Gyroscope value X axis */
	mpu6050_data.Gyroscope_Y = ((short)buffer[10] << 8) + buffer[11];    	// Gyroscope value Y axis */
	mpu6050_data.Gyroscope_Z = ((short)buffer[12] << 8) + buffer[13];    	// Gyroscope value Z axis */
	
	return (MPU6050_t) mpu6050_data;
}

inline void CAccGyro::sendRawMeasure(MPU6050_t& mpu6050_data, BaseType_t* xHigherPriorityTaskWoken) {
	static BaseType_t xStatus;
	xStatus = xQueueSendToBackFromISR(m_mq_measure, &mpu6050_data, xHigherPriorityTaskWoken);
	if (xStatus != pdPASS) {
		
	}
}

slope_t CAccGyro::getAccelAngles(MPU6050_t data)
{
	slope_t motion;
	motion.m_roll = /*radToDeg(*/ atan2((float)data.Accelerometer_Y, sqrt((float) pow((float)data.Accelerometer_Z,2.0f)+ (float) pow((float)data.Accelerometer_X,2.0f)));//);
	motion.m_pitch = /*radToDeg(*/ atan2((float)-data.Accelerometer_X, sqrt((float) pow((float)data.Accelerometer_Z,2.0f)+ (float) pow((float)data.Accelerometer_Y,2.0f)));//);	
	return motion;
}
slope_t CAccGyro::getSlope(void) {
	slope_t slope;
	xSemaphoreTake(m_mtx_slope, portMAX_DELAY);
	
	slope = m_slope;
	
	xSemaphoreGive(m_mtx_slope);
	
	return slope;
}


void CAccGyro::tSlopeMeasure(void* arg) {
	BaseType_t xStatus;

	CAccGyro* slope = static_cast<CAccGyro*>(arg);
	slope_t slope_data, slope_deg;
	MPU6050_t mpu6050_data;

	/* Block for 10ms. */
	const TickType_t xDelay = ACCGYRO_PRD_MS / portTICK_PERIOD_MS;
	
	for(;;) {
		if (xSemaphoreTake(slope->m_sem_startacquisition, portMAX_DELAY) == pdTRUE) {
			if(slope->readAll()) {
				xStatus = xQueueReceive(slope->m_mq_measure, &mpu6050_data, portMAX_DELAY);
				if (xStatus != pdPASS) { // handle error
					// code here
				}
				xSemaphoreTake(slope->m_mtx_slope, portMAX_DELAY);
				slope->m_slope = slope->getAccelAngles(mpu6050_data);
				slope_data = slope->m_slope;
				xSemaphoreGive(slope->m_mtx_slope);
				slope_data.m_roll = degToRad(static_cast<int>(radToDeg(slope_data.m_roll)));
				slope_data.m_pitch = degToRad(static_cast<int>(radToDeg(slope_data.m_pitch)));
				slope_deg.m_roll = radToDeg(slope_data.m_roll);
				slope_deg.m_pitch = radToDeg(slope_data.m_pitch);
				if (slope->m_parent->isBalancing() == false) { 
					xStatus = xQueueSendToBack(slope->m_mq_slope, &slope_data, 0);
					if (xStatus != pdPASS) {
						// handle error
					}
				}
			}
			xSemaphoreGive(slope->m_sem_startacquisition);
			vTaskDelay(xDelay);
		}
	}
}
