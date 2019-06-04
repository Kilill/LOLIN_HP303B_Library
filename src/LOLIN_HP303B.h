#ifndef __LOLIN_HP303B_H
#define __LOLIN_HP303B_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "I2Cdev.h"
//#include <Wire.h>
#include <SPI.h>
#include "util/hp303b_consts.h"

class LOLIN_HP303B
{
public:
	enum Oversample:uint8_t {
		OSample1   = 0b000,
		OSample2   = 0b001,
		OSample4   = 0b010,
		OSample8   = 0b011,
		OSample16  = 0b100,
		OSample32  = 0b101,
		OSample64  = 0b110,
		OSample128 = 0b111
	};
	enum TempRate:uint8_t {
		Rate1		= 0b000,
		Rate2	  	= 0b001<<4,
		Rate4		= 0b010<<4,
		Rate8		= 0b011<<4,
		Rate16		= 0b100<<4,
		Rate32		= 0b101<<4,
		Rate64	 	= 0b110<<4,
		Rate129		= 0b111<<4
	};
	enum TmpExt:uint8_t {
		TmpInternal	= 0,
		TmpExternal = 0b10000000
	};

	// Measurement configuration register
	enum MEAS_ModeType {
		StandBy				= 0,
		SinglePressure		= 0b001,
		SingelTemperature	= 0b010,
		ContinousPressure	= 0b101,
		ContiousTemperature	= 0b110,
		ContinousBoth		= 0b111,
	};
	enum MEAS_CFG_Bits	{
		MEAS_CRL		= 0b111,
		PRS_RDY			= 1<<4,
		TMP_RDY			= 1<<5,
		SENSOR_RDY		= 1<<6,
		COEF_RDY		=1 <<7
	};

	enum CFG_REG_Bits {
		SPI_MODE		= 1,		///< SPI Mode, 					0 = SPI bus 1 = I2C
		FIFO_EN			= 1<<1,		///< Enable Fifo 				0 = Fifo disabled 1 = Fifo enabled
		P_SHIFT			= 1<<2,		///< Pressure shift 			0 = No Shift, 1 = Shift Pressure result
		T_SHIFT			= 1<<3,		///< Temperature shift			0 = No shift 1 = Shift temperature right
		INT_PRS			= 1<<4,		///< Pressure interupt enable	0 = Disable 1 = Enable
		INT_TMP			= 1<<5,		///< Temp interupt enable		0 = Disable 1 = Enable
		INT_FIFO		= 1<<6,		///< Fifo full interupt enable	0 = Disable 1 = Enable
		INT_HL			= 1<<6,		///< Interupt active level		0 = Active Low 1 = Active High
	};
  //constructor
  LOLIN_HP303B(void);
  //destructor
  ~LOLIN_HP303B(void);
  //begin
#ifndef  I2CDEV_IMPLEMENTATION
  void begin(TwoWire &bus, uint8_t slaveAddress);
#endif
  void begin(uint8_t slaveAddress=HP303B__STD_SLAVE_ADDRESS);
  void begin(SPIClass &bus, int32_t chipSelect);
  void begin(SPIClass &bus, int32_t chipSelect, uint8_t threeWire);
  //end
  void end(void);

  //general
  uint8_t getProductId(void);
  uint8_t getRevisionId(void);

  //Idle Mode
  int16_t standby(void);

  //Command Mode
  int16_t measureTempOnce(float &result);
  int16_t measureTempOnce(float &result, uint8_t oversamplingRate);
  int16_t startMeasureTempOnce(void);
  int16_t startMeasureTempOnce(uint8_t oversamplingRate);
  int16_t measurePressureOnce(float &result);
  int16_t measurePressureOnce(float &result, uint8_t oversamplingRate);
  int16_t startMeasurePressureOnce(void);
  int16_t startMeasurePressureOnce(uint8_t oversamplingRate);
  int16_t getSingleResult(float &result);

  //Background Mode
  int16_t startMeasureTempCont(uint8_t measureRate, uint8_t oversamplingRate);
  int16_t startMeasurePressureCont(uint8_t measureRate, uint8_t oversamplingRate);
  int16_t startMeasureBothCont(uint8_t tempMr, uint8_t tempOsr, uint8_t prsMr, uint8_t prsOsr);
  int16_t getContResults(int32_t *tempBuffer, uint8_t &tempCount, int32_t *prsBuffer, uint8_t &prsCount);

  //Interrupt Control
  int16_t setInterruptPolarity(uint8_t polarity);
  int16_t setInterruptSources(uint8_t fifoFull, uint8_t tempReady, uint8_t prsReady);
  int16_t getIntStatusFifoFull(void);
  int16_t getIntStatusTempReady(void);
  int16_t getIntStatusPrsReady(void);

  //function to fix a hardware problem on some devices
  int16_t correctTemp(void);

private:
  //scaling factor table
  static const int32_t scaling_facts[HP303B__NUM_OF_SCAL_FACTS];

  //enum for operating mode
  enum Mode
  {
    IDLE = 0x00,
    CMD_PRS = 0x01,
    CMD_TEMP = 0x02,
    INVAL_OP_CMD_BOTH = 0x03,  //invalid
    INVAL_OP_CONT_NONE = 0x04, //invalid
    CONT_PRS = 0x05,
    CONT_TMP = 0x06,
    CONT_BOTH = 0x07
  };
  Mode m_opMode;

  //flags
  bool m_initFail;
  uint8_t m_productID;
  uint8_t m_revisionID;

  //settings
  uint8_t m_tempMr;
  uint8_t m_tempOsr;
  uint8_t m_prsMr;
  uint8_t m_prsOsr;
  uint8_t m_tempSensor;

  //compensation coefficients
  int32_t m_c0Half;
  int32_t m_c1;
  int32_t m_c00;
  int32_t m_c10;
  int32_t m_c01;
  int32_t m_c11;
  int32_t m_c20;
  int32_t m_c21;
  int32_t m_c30;
  //last measured scaled temperature
  //(necessary for pressure compensation)
  double m_lastTempScal;

  enum BusType {
	  SPIBus,
	  I2CBus
  };
  //bus specific
  BusType m_SpiI2c;

  //used for I2C

#ifndef  I2CDEV_IMPLEMENTATION
  TwoWire *m_i2cbus;
#endif
  uint8_t m_slaveAddress;

  //used for SPI
  SPIClass *m_spibus;
  int32_t m_chipSelect;
  uint8_t m_threeWire;

  //measurement
  void init(void);
  int16_t readcoeffs(void);
  int16_t setOpMode(uint8_t background, uint8_t temperature, uint8_t pressure);
  int16_t setOpMode(uint8_t opMode);
  int16_t configTemp(uint8_t temp_mr, uint8_t temp_osr);
  int16_t configPressure(uint8_t prs_mr, uint8_t prs_osr);
  uint16_t calcBusyTime(uint16_t temp_rate, uint16_t temp_osr);
  int16_t getTemp(float &result);
  int16_t getPressure(float &result);
  int16_t getFIFOvalue(int32_t *value);
  float calcTemp(int32_t raw);
  float calcPressure(int32_t raw);

  //bus specific
  int16_t readByte(uint8_t regAddress);
  int16_t readByteSPI(uint8_t regAddress);
  int16_t readBlock(uint8_t regAddress, uint8_t length, uint8_t *buffer);
  int16_t readBlockSPI(uint8_t regAddress, uint8_t length, uint8_t *readbuffer);
  int16_t writeByte(uint8_t regAddress, uint8_t data);
  int16_t writeByte(uint8_t regAddress, uint8_t data, bool check);
  int16_t writeByteSpi(uint8_t regAddress, uint8_t data, bool check);
  int16_t writeByteBitfield(uint8_t data, uint8_t regAddress, uint8_t mask, uint8_t shift);
  int16_t writeByteBitfield(uint8_t data, uint8_t regAddress, uint8_t mask, uint8_t shift, bool check);
  int16_t readByteBitfield(uint8_t regAddress, uint8_t mask, uint8_t shift);
};

#endif
