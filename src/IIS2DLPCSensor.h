/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __IIS2DLPCSensor_H__
#define __IIS2DLPCSensor_H__

/* Includes -----------------------------------------------------------------*/
#include "Wire.h"
#include "iis2dlpc_reg.h"

/* Defines ------------------------------------------------------------------*/

// SENSITIVITY
#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_2G_LOPOW1_MODE 0.976f /**< Sensitivity value for 2g full scale, Low-power1 mode [mg/LSB] */
#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_2G_OTHER_MODES 0.244f /**< Sensitivity value for 2g full scale, all other modes except Low-power1 [mg/LSB] */

#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_4G_LOPOW1_MODE 1.952f /**< Sensitivity value for 4g full scale, Low-power1 mode [mg/LSB] */
#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_4G_OTHER_MODES 0.488f /**< Sensitivity value for 4g full scale, all other modes except Low-power1 [mg/LSB] */

#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_8G_LOPOW1_MODE 3.904f /**< Sensitivity value for 8g full scale, Low-power1 mode [mg/LSB] */
#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_8G_OTHER_MODES 0.976f /**< Sensitivity value for 8g full scale, all other modes except Low-power1 [mg/LSB] */

#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_16G_LOPOW1_MODE 7.808f /**< Sensitivity value for 16g full scale, Low-power1 mode [mg/LSB] */
#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_16G_OTHER_MODES 1.952f /**< Sensitivity value for 16g full scale, all other modes except Low-power1 [mg/LSB] */

/* Typedefs ------------------------------------------------------------------*/

typedef enum
{
  IIS2DLPC_OK = 0,
  IIS2DLPC_ERROR = -1
} IIS2DLPCStatusTypeDef;

/* To manage events ----------------------------------------------------------*/

// EVENT
typedef struct
{
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
} IIS2DLPC_Event_Status_t;

/*------------------------------------------------------------------------------*/

// OPERATING MODE
typedef enum
{
  IIS2DLPC_HIGH_PERFORMANCE_MODE,
  IIS2DLPC_LOW_POWER_MODE4,
  IIS2DLPC_LOW_POWER_MODE3,
  IIS2DLPC_LOW_POWER_MODE2,
  IIS2DLPC_LOW_POWER_MODE1
} IIS2DLPC_Operating_Mode_t;

// NOISE
typedef enum
{
  IIS2DLPC_LOW_NOISE_DISABLE,
  IIS2DLPC_LOW_NOISE_ENABLE
} IIS2DLPC_Low_Noise_t;

/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of an IIS2DLPC.
 */

class IIS2DLPCSensor
{
public:
  /* default SA0=1, to change into SA0=0 must pass address =  IIS2DLPC_I2C_ADD_L*/
  IIS2DLPCStatusTypeDef begin(uint8_t address = IIS2DLPC_I2C_ADD_H, TwoWire &i2c = Wire);
  IIS2DLPCStatusTypeDef end();
  IIS2DLPCStatusTypeDef ReadID(uint8_t *Id);
  IIS2DLPCStatusTypeDef Enable();
  IIS2DLPCStatusTypeDef Disable();
  IIS2DLPCStatusTypeDef GetSensitivity(float *sensitivity);
  IIS2DLPCStatusTypeDef GetOutputDataRate(float *odr);
  IIS2DLPCStatusTypeDef SetOutputDataRate(float odr);
  IIS2DLPCStatusTypeDef SetOutputDataRate_With_Mode(float odr, IIS2DLPC_Operating_Mode_t Mode, IIS2DLPC_Low_Noise_t Noise);
  IIS2DLPCStatusTypeDef SetOutputDataRate_When_Enable(float Odr, IIS2DLPC_Operating_Mode_t Mode, IIS2DLPC_Low_Noise_t Noise);
  IIS2DLPCStatusTypeDef SetOutputDataRate_When_Disable(float Odr, IIS2DLPC_Operating_Mode_t Mode, IIS2DLPC_Low_Noise_t Noise);
  IIS2DLPCStatusTypeDef GetFullScale(int32_t *fullscale);
  IIS2DLPCStatusTypeDef SetFullScale(int32_t fullscale);
  IIS2DLPCStatusTypeDef GetAxes(int32_t *acceleration);
  IIS2DLPCStatusTypeDef GetAxesRaw(int16_t *value);
  IIS2DLPCStatusTypeDef ReadReg(uint8_t reg, uint8_t *data);
  IIS2DLPCStatusTypeDef WriteReg(uint8_t reg, uint8_t data);
  IIS2DLPCStatusTypeDef SetSelfTest(uint8_t val);
  IIS2DLPCStatusTypeDef GetSelfTest(iis2dlpc_st_t *val);
  IIS2DLPCStatusTypeDef GetDRDYStatus(uint8_t *status);

  IIS2DLPCStatusTypeDef GetEventStatus(IIS2DLPC_Event_Status_t *Status);
  IIS2DLPCStatusTypeDef SetInterruptLatch(uint8_t Status);
  IIS2DLPCStatusTypeDef SetINT1Drdy(uint8_t Status);
  IIS2DLPCStatusTypeDef SetINT2Drdy(uint8_t Status);
  IIS2DLPCStatusTypeDef SetDrdyMode(uint8_t Status);

  // Free Fall
  IIS2DLPCStatusTypeDef EnableFreeFallDetection();
  IIS2DLPCStatusTypeDef DisableFreeFallDetection();
  IIS2DLPCStatusTypeDef SetFreeFallThreshold(uint8_t Threshold);
  IIS2DLPCStatusTypeDef SetFreeFallDuration(uint8_t Duration);

  // Wake Up
  IIS2DLPCStatusTypeDef EnableWakeUpDetection();
  IIS2DLPCStatusTypeDef DisableWakeUpDetection();
  IIS2DLPCStatusTypeDef SetWakeUpThreshold(uint8_t Threshold);
  IIS2DLPCStatusTypeDef SetWakeUpDuration(uint8_t Duration);

  // Sigle Tap
  IIS2DLPCStatusTypeDef EnableSingleTapDetection();
  IIS2DLPCStatusTypeDef DisableSingleTapDetection();

  // Double Tap
  IIS2DLPCStatusTypeDef EnableDoubleTapDetection();
  IIS2DLPCStatusTypeDef DisableDoubleTapDetection();

  IIS2DLPCStatusTypeDef SetTapThreshold(uint8_t Threshold);
  IIS2DLPCStatusTypeDef SetTapShockTime(uint8_t Time);
  IIS2DLPCStatusTypeDef SetTapQuietTime(uint8_t Time);
  IIS2DLPCStatusTypeDef SetTapDurationTime(uint8_t Time);

  // 6D_Orientation
  IIS2DLPCStatusTypeDef Enable6DOrientation();
  IIS2DLPCStatusTypeDef Disable6DOrientation();
  IIS2DLPCStatusTypeDef Set6DOrientationThreshold(uint8_t Threshold);
  IIS2DLPCStatusTypeDef Get6DOrientationThreshold(uint8_t *Threshold);
  IIS2DLPCStatusTypeDef Get6DOrientationXL(uint8_t *XLow);
  IIS2DLPCStatusTypeDef Get6DOrientationXH(uint8_t *XHigh);
  IIS2DLPCStatusTypeDef Get6DOrientationYL(uint8_t *YLow);
  IIS2DLPCStatusTypeDef Get6DOrientationYH(uint8_t *YHigh);
  IIS2DLPCStatusTypeDef Get6DOrientationZL(uint8_t *ZLow);
  IIS2DLPCStatusTypeDef Get6DOrientationZH(uint8_t *ZHigh);

  /**
   * @brief Utility function to read data.
   * @param  pBuffer: pointer to data to be read.
   * @param  RegisterAddr: specifies internal address register to be read.
   * @param  NumByteToRead: number of bytes to be read.
   * @retval 0 if ok, an error code otherwise.
   */
  uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
  {
    if (dev_i2c)
    {
      dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
      dev_i2c->write(RegisterAddr);
      dev_i2c->endTransmission(false);

      dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t)NumByteToRead);

      int i = 0;
      while (dev_i2c->available())
      {
        pBuffer[i] = dev_i2c->read();
        i++;
      }

      return 0;
    }

    return 1;
  }

  /**
   * @brief Utility function to write data.
   * @param  pBuffer: pointer to data to be written.
   * @param  RegisterAddr: specifies internal address register to be written.
   * @param  NumByteToWrite: number of bytes to write.
   * @retval 0 if ok, an error code otherwise.
   */
  uint8_t IO_Write(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
  {
    if (dev_i2c)
    {
      dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

      dev_i2c->write(RegisterAddr);
      for (uint16_t i = 0; i < NumByteToWrite; i++)
      {
        dev_i2c->write(pBuffer[i]);
      }

      dev_i2c->endTransmission(true);

      return 0;
    }

    return 1;
  }

private:
  IIS2DLPCStatusTypeDef Init();
  /*Connection*/
  TwoWire *dev_i2c;

  /*Configuration*/
  uint8_t address;

  uint8_t acc_is_enabled;

  float acc_odr;
  IIS2DLPC_Operating_Mode_t acc_operating_mode;
  IIS2DLPC_Low_Noise_t acc_low_noise;

  iis2dlpc_ctx_t reg_ctx;
};

#ifdef __cplusplus
extern "C"
{
#endif
  int32_t IIS2DLPC_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
  int32_t IIS2DLPC_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif

#endif
