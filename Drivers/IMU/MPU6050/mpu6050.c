
#include "mpu6050.h"

/* Internal register write */
static HAL_StatusTypeDef MPU6050_WriteReg(
    MPU6050_Handle_t *imu,
    uint8_t reg,
    uint8_t value
)
{
    return HAL_I2C_Mem_Write(
        imu->hi2c,
        imu->dev_address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        100
    );
}

/* Internal register read */
static HAL_StatusTypeDef MPU6050_ReadReg(
    MPU6050_Handle_t *imu,
    uint8_t reg,
    uint8_t *value
)
{
    return HAL_I2C_Mem_Read(
        imu->hi2c,
        imu->dev_address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1,
        100
    );
}

/* Initialize MPU6050 */
HAL_StatusTypeDef MPU6050_Init(
    MPU6050_Handle_t *imu,
    I2C_HandleTypeDef *hi2c,
    uint8_t address
)
{
    uint8_t who_am_i;

    if (imu == NULL || hi2c == NULL)
        return HAL_ERROR;

    imu->hi2c = hi2c;
    imu->dev_address = (uint16_t)(address << 1);

    imu->transfer_busy = 0;
    imu->data_ready = 0;
    imu->error = 0;

    /* Check device availability */
    if (HAL_I2C_IsDeviceReady(
            imu->hi2c,
            imu->dev_address,
            5,
            100) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Verify device identity */
    if (MPU6050_ReadReg(
            imu,
            MPU6050_REG_WHO_AM_I,
            &who_am_i) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (who_am_i != MPU6050_WHO_AM_I_VAL)
        return HAL_ERROR;

    /* Reset sensor */
    if (MPU6050_WriteReg(
            imu,
            MPU6050_REG_PWR_MGMT_1,
            0x80) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(100);

    /* Wake up; use PLL with X gyro reference */
    if (MPU6050_WriteReg(
            imu,
            MPU6050_REG_PWR_MGMT_1,
            0x01) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Sample rate: 1000 / (1 + 1) = 500 Hz */
    if (MPU6050_WriteReg(
            imu,
            MPU6050_REG_SMPLRT_DIV,
            0x01) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* DLPF_CFG = 3: approximately 42 Hz gyro bandwidth */
    if (MPU6050_WriteReg(
            imu,
            MPU6050_REG_CONFIG,
            0x03) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Gyroscope: ±2000 degrees/second */
    if (MPU6050_WriteReg(
            imu,
            MPU6050_REG_GYRO_CONFIG,
            0x18) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Accelerometer: ±8g */
    if (MPU6050_WriteReg(
            imu,
            MPU6050_REG_ACCEL_CONFIG,
            0x10) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Interrupt: active-high, push-pull, 50us pulse */
    if (MPU6050_WriteReg(
            imu,
            MPU6050_REG_INT_PIN_CFG,
            0x00) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Enable Data Ready interrupt */
    if (MPU6050_WriteReg(
            imu,
            MPU6050_REG_INT_ENABLE,
            0x01) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* Start 14-byte burst read using DMA */
HAL_StatusTypeDef MPU6050_Read_DMA(MPU6050_Handle_t *imu)
{
    HAL_StatusTypeDef status;

    if (imu == NULL || imu->hi2c == NULL)
        return HAL_ERROR;

    if (imu->transfer_busy)
        return HAL_BUSY;

    imu->transfer_busy = 1;
    imu->data_ready = 0;
    imu->error = 0;

    status = HAL_I2C_Mem_Read_DMA(
        imu->hi2c,
        imu->dev_address,
        MPU6050_REG_ACCEL_XOUT_H,
        I2C_MEMADD_SIZE_8BIT,
        imu->buffer,
        MPU6050_DATA_LENGTH
    );

    if (status != HAL_OK)
    {
        imu->transfer_busy = 0;
        imu->error = 1;
    }

    return status;
}

/* Called after successful DMA reception */
void MPU6050_RxCompleteCallback(MPU6050_Handle_t *imu)
{
    imu->transfer_busy = 0;
    imu->data_ready = 1;
}

/* Called after I2C/DMA error */
void MPU6050_ErrorCallback(MPU6050_Handle_t *imu)
{
    imu->transfer_busy = 0;
    imu->error = 1;
}

/* Convert two bytes into signed 16-bit value */
static int16_t MPU6050_ConvertInt16(
    uint8_t high,
    uint8_t low
)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

/* Parse raw sensor data and convert to physical units */
void MPU6050_ParseData(
    const MPU6050_Handle_t *imu,
    MPU6050_RawData_t *raw,
    MPU6050_Data_t *data
)
{
    const uint8_t *b = imu->buffer;

    raw->accel_x = MPU6050_ConvertInt16(b[0], b[1]);
    raw->accel_y = MPU6050_ConvertInt16(b[2], b[3]);
    raw->accel_z = MPU6050_ConvertInt16(b[4], b[5]);

    raw->temperature = MPU6050_ConvertInt16(b[6], b[7]);

    raw->gyro_x = MPU6050_ConvertInt16(b[8], b[9]);
    raw->gyro_y = MPU6050_ConvertInt16(b[10], b[11]);
    raw->gyro_z = MPU6050_ConvertInt16(b[12], b[13]);

    /* ±8g range: 4096 LSB/g */
    data->accel_x_g = raw->accel_x / 4096.0f;
    data->accel_y_g = raw->accel_y / 4096.0f;
    data->accel_z_g = raw->accel_z / 4096.0f;

    /* ±2000 dps range: 16.4 LSB/(degree/second) */
    data->gyro_x_dps = raw->gyro_x / 16.4f;
    data->gyro_y_dps = raw->gyro_y / 16.4f;
    data->gyro_z_dps = raw->gyro_z / 16.4f;

    /* MPU6050 temperature conversion */
    data->temperature_c =
        (raw->temperature / 340.0f) + 36.53f;
}
