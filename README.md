# IIS2DLPC
Arduino library to support the IIS2DLPC 3D accelerometer

## API

This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:

    IIS2DLPCSensor Accelero(&dev_i2c);
    Accelero.begin();
    Accelero.Enable();

An instance can be created and enabled when the SPI bus is used following the procedure below:

    IIS2DLPCSensor Accelero(&dev_spi, CS_PIN);
    Accelero.begin();
    Accelero.Enable();

The access to the sensor values is done as explained below:

  Read accelerometer.

    int32_t accelerometer[3];
    Accelero.GetAxes(accelerometer);

## Documentation

You can find the source files at
https://github.com/stm32duino/IIS2DLPC

The IIS2DLPC datasheet is available at
https://www.st.com/en/mems-and-sensors/iis2dlpc.html