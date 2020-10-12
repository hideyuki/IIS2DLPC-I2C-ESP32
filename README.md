# IIS2DLPC
Arduino library to support the IIS2DLPC 3D accelerometer

## API

This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensor:
	
	dev_i2c = new TwoWire(I2C_SDA, I2C_SCL);
	dev_i2c->begin();

For SPI it is then required to create a SPI interface before accessing to the sensor:

	dev_spi = new SPIClass(SPI_MOSI, SPI_MISO, SPI_SCK);
	dev_spi->begin();

An instance can be created and enbaled when the I2C bus is used following the procedure below:

	Accelero = new IIS2DLPCSensor(dev_i2c);
	Accelero->Enable();

An instance can be created and enbaled when the SPI bus is used following the procedure below:

	Accelero = new IIS2DLPCSensor(dev_spi);
	Accelero->Enable();

The access to the sensor values is done as explained below:

	Read accelerometer.

	Accelero->GetAxes(&accelerometer);

## Documentation

You can find the source files at
https://github.com/stm32duino/IIS2DLPC

The IIS2DLPC datasheet is available at
https://www.st.com/en/mems-and-sensors/iis2dlpc.html