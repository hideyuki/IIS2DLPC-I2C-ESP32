# IIS2DLPC
Arduino library to support the IIS2DLPC 3D accelerometer

## Example

```
#include <Arudino.h>
#include <IIS2DLPCSensor.h>

// Accelerometer sensor object
IIS2DLPCSensor acc;

void setup()
{
  // Initialize I2C
  Wire.begin();
  
  // Initialize LIS2DH12 Accelerometer
  if (acc.begin() == IIS2DLPC_ERROR)
  {
    Serial.println("Accelerometer not detected. Check address jumper and wiring. Restarting...");
    delay(3000);
    ESP.restart();
  }

  // Enable accelerometer
  acc.Enable();
}

void loop()
{
  int32_t data[3];
  acc.GetAxes(data);
  
  Serial.printf("x: %6.2f, y: %6.2f, z: %6.2f\n", data[0], data[1], data[2]);
  
  delay(1000);
}

```

## Documentation

You can find the source files at
https://github.com/stm32duino/IIS2DLPC

The IIS2DLPC datasheet is available at
https://www.st.com/en/mems-and-sensors/iis2dlpc.html