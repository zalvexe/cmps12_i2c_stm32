# CMPS12 I2C Library for STM32

👾 i made this library last year to access a cmps12 gyro. All addresses i used are based on its [datasheet](https://www.robot-electronics.co.uk/files/cmps12.pdf)    

![image](https://github.com/user-attachments/assets/d5319268-ba6a-44fa-8c6d-03880289c292)

i pushed the entire STM32 base project here so it'll be easier to use.      
❗You can find the cmps12 source file in   ```Core/Src/cmps12_gyro.c``` and header file in ```Core/Src/cmps12_gyro.h```.

👾 In this project i used STM32F407VET6 with I2C3   

<img src="https://github.com/user-attachments/assets/5e809354-1a5f-4f44-8543-20ca709cd8a6" width="75%">

Based on my result on testing this gyro,   
1. it has bearing which you can access and use, it'll give you a very accurate result of rotation (0 to 360 degree)   
2. i tried the gyro z by integrating its acceleration, but it'll give you some error/drifts by time    
3. i give the timeout of 50 and 100 ms. well at first, i set it to ```HAL_MAX_DELAY``` which wasnt a good idea cuz it'll block your executions when i2c failed to do communication with cmps12 

> fun fact: i shorted 1 cmps12 while experimenting with it:"
