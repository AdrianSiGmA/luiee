
/* 
 This function obtains magnetometer readings 
 and applies a low pass filter in order to reduce the noise.
 
 TODO: ++Implement X and Y axes
       -+Implement tilt compensation
       ++Implement hard iron calibration routine
       ++Implement soft iron calibration routine
       ++Add the factory compensation values
       --Add declination to the heading  Heading += Declination; -> Geographic North
         Determine if the alhorithm returns sun position calculated from the magnetic
         or geographic north


 Factory compensation values : ASAX = 1.21094; ASAY = 1.21484; ASAZ=1.16797;

 References: 
 
  [1]Applications of Magnetic Sensors for   
  Low Cost Compass Systems
  Michael J. Caruso; Honeywell, SSEC.
  
*/

/* 
   Inputs and outputs for the functions: 
   
   Mag_hard_iron_calibration  -> [Offset_x Offset_y] -> Mag_soft_iron_calibration
   Mag_soft_iron_calibration  -> [theta_rot]         -> Mag_scaling
   Mag_scaling                -> [Mag_x_scale Mag_y_scale Offset_x_final Offset_y_final] -> Mag_sensor_read

   
*/
void magnetic_sensor_read() {  
  //Serial.print("Magnetic sensor read...");
 /* Start the sensor. Posibly interferes with the GPS. */
  fabo_9axis.begin();
  
 /* Read the magnetic field sensor */
  fabo_9axis.readMagnetXYZ(&mx,&my,&mz);

 /* Apply factory compensation values */
  SmoothData_x = mx * ASAX;
  SmoothData_y = my * ASAY;

 /* Apply rotion matrix for xy axis */

 /* Apply scaling and offset the reading to the origin */
  SmoothData_x = SmoothData_x - Offset_x;
  SmoothData_y = SmoothData_y - Offset_y;
  //delay(250);

  int times_read = 20;
  /* Read the magnetic field sensor 50 times */
  for (int ii=0; ii<times_read; ii++)
  {

   /* Read raw data for x, y and z axes and include factory 
      compensation values */
    fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
    RawData_x = mx * ASAX;
    RawData_y = my * ASAY;
  
   /* Apply factory compensation values */
    RawData_x = mx * ASAX;
    RawData_y = my * ASAY;
 
   /* Apply scaling and offset the reading to the origin */
    RawData_x = RawData_x - Offset_x;
    RawData_y = RawData_y - Offset_y;
   
   /* Low pass filters for x, y and z axes with LPF_Beta = 0.0025  */
    SmoothData_x = SmoothData_x - (LPF_Beta * (SmoothData_x - RawData_x));
    SmoothData_y = SmoothData_y - (LPF_Beta * (SmoothData_y - RawData_y));
    
  }
  /* Determine the heading */
  Heading = atan2(SmoothData_x, SmoothData_y) * RAD_TO_DEG;  // Magnetic North add declination
  LM_data_flag == 0;
  /** Calculate the arithmetic average of the heading for 50 readings. **/
  //Heading = Heading / times_read; // filtered
}
