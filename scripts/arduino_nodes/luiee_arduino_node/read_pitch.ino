void read_pitch()
{
  fabo_9axis.readAccelXYZ(&ax,&ay,&az);
  
  Accel_x = ax-bias_x; // Unbiased acceleration along x axis ;
  Accel_y = ay-bias_y; // Unbiased acceleration along y axis ;
  Accel_z = az-bias_z; // Unbiased acceleration along z axis ;
  
  /* Determine pitch and roll angles*/
  
  Accel_pitch = atan(Accel_x/sqrt(Accel_y*Accel_y+Accel_z*Accel_z))* RAD_TO_DEG; //asin((float)Accel_x/Accel_total_vector) * RAD_TO_DEG;                  //Calculate the pitch angle
  Accel_roll = - atan(Accel_y/sqrt(Accel_x*Accel_x+Accel_z*Accel_z))* RAD_TO_DEG; // asin((float)Accel_y/Accel_total_vector) * RAD_TO_DEG;                   //Calculate the roll angle
  
  /* Adjustments for pitch and roll */
  Accel_roll = Accel_pitch + 0.1;
  Accel_pitch  = Accel_roll  - 0.1;
}
