/**
  This function calibrates the magnetic field sensor. 
  The process includes two steps: Hard iron calibration and soft iron calibration.

  Hard iron calibration compensates the bias that is present in the sensor offseting
  the measurements back to the origin. The magnetic field 
**/


void mag_hard_iron_calibration(){
  fabo_9axis.begin();

  /* Initial values for the low pass filter fo x and y axes. 
     Readings are multiplied by the factory correction values. */
  fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
  //Serial.println(mx);
  SmoothData_x = mx * ASAX;  
  SmoothData_y = my * ASAY;

  auto_calibration(100);
  auto_calibration(-100);
  auto_calibration(-100);
  auto_calibration(100);
  
  /** Perform 1st hard iron calibration. These two results 
      are used for soft iron calibration. **/
  Offset_x = (max_x+min_x)/2;
  Offset_y = (max_y+min_y)/2;
  LM_calibration_flag = 0;
  LM_data_flag = 1;
  sensors_data[n_sensors-3] = Heading;
  sensors_data[n_sensors-2] = 0;
  sensors_data[n_sensors-1] = 0;
  data_console.data = sensors_data;   // it passes the obtained sensor data to the publisher message
  pub.publish(&data_console);
  nh.spinOnce();
  delay(1); 
}

void auto_calibration(int n_steps)
{
    /* The magnetic field is going to be recorded for 1 whole rotation */
  for ( int Steps = 0; Steps < abs(n_steps); Steps ++)
  {      
    for (int ii=0; ii<int(abs(n_steps/2)); ii++)
    {    
      // HEADING STEPPER MOVEMENT
      if(miliSeconds_reached_flag_hs == true)
      {
        timer_hs_prev = millis();
        miliSeconds_reached_flag_hs = false;
      }
      
      // Change in direction
      if(n_steps > 0)
        digitalWrite(LM_pin[0],HIGH);
      else
        digitalWrite(LM_pin[0],LOW); 
         
      desired_hs_steps = abs(n_steps);
      if(counter_hs_steps < desired_hs_steps)
      {
        timer_hs = millis();
        timer_hs_difference = timer_hs - timer_hs_prev;
    
        if(activated_hs_flag == false)
        {
          if(timer_hs_difference >= miliSeconds_hs_stepper)
          {
            activated_hs_flag = true;
            
            counter_hs_steps++;
            timer_hs_prev = millis();
            if(n_steps > 0)
              current_hs_angle+= 1*k_hs;
            else
              current_hs_angle-= 1*k_hs;
          }
          else
            digitalWrite(LM_pin[1],HIGH);   
        }
          
        timer_hs = millis();
        timer_hs_difference = timer_hs - timer_hs_prev;
        
        if(activated_hs_flag == true)
        {
          if(timer_hs_difference >= miliSeconds_hs_stepper)
          {
            activated_hs_flag = false;
            miliSeconds_reached_flag_hs = true;
          }
          else
            digitalWrite(LM_pin[1],LOW);
        }  
      }
      /* Read raw data and apply factory compensation */
      fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
      RawData_x=mx * ASAX;
      RawData_y=my * ASAY;
      //Serial.println(mx);
      
      /* Low pass filters for x, y and z axes with Factor = 0.025  */
      SmoothData_x = SmoothData_x - (Factor * (SmoothData_x - RawData_x));
      SmoothData_y = SmoothData_y - (Factor * (SmoothData_y - RawData_y));
      
      /* Store min / max x and y */
      if (SmoothData_x > max_x) {max_x = SmoothData_x;}
      if (SmoothData_x < min_x) {min_x = SmoothData_x;}
      if (SmoothData_y > max_y) {max_y = SmoothData_y;}
      if (SmoothData_y < min_y) {min_y = SmoothData_y;}
      // ------------- SENSORS READING ----------------
      for(int i = 0; i < n_sensors-3; i++)
        sensors_data[i] = analogRead(sensors_pin[i]);
    
      // FOR DEBUGGING
      /*if(counter_hs_steps < desired_hs_steps)
        sensors_data[0] = n_steps;
      else
        sensors_data[0] = 0;
      sensors_data[1] = LM_data[0];
      sensors_data[2] = LM_data[1];
      sensors_data[3] = LM_data[2];
      sensors_data[4] = LM_calibration_flag;
      sensors_data[5] = LM_data_flag;*/
      sensors_data[10] = abs(n_steps)-Steps;
      sensors_data[11] = current_hs_angle;
        
      data_console.data = sensors_data;   // it passes the obtained sensor data to the publisher message
      pub.publish(&data_console);
      
      nh.spinOnce();
      delay(1);    
    } 
  }
  counter_hs_steps = 0;
}
