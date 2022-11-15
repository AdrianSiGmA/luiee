void nodeHandle_Connected()
{
  // HEADING STEPPER MOVEMENT
  // Change in direction
  if(LM_data[0] > 0)
    digitalWrite(LM_pin[0],HIGH);
  else
    digitalWrite(LM_pin[0],LOW);
    
  if(miliSeconds_reached_flag_hs   == true)
  {
    timer_hs_prev = millis();
    miliSeconds_reached_flag_hs = false;
  }      

  desired_hs_steps = abs(LM_data[0]);
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
        if(LM_data[0] > 0)
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

  // PITCH STEPPER MOVEMENT
  // Change in direction
  if(LM_data[1] > 0)
    digitalWrite(LM_pin[2],HIGH);
  else
    digitalWrite(LM_pin[2],LOW);
    
  if(miliSeconds_reached_flag_ps == true)
  {
    timer_ps_prev = millis();
    miliSeconds_reached_flag_ps = false;
  }      

  desired_ps_steps = abs(LM_data[1]);
  if(counter_ps_steps < desired_ps_steps)
  {
    timer_ps = millis();
    timer_ps_difference = timer_ps - timer_ps_prev;

    if(activated_ps_flag == false)
    {
      if(timer_ps_difference >= miliSeconds_ps_stepper)
      {
        activated_ps_flag = true;
        counter_ps_steps++;
        timer_ps_prev = millis();
        if(LM_data[1] > 0)
          current_ps_angle+= 1*k_ps;
        else
          current_ps_angle-= 1*k_ps;
      }
      else
        digitalWrite(LM_pin[3],HIGH);   
    }
      
    timer_ps = millis();
    timer_ps_difference = timer_ps - timer_ps_prev;
    
    if(activated_ps_flag == true)
    {
      if(timer_ps_difference >= miliSeconds_ps_stepper)
      {
        activated_ps_flag = false;
        miliSeconds_reached_flag_ps = true;
      }
      else
        digitalWrite(LM_pin[3],LOW);
    }  
  }
  
  // PLANE MIRROR MOVEMENT
  if(LM_data[2] > 0)
  {
    stepper.step(1);
    counter_steps_pm++;
    current_pm_angle = current_pm_angle+0.1766;
    desired_steps_pm = int(LM_data[2]*k_pm);
    if(counter_steps_pm >= desired_steps_pm)
    {
      desired_steps_pm = 0;
      LM_data[2] = 0;
      stepper.step(0);
    }
  }
  else if(LM_data[2] < 0)
  {
    stepper.step(-1);
    counter_steps_pm++;
    current_pm_angle = current_pm_angle-0.1766;
    desired_steps_pm = abs(int(LM_data[2]*k_pm));
    if(counter_steps_pm >= desired_steps_pm)
    {
      desired_steps_pm = 0;
      LM_data[2] = 0;
      stepper.step(0);
    }
  }
  
  for(int i = 0; i < 4; i++)
    analogWrite(mot_pin[i], mot_data[i]);
  analogWrite(buzzer_pin,buzzer_data);
  
  for(int i = 0; i < 6; i++)
    analogWrite(linAct_vapChamb_pin[i], linAct_vapChamb_data[i]);
  // ------------- SENSORS READING ----------------
  for(int i = 0; i < n_sensors-3; i++)
    sensors_data[i] = analogRead(sensors_pin[i]);
    //sensors_data[i] = 0;
  read_pitch();

  // FOR DEBUGGING
  /*sensors_data[0] = mot_pwm_right;
  sensors_data[1] = mot_pwm_left;
  sensors_data[2] = LEDring_data;
  sensors_data[3] = buzzer_data;
  sensors_data[4] = linAct1_pwm;
  sensors_data[5] = linAct2_pwm;
  sensors_data[6] = linAct3_pwm;
  if(counter_hs_steps < desired_hs_steps)
    sensors_data[7] = LM_data[0];
  else
    sensors_data[7] = 0;
  sensors_data[8] = LM_data[1];
  sensors_data[9] = LM_data[2];
  sensors_data[10] = LM_calibration_flag;
  sensors_data[11] = LM_data_flag;*/

  // THE REAL SENSORS FRAME
  //for(int i = 0; i < n_sensors-3;i++)
    //sensors_data[i] = i+1;
  sensors_data[n_sensors-3] = Heading;
  sensors_data[n_sensors-2] = Accel_pitch;
  sensors_data[n_sensors-1] = current_pm_angle;
    
  data_console.data = sensors_data;   // it passes the obtained sensor data to the publisher message
  pub.publish(&data_console);

  if(LM_calibration_flag == 1 && LM_data_flag == 0)
    mag_hard_iron_calibration(); // Calibrate magnetoeter - hard iron calibration
}
