void nodeHandle_notConnected()
{
  timer_LEDring = millis();
  timer_difference_LEDring = timer_LEDring - timer_prev_LEDring;

  leds.fadeToBlackBy(1);
  leds[i] = CHSV(50,150,160);   // (50,255,150) YELLOW
                               // (100,255,150) GREEN
                               // (150,255,150) BLUE
                               // (200,255,150) PURPLE
                               // (255,255,150) RED
  //leds[i] = CRGB::Black;                   
  leds(NUM_LEDS/2,NUM_LEDS-1) = leds(0,NUM_LEDS/2 - 1 );
  FastLED.delay(1);
  
  if(i < NUM_LEDS/2)
  {
    if(timer_difference_LEDring > LEDring_DELAY)
    {
      i++;
      timer_prev_LEDring = millis();
    }
  }
  else
  {
    i = 0;
    timer_prev_LEDring = millis();
    LEDring_flag = !LEDring_flag;
  }
  for(int i = 0; i < 4; i++)
    analogWrite(mot_pin[i], 0);
  analogWrite(buzzer_pin,0);
  
  for(int i = 0; i < 6; i++)
    analogWrite(linAct_vapChamb_pin[i], 0);
}
