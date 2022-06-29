  //dataFile = sd.open(log_file_name, FILE_WRITE);
  /*
  if ((millis() - last_write) > 10000)
  {
    digitalWrite(PIN_LED_ACTIVITY, HIGH);
    delay(20); 
    //dataFile.flush();
    dataFile.close();
    dataFile = sd.open(log_file_name, FILE_WRITE);
    digitalWrite(PIN_LED_ACTIVITY, LOW);
    last_write = millis();
  }
  */
  /*
  if(Serial.available())
  {   
    digitalWrite(PIN_LED_ACTIVITY, HIGH);
    dataFile = sd.open(log_file_name, FILE_WRITE);
    uint16_t buffer_size = Serial.available();
    char buff[500];
    for (uint16_t i=0; i<buffer_size; i++)
    {
      buff[i] = Serial.read();
    } 
    dataFile.write(buff, buffer_size);  
    dataFile.close();
    digitalWrite(PIN_LED_ACTIVITY, LOW);
  }
  */