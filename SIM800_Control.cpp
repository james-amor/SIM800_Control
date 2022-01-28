//===================================================================
/* 
 * Copyright (c) 2022, James Amor
 * All rights reserved.

 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree. 
 */
//===================================================================


#include "SIM800_Control.h"

//================================================================================================
SIM800_Control::SIM800_Control()
{
  call_when_idle = NULL;
  
  initialised = false;
  protocol_error_count = 0;
  signal_strength = 0;
  fatal_error_detected = false;
  sim_card_inserted = true; //Assume we have a SIM until we confirm we havent'
  net_registration_denied = false; 
  
  incoming_call_ring_time = 0;
  clear_stored_caller_id();

  clear_sms_buffer();
  
  memset (&tx_buffer, 0, sizeof(char) * TX_BUFFER_SIZE);
  memset (&rx_buffer, 0, sizeof(char) * RX_BUFFER_SIZE);
  rx_buff_pos = 0;
  rx_buff_state = BS_WAITING; 

  website_connected = false;
  
}

//================================================================================================
void SIM800_Control::refresh (void)
{  
  if (initialised == false)
  {
    DebugPrintln (F("Re-Init"));
    initialise(false);
  }
  
  if (check_for_response() == BS_DATA)
  {
    //Unexpected data on the link - handle it as a URC
    process_urc();       
  }  

  //Hang-up / disconnect the call two seconds after it's first detected
  if ((incoming_call_ring_time > 0) && ((millis() - incoming_call_ring_time) > 10000))
  {
    //ATH - Hang-up / disconnect the call
    send_command(F("ATH"));
    if (wait_for_status(20 * SECONDS) == BS_OK)
    {
      //Call disconnected successfully
      incoming_call_ring_time = 0;
      incoming_call_received = true;
    }    
  }
}

//================================================================================================
void SIM800_Control::begin (int baud_Rate)
{

}

//================================================================================================
void SIM800_Control::initialise (bool force_warmstart)
{
  Sim800_Buffer_State return_val;
  bool terminal_online = false;
  
  initialised = false;
  
  //Non-blocking wait for 1 secs
  for (byte wait = 0; wait < 100; wait++)
  {
    if (call_when_idle) call_when_idle();
    delay(10);
  }

  if (force_warmstart == true)
  {
    //Cycle the RESET Pin to restart the module
    pinMode (GSM_RST_PIN, OUTPUT);
  
    //Non-blocking wait for 500ms
    for (byte wait = 0; wait < 50; wait++)
    {
      if (call_when_idle) call_when_idle();
      delay(10);
    }
  
    pinMode (GSM_RST_PIN, INPUT);

    //Non-blocking wait for 500ms
    for (byte wait = 0; wait < 50; wait++)
    {
      if (call_when_idle) call_when_idle();
      delay(10);
    }
  
  }

  //Wait until the Serial port is active
  send_command(F("\r\n"));
  
  let_terminal_settle();

  unsigned long start_time = millis();  
  return_val = BS_UNKNOWN;

  while (((millis() - start_time) < 30000) && (return_val != BS_OK))
  {
    //AT - Check that the module is accepting commands
    send_command(F("AT"));

    return_val = wait_for_status(2 * SECONDS);
    if (return_val != BS_OK)
    {
      DebugPrintln (F("F! NoResp"));
    }    
  }

  if (return_val != BS_OK)
  {
    DebugPrintln (F("F! RstFail"));    
    return;
  }

  return_val = wait_for_data(F("SMS Ready"), 30 * SECONDS);  
  if ((return_val != BS_DATA) && (force_warmstart == true))
  {
    //AT+CCID - Check for a valid SIM card
    send_command(F("AT+CCID"));
  
    wait_for_data(NULL, 2 * SECONDS);  
    
    return_val = wait_for_status(2 * SECONDS);
    if (return_val != BS_OK)
    {
      sim_card_inserted = false;
      DebugPrintln (F("F! NoSim"));
      return;
    }
  
    
    return;
  }

  //Non-blocking wait for 1 secs
  for (byte wait = 0; wait < 100; wait++)
  {
    if (call_when_idle) call_when_idle();
    delay(10);
  }
  
  //AT&F - Reset to Factory Defaults
  send_command(F("AT&F"));

  return_val = wait_for_status(2 * SECONDS);
  if (return_val != BS_OK)
  {    
    DebugPrintln (PROTO_FAILURE_STR); //PROTO_FAILURE_STR);
    protocol_error_count++; 
    return;
  }

  //ATE 0 - Disable Command Echo
  send_command(F("ATE 0"));

  return_val = wait_for_status(2 * SECONDS);
  if (return_val != BS_OK)
  {
    DebugPrintln (PROTO_FAILURE_STR);
    protocol_error_count++; 
    return;
  }

  //AT+CCID - Check for a valid SIM card
  send_command(F("AT+CCID"));

  if (wait_for_data(NULL, 2 * SECONDS) == BS_DATA)
  {
    DebugPrintln (F("SimOk"));  
  }

  return_val = wait_for_status(2 * SECONDS);
  if (return_val != BS_OK)
  {
    DebugPrintln (F("F! NoSim"));
    return;
  }
  
  //AT+CMGF=1 - Manage SMS in Text Format
  send_command(F("AT+CMGF=1"));

  return_val = wait_for_status(2 * SECONDS);
  if (return_val != BS_OK)
  {
    DebugPrintln (F("F! InitFail"));
    return;
  }
  
  //AT+CLIP=1 - Enable Caller ID Presentation
  send_command(F("AT+CLIP=1"));

  return_val = wait_for_status(15 * SECONDS);
  if (return_val != BS_OK)
  {
    DebugPrintln (PROTO_FAILURE_STR);
    protocol_error_count++; 
    return;
  }

  //AT+CUSD=1 - Enable Unstructured Data Responses
  send_command(F("AT+CUSD=1"));

  return_val = wait_for_status(2 * SECONDS);
  if (return_val != BS_OK)
  {
    DebugPrintln (PROTO_FAILURE_STR);
    protocol_error_count++; 
    return;
  }

  wait_for_status(5 * SECONDS);
  send_command(F("AT+CFUN=4"));  
  wait_for_status(15 * SECONDS);
  wait_for_status(5 * SECONDS);
  send_command(F("AT+CFUN=1"));  
  wait_for_status(15 * SECONDS);

       
  DebugPrintln (F("InitOk"));  
  initialised = true;
}
//==================================================================================
//==================================================================================
Sim800_Buffer_State SIM800_Control::check_for_response (void)
{  
  //Initialise the buffer, or clear down any residual data
  if ((rx_buff_state == BS_UNKNOWN) || (rx_buff_state == BS_DATA))
  {
    memset (&rx_buffer, 0, sizeof(char) * RX_BUFFER_SIZE);
    rx_buff_pos = 0;
  }  

  rx_buff_state = BS_WAITING;
  
  //Loop until either a complete line has been read,
  //or there is no more data in the receive buffer
  while ((Sim800_Serial.available()) && (rx_buff_state == BS_WAITING))
  {
    if (call_when_idle) call_when_idle();
    
    rx_buffer[rx_buff_pos] = Sim800_Serial.read();

    if (rx_buffer[rx_buff_pos] == char(10))
    {
      //Do nothing (Strip Line Feeds)
    }
    else if (rx_buffer[rx_buff_pos] == char(13))
    {      
      //Replace the CR with a NULL so that the data forms a valid string
      rx_buffer[rx_buff_pos] = char(0);

      //If something more than a CR on it's own has been received, store the data
      if (rx_buff_pos > 0)
      {
        DebugPrint (F("Rx: "));
        DebugPrintln (rx_buffer);
        rx_buff_state = BS_DATA;
      }
    }
    else
    {      
      if (rx_buff_pos < RX_BUFFER_SIZE)
      {
        rx_buff_pos++;
      }
      else
      {
        //If the buffer has filled, but there's no meaningful data,
        //clear down the buffer
        memset (&rx_buffer, 0, sizeof(char) * RX_BUFFER_SIZE);
        rx_buff_pos = 0;
      }
    }
  }

  return rx_buff_state;
}
//==================================================================================
//==================================================================================
Sim800_Buffer_State SIM800_Control::wait_for_status (byte timeout_secs)
{
  Sim800_Buffer_State return_val = BS_UNKNOWN;
  unsigned long loop_start = millis();

  while (return_val == BS_UNKNOWN)
  {
    if (call_when_idle) call_when_idle();
    
    //Check for a timeout condition
    if (millis() > (loop_start + ((unsigned long)timeout_secs * 1000UL)))
    {
      return_val = BS_TIMEOUT;
    }
    //Check to see whether data has been received
    if (check_for_response() == BS_DATA)
    {
      //Check to see if a status has been received
      if (strstr_P(rx_buffer, PSTR("OK")) != NULL)
      {
        return_val = BS_OK;
      }
      else if (strstr_P(rx_buffer, PSTR("ERROR")) != NULL)
      {
        return_val = BS_ERROR;
      }     
      else if ((rx_buffer[0] == '+') &&
               ((rx_buffer[4] == ':') || (rx_buffer[5] == ':')))
      {
         process_urc();       
      }
    }
  }

  return return_val;  
}
//==================================================================================
//==================================================================================
void SIM800_Control::process_urc (void)
{
  if (strstr_P(rx_buffer, PSTR("+CLIP: ")) != NULL)
  {
    //If this is the first ring, then store the number
    if (incoming_call_ring_time == 0)
    {
      incoming_call_ring_time = millis();
      
      for (byte idx = 7; idx < strlen(rx_buffer); idx++)
      {
        if (rx_buffer[idx] == ',') rx_buffer[idx] = '\0';      
      }

      strcpy(stored_caller_id, (char *)&rx_buffer[7]);

      if (strlen(stored_caller_id) == 2)
      {
        strcpy_P (stored_caller_id, PSTR("\"UNKNOWN\""));
      }
      
      DebugPrint (F("URC=RING "));
      DebugPrintln (stored_caller_id);    
    }       
  }
  else if (strstr_P(rx_buffer, PSTR("+CMTI: \"SM\"")) != NULL)
  {
    DebugPrintln (F("URC=SMS"));
  }
  else if ((initialised == true) &&
           ((strstr_P(rx_buffer, PSTR("SMS Ready")) != NULL) ||
            (strstr_P(rx_buffer, PSTR("Call Ready")) != NULL)))
  {
    //The GSM Module has restarted... re-initialise
    DebugPrintln (F("URC=REBOOT"));
    gsm_resets++;
    initialised = false;
  }  
  else if (strstr_P(rx_buffer, PSTR("CLOSED")) != NULL)
  {
    website_connected = false;   
  }
}
//==================================================================================
//==================================================================================
void SIM800_Control::let_terminal_settle (void)
{
  //Settle for 150ms
  for (byte delay_cnt = 0; delay_cnt < 15; delay_cnt++)
  {
    if (call_when_idle) call_when_idle();
    delay(10);    
  }

  //Clear anything from the receive buffer to ensure we capture the correct reply
  while (check_for_response() == BS_DATA)
  {
    process_urc();
  }  
}
//==================================================================================
//==================================================================================
void SIM800_Control::send_command (const __FlashStringHelper *cmd_string)
{
  let_terminal_settle();
  
  if (strlen_P((PGM_P)cmd_string) <= TX_BUFFER_SIZE)
  {
    strcpy_P (tx_buffer, (PGM_P)cmd_string);

    //Send the required command
    DebugPrint (F("TxC: "));
    DebugPrintln (tx_buffer);
    Sim800_Serial.print (tx_buffer);
    Sim800_Serial.print (F("\r\n"));
  }
  else
  {
    DebugPrintln (F("F! TxCmdTooLong"));
  }

  delay (50);
}
//==================================================================================
//==================================================================================
void SIM800_Control::send_command (char *cmd_string)
{
  let_terminal_settle();
  
  if (strlen(cmd_string) <= TX_BUFFER_SIZE)
  {
    strcpy (tx_buffer, cmd_string);

    //Send the required command
    DebugPrint (F("TxC: "));
    DebugPrintln (tx_buffer);
    Sim800_Serial.print (tx_buffer);
    Sim800_Serial.print (F("\r\n"));
  }
  else
  {
    DebugPrintln (F("F! TxCmdTooLong"));
  }

  delay (50);
}
//==================================================================================
//==================================================================================
Sim800_Buffer_State SIM800_Control::wait_for_data (const __FlashStringHelper *pattern, byte timeoutSecs)
{
  Sim800_Buffer_State return_val = BS_UNKNOWN;
  unsigned long loop_start = millis();

  while (return_val == BS_UNKNOWN)
  {
    if (call_when_idle) call_when_idle();

    //Check for a timeout condition
    if (millis() > (loop_start + ((unsigned long)timeoutSecs * 1000UL)))
    {
      return_val = BS_TIMEOUT;
    }

    //Check to see whether any data has been received
    if (check_for_response() == BS_DATA)
    {
      //Check what data has been received, and whether it matches the data being requested
      if ((strlen(rx_buffer) < 4) && (strstr_P(rx_buffer, PSTR("OK")) != NULL))
      {
        return_val = BS_OK;
      }
      else if ((strlen(rx_buffer) < 7) && (strstr_P(rx_buffer, PSTR("ERROR")) != NULL))
      {
        return_val = BS_ERROR;
      }
      else if ((pattern != NULL) && (strstr_P(rx_buffer, (PGM_P)pattern) != NULL))
      {
        return_val = BS_DATA;
      }
      else if (pattern == NULL)
      {
        return_val = BS_DATA;
      }
      else 
      {
        process_urc();
      }
    }
  }

  return return_val;
}
//==================================================================================
//==================================================================================
bool SIM800_Control::connected_to_network (void)
{

// Test Code
//  if (((millis() > 400000UL) && (millis() <  500000UL)) ||
//      ((millis() > 1000000UL) && (millis() < 1100000UL)))
//  {
//    return false;
//  }

  
	Sim800_Buffer_State return_val = BS_UNKNOWN;
	static unsigned long last_called = 0;
	static bool net_connected = false;

  if (initialised == false) return false;
  
	//Limit polls to every five seconds
	if ((millis() - last_called) > 5000)
	{
		last_called = millis();
    
		send_command(F("\r\n"));
		let_terminal_settle();
		
		//AT+CREG - Query network registration status
		send_command(F("AT+CREG?"));

		if (wait_for_data(F("+CREG: "), 5 * SECONDS) == BS_DATA)
		{
			if ((strstr_P(rx_buffer, PSTR("+CREG: 0,1")) != NULL) ||
			    (strstr_P(rx_buffer, PSTR("+CREG: 0,5")) != NULL))
			{
				net_connected = true;
        net_registration_denied = false;
      }
			else
			{
        if (strstr_P(rx_buffer, PSTR("+CREG: 0,3")) != NULL)
        {
          send_command(F("AT+CFUN=4"));  
          wait_for_status(15 * SECONDS);
          wait_for_status(5 * SECONDS);
          send_command(F("AT+CFUN=1"));  
          wait_for_status(15 * SECONDS);          
          protocol_error_count++; 

          net_registration_denied = true;
        }
        
				net_connected = false;			
			}
		}

		return_val = wait_for_status(5 * SECONDS);
		if (return_val != BS_OK)
		{
			DebugPrintln (PROTO_FAILURE_STR);
			protocol_error_count++; 
			return false;
		}
	}
	
	return net_connected;

}
//==================================================================================
//==================================================================================
byte SIM800_Control::get_signal_percent (void)
{
  byte signal_percent = 0; 
  byte recvd_rssi = get_rssi();

  if (initialised == false) return 0;

  if ((recvd_rssi == 0) || (recvd_rssi == 99)) 
  {
    signal_percent = 0;
  }
  else
  {
    signal_percent = (recvd_rssi * 100) / 31; //(int)float((float(recvd_rssi) / 31.0f) * 100.0f);
  }
  
  return signal_percent;
}
//==================================================================================
//==================================================================================
byte SIM800_Control::get_signal_bars (void)
{
  byte signal_bars = 0;
  byte recvd_rssi = get_rssi();

  if (initialised == false) return 0;

  if ((recvd_rssi == 0) || (recvd_rssi == 99)) signal_bars = 0;
      else if (recvd_rssi < 10) signal_bars = 1;
      else if (recvd_rssi < 15) signal_bars = 2;
      else if (recvd_rssi < 20) signal_bars = 3;
      else signal_bars = 4;

  
  return signal_bars;
}
//==================================================================================
//==================================================================================
byte SIM800_Control::get_rssi (void)
{
  Sim800_Buffer_State return_val = BS_UNKNOWN;
  static unsigned long last_called = 0;
  static byte recvd_rssi = 0;

  if (initialised == false) return 0;
  
  //Limit polls to once a second
  if ((millis() - last_called) > 10000)
  {
    last_called = millis();
    
    send_command(F("\r\n"));
    let_terminal_settle();
    
    //AT+CSQ - Query signal state
    send_command(F("AT+CSQ"));

    if (wait_for_data(F("+CSQ: "), 5 * SECONDS) == BS_DATA)
    {
      for (byte idx = 6; idx < strlen(rx_buffer); idx++)
      {
        if (rx_buffer[idx] == ',') rx_buffer[idx] = '\0';      
      }

      recvd_rssi = atoi((char *)&rx_buffer[6]);           
    }

    return_val = wait_for_status(5 * SECONDS);
    if (return_val != BS_OK)
    {
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 
      return 0;
    }
  }
  
  return recvd_rssi;
}
//==================================================================================
//==================================================================================
bool SIM800_Control::send_sms_from_buffer (char *sms_dest_number)
{  
  if (initialised == false) return false;

  bool message_sent = false;

  //Attempt the send three times before giving up
  for (byte retries = 0; ((retries < 3) && (message_sent == false)); retries++)
  {
    if (call_when_idle) call_when_idle();
    
    send_command(F("\r\n"));
    let_terminal_settle();
    
    char temp_cmd[30];
    memset (&temp_cmd, 0, sizeof(char) * 30);
    strcpy_P (temp_cmd, PSTR("AT+CMGS=\""));
    strcpy (&temp_cmd[9], sms_dest_number);
    temp_cmd[9 + strlen(sms_dest_number)] = '\"';
    
    send_command (temp_cmd);
  
    send_command (sms_buffer);
    
    temp_cmd[0] = char(26);
    temp_cmd[1] = '\0';  
    send_command (temp_cmd);
  
    if (wait_for_status(60 * SECONDS) == BS_OK)
    {
      message_sent = true;
      clear_sms_buffer();
    }  
    else
    {
      message_sent = false;
    }
  }

  
  return message_sent;
}
//==================================================================================
//==================================================================================
bool SIM800_Control::call_number (char *dest_number)
{
  if (initialised == false) return false;
  
  Sim800_Buffer_State return_val = BS_UNKNOWN;
  bool call_completed = false;

  send_command(F("\r\n"));
  let_terminal_settle();
  
  char temp_cmd[30];
  memset (&temp_cmd, 0, sizeof(char) * 30);
  strcpy_P (temp_cmd, PSTR("ATD "));
  strcpy (&temp_cmd[4], dest_number);
  temp_cmd[4 + strlen(dest_number)] = ';';
  
  send_command (temp_cmd);
  
  if (wait_for_status(20 * SECONDS) != BS_OK)
  {
    send_command (F("ATH"));
    DebugPrintln (F("F! CallInit"));      
    return false;
  }  

  //Wait for the call to establish
  wait_for_status(1 * SECONDS);

  //Check for pickup/disconnection
  bool call_complete = false;
  bool call_successful = false;
  unsigned long start_time = millis();
  
  while (((millis() - start_time) < 45000UL) && (call_complete == false))
  {
    send_command (F("AT+CLCC"));

    return_val = wait_for_data(F("+CLCC:"), 5 * SECONDS);        
    if (return_val == BS_DATA)
    {
      byte csv_token = 0;
      byte token_start = 0;
      for (byte idx = 6; idx < strlen(rx_buffer); idx++)
      {
        if (rx_buffer[idx] == ',') 
        {
          csv_token++;
          if (csv_token == 2) token_start = idx+1;
          if (csv_token == 3) rx_buffer[idx] = '\0'; 
        }
      }
  
      switch (rx_buffer[token_start])
      {
        case '0' :        //Active
        case '1' :        //Held
        case '6' :        //Disconnected
                   call_complete = true;
                   call_successful = true;
                   break;      
        case '2' :        //Dialling
        case '3' :        //Ringing
                   break;      
        case '4' :        //Incoming: Invalid for outbound
        case '5' :        //Waiting : Invalid for outbound             
                   call_complete = true;
                   call_successful = false;
                   break;      
      }
    }
    else if (return_val == BS_OK)
    {
      call_complete = true;
      call_successful = true;
    }
    else if (return_val == BS_ERROR)
    {
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 
      call_complete = true;
      call_successful = false;
    }

	//Non-blocking wait for 500ms
	for (byte wait = 0; wait < 50; wait++)
	{
		if (call_when_idle) call_when_idle();
		delay(10);
	}
  }
  
  //Hangup the call
  send_command (F("ATH"));
  if (wait_for_status(20 * SECONDS) != BS_OK)
  {
    DebugPrintln (PROTO_FAILURE_STR);
    protocol_error_count++; 
    call_successful = false;
  }  

  return call_successful;    
}
//==================================================================================
//==================================================================================
bool SIM800_Control::sms_available (void)
{
  Sim800_Buffer_State return_val = BS_UNKNOWN;
  static unsigned long last_called = 0;
  static byte consecutive_errors = 0;
  bool sms_store_populated = false;

  if (initialised == false) return false;
  
  //Limit polls to once a second
  if ((millis() - last_called) > 1000)
  {
    last_called = millis();
    
    send_command(F("\r\n"));
    let_terminal_settle();

    //AT+CMGL="ALL" - Query network registration status
    send_command(F("AT+CMGL=\"ALL\""));

    return_val = wait_for_data(F("+CMGL:"), 20 * SECONDS);
    if (return_val == BS_DATA)
    {
      consecutive_errors = 0;
      sms_store_populated = true;
      if (wait_for_status(20 * SECONDS) != BS_OK)
      {
        //Protocol error
        DebugPrintln (PROTO_FAILURE_STR);
        protocol_error_count++; 
      }
    }
    else if (return_val == BS_OK)
    {
      consecutive_errors = 0;
      sms_store_populated = false;      
    }
    else
    {
      //protocol error
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 

      //If this command keeps erroring, then restart the GSM module
      //as it's likely to have warmstarted
      consecutive_errors++;
      if (consecutive_errors > 5)
      {
        consecutive_errors = 0;
        gsm_resets++;
        initialised = false;
      }      
    }
  }

  return sms_store_populated;
}
//==================================================================================
//==================================================================================
bool SIM800_Control::get_pending_sms (char (*sms_id)[4])
{  
  Sim800_Buffer_State return_val = BS_UNKNOWN;
  memset (sms_id, 0, sizeof(char) * 4);
  
  if (initialised == false) return false;
      
  send_command(F("\r\n"));
  let_terminal_settle();

  //AT+CMGL="ALL" - List all available SMS messages
  send_command(F("AT+CMGL=\"ALL\""));

  return_val = wait_for_data(F("+CMGL:"), 20 * SECONDS);
  if (return_val == BS_DATA)
  {
    //Extract the caller id
    //+CMGL: 1,"REC UNREAD","+447881554465","","19/04/23,15:17:24+04"
    byte csv_token = 0;
    byte token_start = 0;
    for (byte idx = 7; idx < strlen(rx_buffer); idx++)
    {
      if (rx_buffer[idx] == ',') 
      {
        csv_token++;
        if (csv_token == 1) 
        {
          strncpy (*sms_id, ((char *)&rx_buffer[7]), idx-7);
          DebugPrintln (*sms_id);
        }
        else if (csv_token == 2) 
        {
          token_start = idx+1;
        }
        else if (csv_token == 3) 
        {
          rx_buffer[idx] = '\0'; 
        }
      }
    }
    
    strncpy (stored_caller_id, (char *)&rx_buffer[token_start], MAX_CALLER_ID_SIZE);
    
	  //Store the message
    return_val = wait_for_data(F(""), 20 * SECONDS);
    if (return_val == BS_DATA)
    {
      strncpy (sms_buffer, rx_buffer, TX_BUFFER_SIZE);
    }

    //Attempt to spot and decode UCS2 encoded messages (mostly from Lebara)
    byte start_offset = 255;

    if (((stored_caller_id[1] == 'p') || ((stored_caller_id[1] != '+') && (stored_caller_id[1] != '0'))) &&
        (strlen(sms_buffer) >= 14))
    {           
      for (byte idx = 0; ((idx < 8) && (start_offset == 255)) ; idx++)
      {
        if ((sms_buffer[idx] == '0') && (sms_buffer[idx+1] == '0') && (sms_buffer[idx+2] != '0') && 
            (sms_buffer[idx+4] == '0') && (sms_buffer[idx+5] == '0') && (sms_buffer[idx+6] != '0'))
        {
          start_offset = idx;
        }
      }            
    }

    if (start_offset != 255)
    { 
      byte ascii_val = 0;
      byte start_idx = 0;
      byte decoded_length = strlen(sms_buffer) - start_offset;
      decoded_length = (decoded_length - (decoded_length % 4)) / 4;

    
      for (byte idx = 0; idx < decoded_length; idx++)
      {
        start_idx = (idx * 4) + start_offset;
    
//        Serial.print ((char)sms_buffer[start_idx]);
//        Serial.print ((char)sms_buffer[start_idx+1]);
//        Serial.print ((char)sms_buffer[start_idx+2]);
//        Serial.print ((char)sms_buffer[start_idx+3]);
//        Serial.print (" : ");
        
        if ((sms_buffer[start_idx] == '0') &&
            (sms_buffer[start_idx+1] == '0') &&
            ((((int)sms_buffer[start_idx+2] >= 0x30) && ((int)sms_buffer[start_idx+2] <= 0x39)) || (((int)sms_buffer[start_idx+2] >= 0x41) && ((int)sms_buffer[start_idx+2] <= 0x46))) &&
            ((((int)sms_buffer[start_idx+3] >= 0x30) && ((int)sms_buffer[start_idx+3] <= 0x39)) || (((int)sms_buffer[start_idx+3] >= 0x41) && ((int)sms_buffer[start_idx+3] <= 0x46))))        
        {
            
          if ((int)sms_buffer[start_idx+2] >= 0x41) //A-F
          {
            ascii_val = ((int)sms_buffer[start_idx+2] - 55) << 4;        
          }
          else //0-9
          {
            ascii_val = ((int)sms_buffer[start_idx+2] - 48) << 4;
          }
          
          if ((int)sms_buffer[start_idx+3] >= 0x41) //A-F
          {
            ascii_val = ascii_val + ((int)sms_buffer[start_idx+3] - 55);        
          }
          else //0-9
          {
            ascii_val = ascii_val + ((int)sms_buffer[start_idx+3] - 48);
          }      
    
          //Serial.print ((int)ascii_val);
          //Serial.print (" : ");
          sms_buffer[idx] = (char)ascii_val;
        }
        else
        {
          sms_buffer[idx] = '*';
        }
    
        //Serial.println (sms_buffer[idx]);
      }
  
      sms_buffer[decoded_length] = '\0';    
    }

    if (wait_for_status(20 * SECONDS) != BS_OK)
    {
      //Protocol error
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 
    }
  }
  else if (return_val == BS_OK)
  {
    //Do nothing, there's no pending message
  }
  else
  {
    //protocol error
    DebugPrintln (PROTO_FAILURE_STR);
    protocol_error_count++; 
  }

  return (*sms_id[0] != '\0');
}
//==================================================================================
//==================================================================================
void SIM800_Control::delete_sms (char *sms_id)
{
  if (initialised == false) return;
  
  Sim800_Buffer_State return_val = BS_UNKNOWN;

  send_command(F("\r\n"));
  let_terminal_settle();
  
  char temp_cmd[30];
  memset (&temp_cmd, 0, sizeof(char) * 30);
  strcpy (temp_cmd, "AT+CMGD=");
  strcpy (&temp_cmd[8], sms_id);
  
  send_command (temp_cmd);
  
  if (wait_for_status(10 * SECONDS) != BS_OK)
  {
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 
      return;
  }  
  
}
//==================================================================================
//==================================================================================
bool SIM800_Control::put_balance_in_sms_buffer (void)
{
  if (initialised == false) return false;
  
  Sim800_Buffer_State return_val = BS_UNKNOWN;

  clear_sms_buffer();

  send_command(F("\r\n"));
  let_terminal_settle();

  send_command (F("ATD *#1345#;"));
  
  if (wait_for_status(20 * SECONDS) != BS_OK)
  {
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 
      return false;
  }  
  
  if (wait_for_status(20 * SECONDS) != BS_OK)
  {
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 
      return false;
  }  

  return_val = wait_for_data(F("+CUSD:"), 60 * SECONDS);
  if (return_val == BS_DATA)
  {
    byte quote_start = 0;
    for (byte idx = 7; idx < strlen(rx_buffer); idx++)
    {
      if (rx_buffer[idx] == '\"') 
      {
        if (quote_start == 0) quote_start = idx + 1;
        else rx_buffer[idx] = '\0';  
      }
    }

    strcpy (sms_buffer, &rx_buffer[quote_start]);

    Serial.println ("DONE");
  }
  else
  {
    return false;
  }

  return true;
}
//==================================================================================
//==================================================================================
bool SIM800_Control::prep_for_web_submission (void)
{
  static bool hadValidGprsContext = false;
   
  if (initialised == false) return false;

  website_connected = false;  
  Sim800_Buffer_State return_val = BS_UNKNOWN;

  send_command(F("\r\n"));
  let_terminal_settle();

  //AT+CREG - Query network registration status
  send_command(F("AT+CGREG?"));

  if (wait_for_data(F("+CGREG: "), 5 * SECONDS) == BS_DATA)
  {
    if ((strstr_P(rx_buffer, PSTR("+CGREG: 0,1")) == NULL) &&
              (strstr_P(rx_buffer, PSTR("+CGREG: 0,5")) == NULL))
    {
      wait_for_status(5 * SECONDS);  
      return false;
    }
  }

  //AT+CGATT - Query GPRS ready state
  send_command(F("AT+CGATT?"));

  if (wait_for_data(F("+CGATT: "), 5 * SECONDS) == BS_DATA)
  {
    if (strstr_P(rx_buffer, PSTR("+CGATT: 0")) != NULL)
    {
      reset_gprs();
      return false;      
    }
  }

  //AT+CSTT - Define Network APN
  send_command(F("AT+CSTT=\"pp.vodafone.co.uk\",\"wap\",\"wap\""));  
  if (wait_for_status(10 * SECONDS) != BS_OK)
  {
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 
      reset_gprs();
      return false;
  }  

  //AAT+CIICR - Start connection (get an IP address)
  send_command(F("AT+CIICR"));  
  if (wait_for_status(85 * SECONDS) != BS_OK)
  {
      DebugPrintln (F("F! NetStart"));      
      reset_gprs();
      return false;
  }  

  //AT+CIFSR - Report current IP address
  send_command(F("AT+CIFSR"));  
  if (wait_for_data(NULL, 2 * SECONDS) != BS_DATA)
  {
      DebugPrintln (F("FAIL: NoIP"));
      reset_gprs();
      return false;
  }  

  //AT+CIPSTART - Open TCP connection to server
  send_command(F("AT+CIPSTART=\"TCP\",\"lythamrnli.jamesamor.co.uk\",80"));  
  if (wait_for_status(75 * SECONDS) != BS_OK)
  {
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 
      reset_gprs();
      return false;    
  }
  
  return_val = wait_for_data(F("CONNECT OK"), 75 * SECONDS);  
  if (return_val != BS_DATA)
  {
      DebugPrintln (F("F! ServerConnect"));
      reset_gprs();
      return false;
  }  

  //AT+CIPSEND - Prepare for data submission
  send_command(F("AT+CIPSEND"));  

  delay (500);
  
  hadValidGprsContext = true;
  website_connected = true;

  return true; 
  
}
//==================================================================================
//==================================================================================
bool SIM800_Control::complete_web_submission (void)
{
  Sim800_Buffer_State return_val = BS_UNKNOWN;
  bool sendSuccess = true;
  
  website_connected = false;

  send_command(F("\r\n"));
  send_command(F("\r\n"));

  char temp_cmd[2];
  temp_cmd[0] = char(26); //CTRL+Z
  temp_cmd[1] = '\0';  
  send_command (temp_cmd);

  return_val = wait_for_data(F("SEND OK"), 75 * SECONDS);
  if (return_val != BS_DATA)
  {    
    DebugPrintln (F("F! SendFail"));
    reset_gprs();
    return false;    
  }
  
  return_val = wait_for_data(F("+BOB: "), 75 * SECONDS);
  if (return_val == BS_DATA)
  {    
    if (strstr_P(rx_buffer, PSTR("+BOB: 1")) == NULL)
    {
      sendSuccess = false;
    }
  }
  else
  {    
    DebugPrintln (F("F! SiteFail"));
    sendSuccess = false;
  }

  if (return_val != BS_TIMEOUT)
  {
    return_val = wait_for_data(F("CLOSED"), 10 * SECONDS);
    if (return_val != BS_DATA)
    {    
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 
      reset_gprs();
      return false;    
    }
  }

  //AT+CIPCLOSE - CLose the connection (if it's not already closed)
  send_command(F("AT+CIPCLOSE"));  
  wait_for_status(10 * SECONDS);
  
  //AT+CIPSHUT - Force link closure
  send_command(F("AT+CIPSHUT"));  
  return_val = wait_for_data(F("SHUT OK"), 65 * SECONDS);
  if (return_val != BS_DATA)
  {
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 
      reset_gprs();
      return false;
  }    
 
  return sendSuccess;
}
//==================================================================================
//==================================================================================
bool SIM800_Control::connected_to_gprs (void)
{

// Test Code
//  if (((millis() > 300000UL) && (millis() <  600000UL)) ||
//      ((millis() > 900000UL) && (millis() < 1200000UL)))
//  {
//    return false;
//  }
  
  
  Sim800_Buffer_State return_val = BS_UNKNOWN;
  static unsigned long last_called = 0;
  static bool gprs_connected = false;

  if (initialised == false) return false;
  
  //Limit polls to every five seconds
  if ((millis() - last_called) > 5000)
  {
    last_called = millis();
    
    send_command(F("\r\n"));
    let_terminal_settle();

    //AT+CREG - Query network registration status
    send_command(F("AT+CGREG?"));

    if (wait_for_data(F("+CGREG: "), 5 * SECONDS) == BS_DATA)
    {
      if (strstr_P(rx_buffer, PSTR("+CGREG: 0,3")) != NULL)
      {
        wait_for_status(5 * SECONDS);  

        send_command(F("AT+CFUN=4"));  
        wait_for_status(15 * SECONDS);
        wait_for_status(5 * SECONDS);
        send_command(F("AT+CFUN=1"));  
        wait_for_status(15 * SECONDS);

        protocol_error_count++;         
        return false;
      }
      else if ((strstr_P(rx_buffer, PSTR("+CGREG: 0,1")) == NULL) &&
                (strstr_P(rx_buffer, PSTR("+CGREG: 0,5")) == NULL))
      {
        gprs_connected = false;    
        wait_for_status(5 * SECONDS);  
        return false;
      }
    }

    return_val = wait_for_status(5 * SECONDS);
    if (return_val != BS_OK)
    {
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 
      return false;
    }


    //AT+CGATT - Query GPRS ready state
    send_command(F("AT+CGATT?"));

    if (wait_for_data(F("+CGATT: "), 5 * SECONDS) == BS_DATA)
    {
      if (strstr_P(rx_buffer, PSTR("+CGATT: 1")) != NULL)
      {
        gprs_connected = true;             
      }
      else
      {
        gprs_connected = false;     
        wait_for_status(5 * SECONDS); 
        return false;
      }
    }

    return_val = wait_for_status(5 * SECONDS);
    if (return_val != BS_OK)
    {
      DebugPrintln (PROTO_FAILURE_STR);
      protocol_error_count++; 
      return false;
    }
        
  }
  
  return gprs_connected;

}
//==================================================================================
//==================================================================================
void SIM800_Control::reset_gprs (void)
{
    send_command(F("AT+CIPSHUT"));  
    wait_for_data(F("SHUT OK"), 65 * SECONDS);           
    wait_for_status(5 * SECONDS);
    send_command(F("AT+CFUN=4"));  
    wait_for_status(15 * SECONDS);
    wait_for_status(5 * SECONDS);
    send_command(F("AT+CFUN=1"));  
    wait_for_status(15 * SECONDS);
}
