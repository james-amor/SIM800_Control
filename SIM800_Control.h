//===================================================================
/* 
 * Copyright (c) 2022, James Amor
 * All rights reserved.

 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree. 
 */
//===================================================================

#include <Arduino.h>
#include "SoftwareSerial.h"

//-------------------------------------------------------------------
// SIM800 Library v1 (28-01-2022)
//-------------------------------------------------------------------
// Health Warnings / Caveats!
// This isn't a re-entrant library, and most functions
// can take several seconds to complete.  You're likely to need to use the 
// ::call_when_idle callback function to maintain the rest of your 
// system throughput when a SIM800 call is completing
// Library behaviour hasn't been checked/coded to handle a millis() wrap
// use for > 45 days at your own risk!
//-------------------------------------------------------------------
// General Usage:
//
// CONFIGURATION
//   - Open the <SoftwareSerial.h> file (hardware\arduino\avr\libraries), and change 
//     _SS_MAX_RX_BUFF to 162, otherwise this library will not function correctly
//   - Call ::begin(<baud rate>) once at creation to configure the s/w serial port
//   - Point ::call_when_idle to a void function; this will be called when the
//     software is waiting.  Suggested use is to kick the watchdog and handle UI.
//     ** Don't use this to call SIM800_Control functions, as the s/w isn't re-entrant **
//
// STARTUP
//   - Call ::Initialise() before using any other function.  
//   - Periodically check ::initialised is TRUE to ensure GSM module 
//     doesn't need to be reconfigured.  If this drops to FALSE then
//     call ::Initialise() again to recover
// 
//  NORMAL OPERATION
//   - Call ::refresh() periodically (suggest c. 10-20ms) to handle serial traffic & URC codes
//   - Check ::protocol_error_count to confirm health of the SIM800 interface
//     Maybe consider a watchdog reboot if this number gets too high
//   
//  MAKING A CALL
//   - Self explanatory; use ::call_number function
//
//  RECEIVING A CALL
//  - ::incoming_call_received will be set TRUE when a new call has been handled and disconnected
//  - when this flag is set, ::stored_caller_id will be populated with the number
//  - Once handled, call ::clear_stored_caller_id() which will reset the flag
//
//  SENDING AN SMS
//  - Call ::clear_sms_buffer() 
//  - Populate ::sms_buffer with your message
//  - Call ::send_sms_from_buffer("<Phone Num>") to send the message
//  
//  RECEIVING SMS
//  - Call ::sms_available() to check whether a new SMS is available
//  - If TRUE, call ::get_pending_sms(&char[4])  which will populate ::sms_buffer with the message,
//    ::stored_caller_id with the originators number, and the char pointer with the SMS ID
//  - Use ::delete_sms (char[4]) to delete the pending SMS and allow access to newer messages
//
//  CHECKING NETWORK STATE
//  Self explanatory:
//     ::connected_to_network()
//     ::get_signal_bars()
//     ::get_signal_percent()
//
//  SERIAL PORT PASS-THRU
//  When in an idle state, you can use the ::available, ::read and ::write 
//  to get and put commands directly to the device.


//-------------------------------------------------------------------
extern SoftwareSerial Sim800_Serial;
extern const byte GSM_RST_PIN;

#define ENABLE_DEBUG_OUTPUT 0

#ifdef ENABLE_DEBUG_OUTPUT
  #define DebugBegin(a) (Serial.begin(a))
  #define DebugFlush() (Serial.flush())
  #define DebugPrint(a) (Serial.print(a))
  #define DebugPrintVal(a,b) (Serial.print(a,b))
  #define DebugPrintln(a) (Serial.println(a))
  #define DebugAvailable() (Serial.available())
  #define DebugRead() (Serial.read())
#else  
  #define DebugBegin(a) 
  #define DebugFlush() 
  #define DebugPrint(a) 
  #define DebugPrintVal(a,b) 
  #define DebugPrintln(a) 
  #define DebugAvailable() false
  #define DebugRead() '\0'
#endif


typedef void(*Function_Pointer)();

#define SECONDS 1

#define MAX_CALLER_ID_SIZE 20
#define TX_BUFFER_SIZE 162
#define RX_BUFFER_SIZE 162

enum Sim800_Buffer_State
{
  BS_OK,
  BS_ERROR,
  BS_DATA,
  BS_WAITING,
  BS_TIMEOUT,
  BS_UNKNOWN
};

const char PROGMEM PROTO_FAILURE_STR[] = "F! Proto";

class SIM800_Control
{
  public:
    SIM800_Control (void);

    bool initialised;
    bool sim_card_inserted;
    bool net_registration_denied;
    byte protocol_error_count;
    byte signal_strength;
    int gsm_resets;
        
    char stored_caller_id [MAX_CALLER_ID_SIZE];    
    bool incoming_call_received;

    void begin (int baud_Rate);
    void initialise (bool force_warmstart = false);

    void refresh (void);

    inline bool available (void) {      return Sim800_Serial.available();    };
    inline char read (void) {      return Sim800_Serial.read();    };
    inline void write (char to_write) {      Sim800_Serial.write (to_write);    };

    bool connected_to_network (void);
    bool connected_to_gprs (void);
    byte get_signal_bars (void);
    byte get_signal_percent (void);
    bool send_sms_from_buffer (char *sms_dest_number);
    bool call_number (char *dest_number);
    bool sms_available (void);
    bool get_pending_sms (char (*sms_id)[4]);
    void delete_sms (char *sms_id);
    bool put_balance_in_sms_buffer (void);
       
    char sms_buffer[TX_BUFFER_SIZE];

    inline void clear_stored_caller_id (void) {incoming_call_received = false;  memset (&stored_caller_id, 0, sizeof(char) * MAX_CALLER_ID_SIZE);}
    inline void clear_sms_buffer (void) {memset (&sms_buffer, 0, sizeof(char) * TX_BUFFER_SIZE);}
    
    bool prep_for_web_submission (void);   
    bool complete_web_submission (void);
            
        
    Function_Pointer call_when_idle;

  private:
    unsigned long incoming_call_ring_time;
  
    void process_urc (void);
    void send_command (const __FlashStringHelper *cmd_string);
    void send_command (char *cmd_string);
    Sim800_Buffer_State wait_for_data (const __FlashStringHelper *pattern, byte timeoutSecs);
    void let_terminal_settle (void);
    byte get_rssi (void);
    void reset_gprs (void);

//    void flush_sms_store (void);

    Sim800_Buffer_State check_for_response (void);
    Sim800_Buffer_State wait_for_status (byte timeout_secs);

    char tx_buffer[TX_BUFFER_SIZE];    
    char rx_buffer[RX_BUFFER_SIZE];
    byte rx_buff_pos;

    Sim800_Buffer_State rx_buff_state;
    bool fatal_error_detected;

    bool website_connected;
    
};
