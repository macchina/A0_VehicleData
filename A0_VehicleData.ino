// Macchina A0 vehicle data example
// https://docs.macchina.cc/projects/a0-projects/a0-can-vehicle-data

#include <esp32_can.h>

#define LED_PIN     13

// PIDs, as defined in ISO 15031-5
#define PID_ECT     0x05
#define PID_MAP     0x0B
#define PID_RPM     0x0C
#define PID_SPEED   0x0D

// List PIDs we want to read
#define PIDS_COUNT  4
uint8_t pids[PIDS_COUNT] = {PID_ECT, PID_MAP, PID_RPM, PID_SPEED};

unsigned long last_run = 0;
int lastTemp = -1;
float lastMAP = -1;
int lastRPM = -1;
int lastMPH = -1;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Start serial port
  Serial.begin(115200);

  // Start CAN on default pins, at 500k baud
  CAN0.setCANPins(GPIO_NUM_4, GPIO_NUM_5);
  CAN0.begin(500000);
  Serial.println("Ready ...!");

  // Set CAN filters to allow all messages
  for (int filter = 0; filter < 3; filter++)
  {
    CAN0.setRXFilter(filter, 0, 0, false);
  }
}

void loop()
{
  CAN_FRAME incoming;

  // Check for incoming CAN messages
  if (CAN0.available() > 0)
  {
    CAN0.read(incoming);
    // If message ID is in the appropriate range for OBD, process it
    if (incoming.id > 0x7DF && incoming.id < 0x7F0)
    {
      processPID(incoming);
    }
  }

  // Request all listed PIDs every 50ms
  unsigned long now = millis();
  if ((now - last_run) > 50)
  {
    for (int i = 0; i < PIDS_COUNT; i++)
    {
      sendPIDRequest(0x7DF, pids[i]);
    }
    
    last_run = now;
  }
}

// Send a "Request current powertrain diagnostic data" request to get a PID value
void sendPIDRequest(uint32_t id, uint8_t PID)
{
  CAN_FRAME frame;
  frame.id = id;
  frame.extended = 0;
  frame.length = 8;
  
  for (int i = 0; i < 8; i++)
    frame.data.bytes[i] = 0xAA;
  
  frame.data.bytes[0] = 2; // 2 more bytes to follow
  frame.data.bytes[1] = 1;
  frame.data.bytes[2] = PID;
  CAN0.sendFrame(frame);
}

// Process an incoming CAN message with PID data
void processPID(CAN_FRAME &frame)
{
  int ECT, RPM, MPH;
  float MAP;
  
  if (frame.data.bytes[1] != 0x41)
    return; // not a response to PID request
  
  switch (frame.data.bytes[2])
  {
    case PID_ECT:
      ECT = frame.data.bytes[3] - 40;
      if (ECT != lastTemp)
      {
        Serial.print("Coolant temperature (C): ");
        Serial.println(ECT);
        lastTemp = ECT;
      }
      break;
    
    case PID_MAP:
      MAP = frame.data.bytes[3] * 0.145038; // kPA to PSI
      MAP = MAP - 14.8;
      //psi = psi * 100;
      if (MAP != lastMAP)
      {
        Serial.print("Manifold abs pressure (psi): ");
        Serial.println(MAP);
        lastMAP = MAP;
      }
      break;
    
    case PID_RPM:
      RPM = ((frame.data.bytes[3] * 256) + frame.data.bytes[4]) / 4;
      if (RPM != lastRPM)
      {
        Serial.print("Engine RPM: ");
        Serial.println(RPM);
        lastRPM = RPM;
      }
      break;
    
    case PID_SPEED:
      MPH = frame.data.bytes[3] * 0.621371; // km/h to mph
      if (MPH != lastMPH)
      {
        Serial.print("Vehicle Speed (MPH): ");
        Serial.println(MPH);
        lastMPH = MPH;
      }
      break;
  }
}
