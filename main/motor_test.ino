
#include <DynamixelShield.h>

#define PC Serial1
DynamixelShield dxl;

const uint8_t ID1 = 1;
const uint8_t ID2 = 2;
const int32_t TEST_DELTA = 1024;   
const uint32_t SETTLE_MS = 3000;

void waitArrive(uint8_t id, int32_t target) {
  uint32_t t0 = millis();
  while (millis() - t0 < SETTLE_MS) {
    int32_t p = (int32_t)lroundf(dxl.getPresentPosition(id));
    if (labs(p - target) <= 20) break;
    delay(20);
  }
  PC.print("id "); PC.print(id);
  PC.print(" target "); PC.print(target);
  PC.print(" present "); PC.println((int32_t)lroundf(dxl.getPresentPosition(id)));
}

void wiggle(uint8_t id) {
  int32_t p0 = (int32_t)lroundf(dxl.getPresentPosition(id));
  PC.print("motor "); PC.print(id); PC.print(" start "); PC.println(p0);

  dxl.setGoalPosition(id, (float)(p0 + TEST_DELTA));
  waitArrive(id, p0 + TEST_DELTA);
  delay(300);

  dxl.setGoalPosition(id, (float)p0);
  waitArrive(id, p0);
  delay(300);
}

void setup() {
  PC.begin(115200);
  delay(300);
  PC.println("=== motor_test ===");

  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  bool found1 = dxl.ping(ID1);
  bool found2 = dxl.ping(ID2);
  PC.print("ping id1: "); PC.println(found1 ? "OK" : "FAIL");
  PC.print("ping id2: "); PC.println(found2 ? "OK" : "FAIL");

  if (!found1 || !found2) {
    while (true) delay(1000);
  }

  PC.print("model id1: "); PC.println(dxl.getModelNumber(ID1));  
  PC.print("model id2: "); PC.println(dxl.getModelNumber(ID2));

  const uint8_t ids[2] = {ID1, ID2};
  for (uint8_t k = 0; k < 2; k++) {
    uint8_t id = ids[k];
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_EXTENDED_POSITION);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, 100);
    dxl.torqueOn(id);
    dxl.ledOn(id);
  }

  PC.println("moving...");
  wiggle(ID1);
  wiggle(ID2);

  dxl.ledOff(ID1);
  dxl.ledOff(ID2);
  PC.println("done. now run paintbot.ino");
}

void loop() {
}
