#include <Arduino.h>
#include <DynamixelShield.h>
#include <Servo.h>


//il faut changer les parametres avavt de tester

#define PC              Serial1      
const uint32_t DXL_BAUD = 57600;   
const uint8_t  ID_X = 1;             
const uint8_t  ID_Y = 2;            


DynamixelShield dxl;


const float PINION_D_MM   = 40.0f;
const float COUNTS_PER_REV = 4096.0f;                   
const float CPM_BASE = COUNTS_PER_REV / (PI * PINION_D_MM); 
const int8_t DIR_X = +1;
const int8_t DIR_Y = +1;
const float CPM_X = CPM_BASE;
const float CPM_Y = CPM_BASE;
const float MAX_X_MM = 600.0f;
const float MAX_Y_MM = 600.0f;
float speedMMs = 40.0f;             
const uint16_t TICK_MS = 25;         
const int32_t  ARRIVE_TOL = 8;       
const uint32_t ARRIVE_TIMEOUT_MS = 6000;


#define PEN_ENABLE 1
const uint8_t PEN_PIN      = 44;     
const int     PEN_UP_ANGLE   = 70;
const int     PEN_DOWN_ANGLE = 25;
const uint16_t PEN_DELAY_MS  = 250;


#define BUZZER_ENABLE 1
const uint8_t BUZZER_PIN = 22;
const uint8_t LED_PIN    = 13;


Servo pen;
int32_t offsetX = 0, offsetY = 0;     
float curX = 0.0f, curY = 0.0f;       


static long presentCounts(uint8_t id) {
  return (long)lroundf(dxl.getPresentPosition(id));
}

static float presentX() {
  return (float)((presentCounts(ID_X) - offsetX) * DIR_X) / CPM_X;
}
static float presentY() {
  return (float)((presentCounts(ID_Y) - offsetY) * DIR_Y) / CPM_Y;
}

static void setGoals(long cx, long cy) {
  dxl.setGoalPosition(ID_X, (float)(offsetX + (long)DIR_X * cx), UNIT_RAW);
  dxl.setGoalPosition(ID_Y, (float)(offsetY + (long)DIR_Y * cy), UNIT_RAW);
}


static bool parseTwoFloats(const String &line, float &a, float &b) {
  char buf[48];
  line.toCharArray(buf, sizeof(buf));
  char *p = buf + 1;             
  char *end;
  a = (float)strtod(p, &end);
  if (end == p) return false;
  p = end;
  b = (float)strtod(p, &end);
  return end != p;
}

static bool parseOneFloat(const String &line, float &a) {
  char buf[32];
  line.toCharArray(buf, sizeof(buf));
  char *p = buf + 1;
  char *end;
  a = (float)strtod(p, &end);
  return end != p;
}

static void beep(int freq, int ms) {
#if BUZZER_ENABLE
  tone(BUZZER_PIN, freq, ms);
#endif
}

static void penUp() {
#if PEN_ENABLE
  pen.write(PEN_UP_ANGLE);
  delay(PEN_DELAY_MS);
#endif
}

static void penDown() {
#if PEN_ENABLE
  pen.write(PEN_DOWN_ANGLE);
  delay(PEN_DELAY_MS);
#endif
}

static bool stopRequested() {
  if (!PC.available()) return false;
  int c = PC.peek();
  if (c != 'S' && c != 's') return false;
  while (PC.available()) {
    if (PC.read() == '\n') break;
  }
  return true;
}

static void holdHere() {
  float px = presentX();
  float py = presentY();
  setGoals(lroundf(px * CPM_X), lroundf(py * CPM_Y));
  curX = px;
  curY = py;
}

bool moveTo(float x, float y, const char **err) {
  if (x < -0.01f || x > MAX_X_MM || y < -0.01f || y > MAX_Y_MM) {
    *err = "limits";
    return false;
  }

  long cx = lroundf(curX * CPM_X);
  long cy = lroundf(curY * CPM_Y);
  long tx = lroundf(x * CPM_X);
  long ty = lroundf(y * CPM_Y);
  long dx = tx - cx;
  long dy = ty - cy;


  float dist_mm = max(fabsf(dx / CPM_X), fabsf(dy / CPM_Y));
  float mm_per_tick = speedMMs * (float)TICK_MS / 1000.0f;
  int n = max(1, (int)ceilf(dist_mm / max(mm_per_tick, 0.001f)));

  uint32_t t0 = millis();
  for (int i = 1; i <= n; i++) {
    long ix = cx + (long)((int64_t)dx * i / n);
    long iy = cy + (long)((int64_t)dy * i / n);
    setGoals(ix, iy);

    if (stopRequested()) {
      holdHere();
      *err = "stopped";
      return false;
    }

    uint32_t deadline = t0 + (uint32_t)i * TICK_MS;
    while ((int32_t)(millis() - deadline) < 0) {
      if (stopRequested()) {
        holdHere();
        *err = "stopped";
        return false;
      }
    }
  }

  uint32_t start = millis();
  while (millis() - start < ARRIVE_TIMEOUT_MS) {
    long rx = (presentCounts(ID_X) - offsetX) * DIR_X;
    long ry = (presentCounts(ID_Y) - offsetY) * DIR_Y;
    if (labs(rx - tx) <= ARRIVE_TOL && labs(ry - ty) <= ARRIVE_TOL) {
      curX = x;
      curY = y;
      return true;
    }
    if (stopRequested()) {
      holdHere();
      *err = "stopped";
      return false;
    }
    delay(10);
  }

  curX = presentX();
  curY = presentY();
  *err = "timeout";
  return false;
}

void initMotors() {
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(2.0);

  bool ok = true;
  const uint8_t ids[2] = {ID_X, ID_Y};
  for (uint8_t k = 0; k < 2; k++) {
    uint8_t id = ids[k];
    if (!dxl.ping(id)) {
      PC.print("ERR ping id "); PC.println(id);
      ok = false;
    } else {
      PC.print("INFO model id "); PC.print(id); PC.print(" = ");
      PC.println(dxl.getModelNumber(id));
    }
  }
  if (!ok) {
    PC.println("ERR init: motor not found");
    while (true) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      beep(220, 80);
      delay(400);
    }
  }

  for (uint8_t k = 0; k < 2; k++) {
    uint8_t id = ids[k];
    dxl.torqueOff(id);
    if (dxl.readControlTableItem(ControlTableItem::OPERATING_MODE, id)
        != OP_EXTENDED_POSITION) {
      dxl.setOperatingMode(id, OP_EXTENDED_POSITION); 
    }
    dxl.torqueOn(id);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, 0);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, 0);
  }

  
  offsetX = presentCounts(ID_X);
  offsetY = presentCounts(ID_Y);
  curX = curY = 0.0f;
  setGoals(0, 0);

  PC.println("READY");
  beep(880, 80);
  delay(120);
  beep(1320, 100);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  PC.begin(115200);
  PC.setTimeout(200);

  pinMode(BUZZER_PIN, OUTPUT);
#if PEN_ENABLE
  pen.attach(PEN_PIN);
  pen.write(PEN_UP_ANGLE);
  delay(PEN_DELAY_MS);
#endif

  initMotors();
}

void loop() {
  if (!PC.available()) return;

  String line = PC.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = toupper(line[0]);
  const char *err = "unknown";

  switch (cmd) {
    case 'P':  // PING
      PC.println("PONG");
      break;

    case 'Z':  
      offsetX = presentCounts(ID_X);
      offsetY = presentCounts(ID_Y);
      curX = curY = 0.0f;
      setGoals(0, 0);
      PC.println("OK Z");
      break;

    case 'U':
      penUp();
      PC.println("OK U");
      break;

    case 'D':
      penDown();
      PC.println("OK D");
      break;

    case 'V': {
      float v = 40.0f;
      if (parseOneFloat(line, v) && v > 0.5f && v < 200.0f) {
        speedMMs = v;
        PC.println("OK V");
      } else {
        PC.println("ERR speed");
      }
      break;
    }

    case 'G': {
      float x, y;
      if (!parseTwoFloats(line, x, y)) {
        PC.println("ERR syntax");
        break;
      }
      digitalWrite(LED_PIN, LOW);
      bool ok = moveTo(x, y, &err);
      digitalWrite(LED_PIN, HIGH);
      if (ok) PC.println("OK G");
      else { PC.print("ERR "); PC.println(err); }
      break;
    }

    case 'J': {
      float dx, dy;
      if (!parseTwoFloats(line, dx, dy)) {
        PC.println("ERR syntax");
        break;
      }
      bool ok = moveTo(constrain(curX + dx, 0.0f, MAX_X_MM),
                       constrain(curY + dy, 0.0f, MAX_Y_MM), &err);
      if (ok) PC.println("OK J");
      else { PC.print("ERR "); PC.println(err); }
      break;
    }

    case 'R': { 
      PC.print("POS ");
      PC.print(presentX(), 3);
      PC.print(" ");
      PC.println(presentY(), 3);
      break;
    }

    case 'S':
      holdHere();
      PC.println("OK STOP");
      break;

    default:
      PC.println("ERR syntax");
      break;
  }
}
