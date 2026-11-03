#include <Arduino.h>
#include <DynamixelShield.h>
#include <Servo.h>

#define PC Serial1

// ---------- Moteurs ----------
const uint32_t DXL_BAUD    = 57600;
const uint8_t  ID_X        = 5;
const uint8_t  ID_Y        = 4;

const float PINION_D_MM    = 40.0f;
const float COUNTS_PER_REV = 4096.0f;
const float CPM_X          = COUNTS_PER_REV / (PI * PINION_D_MM);
const float CPM_Y          = COUNTS_PER_REV / (PI * PINION_D_MM);

const int8_t DIR_X = +1;
const int8_t DIR_Y = +1;

const float MAX_X_MM = 600.0f;
const float MAX_Y_MM = 600.0f;

const uint16_t TICK_MS           = 25;
const int32_t  ARRIVE_TOL        = 8;      // ~0.25 mm
const uint32_t ARRIVE_TIMEOUT_MS = 6000;

// ---------- Stylo ----------
#define PEN_ENABLE 1
const uint8_t  PEN_PIN        = 44;
const int      PEN_UP_ANGLE   = 70;
const int      PEN_DOWN_ANGLE = 25;
const uint16_t PEN_DELAY_MS   = 250;

// ---------- Signalisation ----------
#define BUZZER_ENABLE 1
const uint8_t BUZZER_PIN = 22;
const uint8_t LED_PIN    = 13;

// ---------- Démo au démarrage ----------
#define DEMO_AU_BOOT 0

// ---------- Image RLE ----------
#define MAX_PIXELS         24000          // ex. 150x160 -> 3 Ko de bitmap
const float DRAW_MARGIN_MM = 10.0f;
const float DRAW_MAX_H_MM  = 250.0f;
#define FLIP_Y 0                          // 1 pour inverser l'axe Y
const uint32_t FIELD_TIMEOUT_MS = 3000;

DynamixelShield dxl;
Servo pen;

int32_t offsetX = 0, offsetY = 0;
float   curX = 0.0f, curY = 0.0f;
float   speedMMs   = 40.0f;
float   drawMaxWmm = 250.0f;              // largeur de dessin par défaut (mm)

uint8_t *imgBits = nullptr;
int      imgW = 0, imgH = 0;

struct PointMM {
  float x;
  float y;
};

// Démo : deux carrés (montre les deux axes en mouvement simultané).
static const PointMM DEMO[] = {
  { 50.0f,  50.0f}, {150.0f,  50.0f}, {150.0f, 150.0f}, { 50.0f, 150.0f}, { 50.0f,  50.0f},
  { 80.0f,  80.0f}, {120.0f,  80.0f}, {120.0f, 120.0f}, { 80.0f, 120.0f}, { 80.0f,  80.0f}
};
static const uint16_t DEMO_N = sizeof(DEMO) / sizeof(DEMO[0]);


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

bool drawPath(const PointMM *pts, uint16_t count, const char **err) {
  if (count < 2) {
    *err = "path";
    return false;
  }

  penUp();
  if (!moveTo(pts[0].x, pts[0].y, err)) return false;

  penDown();
  for (uint16_t i = 1; i < count; i++) {
    if (!moveTo(pts[i].x, pts[i].y, err)) {
      penUp();
      return false;
    }
  }
  penUp();
  return true;
}

void initMotors() {
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(2.0);

  bool ok = true;
  const uint8_t ids[2] = {ID_X, ID_Y};
  for (uint8_t k = 0; k < 2; k++) {
    uint8_t id = ids[k];
    if (!dxl.ping(id)) {
      PC.print("ERR ping id ");
      PC.println(id);
      ok = false;
    } else {
      PC.print("INFO model id ");
      PC.print(id);
      PC.print(" = ");
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

  // La position au démarrage devient l'origine (0,0).
  offsetX = presentCounts(ID_X);
  offsetY = presentCounts(ID_Y);
  curX = curY = 0.0f;
  setGoals(0, 0);

  PC.print("CONFIG CPM_X=");
  PC.print(CPM_X, 4);
  PC.print(" CPM_Y=");
  PC.println(CPM_Y, 4);
  PC.println("READY");

  beep(880, 80);
  delay(120);
  beep(1320, 100);
}

static int pendingChar = -1;

static int readCharTimeout(uint32_t ms) {
  if (pendingChar >= 0) {
    int c = pendingChar;
    pendingChar = -1;
    return c;
  }
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (PC.available()) return PC.read();
  }
  return -1;
}

static bool readField(long &value, int &delim) {
  long v = 0;
  bool any = false;
  int c;

  while ((c = readCharTimeout(FIELD_TIMEOUT_MS)) >= 0) {
    if (c == '\r') {                // tolère les fichiers CRLF
      value = v;
      delim = '\n';
      return any;
    }
    if (!any && (c == ' ' || c == '\t')) continue;

    if (c == ':') {
      if (!any) return false;
      value = v;
      delim = c;
      return true;
    }
    if (c == '\n') {
      value = v;
      delim = c;
      return any;
    }
    if (c >= '0' && c <= '9') {
      v = v * 10 + (c - '0');
      any = true;
      if (v > 10000000L) return false;
      continue;
    }
    return false;
  }

  if (any) {
    value = v;
    delim = '\n';
    return true;
  }
  return false;
}

static inline bool pixelBlack(int r, int c) {
  int idx = r * imgW + c;
  return (imgBits[idx >> 3] & (uint8_t)(1 << (idx & 7))) != 0;
}

static bool drawImage(const char **err) {
  if (!imgBits || imgW < 1 || imgH < 1) {
    *err = "image";
    return false;
  }

  float scale = drawMaxWmm / (float)imgW;
  float maxH = DRAW_MAX_H_MM;
  if (MAX_Y_MM - DRAW_MARGIN_MM < maxH) maxH = MAX_Y_MM - DRAW_MARGIN_MM;
  if ((float)imgH * scale > maxH) scale = maxH / (float)imgH;
  if (scale <= 0.0f) {
    *err = "scale";
    return false;
  }

  float wmm = (float)imgW * scale;
  if (DRAW_MARGIN_MM + wmm > MAX_X_MM) {
    *err = "limits";
    return false;
  }

  PC.print("IMG ");
  PC.print(imgW);
  PC.print("x");
  PC.print(imgH);
  PC.print(" scale=");
  PC.print(scale, 3);
  PC.println(" mm/px");

  penUp();
  for (int r = 0; r < imgH; r++) {
    float y = DRAW_MARGIN_MM + (float)((FLIP_Y) ? (imgH - 1 - r) : r) * scale;

    int c = 0;
    while (c < imgW) {
      while (c < imgW && !pixelBlack(r, c)) c++;
      if (c >= imgW) break;

      int s = c;
      while (c < imgW && pixelBlack(r, c)) c++;
      int e = c;                         // exclusif

      float x0 = DRAW_MARGIN_MM + (float)s * scale;
      float x1 = DRAW_MARGIN_MM + (float)e * scale;

      if (!moveTo(x0, y, err)) {
        penUp();
        return false;
      }
      penDown();
      if (!moveTo(x1, y, err)) {
        penUp();
        return false;
      }
      penUp();
    }

    if ((r & 15) == 15) PC.print(".");
  }
  PC.println();
  return true;
}

// Reçoit l'en-tête + les runs, remplit le bitmap, puis dessine.
static void handleImage() {
  long w, h, col;
  int d;

  if (!readField(w, d) || d != ':') {
    PC.println("ERR rle width");
    return;
  }
  if (!readField(h, d) || d != ':') {
    PC.println("ERR rle height");
    return;
  }
  if (!readField(col, d) || d != ':') {
    PC.println("ERR rle color");
    return;
  }

  if (w < 1 || h < 1 || w > 1000 || h > 1000) {
    PC.println("ERR rle size");
    return;
  }
  long pixels = w * h;
  if (pixels > MAX_PIXELS) {
    PC.print("ERR image too big (");
    PC.print(w);
    PC.print("x");
    PC.print(h);
    PC.print(" > ");
    PC.print(MAX_PIXELS);
    PC.println(" px)");
    return;
  }
  if (col != 0 && col != 1) {
    PC.println("ERR rle color");
    return;
  }

  int nbBytes = (int)((pixels + 7) / 8);
  if (imgBits) {
    free(imgBits);
    imgBits = nullptr;
  }
  imgBits = (uint8_t *)calloc(nbBytes, sizeof(uint8_t));
  if (!imgBits) {
    PC.println("ERR malloc");
    return;
  }
  imgW = (int)w;
  imgH = (int)h;

  long idx = 0;
  uint8_t color = (uint8_t)col;
  bool ok = true;

  PC.print("RLE ");
  PC.print(imgW);
  PC.print("x");
  PC.print(imgH);
  PC.print(" color=");
  PC.println(color);

  while (idx < pixels) {
    long len;
    bool got = readField(len, d);
    if (!got || len < 0 || idx + len > pixels) {
      ok = false;
      break;
    }

    if (color == 1) {
      for (long k = 0; k < len; k++) {
        long p = idx + k;
        imgBits[p >> 3] |= (uint8_t)(1 << (p & 7));
      }
    }
    idx += len;
    color = 1 - color;

    if (d == '\n') break;              // fin de la ligne série
  }

  if (!ok || idx != pixels) {
    PC.print("ERR rle data ");
    PC.print(idx);
    PC.print("/");
    PC.println(pixels);
    free(imgBits);
    imgBits = nullptr;
    imgW = imgH = 0;
    return;
  }

  const char *err = "";
  digitalWrite(LED_PIN, LOW);
  bool done = drawImage(&err);
  digitalWrite(LED_PIN, HIGH);
  beep(done ? 1320 : 330, done ? 120 : 300);

  if (done) {
    PC.println("OK IMG");
  } else {
    PC.print("ERR ");
    PC.println(err);
  }

  free(imgBits);
  imgBits = nullptr;
  imgW = imgH = 0;
}

// ============================================================
//  Parseurs de commandes
// ============================================================

static bool parseTwoFloats(const String &line, float &a, float &b) {
  char buf[48];
  line.toCharArray(buf, sizeof(buf));
  char *p = buf + 1;              // saute la lettre de commande
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

#if DEMO_AU_BOOT
  delay(1500);                    // laisse le temps de couper l'alimentation
  const char *err = "";
  if (drawPath(DEMO, DEMO_N, &err)) {
    PC.println("DEMO OK");
  } else {
    PC.print("DEMO ERR ");
    PC.println(err);
  }
#endif
}

void loop() {
  if (!PC.available()) return;

  int c = PC.read();
  if (c < 0) return;
  if (c == '\r' || c == '\n' || c == ' ' || c == '\t') return;

  char cmd = (char)toupper((unsigned char)c);

  // Image RLE : avec préfixe I, ou directement par un chiffre.
  if (cmd == 'I') {
    handleImage();
    return;
  }
  if (c >= '0' && c <= '9') {
    pendingChar = c;
    handleImage();
    return;
  }

  String rest = PC.readStringUntil('\n');
  rest.trim();

  String full;
  full += cmd;
  full += rest;

  const char *err = "unknown";

  switch (cmd) {
    case 'P':                     // PING
      PC.println("PONG");
      break;

    case 'Z':                     // origine = position actuelle
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
      if (parseOneFloat(full, v) && v > 0.5f && v < 200.0f) {
        speedMMs = v;
        PC.println("OK V");
      } else {
        PC.println("ERR speed");
      }
      break;
    }

    case 'W': {
      float wmm = 0.0f;
      if (parseOneFloat(full, wmm) && wmm > 10.0f && wmm <= 500.0f) {
        drawMaxWmm = wmm;
        PC.println("OK W");
      } else {
        PC.println("ERR width");
      }
      break;
    }

    case 'G':                     // déplacement absolu (mm)
    case 'M': {
      float x, y;
      if (!parseTwoFloats(full, x, y)) {
        PC.println("ERR syntax");
        break;
      }
      digitalWrite(LED_PIN, LOW);
      bool ok = moveTo(x, y, &err);
      digitalWrite(LED_PIN, HIGH);
      if (ok) PC.println("OK G");
      else {
        PC.print("ERR ");
        PC.println(err);
      }
      break;
    }

    case 'J': {                   // déplacement relatif (calibration)
      float dx, dy;
      if (!parseTwoFloats(full, dx, dy)) {
        PC.println("ERR syntax");
        break;
      }
      bool ok = moveTo(constrain(curX + dx, 0.0f, MAX_X_MM),
                       constrain(curY + dy, 0.0f, MAX_Y_MM), &err);
      if (ok) PC.println("OK J");
      else {
        PC.print("ERR ");
        PC.println(err);
      }
      break;
    }

    case 'R':
      PC.print("POS ");
      PC.print(presentX(), 3);
      PC.print(" ");
      PC.println(presentY(), 3);
      break;

    case 'T': {
      digitalWrite(LED_PIN, LOW);
      bool ok = drawPath(DEMO, DEMO_N, &err);
      digitalWrite(LED_PIN, HIGH);
      if (ok) PC.println("OK T");
      else {
        PC.print("ERR ");
        PC.println(err);
      }
      break;
    }

    case 'C':
      PC.print("CONFIG CPM_X=");
      PC.print(CPM_X, 4);
      PC.print(" CPM_Y=");
      PC.print(CPM_Y, 4);
      PC.print(" W=");
      PC.print(drawMaxWmm, 1);
      PC.print(" MAXPX=");
      PC.println(MAX_PIXELS);
      break;

    case 'S':
      holdHere();
      PC.println("OK STOP");
      break;

    default:
      PC.println("ERR syntax");
      break;
  }
}
