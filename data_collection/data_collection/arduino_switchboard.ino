// ======= Switchboard (MEGA) robust reader: D3..D7 =======
// Prints STATE:abcde every 100ms and EV:Sx:y on debounced edges.
// Set WIRING OPTIONS below to match your wiring.

const uint8_t NUM = 5;
const uint8_t PINS[NUM] = {0,4,7,9,12};

// --------- WIRING OPTIONS ---------
// Option A (RECOMMENDED): SPST-to-GND with pull-ups
//   - one end -> GND, center -> pin, other end unused
//   - USE_INTERNAL_PULLUPS = true
//   - LOGIC_ACTIVE_HIGH = false (pressed/on = LOW)
const bool USE_INTERNAL_PULLUPS = true;   // true for Option A; false for SPDT 5V/GND (Option B)
const bool LOGIC_ACTIVE_HIGH    = false;  // true if "ON" = HIGH (Option B), false if "ON" = LOW (Option A)

// Debounce and reporting
const uint8_t  INTEGRATOR_MAX = 8;        // larger = slower but sturdier (try 8–20)
const unsigned SNAPSHOT_MS    = 100;

uint8_t integrator[NUM];   // 0..INTEGRATOR_MAX
uint8_t stableRaw[NUM];    // last stable RAW level (LOW/HIGH)
unsigned long lastSnapshot = 0;

inline bool rawToLogical(uint8_t raw) {
  // Convert electrical level to logical ON(1)/OFF(0) as you want to see it.
  return LOGIC_ACTIVE_HIGH ? (raw == HIGH) : (raw == LOW);
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < NUM; i++) {
    pinMode(PINS[i], USE_INTERNAL_PULLUPS ? INPUT_PULLUP : INPUT);
    // initialize integrator to current raw so we start stable
    uint8_t r = digitalRead(PINS[i]);
    integrator[i] = r == HIGH ? INTEGRATOR_MAX : 0;
    stableRaw[i]  = r;
  }
  delay(50);
  printSnapshot();
}

void loop() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < NUM; i++) {
    uint8_t r = digitalRead(PINS[i]);

    // Integrator debouncer (Jack Ganssle style):
    if (r == HIGH) {
      if (integrator[i] < INTEGRATOR_MAX) integrator[i]++;
    } else {
      if (integrator[i] > 0) integrator[i]--;
    }

    // Only change stable state when integrator saturates
    uint8_t newStableRaw = stableRaw[i];
    if (integrator[i] == 0)            newStableRaw = LOW;
    else if (integrator[i] == INTEGRATOR_MAX) newStableRaw = HIGH;

    if (newStableRaw != stableRaw[i]) {
      // Edge!
      stableRaw[i] = newStableRaw;
      bool on = rawToLogical(stableRaw[i]);
      Serial.print("EV:S"); Serial.print(i + 1); Serial.print(":");
      Serial.println(on ? 1 : 0);
    }
  }

  // if (now - lastSnapshot >= SNAPSHOT_MS) {
  //   lastSnapshot = now;
  //   printSnapshot();
  // }
}

void printSnapshot() {
  Serial.print("STATE:");
  for (uint8_t i = 0; i < NUM; i++) {
    bool on = rawToLogical(stableRaw[i]);
    Serial.print(on ? '1' : '0');
  }
  Serial.println();
}
