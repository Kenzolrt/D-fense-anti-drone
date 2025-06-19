#define ENCODER_A 2
#define ENCODER_B 3

#define MOTOR_PWM1 9
#define MOTOR_PWM2 10

#define TICKS_PAR_TOUR 3459.0  // calibré à partir du test
#define MEASURE_INTERVAL 30    // ms

volatile long encoderCount = 0;

float theta_ref = 0.0;
float Kp = 6.5;
float Ki = 9.0;
float Kd = 1.0;
float integral = 0.0;
float prevError = 0.0;

unsigned long lastTime = 0;

void encoderISR() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoderCount--;
  } else {
    encoderCount++;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  pinMode(MOTOR_PWM1, OUTPUT);
  pinMode(MOTOR_PWM2, OUTPUT);

  analogWrite(MOTOR_PWM1, 0);
  analogWrite(MOTOR_PWM2, 0);

  Serial.println("Système prêt. Commandes : SET x.x | RESET | TEST");
}

void loop() {
  unsigned long currentTime = millis();

  // -------- RÉCEPTION COMMANDE --------
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("SET")) {
      theta_ref = cmd.substring(4).toFloat();
      Serial.print("Nouvelle consigne (rad) : ");
      Serial.println(theta_ref, 4);
    } else if (cmd == "RESET") {
      encoderCount = 0;
      integral = 0;
      prevError = 0;
      Serial.println("Réinitialisé.");
    }
  }

  // -------- BOUCLE PID --------
  if (currentTime - lastTime >= MEASURE_INTERVAL) {
    float dt = MEASURE_INTERVAL / 1000.0;

    //  Correction ici : conversion ticks → rad
    float theta = encoderCount * (2 * PI / TICKS_PAR_TOUR);
    float error = theta_ref - theta;

    integral += error * dt;
    if (integral > 10) integral = 10;
    if (integral < -10) integral = -10;

    float derivative = (error - prevError) / dt;
    float control = Kp * error + Ki * integral + Kd * derivative;
    prevError = error;

    control = constrain(control, -255, 255);

    // -------- COMMANDE MOTEUR --------
    int pwm = abs(control);

    if (pwm < 20) {
      analogWrite(MOTOR_PWM1, 0);
      analogWrite(MOTOR_PWM2, 0);
    } else {
      pwm = constrain(pwm, 0, 255);
      if (control > 0) {
        analogWrite(MOTOR_PWM1, 0);
        analogWrite(MOTOR_PWM2, pwm);
      } else {
        analogWrite(MOTOR_PWM1, pwm);
        analogWrite(MOTOR_PWM2, 0);
      }
    }

    // -------- AFFICHAGE --------
    Serial.print("Theta: ");
    Serial.print(theta, 4);
    Serial.print(" rad | Ref: ");
    Serial.print(theta_ref, 4);
    Serial.print(" | Err: ");
    Serial.print(error, 4);
    Serial.print(" | PWM: ");
    Serial.print(control, 1);
    Serial.print(" | Encoder count: ");
    Serial.println(encoderCount);

    lastTime = currentTime;
  }
}