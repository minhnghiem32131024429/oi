#include <AccelStepper.h>
#include <Servo.h>
#include <math.h> // Thêm thư viện toán học

// --- Định nghĩa hướng quay ---
#define X_DIRECTION 1
#define Y_DIRECTION 1

// --- Chân kết nối ---
#define ENABLE_PIN   8
#define X_STEP_PIN   2
#define X_DIR_PIN    5
#define Y_STEP_PIN   3
#define Y_DIR_PIN    6
#define SERVO_PIN    12

// --- Thông số động cơ ---
#define STEPS_PER_DEG 8.8888888889
#define DEFAULT_SPEED 8000 // Giữ giá trị bạn đã tăng
#define DEFAULT_ACCEL 3000 // Giữ giá trị bạn đã tăng

// --- Thông số Robot (PHẢI GIỐNG PYTHON) ---
#define R_L1 15.0
#define R_L2 25.0
#define R_L3 25.0
#define R_L4 15.0
#define R_X1 0.0
#define R_Y1 0.0
#define R_X5 -20.0
#define R_Y5 0.0

// --- Thông số Servo ---
#define PEN_UP_ANGLE   80
#define PEN_DOWN_ANGLE 54.8
bool isPenDown = false;

// --- Biến trạng thái ---
float currentX_angle = 90.0; 
float currentY_angle = 90.0;
bool isProcessingGcode = false;
bool commandCompleted = true;
bool absolutePositioning = true;

// --- Khởi tạo đối tượng ---
Servo penServo;
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
enum Mode { IDLE, MOVE, HOME } mode = IDLE;

// --- Hàm Forward Kinematics (Arduino version) ---
// Trả về true nếu tính toán thành công, false nếu lỗi
// Kết quả được lưu vào tham chiếu x3_out, y3_out
bool forwardKinematics(float theta1_deg, float theta2_deg, float &x3_out, float &y3_out) {
  float theta1 = radians(theta1_deg);
  float theta2 = radians(theta2_deg);

  float x2 = R_X1 + R_L1 * cos(theta1);
  float y2 = R_Y1 + R_L1 * sin(theta1);
  float x4 = R_X5 + R_L4 * cos(theta2);
  float y4 = R_Y5 + R_L4 * sin(theta2);

  float dx = x4 - x2;
  float dy = y4 - y2;
  float d_sq = dx * dx + dy * dy;

  if (d_sq < 1e-9) return false; 
  float d = sqrt(d_sq);

  if (d > R_L2 + R_L3 + 1e-5 || d < abs(R_L2 - R_L3) - 1e-5) return false;

  float div = 2.0 * R_L2 * d;
  if (abs(div) < 1e-9) return false;

  float cos_angle_at_j1 = (R_L2 * R_L2 + d_sq - R_L3 * R_L3) / div;
  if (cos_angle_at_j1 > 1.0) cos_angle_at_j1 = 1.0;
  if (cos_angle_at_j1 < -1.0) cos_angle_at_j1 = -1.0;

  float angle_at_j1 = acos(cos_angle_at_j1);
  float angle_j1_to_j2 = atan2(dy, dx);

  float angle_L2 = angle_j1_to_j2 - angle_at_j1;

  x3_out = x2 + R_L2 * cos(angle_L2);
  y3_out = y2 + R_L2 * sin(angle_L2);

  return true;
}


// --- Hàm gửi phản hồi hoàn thành chuẩn ---
// Luôn gửi MOVE_COMPLETE AT:X,Y theo sau là OK
void sendCompletionResponse(float finalAngleX, float finalAngleY) {
  float finalPosX, finalPosY;
  // Tính toán tọa độ XY cuối cùng
  if (forwardKinematics(finalAngleX, finalAngleY, finalPosX, finalPosY)) {
    Serial.print(F("MOVE_COMPLETE AT:"));
    Serial.print(finalPosX, 2); // Gửi tọa độ X
    Serial.print(F(","));
    Serial.println(finalPosY, 2); // Gửi tọa độ Y
  } else {
    // Nếu FK lỗi, vẫn gửi nhưng với góc để Python biết có vấn đề
    Serial.print(F("MOVE_COMPLETE AT:FK_ERR,")); // Báo lỗi FK
    Serial.print(finalAngleX, 2);
    Serial.print(F(","));
    Serial.println(finalAngleY, 2);
    Serial.println(F("ERROR: FK calculation failed on Arduino")); // Thêm log lỗi
  }
  Serial.println(F("OK")); // Luôn gửi OK sau MOVE_COMPLETE
}


// --- Hàm kiểm tra giới hạn góc (giữ nguyên) ---
bool checkAngleLimits(float theta1, float theta2) {
  const float THETA1_MIN = -120, THETA1_MAX = 150;
  const float THETA2_MIN = 30, THETA2_MAX = 300;
  if (theta1 < THETA1_MIN || theta1 > THETA1_MAX) { /* ... lỗi ... */ return false; }
  if (theta2 < THETA2_MIN || theta2 > THETA2_MAX) { /* ... lỗi ... */ return false; }
  return true;
}

// --- Hàm di chuyển (giữ nguyên) ---
void moveToAngle(AccelStepper &stepper, float targetAngle, int direction = 1) {
  long targetSteps = (long)(targetAngle * STEPS_PER_DEG * direction);
  stepper.moveTo(targetSteps);
}

// --- Hàm enable/disable (giữ nguyên) ---
void enableMotors(bool enabled = true) { digitalWrite(ENABLE_PIN, !enabled); }

// --- Hàm pen (sửa để không gửi OK nữa, sendCompletionResponse sẽ gửi) ---
void penDown() {
  if (!isPenDown) {
    penServo.write(PEN_DOWN_ANGLE); delay(250); isPenDown = true;
    Serial.println(F("PEN_DOWN"));
    Serial.println(F("OK")); // Lệnh tức thời, vẫn gửi OK ngay
  } else {
    Serial.println(F("PEN_ALREADY_DOWN"));
    Serial.println(F("OK"));
  }
}
void penUp() {
  if (isPenDown) {
    penServo.write(PEN_UP_ANGLE); delay(250); isPenDown = false;
    Serial.println(F("PEN_UP"));
    Serial.println(F("OK")); // Lệnh tức thời, vẫn gửi OK ngay
  } else {
     Serial.println(F("PEN_ALREADY_UP"));
     Serial.println(F("OK"));
  }
}

// --- Hàm calibrate (sửa để dùng sendCompletionResponse) ---
void calibratePosition() {
  long stepsX = (long)(90.0 * STEPS_PER_DEG * X_DIRECTION);
  long stepsY = (long)(90.0 * STEPS_PER_DEG * Y_DIRECTION);
  stepperX.setCurrentPosition(stepsX);
  stepperY.setCurrentPosition(stepsY);
  currentX_angle = 90.0;
  currentY_angle = 90.0;
  Serial.println(F("CALIBRATED_TO_90_90"));
  // Gửi phản hồi chuẩn sau khi calibrate
  sendCompletionResponse(currentX_angle, currentY_angle);
}

// --- Hàm home (sửa để dùng sendCompletionResponse) ---
void homePosition() {
  enableMotors(true);
  mode = HOME;
  commandCompleted = false;

  long homeStepsX = (long)(90.0 * STEPS_PER_DEG * X_DIRECTION);
  long homeStepsY = (long)(90.0 * STEPS_PER_DEG * Y_DIRECTION);
  long toleranceSteps = (long)(1.0 * STEPS_PER_DEG);

  if (abs(stepperX.currentPosition() - homeStepsX) < toleranceSteps &&
      abs(stepperY.currentPosition() - homeStepsY) < toleranceSteps)
  {
    Serial.println(F("ALREADY_AT_HOME"));
    commandCompleted = true;
    mode = IDLE;
    // Gửi phản hồi chuẩn ngay cả khi đã ở home
    sendCompletionResponse(90.0, 90.0);
  } else {
    Serial.println(F("HOMING"));
    moveToAngle(stepperX, 90.0, X_DIRECTION);
    moveToAngle(stepperY, 90.0, Y_DIRECTION);
    // loop() sẽ gọi sendCompletionResponse khi di chuyển xong
  }
}

// --- setup() (không đổi nhiều) ---
void setup() {
  Serial.begin(115200);
  pinMode(X_STEP_PIN, OUTPUT); pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT); pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  stepperX.setEnablePin(ENABLE_PIN); stepperX.setPinsInverted(false, false, true);
  stepperY.setEnablePin(ENABLE_PIN); stepperY.setPinsInverted(false, false, true);
  enableMotors(true);

  penServo.attach(SERVO_PIN);
  penUp(); // Gửi OK nội bộ
  delay(300);

  stepperX.setMaxSpeed(DEFAULT_SPEED); stepperX.setAcceleration(DEFAULT_ACCEL);
  stepperY.setMaxSpeed(DEFAULT_SPEED); stepperY.setAcceleration(DEFAULT_ACCEL);

  long initStepsX = (long)(90.0 * STEPS_PER_DEG * X_DIRECTION);
  long initStepsY = (long)(90.0 * STEPS_PER_DEG * Y_DIRECTION);
  stepperX.setCurrentPosition(initStepsX);
  stepperY.setCurrentPosition(initStepsY);
  currentX_angle = 90.0;
  currentY_angle = 90.0;

  Serial.println(F("SCARA_READY"));
  Serial.println(F("OK"));
}

// --- loop() (sửa để dùng sendCompletionResponse) ---
void loop() {
  if (Serial.available() && commandCompleted) {
    char buffer[64];
    uint8_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';
    processCommand(buffer);
  } else if (Serial.available() && !commandCompleted) {
     while(Serial.available()) Serial.read(); // Xóa buffer nếu bận
  }

  bool xMoving = stepperX.distanceToGo() != 0;
  bool yMoving = stepperY.distanceToGo() != 0;
  if (xMoving) stepperX.run();
  if (yMoving) stepperY.run();

  static bool wasMoving = false;
  bool isMovingNow = (xMoving || yMoving);

  if (wasMoving && !isMovingNow && !commandCompleted && (mode == MOVE || mode == HOME)) {
    commandCompleted = true;
    mode = IDLE;

    // Cập nhật góc hiện tại từ steps
    currentX_angle = (float)stepperX.currentPosition() / (STEPS_PER_DEG * X_DIRECTION);
    currentY_angle = (float)stepperY.currentPosition() / (STEPS_PER_DEG * Y_DIRECTION);

    // Gửi phản hồi chuẩn với tọa độ XY cuối cùng
    sendCompletionResponse(currentX_angle, currentY_angle);
  }
  wasMoving = isMovingNow;
}

// --- processCommand() (sửa để không gửi OK cho lệnh di chuyển/home/calibrate) ---
void processCommand(const char* buffer) {
  if (strlen(buffer) == 0) return;

  // Các lệnh tức thời (status, enable, disable, pen, speed, accel, gstart, gend, servo)
  // vẫn gửi OK trực tiếp như trước.
  if (strcmp(buffer, "status") == 0) { /* ... */ Serial.println(F("OK")); }
  else if (strcmp(buffer, "enable") == 0) { /* ... */ Serial.println(F("OK")); }
  else if (strcmp(buffer, "disable") == 0) { /* ... */ Serial.println(F("OK")); }
  else if (strcmp(buffer, "u") == 0 || strcmp(buffer, "up") == 0) { penUp(); } // OK nội bộ
  else if (strcmp(buffer, "d") == 0 || strcmp(buffer, "down") == 0) { penDown(); } // OK nội bộ
  else if (buffer[0] == 's' && (isdigit(buffer[1]) || buffer[1] == '-')) { /* ... */ Serial.println(F("OK")); }
  else if (buffer[0] == 'f' && isdigit(buffer[1])) { /* ... */ Serial.println(F("OK")); }
  else if (buffer[0] == 'a' && isdigit(buffer[1])) { /* ... */ Serial.println(F("OK")); }
  else if (strcmp(buffer, "gstart") == 0) { /* ... */ Serial.println(F("OK")); }
  else if (strcmp(buffer, "gend") == 0) { /* ... */ Serial.println(F("OK")); }

  // Lệnh Calibrate 
  else if (strcmp(buffer, "calibrate") == 0) {
    calibratePosition(); // calibratePosition sẽ gọi sendCompletionResponse (đã bao gồm OK)
  }
  // Lệnh Home
  else if (buffer[0] == 'h' || strcmp(buffer, "home") == 0) {
    homePosition(); // homePosition sẽ gọi sendCompletionResponse nếu đã ở home, hoặc chờ loop() gọi
  }
  // Xử lý G-code
  else if (isProcessingGcode) {
    processGcode(buffer); // processGcode sẽ xử lý việc gửi OK/chờ đợi
  }
  // Lệnh di chuyển góc X,Y 
  else {
    char* comma = strchr(buffer, ',');
    if (comma != nullptr) {
      *comma = '\0';
      float targetX_angle = atof(buffer);
      float targetY_angle = atof(comma + 1);

      if (checkAngleLimits(targetX_angle, targetY_angle)) {
        enableMotors(true);
        mode = MOVE;
        commandCompleted = false;
        moveToAngle(stepperX, targetX_angle, X_DIRECTION);
        moveToAngle(stepperY, targetY_angle, Y_DIRECTION);
        if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {
          commandCompleted = true;
          mode = IDLE;
          sendCompletionResponse(targetX_angle, targetY_angle);
        } else {
          Serial.print(F("MOVING_TO:"));
          Serial.print(targetX_angle, 2); Serial.print(F(","));
          Serial.println(targetY_angle, 2);
        }
      }
    }
    // Lệnh không xác định
    else {
      Serial.print(F("ERROR: Unknown command - ")); Serial.println(buffer); // Không OK
    }
  }
}

// --- processGcode() 
void processGcode(const char* gcode) {
  if (strlen(gcode) < 1 || gcode[0] == ';') { Serial.println(F("OK")); return; } // OK cho comment/trống

  char gcopy[64]; strncpy(gcopy, gcode, sizeof(gcopy) - 1); gcopy[sizeof(gcopy) - 1] = '\0';
  for (char* p = gcopy; *p; ++p) *p = toupper(*p);

  float x = NAN, y = NAN, z = NAN, f = NAN, p = NAN;
  bool hasX = false, hasY = false, hasZ = false, hasF = false, hasP = false;
  // ... (phần parse giữ nguyên) ...
  char* command = strtok(gcopy, " ");
  if (!command) { Serial.println(F("ERROR: Invalid G-code line")); return; }
  char* token;
  while ((token = strtok(NULL, " "))) {
      if (strlen(token) < 2) continue; char code = token[0]; float value = atof(token + 1);
      switch(code) {
          case 'X': x = value; hasX = true; break; case 'Y': y = value; hasY = true; break;
          case 'Z': z = value; hasZ = true; break; case 'F': f = value; hasF = true; break;
          case 'P': p = value; hasP = true; break;
      }
  }

  bool commandRequiresWaiting = false;

  if (strncmp(command, "G0", 2) == 0 || strncmp(command, "G1", 2) == 0) {
    // Xử lý Z (penUp/penDown gửi OK nội bộ)
    if (hasZ) { if (z <= 0) penDown(); else penUp(); }
    // Xử lý F (gửi OK trực tiếp)
    if (hasF && f > 0) { /* ... set speed ... */ Serial.println(F("OK")); }
    // Xử lý XY Move -> Không gửi OK ở đây
    if (hasX && hasY) {
      if (checkAngleLimits(x, y)) {
        enableMotors(true); mode = MOVE; commandCompleted = false;
        commandRequiresWaiting = true; // Đánh dấu chờ loop()
        moveToAngle(stepperX, x, X_DIRECTION); moveToAngle(stepperY, y, Y_DIRECTION);
        Serial.print(F("MOVING_TO:")); /* ... */ Serial.println(y, 2);
        // KHÔNG GỬI OK
      } else { Serial.println(F("ERROR: Angle limits check failed (G-code)")); /* Không OK */ }
    } else if (!hasX && !hasY && (hasZ || hasF)) {
        // Chỉ có Z/F đã xử lý và gửi OK rồi, nhưng gửi thêm OK cho dòng G-code này
         Serial.println(F("OK"));
    } else if (!hasX && !hasY && !hasZ && !hasF) {
        Serial.println(F("OK")); // Dòng G0/G1 trống
    }
    // Thiếu X hoặc Y không gửi OK
  }
  // G4: Dwell (gửi OK sau khi delay)
  else if (strncmp(command, "G4", 2) == 0) { if (hasP && p >= 0) { /* ... delay ... */ Serial.println(F("OK")); } else { /* Lỗi, không OK */ } }
  // G28: Home -> Không gửi OK ở đây
  else if (strncmp(command, "G28", 3) == 0) { homePosition(); commandRequiresWaiting = (mode == HOME); }
  // G90, G91 (gửi OK trực tiếp)
  else if (strncmp(command, "G90", 3) == 0) { /* ... */ Serial.println(F("OK")); }
  else if (strncmp(command, "G91", 3) == 0) { /* ... */ Serial.println(F("OK")); }
  // M3, M5 (penUp/penDown gửi OK nội bộ)
  else if (strncmp(command, "M3", 2) == 0 || strncmp(command, "M03", 3) == 0) { penDown(); }
  else if (strncmp(command, "M5", 2) == 0 || strncmp(command, "M05", 3) == 0) { penUp(); }
  // M17, M18, M84 (gửi OK trực tiếp)
  else if (strncmp(command, "M17", 3) == 0) { /* ... */ Serial.println(F("OK")); }
  else if (strncmp(command, "M84", 3) == 0 || strncmp(command, "M18", 3) == 0) { /* ... */ Serial.println(F("OK")); }
  // Lệnh không xác định -> Không gửi OK
  else { Serial.print(F("ERROR: Unknown G-code - ")); Serial.println(gcode); }

  // Không cần gửi OK cuối cùng ở đây nữa
}