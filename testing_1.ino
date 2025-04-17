#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#define ENABLE_ACCELERATION_PLANNING true   // Bật lập kế hoạch gia tốc
#define MINIMUM_SEGMENT_TIME 20             // Thời gian tối thiểu cho mỗi segment (ms)
#define DEFAULT_ACCELERATION 800            // Gia tốc mặc định (mm/s²)
// Trong Arduino
#define GCODE_BUFFER_SIZE 8      // Số lệnh có thể đệm
#define SEGMENT_BUFFER_SIZE 32   // Số đoạn chuyển động nhỏ có thể đệm

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
#define DEFAULT_SPEED 8000
#define DEFAULT_ACCEL 3000

// --- Mode mở rộng cho đường thẳng ---
#define MAX_LINE_SEGMENTS 100  // Số đoạn tối đa cho nội suy đường thẳng

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
#define PEN_DOWN_ANGLE 45
bool isPenDown = false;

// --- Biến trạng thái ---
float currentX_angle = 90.0; 
float currentY_angle = 90.0;
bool isProcessingGcode = false;
bool commandCompleted = true;
bool absolutePositioning = true;

// --- Dữ liệu cho đường thẳng ---
float straightLineX[MAX_LINE_SEGMENTS];
float straightLineY[MAX_LINE_SEGMENTS];
float segmentVelocity[MAX_LINE_SEGMENTS]; 
int numLineSegments = 0;
int currentLineSegment = 0;

// --- Khởi tạo đối tượng ---
Servo penServo;
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
enum Mode { IDLE, MOVE, HOME, STRAIGHT_LINE } mode = IDLE;

// --- Hàm Forward Kinematics (Arduino version) ---
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

// --- Hàm Inverse Kinematics ---
bool inverseKinematics(float x3, float y3, float &theta1_out, float &theta2_out) {
  // Giới hạn góc
  const float THETA1_MIN = -120.0, THETA1_MAX = 150.0;
  const float THETA2_MIN = 30.0, THETA2_MAX = 300.0;
  
  // Hằng số xử lý sai số
  const float epsilon = 1e-6;
  
  // Tính toán khoảng cách từ điểm (x3,y3) đến gốc motor 1
  float dx13 = x3 - R_X1;
  float dy13 = y3 - R_Y1;
  float L13_sq = dx13*dx13 + dy13*dy13;
  
  float L13;
  if (L13_sq < epsilon*epsilon) {
    L13 = epsilon; // Tránh chia cho 0
  } else {
    L13 = sqrt(L13_sq);
  }
  
  // Kiểm tra xem điểm có nằm trong phạm vi đến được của tay robot không
  if (L13 > R_L1 + R_L2 + epsilon || L13 < abs(R_L1 - R_L2) - epsilon) {
    return false; // Điểm ngoài phạm vi
  }
  
  // Tính beta1 (góc từ gốc đến điểm x3,y3)
  float beta1 = atan2(dy13, dx13);
  
  // Tính alpha1 (góc từ centerline đến link L1)
  float denominator1 = 2 * R_L1 * L13;
  if (denominator1 < epsilon) return false; // Tránh chia cho 0
  
  float cos_alpha1 = (R_L1*R_L1 + L13_sq - R_L2*R_L2) / denominator1;
  
  // Clip giá trị cos_alpha1 vào khoảng [-1,1]
  if (cos_alpha1 > 1.0) cos_alpha1 = 1.0;
  if (cos_alpha1 < -1.0) cos_alpha1 = -1.0;
  
  float alpha1 = acos(cos_alpha1);
  
  // Tính theta1 (2 nghiệm: elbow up và elbow down)
  float theta1_sol1 = degrees(beta1 - alpha1); // elbow down
  float theta1_sol2 = degrees(beta1 + alpha1); // elbow up
  
  // Tương tự cho motor 2
  float dx53 = x3 - R_X5;
  float dy53 = y3 - R_Y5;
  float L53_sq = dx53*dx53 + dy53*dy53;
  
  float L53;
  if (L53_sq < epsilon*epsilon) {
    L53 = epsilon;
  } else {
    L53 = sqrt(L53_sq);
  }
  
  if (L53 > R_L4 + R_L3 + epsilon || L53 < abs(R_L4 - R_L3) - epsilon) {
    return false; // Điểm ngoài phạm vi
  }
  
  float beta5 = atan2(dy53, dx53);
  
  float denominator5 = 2 * R_L4 * L53;
  if (denominator5 < epsilon) return false;
  
  float cos_alpha5 = (R_L4*R_L4 + L53_sq - R_L3*R_L3) / denominator5;
  
  if (cos_alpha5 > 1.0) cos_alpha5 = 1.0;
  if (cos_alpha5 < -1.0) cos_alpha5 = -1.0;
  
  float alpha5 = acos(cos_alpha5);
  
  // Tính theta2 (2 nghiệm: elbow up và elbow down)
  float theta2_sol1 = degrees(beta5 + alpha5); // elbow up
  float theta2_sol2 = degrees(beta5 - alpha5); // elbow down
  
  // Mặc định sử dụng cấu hình elbow-down-down
  float selected_theta1 = theta1_sol1;
  float selected_theta2 = theta2_sol2;
  
  // Kiểm tra xem góc đã tính có nằm trong giới hạn robot không
  if (selected_theta1 < THETA1_MIN || selected_theta1 > THETA1_MAX ||
      selected_theta2 < THETA2_MIN || selected_theta2 > THETA2_MAX) {
    
    // Thử nghiệm với cấu hình khác
    selected_theta1 = theta1_sol2;
    selected_theta2 = theta2_sol2;
    
    if (selected_theta1 < THETA1_MIN || selected_theta1 > THETA1_MAX ||
        selected_theta2 < THETA2_MIN || selected_theta2 > THETA2_MAX) {
      
      // Thử nghiệm với cấu hình khác nữa
      selected_theta1 = theta1_sol1;
      selected_theta2 = theta2_sol1;
      
      if (selected_theta1 < THETA1_MIN || selected_theta1 > THETA1_MAX ||
          selected_theta2 < THETA2_MIN || selected_theta2 > THETA2_MAX) {
        
        // Cuối cùng thử với cấu hình còn lại
        selected_theta1 = theta1_sol2;
        selected_theta2 = theta2_sol1;
        
        if (selected_theta1 < THETA1_MIN || selected_theta1 > THETA1_MAX ||
            selected_theta2 < THETA2_MIN || selected_theta2 > THETA2_MAX) {
          
          // Không có cấu hình nào thỏa mãn
          return false;
        }
      }
    }
  }
  
  // Gán kết quả cho các tham chiếu đầu ra
  theta1_out = selected_theta1;
  theta2_out = selected_theta2;
  
  // Xác nhận đã tìm được góc hợp lệ
  return true;
}

// --- Hàm gửi phản hồi hoàn thành chuẩn ---
void sendCompletionResponse(float finalAngleX, float finalAngleY) {
  float finalPosX, finalPosY;
  if (forwardKinematics(finalAngleX, finalAngleY, finalPosX, finalPosY)) {
    Serial.print(F("MOVE_COMPLETE AT:"));
    Serial.print(finalPosX, 2);
    Serial.print(F(","));
    Serial.println(finalPosY, 2);
  } else {
    Serial.print(F("MOVE_COMPLETE AT:FK_ERR,"));
    Serial.print(finalAngleX, 2);
    Serial.print(F(","));
    Serial.println(finalAngleY, 2);
    Serial.println(F("ERROR: FK calculation failed on Arduino"));
  }
  Serial.println(F("OK"));
}

// --- Hàm kiểm tra giới hạn góc ---
bool checkAngleLimits(float theta1, float theta2) {
  const float THETA1_MIN = -120, THETA1_MAX = 150;
  const float THETA2_MIN = 30, THETA2_MAX = 300;
  if (theta1 < THETA1_MIN || theta1 > THETA1_MAX) return false;
  if (theta2 < THETA2_MIN || theta2 > THETA2_MAX) return false;
  return true;
}

// --- Hàm di chuyển ---
void moveToAngle(AccelStepper &stepper, float targetAngle, int direction = 1) {
  long targetSteps = (long)(targetAngle * STEPS_PER_DEG * direction);
  stepper.moveTo(targetSteps);
}

// --- Hàm enable/disable ---
void enableMotors(bool enabled = true) { 
  digitalWrite(ENABLE_PIN, !enabled); 
}

// --- Hàm tạo đoạn đường thẳng với đường cong vận tốc ---
void generateLineSegments(float startX, float startY, float endX, float endY, int feedrate) {
  // Tính khoảng cách
  float dx = endX - startX;
  float dy = endY - startY;
  float distance = sqrt(dx * dx + dy * dy);
  
  // Tính số đoạn 
  int segments = min((int)(distance / 0.5) + 1, MAX_LINE_SEGMENTS);
  if (segments < 2) segments = 2;  // Đảm bảo ít nhất 2 đoạn
  
  numLineSegments = segments;
  currentLineSegment = 0;
  
  // Tạo đường cong vận tốc hình thang
  float baseVelocity = (float)feedrate;
  float accelFactor = 0.3;  // Tốc độ tăng/giảm (30%)
  
  int accelSegments = min(segments / 4, 5);  // Số đoạn tăng tốc (tối đa 5)
  int decelSegments = min(segments / 4, 5);  // Số đoạn giảm tốc (tối đa 5)
  int steadySegments = segments - accelSegments - decelSegments;  // Số đoạn tốc độ đều
  
  Serial.print(F("Creating line: segments="));
  Serial.println(segments);

  // Tính tốc độ và vị trí cho từng đoạn
  for (int i = 0; i < segments; i++) {
    // Tính tọa độ nội suy
    float t = (float)i / (segments - 1);
    float x = startX + dx * t;
    float y = startY + dy * t;
    
    // Tính tốc độ theo hình thang
    if (i < accelSegments) {
      // Pha tăng tốc
      float factor = accelFactor + (1.0 - accelFactor) * ((float)i / accelSegments);
      segmentVelocity[i] = baseVelocity * factor;
    } else if (i >= accelSegments + steadySegments) {
      // Pha giảm tốc
      int decelIdx = i - (accelSegments + steadySegments);
      float factor = 1.0 - (accelFactor * (float)decelIdx / decelSegments);
      segmentVelocity[i] = max(baseVelocity * factor, baseVelocity * 0.5);
    } else {
      // Pha tốc độ đều
      segmentVelocity[i] = baseVelocity;
    }
    
    // Thực hiện inverse kinematics
    float theta1, theta2;
    if (inverseKinematics(x, y, theta1, theta2)) {
      // Lưu góc cho từng điểm
      straightLineX[i] = theta1;
      straightLineY[i] = theta2;
    } else {
      // Nếu không thể đạt được điểm này
      Serial.print(F("ERROR: IK failed at point "));
      Serial.print(i);
      Serial.print(F(" ("));
      Serial.print(x, 2);
      Serial.print(F(","));
      Serial.print(y, 2);
      Serial.println(F(")"));
      numLineSegments = 0;
      return;
    }
  }
  
  Serial.print(F("Line created with "));
  Serial.print(numLineSegments);
  Serial.println(F(" segments"));
}

// --- Hàm pen ---
void penDown() {
  if (!isPenDown) {
    penServo.write(PEN_DOWN_ANGLE);
    delay(250);
    isPenDown = true;
    Serial.println(F("PEN_DOWN"));
    Serial.println(F("OK"));
  } else {
    Serial.println(F("PEN_ALREADY_DOWN"));
    Serial.println(F("OK"));
  }
}

void penUp() {
  if (isPenDown) {
    penServo.write(PEN_UP_ANGLE);
    delay(250);
    isPenDown = false;
    Serial.println(F("PEN_UP"));
    Serial.println(F("OK"));
  } else {
     Serial.println(F("PEN_ALREADY_UP"));
     Serial.println(F("OK"));
  }
}

// --- Hàm calibrate ---
void calibratePosition() {
  long stepsX = (long)(90.0 * STEPS_PER_DEG * X_DIRECTION);
  long stepsY = (long)(90.0 * STEPS_PER_DEG * Y_DIRECTION);
  stepperX.setCurrentPosition(stepsX);
  stepperY.setCurrentPosition(stepsY);
  currentX_angle = 90.0;
  currentY_angle = 90.0;
  Serial.println(F("CALIBRATED_TO_90_90"));
  sendCompletionResponse(currentX_angle, currentY_angle);
}

// --- Hàm home ---
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
    sendCompletionResponse(90.0, 90.0);
  } else {
    Serial.println(F("HOMING"));
    moveToAngle(stepperX, 90.0, X_DIRECTION);
    moveToAngle(stepperY, 90.0, Y_DIRECTION);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  stepperX.setEnablePin(ENABLE_PIN);
  stepperX.setPinsInverted(false, false, true);
  stepperY.setEnablePin(ENABLE_PIN);
  stepperY.setPinsInverted(false, false, true);
  enableMotors(true);

  penServo.attach(SERVO_PIN);
  penUp();
  delay(300);

  stepperX.setMaxSpeed(DEFAULT_SPEED);
  stepperX.setAcceleration(DEFAULT_ACCEL);
  stepperY.setMaxSpeed(DEFAULT_SPEED);
  stepperY.setAcceleration(DEFAULT_ACCEL);

  long initStepsX = (long)(90.0 * STEPS_PER_DEG * X_DIRECTION);
  long initStepsY = (long)(90.0 * STEPS_PER_DEG * Y_DIRECTION);
  stepperX.setCurrentPosition(initStepsX);
  stepperY.setCurrentPosition(initStepsY);
  currentX_angle = 90.0;
  currentY_angle = 90.0;

  Serial.println(F("SCARA_READY"));
  Serial.println(F("OK"));
}

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

  // Xử lý các loại chuyển động khác nhau
  if (wasMoving && !isMovingNow) {
    if (mode == STRAIGHT_LINE) {
      // Đã hoàn thành một đoạn, di chuyển đến đoạn tiếp theo
      if (currentLineSegment < numLineSegments) {
        // Thiết lập vận tốc cho đoạn tiếp theo
        float speed = segmentVelocity[currentLineSegment];
        stepperX.setMaxSpeed(speed);
        stepperY.setMaxSpeed(speed);
        
        // Di chuyển đến điểm tiếp theo
        float nextX = straightLineX[currentLineSegment];
        float nextY = straightLineY[currentLineSegment];
        moveToAngle(stepperX, nextX, X_DIRECTION);
        moveToAngle(stepperY, nextY, Y_DIRECTION);
        
        // Cập nhật góc hiện tại
        currentX_angle = nextX;
        currentY_angle = nextY;
        
        // Tăng chỉ số đoạn
        currentLineSegment++;
      } else {
        // Hoàn thành tất cả các đoạn
        commandCompleted = true;
        mode = IDLE;
        
        // Khôi phục tốc độ mặc định
        stepperX.setMaxSpeed(DEFAULT_SPEED);
        stepperY.setMaxSpeed(DEFAULT_ACCEL);
        
        // Gửi phản hồi hoàn thành với vị trí cuối cùng
        sendCompletionResponse(currentX_angle, currentY_angle);
      }
    } 
    else if (mode == MOVE || mode == HOME) {
      // Hoàn thành chuyển động đơn
      commandCompleted = true;
      mode = IDLE;

      // Cập nhật góc hiện tại từ steps
      currentX_angle = (float)stepperX.currentPosition() / (STEPS_PER_DEG * X_DIRECTION);
      currentY_angle = (float)stepperY.currentPosition() / (STEPS_PER_DEG * Y_DIRECTION);

      // Gửi phản hồi hoàn thành
      sendCompletionResponse(currentX_angle, currentY_angle);
    }
  }
  wasMoving = isMovingNow;
}

void processCommand(const char* buffer) {
  Serial.print(F("RECEIVED: "));
  Serial.println(buffer);
  if (strlen(buffer) == 0) return;

  // Xử lý lệnh đường thẳng đặc biệt (L command)
  if (buffer[0] == 'L') {
    Serial.println(F("L COMMAND DETECTED"));
    // Lệnh định dạng: LstartX,startY,endX,endY[,Ffeedrate]
    char temp[64];
    strncpy(temp, buffer, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    
    char* startX_str = strchr(temp, 'L') + 1;
    char* startY_str = strchr(startX_str, ',');
    if (!startY_str) { 
      Serial.println(F("ERROR: Invalid line format")); 
      return; 
    }
    *startY_str++ = '\0';
    
    char* endX_str = strchr(startY_str, ',');
    if (!endX_str) { 
      Serial.println(F("ERROR: Invalid line format")); 
      return; 
    }
    *endX_str++ = '\0';
    
    char* endY_str = strchr(endX_str, ',');
    if (!endY_str) { 
      Serial.println(F("ERROR: Invalid line format")); 
      return; 
    }
    *endY_str++ = '\0';
    
    // Parse feedrate nếu có
    int feedrate = DEFAULT_SPEED;
    char* feedrate_str = strchr(endY_str, ',');
    if (feedrate_str && feedrate_str[1] == 'F') {
      *feedrate_str++ = '\0';
      feedrate = atoi(feedrate_str + 1);
    }
    
    // Chuyển đổi chuỗi sang số thực
    float startX = atof(startX_str);
    float startY = atof(startY_str);
    float endX = atof(endX_str);
    float endY = atof(endY_str);
    
    Serial.print(F("Line: ("));
    Serial.print(startX); Serial.print(F(","));
    Serial.print(startY); Serial.print(F(") to ("));
    Serial.print(endX); Serial.print(F(","));
    Serial.print(endY); Serial.print(F("), speed="));
    Serial.println(feedrate);
    
    // Tạo các đoạn đường thẳng nội suy
    enableMotors(true);
    generateLineSegments(startX, startY, endX, endY, feedrate);
    
    if (numLineSegments > 0) {
      // Bắt đầu chế độ vẽ đường thẳng
      mode = STRAIGHT_LINE;
      commandCompleted = false;
      currentLineSegment = 0;
      
      // Di chuyển đến đoạn đầu tiên
      float speed = segmentVelocity[0];
      stepperX.setMaxSpeed(speed);
      stepperY.setMaxSpeed(speed);
      
      float nextX = straightLineX[0];
      float nextY = straightLineY[0];
      moveToAngle(stepperX, nextX, X_DIRECTION);
      moveToAngle(stepperY, nextY, Y_DIRECTION);
      
      Serial.println(F("DRAWING_STRAIGHT_LINE"));
    }
    return; // Không gửi OK ngay, sẽ gửi sau khi hoàn thành
  }
  // Các lệnh tức thời (status, enable, disable, pen, speed, accel)
  else if (strcmp(buffer, "status") == 0) { 
    // Gửi trạng thái
    Serial.print(F("STATUS:")); 
    Serial.print(currentX_angle); Serial.print(F(",")); 
    Serial.println(currentY_angle);
    Serial.println(F("OK")); 
  }
  else if (strcmp(buffer, "enable") == 0) { 
    enableMotors(true); 
    Serial.println(F("MOTORS_ENABLED"));
    Serial.println(F("OK")); 
  }
  else if (strcmp(buffer, "disable") == 0) { 
    enableMotors(false); 
    Serial.println(F("MOTORS_DISABLED"));
    Serial.println(F("OK")); 
  }
  else if (strcmp(buffer, "u") == 0 || strcmp(buffer, "up") == 0) { 
    penUp(); // Bao gồm gửi OK
  }
  else if (strcmp(buffer, "d") == 0 || strcmp(buffer, "down") == 0) { 
    penDown(); // Bao gồm gửi OK
  }
  else if (buffer[0] == 'f' && isdigit(buffer[1])) {
    // Đặt tốc độ
    int speed = atoi(buffer + 1);
    if (speed > 0) {
      stepperX.setMaxSpeed(speed);
      stepperY.setMaxSpeed(speed);
      Serial.print(F("SPEED_SET:")); Serial.println(speed);
    }
    Serial.println(F("OK"));
  }
  else if (buffer[0] == 'a' && isdigit(buffer[1])) {
    // Đặt gia tốc
    int accel = atoi(buffer + 1);
    if (accel > 0) {
      stepperX.setAcceleration(accel);
      stepperY.setAcceleration(accel);
      Serial.print(F("ACCEL_SET:")); Serial.println(accel);
    }
    Serial.println(F("OK"));
  }
  // Lệnh Calibrate 
  else if (strcmp(buffer, "calibrate") == 0) {
    calibratePosition(); // Bao gồm gửi OK
  }
  // Lệnh Home
  else if (buffer[0] == 'h' || strcmp(buffer, "home") == 0) {
    homePosition(); // Gửi OK sau khi hoàn thành
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
      } else {
        Serial.println(F("ERROR: Angle limits check failed"));
      }
    }
    // Lệnh không xác định
    else {
      Serial.print(F("ERROR: Unknown command - ")); 
      Serial.println(buffer);
    }
  }
}
// Thêm vào firmware Arduino
void plan_buffer_line(float x, float y, float feedrate) {
    // Tối ưu đường cong vận tốc cho chuyển động mượt mà
    static float prev_feedrate = 0;
    
    // Tính toán vector chuyển động
    float dx = x - current_position[X_AXIS];
    float dy = y - current_position[Y_AXIS];
    float distance = sqrt(dx*dx + dy*dy);
    
    // Tối ưu vận tốc dựa trên góc và khoảng cách
    float entry_speed = min(prev_feedrate, feedrate);
    float exit_speed = feedrate;
    
    // Thêm vào hàng đợi chuyển động
    addPlannerLine(x, y, entry_speed, exit_speed, feedrate);
    
    // Lưu vận tốc cho lần tiếp theo
    prev_feedrate = feedrate;
}