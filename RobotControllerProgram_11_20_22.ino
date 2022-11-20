#include <SoftwareSerial.h>
#include <Stepper.h>
#include <math.h>
#include <MatrixMath.h>
#include <MemoryUsage.h>

/*
int stepper_direction[6] = {7,9,11,13,15,17};
int stepper_steps[6] = {6,8,10,12,14,16};
*/
int stepper_direction[6] = {6,10,14,18,22,26};
int stepper_direction2[6] = {7,11,15,19,23,27};
int stepper_steps[6] = {8,12,16,20,24,28};
int stepper_steps2[6] = {9,13,17,21,25,29};

String message = "";
const int row = 4;
const int col = 4;
String p_m = "";
String velocity_direction = "";
String sendBackString = "stopping";
int step_max = 1000;
float step_min = 100; //90000
int step_get = 100;
int step_fast = 0;
int out = 0;
int mover = 0;
int io_counter = 0;
int joint = 0;
int steps_loc_start = 0;
int steps_loc_stop = 0;
int direction_loc_start = 0;
int direction_loc_stop = 0;
float x_pos = 0;
float y_pos = 0;
float z_pos = 0;
float end_x = 0;
float end_y = 0;
float end_z = 0;
int y_index = 0;
int z_index = 0;
int w_index = 0;
int p_index = 0;
int r_index = 0;
int p_m_int = 0;
int end_index = 0;
int end_x_index = 0;
int end_y_index = 0;
int end_z_index = 0;
int end_w_index = 0;
int end_p_index = 0;
int end_r_index = 0;
int end_start_index = 0;
float work_frame[4][4] = {
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1}
}; //was float
/*float input_matrix[6][4] = {
  {0, radians(-90), 41.694, 0},
  {0, 0, 0, 150},
  {0, radians(90), 0, 0},
  {0, radians(-90), -159.5, 0},
  {0, radians(90), 0, 0},
  {0, 0, -65, 0}
};*/
int start = 0;
int ending = 0;
int led_pin = 12;
int complete_count = 0;
float motor_positions[6] = {90,60,0,0,-60,0};
float v_d_matrix[6] = {0,0,0,0,0,0};
float end_loc[6] = {0,0,0,0,0,0};
int end_loc_len = 6;
float* temp_positions;
int linear_velocity = 5;
float currentPos[6] = {0,0,0,0,0,0};

void setup() {
  
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(300);
  //delay(100); 
  /*
  for (int i=0; i<10; i+=2) {
    stepper_direction[i] = i+3;
    stepper_steps[i] = i+4;
  }
  */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(7, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  
    
  //Serial.println("Running");
  // put your main code here, to run repeatedly:
  //if (Serial.available() > 0) {
  message = Serial.readString();
  sendBackString = "stopping";
  //Serial.print("Recieved!");
  //delayMicroseconds(75);
  //delay(10);
  if (message.startsWith("connect")) {
    //delay(10);

    float motor_positions[6] = {0,60,0,0,-60,0};
    float v_d_matrix[6] = {0,0,0,0,0,0};
    float end_loc[6] = {0,0,0,0,0,0};
    float potMotorZero[6] = {0,0,0,0,0,0};
    float currentDegree[6] = {0,0,0,0,0,0};
    String abcdef = "ABCDEF";
    int startLoc = 6;
    int indexStartLoc = 0;
    int indexEndLoc = 0;
    int numMotors = 1;
    int smallGear = 26;
    int largeGear = 86;
    double gearRatio = largeGear / smallGear;
    double onePotTurn = 102.3;
    float potDegreeInV = onePotTurn / 360;
    float motorDegreeInPot = gearRatio * potDegreeInV;
    // float getPotPos[6] = {0,0,0,0,0,0};
    for (int i=0; i<numMotors; i++) {
      if (i == 0) {
        indexStartLoc = startLoc + 1;
        indexEndLoc = message.indexOf(abcdef[i], startLoc);
      }
      else {
        indexStartLoc = message.indexOf(abcdef[i], startLoc) + 1;
        indexEndLoc = message.indexOf(abcdef[i+1], startLoc) - 1;
      }
      if (i < 6) {
        potMotorZero[i] =  message.substring(indexStartLoc, indexEndLoc).toInt();
      }
      else {
        potMotorZero[i] =  message.substring(indexStartLoc).toInt();
      }
      /*
      Serial.print("startIndex = ");
      Serial.println(indexStartLoc);
      Serial.print("endIndex = ");
      Serial.println(indexEndLoc);
      Serial.print("message = ");
      Serial.println(message.substring(indexStartLoc, indexEndLoc));
      
      Serial.print("currentPos[i] = ");
      Serial.println(currentPos[i]);
      Serial.print("potMotorZero[i] = ");
      Serial.println(potMotorZero[i]);
      Serial.print("pot diff = ");
      Serial.println(potMotorZero[i] - currentPos[i]);
      */
      
      /*
       * ---------------------------------------------------------------------
       * commenting out when not connected to motors
       * ---------------------------------------------------------------------
       * */
      getPotPos(i);
      currentDegree[i] = ((potMotorZero[i] - currentPos[i]) * motorDegreeInPot);
      if (currentDegree[i] != motor_positions[i]) {
        StepperMove(stepper_steps[i], stepper_direction[i], "?", 10, currentDegree[i], motor_positions[i], i);
      }
      
    }
    /*
    for (int i=0; i<6; i++) {
      Serial.println(potMotorZero[i]);
    }
    */

    Serial.print("Connected");
  }
  if (message.startsWith("Set IO")) {
    Serial.print("IO Ready");
  }
  if (message.startsWith("Joint")) {
  //while (out == 0) {
    //message = Serial.readString();
    joint = message.substring(4,5).toInt();
    steps_loc_start = message.indexOf("p", 7) + 1;
    steps_loc_stop = message.indexOf("D", steps_loc_start);
    direction_loc_start = message.indexOf("n", steps_loc_stop) + 1;
    stepper_steps[joint] = message.substring(steps_loc_start, steps_loc_stop).toInt();
    stepper_direction[joint] = message.substring(direction_loc_start).toInt();
    pinMode(stepper_steps[joint], OUTPUT);
    pinMode(stepper_direction[joint], OUTPUT);
    //Serial.print("Step");
    //Serial.print(stepper_steps[joint]);
    //Serial.print("Direction");
    //Serial.print(stepper_direction[joint]);
    Serial.print("Next");
    message = Serial.readString();   
    complete_count += 1;
    if (complete_count == 4) {
      complete_count = 0; 
    }
    
  }
  if (message.startsWith("Speed")) {
    step_get = message.substring(5,message.length()).toInt();
    Serial.print("Speed Set");
    step_fast = (step_min*(step_get/100));
  }
  /*if (message.startsWith("LMove") || message.startsWith("JMove")) {
    start = message.indexOf("J1")+2;
    ending = message.indexOf("J2")-1;
    reverse_angles[0] = message.substring(start, ending).toInt();
    start = message.indexOf("J2")+2;
    ending = message.indexOf("J3")-1;
    reverse_angles[1] = message.substring(start, ending).toInt();
    start = message.indexOf("J3")+2;
    ending = message.indexOf("J4")-1;
    reverse_angles[2] = message.substring(start, ending).toInt();
    start = message.indexOf("J4")+2;
    ending = message.indexOf("J5")-1;
    reverse_angles[3] = message.substring(start, ending).toInt();
    start = message.indexOf("J5")+2;
    reverse_angles[4] = message.substring(start).toInt();
    if (message.startsWith("JMove")) {
      for (int i=0; i<5; i++) {
        if (reverse_angles[i] != 0) {
          Serial.print(reverse_angles[i]);
          StepperMove(stepper_steps[i], stepper_direction[i], reverse_angles[i], step_fast, i);
        }
      }
    }
    Serial.print("Reached");      
  }*/
  if (message.startsWith("JMove")) {
    Serial.print("Spinning");
    p_m = message.substring(7);
    //Serial.print(p_m);
    /*
    Serial.println("starting motor positions");
    for (int i=0; i<6; i++) {
      Serial.println(motor_positions[i]);
    }
    
    Serial.print("(message.substring(6).toInt()-1) = ");
    Serial.println(stepper_direction[(message.substring(6).toInt()-1)]);
    */
    Serial.print("(message.substring(6).toInt()-1) = ");
    Serial.println((message.substring(6).toInt()-1));
    if (p_m.indexOf("-") != -1) {
      //Serial.println("here i am");
      motor_positions[(message.substring(6).toInt()-1)] = StepperMove(stepper_steps[(message.substring(6).toInt()-1)], stepper_direction[(message.substring(6).toInt()-1)], p_m, step_min, motor_positions[(message.substring(6).toInt()-1)], -800, (message.substring(6).toInt()-1));
    }
    else if (p_m.indexOf("+") != -1) {
      //Serial.println("here i am");
      motor_positions[(message.substring(6).toInt()-1)] = StepperMove(stepper_steps[(message.substring(6).toInt()-1)], stepper_direction[(message.substring(6).toInt()-1)], p_m, step_min, motor_positions[(message.substring(6).toInt()-1)], 800, (message.substring(6).toInt()-1));
    }
    
    //motor_positions[(message.substring(6).toInt()-1)] = StepperMove(stepper_steps[(message.substring(6).toInt()-1)], stepper_direction[(message.substring(6).toInt()-1)], p_m, step_min, motor_positions[(message.substring(6).toInt()-1)], 400, (message.substring(6).toInt()-1));
    //Serial.print("stopping");
    //Serial.print(String(motor_positions[(message.substring(6).toInt()-1)]));
    sendBackString = sendBackString + String(motor_positions[0]) + "A" + String(motor_positions[1]) + "B" + String(motor_positions[2]) + "C" + String(motor_positions[3]) + "D" + String(motor_positions[4]) + "E" + String(motor_positions[5]) + "F"; 
    Serial.print(sendBackString);
    /*
    Serial.println("ending motor positions");
    for (int i=0; i<6; i++) {
      Serial.println(motor_positions[i]);
    }
    
    Serial.print("Motor Position[");
    Serial.print((message.substring(6).toInt()-1));
    Serial.print("] = ");
    Serial.println(motor_positions[(message.substring(6).toInt()-1)]);
    */
  }
  if (message.startsWith("LMove")) {
    Serial.print("Spinning");
    
    p_m = message.substring(6,7);
    velocity_direction = message.substring(5,6);
    if (p_m == "+") {
      p_m_int = -1;
    }
    else if (p_m == "-") {
      p_m_int = 1;
    }
    if (velocity_direction == "x") {
      v_d_matrix[0] = linear_velocity * p_m_int;
    }
    else if (velocity_direction == "y") {
      v_d_matrix[1] = linear_velocity * p_m_int;
    }
    else if (velocity_direction == "z") {
      v_d_matrix[2] = linear_velocity * p_m_int;
    }
    //Linear_Move((float*)motor_positions, 6, (float*)v_d_matrix, 6);

    temp_positions = Linear_Move(motor_positions, 6, v_d_matrix, 6, end_loc, 6, 2);
    for (int i=0; i<sizeof(temp_positions); i++) {
      motor_positions[i] = temp_positions[i];
    }

    for (int i=0; i<6; i++) {
      v_d_matrix[i] = 0;
    }
  }
  
  if (message.startsWith("Run")) {
    //Serial.print("Running");
    /*
    y_index = message.indexOf("Y", 0);
    z_index = message.indexOf("Z", y_index);
    end_x_index = message.indexOf("X", z_index);
    end_y_index = message.indexOf("Y", end_x_index);
    end_z_index = message.indexOf("Z", end_y_index);
    v_d_matrix[0] = message.substring(4, y_index).toFloat();
    v_d_matrix[1] = message.substring(y_index+1, z_index).toFloat();
    v_d_matrix[2] = message.substring(z_index+1, end_x_index).toFloat();
    end_loc[0] = message.substring(end_x_index+1,end_y_index).toFloat();
    end_loc[1] = message.substring(end_y_index+1,end_z_index).toFloat();
    end_loc[2] = message.substring(end_z_index+1,message.length()).toFloat();
    */
    
    y_index = message.indexOf("Y", 0);
    z_index = message.indexOf("Z", y_index);
    w_index = message.indexOf("W", z_index);
    p_index = message.indexOf("P", w_index);
    r_index = message.indexOf("R", p_index);
    end_start_index = message.indexOf("E", r_index);
    end_x_index = message.indexOf("X", end_start_index);
    end_y_index = message.indexOf("Y", end_x_index);
    end_z_index = message.indexOf("Z", end_y_index);
    end_w_index = message.indexOf("W", end_z_index);
    end_p_index = message.indexOf("P", end_w_index);
    end_r_index = message.indexOf("R", end_p_index);
    
    v_d_matrix[0] = message.substring(4, y_index).toFloat();
    v_d_matrix[1] = message.substring(y_index+1, z_index).toFloat();
    v_d_matrix[2] = message.substring(z_index+1, w_index).toFloat();
    v_d_matrix[3] = message.substring(w_index+1, p_index).toFloat();
    v_d_matrix[4] = message.substring(p_index+1, r_index).toFloat();
    v_d_matrix[5] = message.substring(r_index+1, end_start_index).toFloat();
    
    end_loc[0] = message.substring(end_start_index+4,end_y_index).toFloat();
    end_loc[1] = message.substring(end_y_index+1,end_z_index).toFloat();
    end_loc[2] = message.substring(end_z_index+1,end_w_index).toFloat();
    end_loc[3] = message.substring(end_w_index+1,end_p_index).toFloat();
    end_loc[4] = message.substring(end_p_index+1,end_r_index).toFloat();
    end_loc[5] = message.substring(end_r_index+1,message.length()).toFloat();

    /*
    Serial.println("Velocity Matrix");
    for (int i=0; i<6; i++) {
      Serial.println(v_d_matrix[i]);
    }
    Serial.println("End Locations");
    for (int i=0; i<6; i++) {
      Serial.println(end_loc[i]);
    }
    
    Serial.println(message);
    Serial.print("X = ");
    Serial.println(v_d_matrix[0]);
    Serial.print("Y = ");
    Serial.println(v_d_matrix[1]);
    Serial.print("Z = ");
    Serial.println(v_d_matrix[2]);
    Serial.print("Next X = ");
    Serial.println(end_loc[0]);
    Serial.print("Next Y = ");
    Serial.println(end_loc[1]);
    Serial.print("Next Z = ");
    Serial.println(end_loc[2]);
    */
    temp_positions = Linear_Move(motor_positions, 6, v_d_matrix, 6, end_loc, 6, 1);
    for (int i=0; i<sizeof(temp_positions); i++) {
      motor_positions[i] = temp_positions[i];
    }
    //Serial.print("Reached");
    //Serial.print("stopping");
  }
  if (message.startsWith("ZeroMotors")) {
    String calString = "cal";
    String indicators = "ABCDEF";
    for (int i=0; i<6; i++) {
      getPotPos(i);
      calString = calString + String(currentPos[i]) + indicators[i];
      // calString = calString + indicators[i];
      motor_positions[i] = 0;
    }
    Serial.println(calString);
    //motor_positions[6] = {0,0,0,0,0,0};
  }

  if (message.startsWith("start")) {
    delay(5);
    Serial.print("s!");
    //StepperMove(stepper_steps[message[6]-1], stepper_direction[message[6]-1], message[7], step_fast, message[6]-1);
  }
  //}
}

String ReadInput(){
  String message = "";
  if (Serial.available() > 0) {
    message = Serial.readString();
    //Serial.print(message);
  }
  /*if (Bluetooth.available() > 0) {
    message = Bluetooth.readString();  
  }*/

  return message;
}

float getPotPos(int jointNum) {
  int pinNum[6] = {A0, A1, A2, A3, A4, A5};
  int sampleAnalogInSum = 0;
  float averageAnalogIn = 0;
  int numSamples = 10;
  for (int i=0; i<numSamples; i++) {
    sampleAnalogInSum = sampleAnalogInSum + analogRead(pinNum[jointNum]);
    delay(10);
  }
  averageAnalogIn = sampleAnalogInSum / numSamples;
  // currentPos[jointNum] = analogRead(pinNum[jointNum]);
  currentPos[jointNum] = averageAnalogIn;
  Serial.print("current pot pos in function = ");
  Serial.println(currentPos[jointNum]);
  return currentPos[jointNum];
}

float StepperMove(int step_pin, int direct_pin, String way, int fast, float joint_pos, float joint_pos_final, int jointNum) {
  int go = 0;
  float steps = 0;
  int deg_moved = 0;
  int check_new_message = -1;
  int way_index = -1;
  float plus_minus = 0;
  String new_message = "";
  String first_part = "";
  String return_string = "";
  String stop_message = "stop";
  const int stepsPerRevolution = 8000; //200
  const int currentSpeed = 60;
  float val = 0;
  float prev_motor_pos = 0;
  float current_motor_pos = 0;
  int joint_num = 0;
  float gear_ratio[6] = {1, 2, 2, 2, 2, 2};
  float current_gear_ratio = 0;
  float micro_steps = 1; //16
  float threshold = 0.1;
  float updatePos = 0;
  bool firstTimeLoop = true;

  Stepper currentStepper(stepsPerRevolution, direct_pin, direct_pin+1, step_pin, step_pin+1);
  currentStepper.setSpeed(currentSpeed);
  Serial.print("joint number - ");
  Serial.println(jointNum);
  if ((way.indexOf("?") != -1)) {
    val = joint_pos_final - joint_pos;
    if (val > 0) {
      way = "+";
      //Serial.println("way = +");
    }
    else if (val < 0) {
      way = "-";
      //Serial.println("way = -");
    }
  }

  if (way.indexOf("+") != -1) {
    //Serial.println("Confirmation of way = +");
    digitalWrite(direct_pin, HIGH);
    first_part = "+";
    plus_minus = 1;
  }
  else if (way.indexOf("-") != -1) {
    //Serial.println("Confirmation of way = -");
    digitalWrite(direct_pin, LOW);
    first_part = "-";
    plus_minus = -1;
  }
  //Serial.print("direct_pin = ");
  //Serial.println(direct_pin);
  for (int i=0; i<6; i++) {
    if (direct_pin == stepper_direction[i]) {
      joint_num = i+1;
      current_gear_ratio = gear_ratio[i];
    }
  }
  //Serial.println("before while loop");
  //while (new_message != "stop") {
  
  while (check_new_message == -1) {  
    //Serial.println(fast);
    //prev_motor_pos = (((((steps / stepsPerRevolution) * 360) * plus_minus) / 2000) + joint_pos) * 100;
    prev_motor_pos = (((((steps / stepsPerRevolution) * 360) * plus_minus) / (current_gear_ratio*micro_steps)) + joint_pos);// * 100;
    /*
    digitalWrite(7, HIGH);
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(fast);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(fast);
    digitalWrite(7, HIGH);
    digitalWrite(step_pin, HIGH);
    delay(fast);
    digitalWrite(step_pin, LOW);
    delay(fast);
    */

    
    currentStepper.step(plus_minus);
    
    steps += 1;
    current_motor_pos = (((((steps / stepsPerRevolution) * 360) * plus_minus) / (current_gear_ratio*micro_steps)) + joint_pos);// * 100;
    //current_motor_pos = (((((steps / stepsPerRevolution) * 360) * plus_minus) / 2000) + joint_pos) * 100;
    /*
    //this gets commented out when really running
    Serial.print("steps = ");
    Serial.print(steps);
    
    //Serial.print("calculation = ");
    //Serial.print(((((steps / stepsPerRevolution) * 360) * plus_minus)) + joint_pos); //

    Serial.print("current_motor_pos = ");
    Serial.print(current_motor_pos); //
    
    Serial.print("joint_pos_final = ");
    Serial.print(joint_pos_final);

    Serial.print("way = ");
    Serial.println(way);
    */
    
    /*
    Serial.print("prev_motor_pos = ");
    Serial.println(prev_motor_pos);
    Serial.print("current_motor_pos = ");
    Serial.println(current_motor_pos);
    
    if (prev_motor_pos != current_motor_pos && joint_pos_final == 800) {
      Serial.print("cnt");
      Serial.print(((((steps / stepsPerRevolution) * 360) * plus_minus) / 2000) + joint_pos);
    }
    */
    //if (prev_motor_pos != current_motor_pos) {
    //THIS IS THE MESSAGE THAT I COMMENTED OUT!!!!!
    
    //if ((abs(abs(prev_motor_pos) - abs(current_motor_pos)) > threshold) || firstTimeLoop == true) {
    if ((abs(abs(updatePos) - abs(current_motor_pos)) > threshold) || firstTimeLoop == true) {
      Serial.print("J");
      Serial.print(joint_num);
      Serial.print("cnt");
      Serial.println(((((steps / stepsPerRevolution) * 360) * plus_minus) / (current_gear_ratio*micro_steps)) + joint_pos);
      //Serial.print(((((steps / stepsPerRevolution) * 360) * plus_minus) / 2000) + joint_pos);
      updatePos = current_motor_pos;
    }
    
    if (way == "-") {
      //if ((((((steps / stepsPerRevolution) * 360) * plus_minus) / 2000) + joint_pos) <= joint_pos_final) {
      if ((((((steps / stepsPerRevolution) * 360) * plus_minus) / (current_gear_ratio*micro_steps)) + joint_pos) <= joint_pos_final) {
        new_message = "stop";
        check_new_message = 0;
      }
    }
    else if (way == "+") {
      //if ((((((steps / stepsPerRevolution) * 360) * plus_minus) / 2000) + joint_pos) >= joint_pos_final) {
      if ((((((steps / stepsPerRevolution) * 360) * plus_minus) / (current_gear_ratio*micro_steps)) + joint_pos) >= joint_pos_final) {
        new_message = "stop";
        check_new_message = 0;
      }
    }
    
    if (val == 0) {
      if (Serial.available() > 0) {
        //Serial.println("DID I GO RIGHT HERE??????????????????????????");
        new_message = Serial.readString();
        check_new_message = new_message.indexOf(stop_message);
      }
    }
    /*
    Serial.print("Check New Message = ");
    Serial.print(check_new_message);
    
    Serial.print("New Message = ");
    Serial.println(new_message);
    */
    firstTimeLoop = false;
  }
  getPotPos(jointNum);
  Serial.print("J");
  Serial.print(joint_num);
  Serial.print("cnt");
  Serial.println(((((steps / stepsPerRevolution) * 360) * plus_minus) / (current_gear_ratio*micro_steps)) + joint_pos);

  //Serial.println("");
  digitalWrite(7, LOW);
  /*
  if (joint_pos_final == 400) {
    return_string = "stopping" + first_part + String(steps);
    Serial.print(return_string);
    first_part = "";
  }
  */
  //joint_pos = ((((steps / stepsPerRevolution) * 360) * plus_minus) / 2000) + joint_pos;
  joint_pos = ((((steps / stepsPerRevolution) * 360) * plus_minus) / (current_gear_ratio*micro_steps)) + joint_pos;
  return joint_pos;
}

float** Forward_Kinematics(float joint_input[], int joint_input_len) {
  float a1 = 10;
  float a2 = 20;
  float a3 = 5;
  float a4 = 5;
  float a5 = 2;
  float a6 = 2;
  float input_matrix[6][4] = {{0, radians(90), 0, a1},
                   {0, 0, a2, 0},
                   {0, radians(+90), 0, 0},
                   {0, radians(-90), 0, (a3+a4)},
                   {0, radians(+90), 0, 0},
                   {0, 0, 0, (a5+a6)}};
  int work_frame[4][4] = {{0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0}};
  int tool_frame[4][4] = {{0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0}};
  float J1_matrix[4][4] = {{0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0}};
  float J2_matrix[4][4] = {{0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0}};
  float J3_matrix[4][4] = {{0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0}};
  float J4_matrix[4][4] = {{0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0}};
  float J5_matrix[4][4] = {{0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0}};
  float J6_matrix[4][4] = {{0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0},
                          {0,0,0,0}};
  float h_temp[4][4] = {{0,0,0,0},
                        {0,0,0,0},
                        {0,0,0,0},
                        {0,0,0,0}};
  float H0_6_Return[4][4] = {{0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0}};
  float** H0_2;
  float** H0_3;
  float** H0_4;
  float** H0_5;
  float** H0_6;
  
  /*float work_frame[4][4] = {
                  {1, 0, 0, 0},
                  {0, 1, 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}
                  };
  float tool_frame[4][4] = {
                  {1, 0, 0, 0},
                  {0, 1, 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}
                  };*/
  //Serial.println("Joint Input");
  for (int i=0; i<joint_input_len; i++) {
    //Serial.println(radians(joint_input[i]));
    //Serial.println((joint_input[i]));
    input_matrix[i][0] = radians(joint_input[i]);
  }
  for (int i=0; i<4; i++) {
    tool_frame[i][i] = 1;
    work_frame[i][i] = 1;
  }
  /*
  Serial.println("J1_matrix before definition \n");
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      Serial.print(J1_matrix[i][j]);
      Serial.print("  ");
    }
    Serial.println("");
  }
  */
  //digitalWrite(LED_BUILTIN, HIGH);
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      if (i == 0) {
        if (j == 0) {
          J1_matrix[i][j] = cos(input_matrix[0][0]);
          J2_matrix[i][j] = cos(input_matrix[1][0]);
          J3_matrix[i][j] = cos(input_matrix[2][0]);
          J4_matrix[i][j] = cos(input_matrix[3][0]);
          J5_matrix[i][j] = cos(input_matrix[4][0]);
          J6_matrix[i][j] = cos(input_matrix[5][0]);
        }
        if (j == 1) {
            J1_matrix[i][j] = -1 * sin(input_matrix[0][0]) * cos(input_matrix[0][1]);
            J2_matrix[i][j] = -1 * sin(input_matrix[1][0]) * cos(input_matrix[1][1]);
            J3_matrix[i][j] = -1 * sin(input_matrix[2][0]) * cos(input_matrix[2][1]);
            J4_matrix[i][j] = -1 * sin(input_matrix[3][0]) * cos(input_matrix[3][1]);
            J5_matrix[i][j] = -1 * sin(input_matrix[4][0]) * cos(input_matrix[4][1]);
            J6_matrix[i][j] = -1 * sin(input_matrix[5][0]) * cos(input_matrix[5][1]);
        }
        if (j == 2) {
            J1_matrix[i][j] = sin(input_matrix[0][0]) * sin(input_matrix[0][1]);
            J2_matrix[i][j] = sin(input_matrix[1][0]) * sin(input_matrix[1][1]);
            J3_matrix[i][j] = sin(input_matrix[2][0]) * sin(input_matrix[2][1]);
            J4_matrix[i][j] = sin(input_matrix[3][0]) * sin(input_matrix[3][1]);
            J5_matrix[i][j] = sin(input_matrix[4][0]) * sin(input_matrix[4][1]);
            J6_matrix[i][j] = sin(input_matrix[5][0]) * sin(input_matrix[5][1]);
        }
        if (j == 3) {
            J1_matrix[i][j] = input_matrix[0][2]*cos(input_matrix[0][0]);
            J2_matrix[i][j] = input_matrix[1][2]*cos(input_matrix[1][0]);
            J3_matrix[i][j] = input_matrix[2][2]*cos(input_matrix[2][0]);
            J4_matrix[i][j] = input_matrix[3][2]*cos(input_matrix[3][0]);
            J5_matrix[i][j] = input_matrix[4][2]*cos(input_matrix[4][0]);
            J6_matrix[i][j] = input_matrix[5][2]*cos(input_matrix[5][0]);
        }
      }
      if (i == 1) {
        if (j == 0) {
          J1_matrix[i][j] = sin(input_matrix[0][0]);
          J2_matrix[i][j] = sin(input_matrix[1][0]);
          J3_matrix[i][j] = sin(input_matrix[2][0]);
          J4_matrix[i][j] = sin(input_matrix[3][0]);
          J5_matrix[i][j] = sin(input_matrix[4][0]);
          J6_matrix[i][j] = sin(input_matrix[5][0]);
        }
        if (j == 1) {
          J1_matrix[i][j] = cos(input_matrix[0][0]) * cos(input_matrix[0][1]);
          J2_matrix[i][j] = cos(input_matrix[1][0]) * cos(input_matrix[1][1]);
          J3_matrix[i][j] = cos(input_matrix[2][0]) * cos(input_matrix[2][1]);
          J4_matrix[i][j] = cos(input_matrix[3][0]) * cos(input_matrix[3][1]);
          J5_matrix[i][j] = cos(input_matrix[4][0]) * cos(input_matrix[4][1]);
          J6_matrix[i][j] = cos(input_matrix[5][0]) * cos(input_matrix[5][1]);
        }
        if (j == 2) {
          J1_matrix[i][j] = -1 * cos(input_matrix[0][0]) * sin(input_matrix[0][1]);
          J2_matrix[i][j] = -1 * cos(input_matrix[1][0]) * sin(input_matrix[1][1]);
          J3_matrix[i][j] = -1 * cos(input_matrix[2][0]) * sin(input_matrix[2][1]);
          J4_matrix[i][j] = -1 * cos(input_matrix[3][0]) * sin(input_matrix[3][1]);
          J5_matrix[i][j] = -1 * cos(input_matrix[4][0]) * sin(input_matrix[4][1]);
          J6_matrix[i][j] = -1 * cos(input_matrix[5][0]) * sin(input_matrix[5][1]);
        }
        if (j == 3) {
          J1_matrix[i][j] = input_matrix[0][2] * sin(input_matrix[0][0]);
          J2_matrix[i][j] = input_matrix[1][2] * sin(input_matrix[1][0]);
          J3_matrix[i][j] = input_matrix[2][2] * sin(input_matrix[2][0]);
          J4_matrix[i][j] = input_matrix[3][2] * sin(input_matrix[3][0]);
          J5_matrix[i][j] = input_matrix[4][2] * sin(input_matrix[4][0]);
          J6_matrix[i][j] = input_matrix[5][2] * sin(input_matrix[5][0]);
        }
      }
        if (i == 2) {
          if (j == 0) {
              J1_matrix[i][j] = 0;
              J2_matrix[i][j] = 0;
              J3_matrix[i][j] = 0;
              J4_matrix[i][j] = 0;
              J5_matrix[i][j] = 0;
              J6_matrix[i][j] = 0;
          }
          if (j == 1) {
            J1_matrix[i][j] = sin(input_matrix[0][1]);
            J2_matrix[i][j] = sin(input_matrix[1][1]);
            J3_matrix[i][j] = sin(input_matrix[2][1]);
            J4_matrix[i][j] = sin(input_matrix[3][1]);
            J5_matrix[i][j] = sin(input_matrix[4][1]);
            J6_matrix[i][j] = sin(input_matrix[5][1]);
          }
          if (j == 2) {
            J1_matrix[i][j] = cos(input_matrix[0][1]);
            J2_matrix[i][j] = cos(input_matrix[1][1]);
            J3_matrix[i][j] = cos(input_matrix[2][1]);
            J4_matrix[i][j] = cos(input_matrix[3][1]);
            J5_matrix[i][j] = cos(input_matrix[4][1]);
            J6_matrix[i][j] = cos(input_matrix[5][1]);
          }
          if (j == 3) {
            J1_matrix[i][j] = (input_matrix[0][3]);
            J2_matrix[i][j] = (input_matrix[1][3]);
            J3_matrix[i][j] = (input_matrix[2][3]);
            J4_matrix[i][j] = (input_matrix[3][3]);
            J5_matrix[i][j] = (input_matrix[4][3]);
            J6_matrix[i][j] = (input_matrix[5][3]);
          }
        }
        if (i == 3) {
          if (j == 0) {
            J1_matrix[i][j] = 0;
            J2_matrix[i][j] = 0;
            J3_matrix[i][j] = 0;
            J4_matrix[i][j] = 0;
            J5_matrix[i][j] = 0;
            J6_matrix[i][j] = 0;
          }
          if (j == 1) {
            J1_matrix[i][j] = 0;
            J2_matrix[i][j] = 0;
            J3_matrix[i][j] = 0;
            J4_matrix[i][j] = 0;
            J5_matrix[i][j] = 0;
            J6_matrix[i][j] = 0;
          }
          if (j == 2) {
            J1_matrix[i][j] = 0;
            J2_matrix[i][j] = 0;
            J3_matrix[i][j] = 0;
            J4_matrix[i][j] = 0;
            J5_matrix[i][j] = 0;
            J6_matrix[i][j] = 0;
          }
          if (j == 3) {
            J1_matrix[i][j] = 1;
            J2_matrix[i][j] = 1;
            J3_matrix[i][j] = 1;
            J4_matrix[i][j] = 1;
            J5_matrix[i][j] = 1;
            J6_matrix[i][j] = 1;
          }
        } 
    }
  }
  /*
  Serial.println("J1_matrix");
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      Serial.print(J1_matrix[i][j]);
      Serial.print("  ");
    }
    Serial.println("");
  }
  */
  //Weird stuff happens with these Dot_Product functions
  //Serial.println("");
  //Serial.println("About to enter function");
  H0_2 = Dot_Product((float*)J1_matrix, (float*)J2_matrix, 4, 4, 4, 4, 1);
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      h_temp[i][j] = H0_2[i][j];
    }
  }
  for (int i = 0; i < 4; i++) {
    delete[] H0_2[i];
  }
  delete[] H0_2;
  
  /*
  Serial.println("H0_2");
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      Serial.print(H0_2[i][j]);
      Serial.print("  ");
    }
    Serial.println("");
  }
  */
  H0_3 = Dot_Product((float*)h_temp, (float*)J3_matrix, 4, 4, 4, 4, 1);
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      h_temp[i][j] = H0_3[i][j];
    }
  }
  for (int i = 0; i < 4; i++) {
    delete[] H0_3[i];
  }
  delete[] H0_3;
  H0_4 = Dot_Product((float*)h_temp, (float*)J4_matrix, 4, 4, 4, 4, 1); 
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      h_temp[i][j] = H0_4[i][j];
    }
  }
  for (int i = 0; i < 4; i++) {
    delete[] H0_4[i];
  }
  delete[] H0_4;
  H0_5 = Dot_Product((float*)h_temp, (float*)J5_matrix, 4, 4, 4, 4, 1);
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      h_temp[i][j] = H0_5[i][j];
    }
  }
  for (int i = 0; i < 4; i++) {
    delete[] H0_5[i];
  }
  delete[] H0_5;
  
  H0_6 = Dot_Product((float*)h_temp, (float*)J6_matrix, 4, 4, 4, 4, 1);

  /*
  Serial.println("H0_6");
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      Serial.print(H0_6[i][j]);
      Serial.print("   ");
    }
    Serial.println("");
  }
  */
  /*
  for (int i = 0; i < 4; i++) {
    delete[] H0_2[i];
    delete[] H0_3[i];
    delete[] H0_4[i];
    delete[] H0_5[i];
  }
  delete[] H0_2;
  delete[] H0_3;
  delete[] H0_4;
  delete[] H0_5;
  */
  float** H0_6_pointer = new float*[4];
  for (int k = 0; k < 4; k++) {
    H0_6_pointer[k] = new float[4];
    for (int i = 0; i < 4; i++) {
      H0_6_pointer[k][i] = H0_6[k][i];
    }
  }
  for (int i = 0; i < 4; i++) {
    delete[] H0_6[i];
  }
  delete[] H0_6;
  /*for (int k = 0; k < (row); k++) {
    for (int i = 0; i < (col); i++) {
      for (int j = 0; j < (row); j++) {
        H0_6_pointer[k][i] = ((*(H0_6 + k * col + j)) * (*(H0_6 + j * col + i)));
      }
    }
  }*/
  
  return H0_6_pointer;
}

float* Jacobian(float joint_pos1[], int joint_pos_len, float velocity[], int veloctiy_len) {
  float delta = 0.25; //degree change for calculating partial derivative;
  float jacobian_matrix[6][6] = {{0,0,0,0,0,0},
                                {0,0,0,0,0,0},
                                {0,0,0,0,0,0},
                                {0,0,0,0,0,0},
                                {0,0,0,0,0,0},
                                {0,0,0,0,0,0}};
  float jacobian_inv[6][6] = {{0,0,0,0,0,0},
                              {0,0,0,0,0,0},
                              {0,0,0,0,0,0},
                              {0,0,0,0,0,0},
                              {0,0,0,0,0,0},
                              {0,0,0,0,0,0}};
  float jacobian_inv_flip[6][6] = {{0,0,0,0,0,0},
                                  {0,0,0,0,0,0},
                                  {0,0,0,0,0,0},
                                  {0,0,0,0,0,0},
                                  {0,0,0,0,0,0},
                                  {0,0,0,0,0,0}};
  float partial_derivative_matrix[4][4] = {{0,0,0,0},
                                          {0,0,0,0},
                                          {0,0,0,0},
                                          {0,0,0,0}};
  float rotation_matrix_transpose[3][3] = {{0,0,0},
                                          {0,0,0},
                                          {0,0,0}};
  float rotation_matrix_derivative[3][3] = {{0,0,0},
                                          {0,0,0},
                                          {0,0,0}};
  float rotation_matrix_home[3][3] = {{0,0,0},
                                      {0,0,0},
                                      {0,0,0}};
  float joint_velocity_matrix[6] = {0,0,0,0,0,0};
  float* joint_velocity_matrix_pointer = new float[6];
  //float* joint_velocity_matrix_pointer[6];
  float** skew_symetric_matrix;
  float** temp;
  float** delta_matrix;
  float** home_matrix;
  float test1[4][4] = {{5,8,6,7},
                        {3,1,7,2},
                        {7,4,5,6},
                        {3,2,8,7}};
  float test2[4][4] = {{1,3,6,9},
                        {9,1,4,5},
                        {9,4,6,7},
                        {2,2,1,1}};
  /*                      
  Serial.println("jacobian_inv flip start");
  for (int i=0; i<6; i++) {
    for (int j=0; j<6; j++) {
      Serial.print(jacobian_inv_flip[j][i]);
      Serial.print("   ");
    }
    Serial.println("");
  }
  */
  //digitalWrite(LED_BUILTIN, HIGH);
  //Serial.println("Home matrix");
  
  //TAKES JOINT POSITIONS AND CONVERTS TO CARTESIAN COORDINATES
  home_matrix = Forward_Kinematics(joint_pos1, joint_pos_len);
  
  /*
  Serial.println("Joint Position");
  for (int i=0; i<joint_pos_len; i++) {
    Serial.println(joint_pos1[i]);
  }
  
  Serial.println("Home Matrix");
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      Serial.print(home_matrix[i][j]);
      Serial.print("   ");
    }
    Serial.println("");
  }
  */
  //digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i < 6; i++) {
    /*
    Serial.println("delta matrix");
    Serial.print("i = ");
    Serial.println(i);
       
    Serial.print("joint_pos1[i]+delta = ");
    Serial.println(joint_pos1[i]);
    */
    joint_pos1[i] = joint_pos1[i] + (delta); 
    delta_matrix = Forward_Kinematics(joint_pos1, joint_pos_len);
    /*
    Serial.println("Delta Matrix");
    for (int j=0; j<4; j++) {
      for (int k=0; k<4; k++) {
        Serial.print(delta_matrix[j][k]);
        Serial.print("   ");
      }
      Serial.println("");
    }
    */
    
    //joint_pos1[i] = 8;
    //Serial.print("joint_pos1[i] before for loops = ");
    //Serial.println(joint_pos1[i]);
    for (int j=0; j<4; j++) {
      for (int k=0; k<4; k++) {
        partial_derivative_matrix[j][k] = ((home_matrix[j][k] - delta_matrix[j][k]) / delta);
        //partial_derivative_matrix[j][k] = ((test1[j][k] - test2[j][k]) / delta);
        /*
        if (j < 3 || k < 3) {
          //Serial.println(home_matrix[j][k]);
          //Serial.println(partial_derivative_matrix[j][k]);
          rotation_matrix_transpose[k][j] = home_matrix[j][k]; // rad
          rotation_matrix_home[j][k] = home_matrix[j][k]; // rad
          rotation_matrix_derivative[j][k] = partial_derivative_matrix[j][k]; //rad/rad
        }
        */
      }
    }
    int counter = 0;
    for (int j=0; j<3; j++) {
      for (int k=0; k<3; k++) {
        //Serial.println(home_matrix[j][k]);
        //Serial.println(partial_derivative_matrix[j][k]);
        //Serial.print("joint_pos1[i] inside for loops = ");
        //Serial.println(joint_pos1[i]);
        rotation_matrix_transpose[k][j] = home_matrix[j][k]; // rad
        rotation_matrix_home[j][k] = home_matrix[j][k]; // rad
        rotation_matrix_derivative[j][k] = partial_derivative_matrix[j][k]; //rad/rad
        //Serial.println(counter);
        //counter++;
      }
    }
    //Serial.print("joint_pos1[i] after for loops = ");
    //Serial.println(joint_pos1[i]);
    skew_symetric_matrix = Dot_Product((float*)rotation_matrix_derivative, (float*)rotation_matrix_transpose, 3, 3, 3, 3, 1);
    //Matrix.Multiply((float*)rotation_matrix_derivative, (float*)rotation_matrix_transpose, 3, 3, 3,(float*)skew_symetric_matrix);  

    /*
    Serial.print("skew symetric matrix ");
    Serial.println(i+1);
    for (int j=0; j<3; j++) {
      for (int k=0; k<3; k++) {
        Serial.print(skew_symetric_matrix[j][k] * 1000);
        Serial.print("   ");
      }
      Serial.println("");
    }
    */
    
    jacobian_matrix[i][0] = partial_derivative_matrix[0][3]; //mm/rad
    jacobian_matrix[i][1] = partial_derivative_matrix[1][3]; //mm/rad
    jacobian_matrix[i][2] = partial_derivative_matrix[2][3]; //mm/rad
    jacobian_matrix[i][3] = skew_symetric_matrix[1][2]; //rad
    jacobian_matrix[i][4] = -1 * skew_symetric_matrix[0][2]; //rad
    jacobian_matrix[i][5] = skew_symetric_matrix[0][1]; //rad
    
    joint_pos1[i] = joint_pos1[i] - delta;
    //Serial.print("joint_pos1[i]-delta = ");
    //Serial.println(joint_pos1[i]);
    //Serial.println("before I delete skew symetric");
    for (int j = 0; j < 4; j++) {
      delete[] delta_matrix[j];
      if (j < 3) {
        delete[] skew_symetric_matrix[j];
      }
    }
    delete[] delta_matrix;
    /*for (int i = 0; i < 3; i++) {
      delete[] skew_symetric_matrix[i];
    }*/
    delete[] skew_symetric_matrix;
  }
  //Serial.println("Jacobian Post for loop");
  /*
  Serial.print("x position = ");
  Serial.println(home_matrix[0][3]);
  Serial.print("y position = ");
  Serial.println(home_matrix[1][3]);
  Serial.print("z position = ");
  Serial.println(home_matrix[2][3]);
  */
  for (int i = 0; i < 4; i++) {
    
    delete[] home_matrix[i];
  }
  delete[] home_matrix;
  //Serial.println("Jacobian Inv");
  for (int i=0; i<6; i++) {
    for (int j=0; j<6; j++) {
      jacobian_inv[i][j] = jacobian_matrix[i][j];
      //Serial.print(jacobian_inv[j][i]);
      //Serial.print("   ");
    }
    //Serial.println("");
  }
  /*
  Serial.println("Velocity Matrix");
  for (int i=0; i<6; i++) {
    Serial.println(velocity[i]);
  }
  */
  Matrix.Invert((double*)jacobian_inv, 6);
  for (int i=0; i<6; i++) {
    for (int j=0; j<6; j++) {
      jacobian_inv_flip[j][i] = jacobian_inv[i][j];
      //Serial.print(jacobian_inv_flip[j][i]);
      //Serial.print("   ");
    }
    //Serial.println("");
  }
  /*
  Serial.println("Jacobian");
  //Serial.println("jacobian_inv flip");
  for (int i=0; i<6; i++) {
    for (int j=0; j<6; j++) {
      Serial.print(jacobian_matrix[j][i]);
      Serial.print("   ");
    }
    Serial.println("");
  }
  
  Serial.println("Jacobian Inverse Flip");
  //Serial.println("jacobian_inv flip");
  for (int i=0; i<6; i++) {
    for (int j=0; j<6; j++) {
      Serial.print(jacobian_inv_flip[j][i]);
      Serial.print("   ");
    }
    Serial.println("");
  }
  */
  temp = Dot_Product((float*)jacobian_inv_flip, (float*)velocity, 6, 6, 6, 1, 2);
  //temp = Dot_Product((float*)jacobian_inv, (float*)velocity, 6, 6, 6, 1, 2);
  
  //Serial.println("Joint Velocity Matrix");
  for (int i=0; i<6; i++) {
    joint_velocity_matrix_pointer[i] = temp[i][0];
    //Serial.println(joint_velocity_matrix_pointer[i]*100);
    //Serial.println(temp[i][0]);
    //Serial.println(temp[1][i]);
  }
  
  for (int k = 0; k < 6; k++) {
    delete temp[k];
  }
  delete[] temp;
  /*
  for (int i=0; i<6; i++) {
    Serial.print("joint_velocity_matrix_pointer[i] = ");
    Serial.println(joint_velocity_matrix_pointer[i]);
    //Serial.println(temp[i][0]);
    //Serial.println(temp[1][i]);
  }
  */
  return joint_velocity_matrix_pointer;
}

float* Linear_Move (float joint_angles[], int joint_angle_len, float EOAT_velocity[], int EOAT_veloctiy_len, float end_point[], int end_point_len, int type) {
  float* j_v_matrix;
  String back_string = "stopping";
  String cnt_string = "cnt";
  String finish_message = "empty";
  float delta_t = 10; ///5delta_t needs to be 10 divided by velocity
  //float delta_t = [10, 10, 10, 10, 10, 10];
  String linear_message = "";
  int times = 0;
  float temp_joint = 0;
  float joint_angles_actual[joint_angle_len];
  float testing = 0;
  //float j_thresh = 0.005;
  float j_thresh = 0.01;
  float** check_pos;
  float EOAT_vector = 0;
  float distLeft = 0;
  float prevCheckPos[4] = {0.0, 0.0 ,0.0, 0.0};
  float checkStep = 0;
  float prevDistLeft = 0;
  float maxV = 0;
  int atEnd[6] = {0, 0, 0, 0, 0, 0};
  float rots[3] = {0, 0, 0};
  int atEndSum = 0;
  bool firstTimeLoop = false;
  int roundedTempJointInt = 0;
  float roundedTempJointFloat = 0.0;

  EOAT_vector = sqrt(pow(EOAT_velocity[0],2) + pow(EOAT_velocity[1],2) + pow(EOAT_velocity[2],2));
  if (EOAT_vector != 0) {
    delta_t = 10/EOAT_vector; 
  }
  else if (EOAT_vector == 0 && (EOAT_velocity[3] != 0 || EOAT_velocity[4] != 0 || EOAT_velocity[5] != 0)) {
    for (int i = 0; i < EOAT_veloctiy_len; i++) {
      // Serial.print("EOAT_velocity[i] = ");
      // Serial.println(abs(EOAT_velocity[i]));
      if (maxV < abs(EOAT_velocity[i])) {
        maxV = abs(EOAT_velocity[i]);
      }
    }
    delta_t = 10 / maxV; 
  }

  //while (linear_message != "stop") {

  check_pos = Forward_Kinematics(joint_angles, joint_angle_len);
  rots[0] = -1 * degrees(atan2(check_pos[2][1], check_pos[2][2]));
  rots[1] = degrees(atan2((-1 * check_pos[2][0]), sqrt(pow(check_pos[2][1], 2) + pow(check_pos[2][2], 2))));
  rots[2] = degrees(atan2(check_pos[1][0], check_pos[0][0]));
  
  if (EOAT_vector != 0) {
    distLeft = sqrt(pow((check_pos[0][3]-end_point[0]),2) + pow((check_pos[1][3]-end_point[1]),2) + pow((check_pos[2][3]-end_point[2]),2));
    checkStep = abs(distLeft - prevDistLeft);
  }
  else if (EOAT_vector == 0 && (EOAT_velocity[3] != 0 || EOAT_velocity[4] != 0 || EOAT_velocity[5] != 0)) {
    distLeft = sqrt(pow((rots[0]-end_point[3]),2) + pow((rots[1]-end_point[4]),2) + pow((rots[2]-end_point[5]),2));
    checkStep = abs(distLeft - prevDistLeft); 
  }
  /*
  Serial.print("distLeft = ");
  Serial.println(distLeft);
  Serial.print("checkStep = ");
  Serial.println(checkStep);
  */
  prevDistLeft = distLeft;
      
  while (linear_message.indexOf("stop") == -1) {
    /*
    linear_message = "";
    Serial.println("Start of While Loop");
    Serial.print("Linear Message = ");
    Serial.println(linear_message);
    
    Serial.println("EOAT_Velocity");
    for (int i=0; i<6; i++) {
      Serial.println(EOAT_velocity[i]);
    }
    */
    /*
    Serial.println("starting joint positions");
    for (int i=0; i<6; i++) {
      Serial.println(joint_angles[i]);
    }
    */
    //j_v_matrix = Jacobian(float joint_angles[], int joint_angle_len, float EOAT_velocity[], int EOAT_veloctiy_len);
    j_v_matrix = Jacobian(joint_angles, joint_angle_len, EOAT_velocity, EOAT_veloctiy_len);

    /*
    Serial.println("j_v_matrix");
    for (int i=0; i<6; i++) {
      Serial.println(j_v_matrix[i]);
    }
    */
    
    //Serial.print("joint angle length = ");
    //Serial.println(joint_angle_len);
    float jointTotal = 0;
    int multiplier = 1000;
    Serial.println("-----------------------------------------------------------------");
    for (int i=0; i<joint_angle_len; i++) {
      /*
      if (EOAT_velocity[i] != 0) {
        delta_t = 10 / EOAT_velocity[i];
      }
      */
      temp_joint = ((delta_t/1000) * j_v_matrix[i]);
      roundedTempJointInt = temp_joint * multiplier;
      roundedTempJointFloat = float(roundedTempJointInt) / multiplier;
      /*
      Serial.print("roundedTempJointInt = ");
      Serial.println(roundedTempJointInt);
      Serial.print("roundedTempJointFloat = ");
      Serial.println(roundedTempJointFloat);
      //temp_joint = ((delta_t/1000));
      */
      
      if (abs(j_v_matrix[i]) > 0.01) {
        Serial.print("joint = ");
        Serial.println(i+1);
        Serial.print("start joint angle = ");
        Serial.println(joint_angles[i]);
      }
      jointTotal = jointTotal + roundedTempJointFloat;
      temp_joint = roundedTempJointFloat;
      joint_angles[i] = joint_angles[i] + temp_joint;
      /*
      Serial.print("temp joint ");
      Serial.print(i+1);
      Serial.print(" = ");
      Serial.println(temp_joint);
      //Serial.println("");
      */
      if (abs(j_v_matrix[i]) > 0.01) {
        Serial.print("joint addition = ");
        Serial.println(temp_joint);
        Serial.print("end joint angle = ");
        Serial.println(joint_angles[i]);
      }
      Serial.println("");
      
      if (temp_joint > 0.01 || temp_joint < -0.01) {
        joint_angles_actual[i] = StepperMove(stepper_steps[i], stepper_direction[i], "?", step_min, (joint_angles[i]-temp_joint), joint_angles[i], i);
        joint_angles[i] = joint_angles_actual[i];
      }
      
      /*
      Serial.println("past stepper function");
      Serial.print("i = ");
      Serial.println(i);
      
      Serial.print("j_v = ");
      Serial.println(j_v_matrix[i]);
      Serial.print("temp = ");
      Serial.println(temp_joint);
      */
    }
    
    Serial.println("");
    Serial.print("joint total = ");
    Serial.println(jointTotal*multiplier);
    if (jointTotal == 0) {
      Serial.println("True");
    }
    else {
      Serial.println("False");
    }
    Serial.println("");
    
    delay(2);
    if (Serial.available() > 0) {
      //Serial.println("Did I go in here????????????????????????????????????????????????????");
      linear_message = Serial.readString();
    }
    if (type == 1) {
      check_pos = Forward_Kinematics(joint_angles, joint_angle_len);
      rots[0] = -1 * degrees(atan2(check_pos[2][1], check_pos[2][2]));
      rots[1] = degrees(atan2((-1 * check_pos[2][0]), sqrt(pow(check_pos[2][1], 2) + pow(check_pos[2][2], 2))));
      rots[2] = degrees(atan2(check_pos[1][0], check_pos[0][0]));

      if (EOAT_vector != 0) {
        distLeft = sqrt(pow((check_pos[0][3]-end_point[0]),2) + pow((check_pos[1][3]-end_point[1]),2) + pow((check_pos[2][3]-end_point[2]),2));
        checkStep = abs(distLeft - prevDistLeft);
      }
      else if (EOAT_vector == 0 && (EOAT_velocity[3] != 0 || EOAT_velocity[4] != 0 || EOAT_velocity[5] != 0)) {
        distLeft = sqrt(pow((rots[0]-end_point[3]),2) + pow((rots[1]-end_point[4]),2) + pow((rots[2]-end_point[5]),2));
        checkStep = abs(distLeft - prevDistLeft);
      }
      /*
      Serial.print("distLeft = ");
      Serial.println(distLeft);
      Serial.print("checkStep = ");
      Serial.println(checkStep);
      */
      // if (distLeft < checkStep || checkStep <= j_thresh) {
      if (distLeft < checkStep) {
        // change EOAT velocity
        if (EOAT_vector != 0) {
          // EOAT_vector = EOAT_vector * 2;
          EOAT_vector = EOAT_vector * (checkStep / distLeft);
          delta_t = 10/EOAT_vector; 
        }
        else if (EOAT_vector == 0 && (EOAT_velocity[3] != 0 || EOAT_velocity[4] != 0 || EOAT_velocity[5] != 0)) {
          // maxV = maxV * 2;
          maxV = maxV * (checkStep / distLeft);
          delta_t = 10/maxV;
        }
      }
      
      /*
      Serial.print("Check X Position = ");
      Serial.println(check_pos[0][3]);
      Serial.print("Check Y Position = ");
      Serial.println(check_pos[1][3]);
      Serial.print("Check Z Position = ");
      Serial.println(check_pos[2][3]);
      Serial.print("times = ");
      Serial.println(times);
      Serial.println("");
      
      Serial.print("End X Position = ");
      Serial.println(end_point[0]);
      Serial.print("End Y Position = ");
      Serial.println(end_point[1]);
      Serial.print("End Z Position = ");
      Serial.println(end_point[2]);
      */
      for (int i=0; i<6; i++) {
        if (i < 3) {
          if (abs(check_pos[i][3]-end_point[i]) < j_thresh) {
            atEnd[i] = 1;
          }
        }
        else if (i >= 3) {
          if (abs(rots[i-3]-end_point[i]) < j_thresh) {
            atEnd[i] = 1;
          }
        }
      }
      for (int i=0; i<6; i++) {
        atEndSum = atEnd[i] + atEndSum;
      }
      if (atEndSum == 6) {
        // Serial.println("THIS IS A BREAK POINT!!!!!!!!!!!!!!!");
        linear_message = "stop";
      }
      atEndSum = 0;
      int atEnd[6] = {0,0,0,0,0,0};
      
      /*
      // REMOVED POSITION CHECKER
      if (abs(check_pos[0][3]-end_point[0]) < j_thresh && abs(check_pos[1][3]-end_point[1]) < j_thresh && abs(check_pos[2][3]-end_point[2]) < j_thresh){
      //if (check_pos[0][3] == end_point[0] && check_pos[1][3] == end_point[1] && check_pos[2][3] == end_point[2]) {
        //Serial.println("STOPPING");
        linear_message = "stop";
      }
      */
      
      prevDistLeft = distLeft;
      for (int i = 0; i < 4; i++) {
        prevCheckPos[i] = check_pos[i][3];
        delete[] check_pos[i];
      }
      delete[] check_pos;
      
    }
    //Serial.print("Times through loop = ");
    //Serial.println(times);
    times++;
    //FREERAM_PRINT;
    //Serial.println("End of While Loop");
    delete[] j_v_matrix;
    
    cnt_string = cnt_string + String(joint_angles[0]) + "A" + String(joint_angles[1]) + "B" + String(joint_angles[2]) + "C" + String(joint_angles[3]) + "D" + String(joint_angles[4]) + "E" + String(joint_angles[5]) + "F"; 
    //I COMMENTED THIS OUT!!!!
    //Serial.print(cnt_string);
    cnt_string = "cnt";
    
  }
  //Serial.println("");
  //Serial.println("DID I BREAK THE WHILE LOOP?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?");
  back_string = back_string + String(joint_angles[0]) + "A" + String(joint_angles[1]) + "B" + String(joint_angles[2]) + "C" + String(joint_angles[3]) + "D" + String(joint_angles[4]) + "E" + String(joint_angles[5]) + "F"; 
  /*
  while(finish_message == "empty"){
    Serial.println(back_string);
    digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds(50);
    if (Serial.available() > 0) {
      //Serial.println("Did I go in here????????????????????????????????????????????????????");
      finish_message = Serial.readString();
      Serial.println("done");
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  */
  Serial.println(back_string);
  return joint_angles; 
}

float** Dot_Product (float* matrix_1, float* matrix_2, int row_1, int col_1, int row_2, int col_2, int type) {
  float** matrix_solution = new float*[row_2];
  float temp1 = 0;
  float temp2 = 0;
  float test_math = 0;
  float temp3 = 0;
  float temp4[row_2];
  if (type == 1) {
    for (int k = 0; k < row_2; k++) {
      matrix_solution[k] = new float[col_1+1];
      // the second time calling this function we get stuck in this loop
      for (int ii = 0; ii < row_2; ii++) {
        //Serial.print("i = ");
        //Serial.println(ii);
        matrix_solution[k][ii] = 0;
      }
      //Serial.println("BREAK");
      //digitalWrite(LED_BUILTIN, LOW);
    }
    for (int i = 0; i < row_1; i++) {
      for (int j = 0; j < col_2; j++) {
        for (int k = 0; k < row_2; k++) {
          matrix_solution[i][j] = ((*(matrix_1 + i * col_1 + k)) * (*(matrix_2 + k * row_1 + j))) + matrix_solution[i][j];
        }
      }
    }
  }
  else if (type == 2) {
    for (int k = 0; k < row_2; k++) {
      matrix_solution[k] = new float[col_2+1];
      for (int i = 0; i < (col_2+1); i++) {
        matrix_solution[k][i] = 0;
      }
    }
    for (int ii = 0; ii < row_2; ii++) {
      temp4[ii] = 0;
      for (int j = 0; j < row_2; j++) {
        temp1 = (*(matrix_1 + ii * col_1 + j));
        //temp2 = (*(matrix_2 + j * col_2 + 0));
        temp2 = (*(matrix_2 + j));
        //for (int k = 0; k < row_2; k++) {
        //matrix_solution[i][0] = ((*(matrix_1 + i * col_1 + j)) * (*(matrix_2 + j * col_2 + 0))) + matrix_solution[i][0];
        test_math = temp1*temp2;
        temp3 = matrix_solution[ii][0];
        //matrix_solution[ii][0] = (temp1 * temp2) + matrix_solution[ii][0];
        temp4[ii] = test_math + temp4[ii];
        //matrix_solution[ii][0] = test_math + temp3;
        /*
        Serial.print("test_math = ");
        Serial.println(test_math);
        Serial.print("temp4 = ");
        Serial.println(temp4[ii]);
        */
        //}
      }
      //matrix_solution[ii][0] = temp4;
      //Serial.print("final row solution = ");
      //Serial.println(temp4[ii]);
    }
    for (int k=0; k<row_2; k++) {
      matrix_solution[k][0] = temp4[k];
    }
  }
  return matrix_solution;
}


/*int doSomething(x, y, z) {
  
  return 
}*/
