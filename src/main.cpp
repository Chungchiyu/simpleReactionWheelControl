#include <Arduino.h>
#include <EEPROM.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <Servo.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

typedef struct CONFIG {
  bool not_first_time = false;
  unsigned long motor_on_time = 0;
  unsigned long motor_off_time = 0;
  float launch_Gforce = 0;
  unsigned long pid_on_time = 0;
  bool pid_dir = 0;
  bool pid_delay_on_boot = false;
  double kp = 0, ki = 0, kd = 0;
  double gy_target = 0;
  double motor_init_speed = 0;
  uint8_t motor_raw_max_speed = 180;
} config_t;

enum STATE { UNKONWN, INIT, IDLE, PRELAUNCH, LAUNCH, STOP };
typedef struct _STATUS {
  STATE state = UNKONWN;
  bool pid_on = false;
  int motor_speed = 0;
} status_t;

config_t config;
status_t status;
PID *pid;
double gy_input = 0, motor_output = 0;
WebSocketsServer webSocket = WebSocketsServer(81);
const char *ssid = "ESP reaction wheel";
const char *pwd = "America~FUCKYEAH";
Servo motor;

int eeprom_read(config_t *configs) {
  uint8_t *ptr = (uint8_t *)(configs);
  for (size_t i = 0; i < sizeof(config_t); i++)
    *(ptr + i) = EEPROM.read(i);
  return 0;
}
int eeprom_write(config_t configs) {
  uint8_t *ptr = (uint8_t *)(&configs);
  for (size_t i = 0; i < sizeof(config_t); i++)
    EEPROM.write(i, *(ptr + i));
  return EEPROM.commit();
}

String command(uint8_t *payload) {
  String cmd = (const char *)payload;
  cmd.toLowerCase();
  String msg = "";

  // Set pid control parameter and status command
  if (cmd == "pid on") {
    status.pid_on = true;
    msg = "pid turning on";
  } else if (cmd == "pid off") {
    status.pid_on = false;
    msg = "pid turning off";
  } else if (cmd.substring(0, 11) == "pid on time") {
    config.pid_on_time = (unsigned long)cmd.substring(11).toInt();
    msg = "/set pid on delay time: " + String(config.pid_on_time) + " ms";
  } else if (cmd.substring(0, 7) == "pid dir") {
    bool temp = (bool)cmd.substring(7).toInt();
    if (temp != 0 && temp != 1)
      msg = "can only be 0 or 1";
    else {
      config.pid_dir = temp;
      msg = "/set pid dir: " + String(config.pid_dir);
    }
  } else if (cmd == "pid on boot") {
    config.pid_delay_on_boot = true;
    msg = "/pid turn on delay after boot up";
  } else if (cmd == "pid on launch") {
    config.pid_delay_on_boot = false;
    msg = "/pid turn on delay after launch";
  } else if (cmd.substring(0, 10) == "pid target") {
    double temp = cmd.substring(10).toDouble();
    if (temp < 0 || temp > 100)
      msg = "can not be less than 0% or over 100%";
    else {
      config.gy_target = temp;
      msg = "/set rocket angular velocity target: " + String(config.gy_target) +
            "%";
    }
  } else if (cmd.substring(0, 2) == "kp") {
    config.kp = cmd.substring(2).toDouble();
    msg = "/set kp: " + String(config.kp);
  } else if (cmd.substring(0, 2) == "ki") {
    config.ki = cmd.substring(2).toDouble();
    msg = "/set ki: " + String(config.ki);
  } else if (cmd.substring(0, 2) == "kd") {
    config.kd = cmd.substring(2).toDouble();
    msg = "/set kd: " + String(config.kd);
  }

  // set motor configuration or action command
  else if (cmd.substring(0, 11) == "motor speed") {
    int temp = cmd.substring(11).toInt();
    if (temp < 0 || temp > 100)
      msg = "can not be less than 0% or over 100%";
    else {
      status.motor_speed = temp;
      int speed =
          map(status.motor_speed, 0, 100, 0, config.motor_raw_max_speed);
      motor.write(speed);
      msg = "set motor speed: " + String(status.motor_speed) + "%";
    }
  } else if (cmd.substring(0, 9) == "motor max") {
    uint8_t temp = (uint8_t)cmd.substring(9).toInt();
    if (temp < 0 || temp > 180)
      msg = "can not be less than 0 or over 180 PWM";
    else {
      config.motor_raw_max_speed = temp;
      msg = "/set motor raw max speed: " + String(config.motor_raw_max_speed) +
            " PWM";
    }
  } else if (cmd.substring(0, 10) == "motor init") {
    double temp = cmd.substring(10).toDouble();
    if (temp < 0 || temp > 100)
      msg = "can not be less than 0% or over 100%";
    else {
      config.motor_init_speed = temp;
      msg =
          "/set motor initial speed: " + String(config.motor_init_speed) + "%";
    }
  } else if (cmd.substring(0, 13) == "motor on time") {
    config.motor_on_time = (unsigned long)cmd.substring(13).toInt();
    msg = "/set motor on delay time: " + String(config.motor_on_time) + " ms";
  } else if (cmd.substring(0, 14) == "motor off time") {
    config.motor_off_time = (unsigned long)cmd.substring(14).toInt();
    msg = "/set motor off delay time: " + String(config.motor_off_time) + " ms";
  }

  else if (cmd.substring(0, 6) == "gforce") {
    config.launch_Gforce = cmd.substring(6).toFloat();
    msg = "/set launch detect g-force: " + String(config.launch_Gforce) + "g";
  }

  else if (cmd == "status") {
    msg += "state: " + String(status.state) + "\n";
    msg += "pid is " + String(status.pid_on ? "on" : "off") + "\n";
    msg += "motor is at " + String(status.motor_speed) + "% speed\n";
    msg += "pid last output is " + String(motor_output) + "% speed\n";
    msg += "rocket is at " + String(123) + " rpm\n";
  } else if (cmd.substring(0, 6) == "config") {
    if (cmd.substring(7, 12) == "motor" || cmd.substring(6).isEmpty()) {
      msg += "motor maximum speed is " + String(config.motor_raw_max_speed) +
             " PWM\n";
      msg +=
          "motor initial speed is " + String(config.motor_init_speed) + "%\n";
      msg += "motor on time is " + String(config.motor_on_time) + " ms\n";
      msg += "motor off time is " + String(config.motor_off_time) + " ms\n";
    }
    if (cmd.substring(7, 10) == "pid" || cmd.substring(6).isEmpty()) {
      msg += "pid on time is " + String(config.pid_on_time) + " ms\n";
      msg += "pid direction is " + String(config.pid_dir) + "\n";
      msg += "pid turns on on " +
             String(config.pid_delay_on_boot ? "boot" : "launch") + "\n";
      msg += "pid target is " + String(config.gy_target) + "%\n";
      msg += "kp: " + String(config.kp) + "\n";
      msg += "ki: " + String(config.ki) + "\n";
      msg += "kd: " + String(config.kd) + "\n";
    }
    if (cmd.substring(7, 13) == "launch" || cmd.substring(6).isEmpty()) {
      msg += "launch trigger G-force is at " + String(config.launch_Gforce) +
             "g\n";
    }
    if (msg == "") {
      msg = cmd + "no such command";
    }
  }

  else if (cmd == "restart") {
    webSocket.broadcastTXT("restarting esp32, please reconnect wifi manually");
    ESP.restart();
  } else if (cmd == "format") {
    config_t config_reset;
    config_reset.not_first_time = true;
    eeprom_write(config_reset);
    eeprom_read(&config);
    msg = "format all configuration";
  } else {
    msg = cmd + ": no such command";
  }

  if (msg.substring(0, 1) == "/") {
    eeprom_write(config);
    Serial.println(msg.substring(1));
    return msg.substring(1);
  }

  Serial.println(msg);
  return msg;
}

void deSpinControl(bool ON) {
  if (ON) {
    pid->SetMode(ON);
    // gy_input = (double) sensor.getGyro().y;
    pid->Compute();
    double output = config.motor_init_speed + config.gy_target;
    output = output > 100 ? 100 : output;
    output = output < 0 ? 0 : output;
    output = map(output, 0, 100, 0, config.motor_raw_max_speed);
    motor.write((int)round(output));
  } else {
    pid->SetMode(0);
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length) {

  switch (type) {
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;
  case WStype_CONNECTED: {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0],
                  ip[1], ip[2], ip[3], payload);

    // send message to client
    webSocket.sendTXT(num, "Connected");
  } break;
  case WStype_TEXT:
    Serial.printf("[%u] get Text: %s\n", num, payload);

    webSocket.sendTXT(num, command(payload).c_str());

    break;
  case WStype_BIN:
  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
    break;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("\n\n\nbooting up...");

  EEPROM.begin(sizeof(config_t));
  unsigned long not_first = EEPROM.read(0);
  if (not_first != true) {
    config.not_first_time = true;
    eeprom_write(config);
    Serial.println("EEPROM first time initalizing...");
  } else if (!eeprom_read(&config)) {
    Serial.println("EEPROM reading...");
  };

  // Attach pin and calibrate ESC max throttle
  motor.attach(4, 1000, 2000);
  motor.write(config.motor_raw_max_speed);
  Serial.println("Calibrating motor...");

  pid = new PID(&gy_input, &motor_output, &config.gy_target, config.kp,
                config.ki, config.kd, P_ON_M, config.pid_dir);

  WiFi.softAP(ssid, pwd);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("IP address:\t");
  Serial.println(ip);
  Serial.println("ssid:\t\t" + String(ssid));
  Serial.println("passward:\t" + String(pwd));

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Delay for calibrate ESC
  delay(3000);
  motor.write(0);
  Serial.println("Motor calibration complete");

  Serial.println("System ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  webSocket.loop();

  deSpinControl(status.pid_on);
}