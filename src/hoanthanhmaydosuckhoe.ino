#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_MLX90614.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_GFX.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSerifBold12pt7b.h>
#include <FreeRTOSConfig.h>

uint8_t max30100_address = 0x57;
uint8_t irmlx90614_address = 0x5A;
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
double new_emissivity = 0.98;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
PulseOximeter pox;
#define BUTTON_UP_PIN 12
#define BUTTON_DOWN_PIN 13
#define BUTTON_SELECT_PIN 14
#define BUTTON_BACK_PIN 26
#define BUZZER_PIN 33
         
#define EVENT_INITIAL_DISPLAY (1 << 0)             //       1
#define EVENT_DISPLAY_VALUES (1 << 1)              //      10
#define EVENT_DISPLAY_CALCULATING (1 << 2)         //     100
#define EVENT_DISPLAY_TEMPERATURE (1 << 3)         //    1000
#define EVENT_DISPLAY_SAVE_MESSAGE (1 << 4)        //   10000  
#define EVENT_DRAW_MENU (1 << 5)                   //  100000

#define EVENT_BUZZER_1   (1 << 0) // Event 1: Buzzer kêu 3 tiếng trong 0.5s
#define EVENT_BUZZER_2   (1 << 1) // Event 2: Buzzer kêu liên tục trong 2s
#define EVENT_BUZZER_3   (1 << 2) // Event 3: Kêu chu kỳ 0.5s 30 lần
#define EVENT_BUZZER_4   (1 << 3) // Event 1: Buzzer kêu liên tục và ngắt quãng báo người dùng đến lúc đo

unsigned long lastActivityTime = 0;
unsigned long buttonPressTime = 0;
struct MenuItem {
  const char* name;
  char value[16];
  const char* unit;
};

MenuItem main_menu[] = {
  { "Hbeat SpO2", "", "" },
  { "Body Temp", "", "" },
  { "Config", "", "" },
  { "History", "", "" },
  { "Timer", "", "" },
  { "Perfect", "", "" }
};

MenuItem submenu_item6[] = {
  { "t", "36-37.2", "oC" },
  { "h", "60-100", "bpm" },
  { "spo2", ">94", "%" },
};

MenuItem submenu_item5[] = {
  { "", "", "second" },
  { "", "", "minutes"},
  { "", "", "hours" },
  { "Repeat", "0", "" },
  { "Save", "", "" }
};

MenuItem submenu_item4[] = {
  { "Temp", "", "oC" },
  { "HR", "", "bpm" },
  { "Spo2", "", "%" },
};

MenuItem submenu_item3[] = {
  { "tem", "", "oC" },
  { "lhr", "", "bpm" },
  { "hhr", "", "bpm" },
  { "spO2", "", "%" },
  { "Save", "", "" }
};

// Số lượng phần tử
const int num_main_items = sizeof(main_menu) / sizeof(main_menu[0]);
const int num_submenu4_items = sizeof(submenu_item4) / sizeof(submenu_item4[0]);
const int num_submenu3_items = sizeof(submenu_item3) / sizeof(submenu_item3[0]);
const int num_submenu5_items = sizeof(submenu_item5) / sizeof(submenu_item5[0]);
const int num_submenu6_items = sizeof(submenu_item6) / sizeof(submenu_item6[0]);
uint32_t tsLastReport1 = 0;
float filterweight = 0.5;
uint32_t last_beat = 0;
int readIndex = 0;
int average_beat = 60;
int average_SpO2 = 97;
float average_temp = 36;
bool calculation_complete = false;
bool calculating = false;

bool is_editing = false;    // Trạng thái chỉnh sửa giá trị
int selected_item = 0;      // Currently selected item
int window_start = 0;       // First item in the display window
int window_back = 0;        // First item in the display window
const int window_size = 3;  // Maximum number of items visible at once

// Menu Level: 0 = Main Menu, 1 = Submenu
int current_level = 0;
int current_parent = -1;  // Parent item index (-1 indicates main menu)
bool br = 0;
float s, p, t;
// Task Handles
TaskHandle_t TaskOLEDHandle = NULL;
TaskHandle_t TaskMenuHandle = NULL;
TaskHandle_t mytaskSensor2 = NULL;
TaskHandle_t mytaskSensor1 = NULL;

TimerHandle_t timer;
EventGroupHandle_t xBuzzerEventGroup; 
EventGroupHandle_t xEventGroup;
SemaphoreHandle_t tem;
SemaphoreHandle_t bpm;
SemaphoreHandle_t sem_timer;


// Function Prototypes
void TaskOLED(void* pvParameters);
void TaskMenu(void* pvParameters);
void adjustValue(int delta);  // Hàm điều chỉnh giá trị
void drawMenu();              // Hàm vẽ menu
void displaySaveMessage();
void saveToEEPROM();
void loadFromEEPROM();
void initializeEEPROM();
void nextItem();
void prevItem();
void enterSubmenu();
void exitSubmenu();
int getNumItems();
void onBeatDetected();
void timerCallback(TimerHandle_t xTimer) {
    xEventGroupSetBits(xBuzzerEventGroup, EVENT_BUZZER_3);
  }

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_BACK_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  display.setTextSize(1);               
  display.setTextColor(SSD1306_WHITE); 
  EEPROM.begin(50);
  loadFromEEPROM();
  // Create tasks
  xBuzzerEventGroup = xEventGroupCreate();
  xEventGroup = xEventGroupCreate();
  xEventGroupSetBits(xBuzzerEventGroup, EVENT_BUZZER_1);
  tem = xSemaphoreCreateBinary();
  bpm = xSemaphoreCreateBinary();
  sem_timer = xSemaphoreCreateBinary();
  xTaskCreate(TaskOLED, "TaskOLED", 4096, NULL, 1, &TaskOLEDHandle);
  xTaskCreate(taskSensor2, "Sensor Task2", 4096, NULL, 2, &mytaskSensor2);
  xTaskCreate(TaskMenu, "TaskMenu", 4096, NULL, 3, &TaskMenuHandle);
  xTaskCreate(taskSensor1, "Sensor Task", 4096, NULL, 2, &mytaskSensor1);
  xTaskCreate(buzzerTask, "Buzzer Task", 2048, NULL, 1, NULL);
  xTaskCreate(software_timer, "timer Task", 2048, NULL, 1, NULL);
}
void loop(){}

void TaskMenu(void* pvParameters) {
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 200;
  MenuItem* menu = nullptr;
  int num_items = 0;
  while (1) {
    if (current_level == 0) {
      menu = main_menu;
      num_items = num_main_items;
    } else if (current_level == 1 && current_parent == 2) {
      menu = submenu_item3;
      num_items = num_submenu3_items;
    }
    // Handle UP button
    if ((digitalRead(BUTTON_UP_PIN) == LOW) && (!br) && (millis() - lastDebounceTime > debounceDelay)) {
      lastDebounceTime = millis();
      if (is_editing) {
        adjustValue(1);  // Tăng giá trị
      } else {
        prevItem();  // Di chuyển lên
      }
      xEventGroupSetBits(xEventGroup, EVENT_DRAW_MENU);
    }

    if (digitalRead(BUTTON_DOWN_PIN) == LOW && (!br) && millis() - lastDebounceTime > debounceDelay) {
      lastDebounceTime = millis();
      if (is_editing) {
        adjustValue(-1);  // Giảm giá trị
      } else {
        nextItem();  // Di chuyển xuống
      }
      xEventGroupSetBits(xEventGroup, EVENT_DRAW_MENU);
    }

    if (digitalRead(BUTTON_SELECT_PIN) == LOW && (!br) && millis() - lastDebounceTime > debounceDelay) {
      lastDebounceTime = millis();
      if (current_level == 1 && (current_parent == 2)||(current_parent == 4)) {
        if (selected_item != 4) {
          is_editing = true;  // Bắt đầu chỉnh sửa giá trị
        } else {
          if(current_parent==4){
          xSemaphoreGive(sem_timer);
          }
          saveToEEPROM();
          xEventGroupSetBits(xEventGroup, EVENT_DISPLAY_SAVE_MESSAGE);  // Hiển thị thông báo "Đã lưu"
        }
      } else if (current_level == 0 && (selected_item == 0)) {
        br = 1;
        xSemaphoreGive(bpm);
      } else if (current_level == 0 && (selected_item == 1)) {
        br = 1;
        xSemaphoreGive(tem);
      } else {
        enterSubmenu();  // Chuyển sang submenu nếu không có giá trị để chỉnh sửa
      }
      xEventGroupSetBits(xEventGroup, EVENT_DRAW_MENU);
    }

    if (digitalRead(BUTTON_BACK_PIN) == LOW && millis() - lastDebounceTime > debounceDelay) {
      lastDebounceTime = millis();
      if (is_editing) {
        is_editing = false;  // Thoát chế độ chỉnh sửa
      } else if (current_level == 0 && ((selected_item == 0) || (selected_item == 1))) {
        br=0;
        xSemaphoreTake(tem, 0);
        xSemaphoreTake(bpm, 0);
      } else {
        exitSubmenu();  // Quay lại menu cha
      }
      xEventGroupSetBits(xEventGroup, EVENT_DRAW_MENU);
    }
    vTaskDelay(10);
  }
}

void TaskOLED(void* pvParameters) {
  display.clearDisplay();
  display.setFont(&FreeSerifBold12pt7b);
  display.setCursor(20, 15);
  display.println("Welcom");
  display.setCursor(27, 45);
  display.println("BACK");
  display.display();
  display.setTextSize(1);
  vTaskDelay(2000);
  xEventGroupSetBits(xEventGroup, EVENT_DRAW_MENU);
  while (1) {
    EventBits_t uxBits = xEventGroupWaitBits(
      xEventGroup,
      EVENT_INITIAL_DISPLAY | EVENT_DISPLAY_VALUES | EVENT_DISPLAY_CALCULATING | EVENT_DISPLAY_TEMPERATURE | EVENT_DISPLAY_SAVE_MESSAGE | EVENT_DRAW_MENU,
      pdTRUE,   // Clear the bits after handling
      pdFALSE,  // Do not wait for all bits
      portMAX_DELAY);
    if (uxBits & EVENT_INITIAL_DISPLAY) {
      initial_display();
    }
        ///010
        ///100
        ///110
    if (uxBits & EVENT_DISPLAY_VALUES) {
      display_values();
    }

    if (uxBits & EVENT_DISPLAY_CALCULATING) {
      display_calculating();
    }

    if (uxBits & EVENT_DISPLAY_TEMPERATURE) {
      display_temp();
    }

    if (uxBits & EVENT_DISPLAY_SAVE_MESSAGE) {
      displaySaveMessage();
    }

    if (uxBits & EVENT_DRAW_MENU) {
      drawMenu();
    }
  }
}

void taskSensor1(void* pvParameters) {
  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;);
  } else {
    Serial.println("SUCCESS");
  }
  pox.setOnBeatDetectedCallback(onBeatDetected);
  for (;;) {
    if (xSemaphoreTake(bpm, 0) == pdTRUE){
      pox.resume();
      calculation_complete = false;
      xEventGroupSetBits(xEventGroup, EVENT_INITIAL_DISPLAY);
      vTaskDelay(1);
      while (!calculation_complete){
        if (!br){
             = false;
          average_beat = atoi(submenu_item4[1].value);
          average_SpO2 = atoi(submenu_item4[2].value);
          break;
        }
        pox.update();
        if ((millis() - last_beat > 500) && (calculating)){
          p = pox.getHeartRate();
          s = pox.getSpO2();
          if ((p > 30) && (p < 220) && (s > 50)){
            if (readIndex == 10) {
              snprintf(submenu_item4[1].value, sizeof(submenu_item4[1].value), "%d", average_beat);  // Cập nhật giá trị
              EEPROM.put(20, atoi(submenu_item4[1].value));
              snprintf(submenu_item4[2].value, sizeof(submenu_item4[2].value), "%d", average_SpO2);  // Cập nhật giá trị
              EEPROM.put(44, atoi(submenu_item4[2].value));
              EEPROM.commit();
              calculation_complete = true;
              readIndex = 0;
              br = 0;
              if((average_SpO2 < atoi(submenu_item3[3].value))||(average_beat < atoi(submenu_item3[1].value))||(average_beat > atoi(submenu_item3[2].value))){
                  xEventGroupSetBits(xBuzzerEventGroup, EVENT_BUZZER_2);
              }else{
                xEventGroupSetBits(xBuzzerEventGroup, EVENT_BUZZER_1);
                }
              xEventGroupSetBits(xEventGroup, EVENT_DISPLAY_VALUES);
              break;
            }
            average_beat = filterweight * (p) + (1 - filterweight) * average_beat;
            average_SpO2 = filterweight * (s) + (1 - filterweight) * average_SpO2;
            readIndex++;
            calculating = false;
            xEventGroupSetBits(xEventGroup, EVENT_DISPLAY_CALCULATING);
          }
        }
        if ((millis() - last_beat > 5000)){
          calculation_complete = false;
          readIndex=0;
          average_beat = 0;
          average_SpO2 = 0;
          xEventGroupSetBits(xEventGroup, EVENT_INITIAL_DISPLAY);
        }
        vTaskDelay(1);
      }
    }
    portYIELD();
    pox.shutdown();
  }
}

void taskSensor2(void* pvParameters){
  mlx.begin(0x5A, &Wire);
  Serial.print("Current emissivity = ");
  Serial.println(mlx.readEmissivity());
  Serial.print("Setting emissivity = ");
  Serial.println(new_emissivity);
  mlx.writeEmissivity(new_emissivity);  
  Serial.print("New emissivity = ");
  Serial.println(mlx.readEmissivity());
  while (1) {
    if (xSemaphoreTake(tem, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < 5; i++) {
        if (br == 0) {
          average_temp = atof(submenu_item4[0].value);  
          break;
        }
        if (millis() - tsLastReport1 > 1000) {
          t = mlx.readObjectTempC();
          Serial.println(t);
          average_temp = filterweight * (t) + (1 - filterweight) * average_temp;
          tsLastReport1 = millis();
          xEventGroupSetBits(xEventGroup, EVENT_DISPLAY_TEMPERATURE);
        }
        vTaskDelay(1000);
      }
        if((average_temp > atoi(submenu_item3[0].value))){
          xEventGroupSetBits(xBuzzerEventGroup, EVENT_BUZZER_2);
      }else{
         xEventGroupSetBits(xBuzzerEventGroup, EVENT_BUZZER_1);              
      }
      snprintf(submenu_item4[0].value, sizeof(submenu_item4[0].value), "%.1f", average_temp);
      EEPROM.writeFloat(16, atof(submenu_item4[0].value));
      EEPROM.commit();
  }
 }
}

void buzzerTask(void *parameter) {
  while (true) {
    // Chờ sự kiện từ Event Group mới
    EventBits_t eventBits = xEventGroupWaitBits(xBuzzerEventGroup, EVENT_BUZZER_1 | EVENT_BUZZER_2 | EVENT_BUZZER_3, pdTRUE, pdFALSE, portMAX_DELAY);

    // Xử lý sự kiện 1: Buzzer kêu 3 tiếng trong 0.5s
    if (eventBits & EVENT_BUZZER_1) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH); 
        vTaskDelay(80); 
        digitalWrite(BUZZER_PIN, LOW);  
        vTaskDelay(80); 
      }
    }
    // Xử lý sự kiện 2: Buzzer kêu liên tục trong 2s
    if (eventBits & EVENT_BUZZER_2) {
      digitalWrite(BUZZER_PIN, HIGH);  // Buzzer kêu liên tục
      vTaskDelay(2000); // Kêu trong 2s
      digitalWrite(BUZZER_PIN, LOW);   // Tắt buzzer
    }

    // Xử lý sự kiện 3: Buzzer kêu chu kỳ 0.5s 30 lần
    if (eventBits & EVENT_BUZZER_3) {
      for (int i = 0; i < 30; i++){
        digitalWrite(BUZZER_PIN, HIGH); // Buzzer kêu
        vTaskDelay(50); // Kêu trong 0.5s
        digitalWrite(BUZZER_PIN, LOW);  // Tắt buzzer
        vTaskDelay(50); // Tắt trong 0.5s
      }
    }
  }
}

void software_timer(void *parameter) {
  int giay,phut,gio;
  while (1) {
    if (xSemaphoreTake(sem_timer, portMAX_DELAY) == pdTRUE) {
   giay=atoi(submenu_item5[0].value);
   phut=atoi(submenu_item5[1].value);
   gio=atoi(submenu_item5[2].value);
     int time = (giay+phut*60+gio*360)*1000;//số giây hẹn giờ 
     Serial.println(time);
    if (timer != NULL){
      xTimerDelete(timer, 0);  // Xóa timer cũ
    }
     timer = xTimerCreate("Timer_buzzer",pdMS_TO_TICKS(time),atoi(submenu_item5[3].value),(void *) 0,timerCallback);
     xTimerStart(timer, 0);
   }
  }
}

void nextItem() {
  int num_items = getNumItems();
  selected_item = (selected_item + 1) % num_items;

  // Kiểm tra và điều chỉnh `window_start` khi danh sách cuộn vòng
  if (selected_item == 0) {
    window_start = 0;  // Reset về đầu khi cuộn vòng
  } else if (selected_item >= window_start + window_size) {
    window_start++;
  }
}

void prevItem() {
  int num_items = getNumItems();
  selected_item = (selected_item - 1 + num_items) % num_items;
  // Kiểm tra và điều chỉnh `window_start` khi danh sách cuộn vòng
  if (selected_item == num_items - 1) {
    window_start = num_items - window_size;  // Đặt về cuối khi cuộn vòng
                                             // if (window_start < 0) window_start = 0; // Đảm bảo không bị âm
  } else if (selected_item < window_start) {
    window_start--;
  }
}

void enterSubmenu() {
  if (current_level == 0) {    // Only allow entering submenu from main menu
      current_level = 1;
      current_parent = selected_item;
      selected_item = 0;
      window_back = window_start;
      window_start = 0;
    }
}

void exitSubmenu() {
  if (current_level == 1) {  
    current_level = 0;
    selected_item = current_parent;
    window_start = window_back;
    current_parent = -1;
  }
}

int getNumItems() {
  if (current_level == 0) {
    return num_main_items;
  } else if (current_level == 1) {
    if (current_parent == 2) {
      return num_submenu3_items;
    } else if (current_parent == 3) {
      return num_submenu4_items;
    }else if (current_parent == 4) {
      return num_submenu5_items;
    }else if (current_parent == 5) {
      return num_submenu6_items;
    }
  }
  return 0;
}

void drawMenu() {
  display.clearDisplay();
  display.setFont(&FreeSansBold9pt7b);
  
  MenuItem* menu = nullptr;
  int num_items = 0;

  if (current_level == 0) {
    // Menu chính
    menu = main_menu;
    num_items = num_main_items;
  } else if (current_level == 1) {
    // Submenu
    if (current_parent == 3){
      menu = submenu_item4;
      num_items = num_submenu4_items;
    } else if (current_parent == 2) {
      menu = submenu_item3;
      num_items = num_submenu3_items;
    }else if (current_parent == 3) {
      menu = submenu_item4;
      num_items = num_submenu4_items;
    }else if (current_parent == 4) {
      menu = submenu_item5;
      num_items = num_submenu5_items;
    }else if (current_parent == 5) {
      menu = submenu_item6;
      num_items = num_submenu6_items;
    }
  }
 

  for (int i = 0; i < window_size && i + window_start < num_items; i++) {
    int item_index = window_start + i;
    //display.setCursor(0, i * 20);

    if (item_index == selected_item) {
        display.setCursor(0, i * 22+15);
      display.print(">");
      if (is_editing) {
        display.print("*");
      }
    } else {
        display.setCursor(5, i * 22+15);
    }

    // Lấy dữ liệu từ menu
    const char* name = menu[item_index].name;
    const char* value = (menu[item_index].value[0] != '\0') ? menu[item_index].value : "";
    const char* unit = (menu[item_index].unit != nullptr) ? menu[item_index].unit : "";
    char buffer[32];
    sprintf(buffer, "%s %s %s", name, value, unit);
    Serial.println(buffer);
    display.print(buffer);
  }
  display.display();
}

void onBeatDetected(){
  calculating = true;
  last_beat = millis();
  Serial.println("beat");
}

void display_calculating() {
  display.clearDisplay();
  display.setFont(&FreeSansBold9pt7b);
  display.setCursor(0, 20);
  display.print("Measuring");
  display.setCursor(0, 40);
  for (int i = 0; i <= readIndex; i++){
    display.print(". ");
  }
  display.display();
}

void display_values() {
  display.clearDisplay();
  display.setFont(&FreeSansBold9pt7b);
  display.setCursor(0, 20);
  display.print("Heart rate:");
   display.setCursor(0, 40);
  display.print(pox.getHeartRate(), 0);
  display.print(" bpm");
  display.setCursor(0, 60);
  display.print("SpO2: ");
  //display.setCursor(62,59);
  display.print(pox.getSpO2());
  display.print(" %");
  display.display();
}

void initial_display() {
  display.clearDisplay();
  display.setFont(&FreeSansBold9pt7b);
  display.setCursor(0, 20);
  display.print("Place Finger");
  display.setCursor(0, 40);
  display.print("On the sensor");
  display.setCursor(0, 60);
  display.print("Thank you!");
  display.display();
}

void display_temp() {
  display.clearDisplay();
  display.setCursor(30, 20);
  display.print(" Body ");
  display.setCursor(0, 40);
  display.print("Temperature");
  display.setCursor(10,60);
  display.print(average_temp);
  display.print(" oC");
  display.display();
}

void adjustValue(int delta){
  if (current_level == 1){
    MenuItem* menu = nullptr;
    if (current_parent == 2){
      menu = submenu_item3;  // Sử dụng submenu_item2
      // Chỉ chỉnh sửa mục có giá trị
      if (menu[selected_item].value[0] != '\0') {
        if (strcmp(menu[selected_item].unit, "oC") == 0) {
          // Điều chỉnh Temp (dạng float)
          float value = atof(menu[selected_item].value);  // Lấy giá trị hiện tại
          value += delta * 0.5;                           // Tăng hoặc giảm 0.2
          if (value < 30) value = 50;                     // Đảm bảo không âm
          else if (value > 50) value = 30;
          snprintf(menu[selected_item].value, sizeof(menu[selected_item].value), "%.1f", value);  // Cập nhật giá trị
        } else if (strcmp(menu[selected_item].name, "lhr") == 0) {
          // Điều chỉnh BPM (dạng int)
          int value = atoi(menu[selected_item].value);                      // Lấy giá trị hiện tại
          value += delta;                                                   // Tăng hoặc giảm 1
          if (value < 40) value = atoi(menu[selected_item + 1].value) - 1;  // Đảm bảo không âm
          else if (value > atoi(menu[selected_item + 1].value) - 1) value = 40;
          snprintf(menu[selected_item].value, sizeof(menu[selected_item].value), "%d", value);  // Cập nhật giá trị
        } else if (strcmp(menu[selected_item].name, "hhr") == 0) {
          // Điều chỉnh BPM (dạng int)
          int value = atoi(menu[selected_item].value);                       // Lấy giá trị hiện tại
          value += delta;                                                    // Tăng hoặc giảm 1
          if (value > 120) value = atoi(menu[selected_item - 1].value) + 1;  // Đảm bảo không âm
          else if (value < atoi(menu[selected_item - 1].value) + 1) value = 120;
          snprintf(menu[selected_item].value, sizeof(menu[selected_item].value), "%d", value);  // Cập nhật giá trị
        } else if (strcmp(menu[selected_item].unit, "%") == 0) {
          // Điều chỉnh Spo2 (dạng int)
          int value = atoi(menu[selected_item].value);                                          // Lấy giá trị hiện tại
          value += delta;                                                                       // Tăng hoặc giảm 1
          if (value < 80) value = 100;                                                          // Đảm bảo không âm
          else if (value > 100) value = 80;                                                     // Giới hạn Spo2 tối đa là 100
          snprintf(menu[selected_item].value, sizeof(menu[selected_item].value), "%d", value);  // Cập nhật giá trị
        }
      }
    }else if(current_parent == 4){
       menu = submenu_item5;  // Sử dụng submenu_item2
      // Chỉ chỉnh sửa mục có giá trị
      if (menu[selected_item].value[0] != '\0') {
          if (strcmp(menu[selected_item].unit, "second") == 0) {
          // Điều chỉnh Temp (dạng float)
          int value = atoi(menu[selected_item].value);  // Lấy giá trị hiện tại
          value += delta * 1;                           // Tăng hoặc giảm 0.2
          if (value < 0) value = 59;                     // Đảm bảo không âm
          else if (value > 59) value = 0;
          snprintf(menu[selected_item].value, sizeof(menu[selected_item].value), "%d", value);  // Cập nhật giá trị
        } else if (strcmp(menu[selected_item].unit, "minutes") == 0) {
          // Điều chỉnh Temp (dạng float)
          int value = atoi(menu[selected_item].value);  // Lấy giá trị hiện tại
          value += delta * 1;                           // Tăng hoặc giảm 0.2
          if (value < 0) value = 59;                     // Đảm bảo không âm
          else if (value > 59) value = 0;
          snprintf(menu[selected_item].value, sizeof(menu[selected_item].value), "%d", value);  // Cập nhật giá trị
        }else if (strcmp(menu[selected_item].unit, "hours") == 0) {
          // Điều chỉnh Temp (dạng float)
          int value = atoi(menu[selected_item].value);  // Lấy giá trị hiện tại
          value += delta * 1;                           // Tăng hoặc giảm 0.2
          if (value < 0) value = 23;                     // Đảm bảo không âm
          else if (value > 23) value = 0;
          snprintf(menu[selected_item].value, sizeof(menu[selected_item].value), "%d", value);  // Cập nhật giá trị
        }else if (strcmp(menu[selected_item].name, "Repeat") == 0) {
          // Điều chỉnh Temp (dạng float)
          int value = atoi(menu[selected_item].value);  // Lấy giá trị hiện tại
          value =1-value;                           // Tăng hoặc giảm 0.2
          snprintf(menu[selected_item].value, sizeof(menu[selected_item].value), "%d", value);  // Cập nhật giá trị
        }
      }
    }
  }
}

void loadFromEEPROM() {
  float temp,tem;
  int lowerHR, higherHR, spo2, HR, oxi;
  int giay,phut,gio,ngay;
  
  // Đọc dữ liệu từ EEPROM
  EEPROM.get(0, temp);
  EEPROM.get(4, lowerHR);
  EEPROM.get(8, higherHR);
  EEPROM.get(12, spo2);
  EEPROM.get(16, tem);
  EEPROM.get(20, HR);
  EEPROM.get(44, oxi);

  EEPROM.get(28, giay);
  EEPROM.get(32, phut);
  EEPROM.get(36, gio);
  EEPROM.get(40, ngay);
  
  if (isnan(temp) || temp < 10.0 || temp > 50.0){
    temp = 37.5;  // Giá trị mặc định
  }
  if (lowerHR < 30 || lowerHR > 200) {  // Giá trị hợp lệ: 30 bpm đến 200 bpm
    lowerHR = 60;                      // Giá trị mặc định
  }
  if (higherHR < 30 || higherHR > 200) {  // Giá trị hợp lệ: 30 bpm đến 200 bpm
    higherHR = 100;                      // Giá trị mặc định
  }

  if (spo2 < 50 || spo2 > 100) {  // Giá trị hợp lệ: 70% đến 100%
    spo2 = 94;                   // Giá trị mặc định
  }
  if(isnan(tem)||temp < 0){
    tem=0.0;
  }
  if (HR < 30 || HR > 200) {  // Giá trị hợp lệ: 30 bpm đến 200 bpm
    HR = 0;                      // Giá trị mặc định
  }
  if (oxi < 50 || oxi > 100) {  
     oxi= 0;                   
  }
  if(giay<0||ngay<0||phut<0||gio<0){
    giay=0;phut=30;gio=0;ngay=0;
  }
  
  // Cập nhật giá trị vào `submenu_item3`
  snprintf(submenu_item3[0].value, sizeof(submenu_item3[0].value), "%.1f", temp);
  snprintf(submenu_item3[1].value, sizeof(submenu_item3[1].value), "%d", lowerHR);
  snprintf(submenu_item3[2].value, sizeof(submenu_item3[2].value), "%d", higherHR);
  snprintf(submenu_item3[3].value, sizeof(submenu_item3[3].value), "%d", spo2);
  snprintf(submenu_item4[0].value, sizeof(submenu_item4[0].value), "%.1f", tem);
  snprintf(submenu_item4[1].value, sizeof(submenu_item4[1].value), "%d", HR);
  snprintf(submenu_item4[2].value, sizeof(submenu_item4[2].value), "%d", oxi);
 
  snprintf(submenu_item5[0].value, sizeof(submenu_item5[0].value), "%d", giay);
  snprintf(submenu_item5[1].value, sizeof(submenu_item5[1].value), "%d", phut);
  snprintf(submenu_item5[2].value, sizeof(submenu_item5[2].value), "%d", gio);
  snprintf(submenu_item5[3].value, sizeof(submenu_item5[3].value), "%d", ngay);
}

void saveToEEPROM() {
  EEPROM.writeFloat(0, atof(submenu_item3[0].value));  // Lưu Temp
  EEPROM.put(4, atoi(submenu_item3[1].value));         // Lưu BPM
  EEPROM.put(8, atoi(submenu_item3[2].value));         // Lưu BPM
  EEPROM.put(12, atoi(submenu_item3[3].value));        // Lưu Spo2
  
  EEPROM.put(28, atoi(submenu_item5[0].value));  
  EEPROM.put(32, atoi(submenu_item5[1].value));         
  EEPROM.put(36, atoi(submenu_item5[2].value));        
  EEPROM.put(40, atoi(submenu_item5[3].value));        

  EEPROM.commit();  // Đảm bảo lưu vào EEPROM
}

void displaySaveMessage() {
  display.clearDisplay();
  display.setFont(&FreeSansBold9pt7b);
  display.setCursor(30, 20);
  display.print("saved");
  display.setCursor(0, 40);
  display.print("successfully");
  display.display();
  vTaskDelay(2000);  // Hiển thị trong 2 giây
}
