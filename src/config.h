// Wemos Brew user config

// ************************************************
// Pin definitions
// ************************************************

#define OLED_RESET 0  // GPIO0
#define STATUSLED_PIN D3 // optional feature
#define RELAY_PIN D0 // Output Relay heat
#define PWM_OUT D8 // Output PWM Pump
#define ONE_WIRE_BUS D4 // One-Wire Temperature Sensor
#define BUTTON_DOWN_PIN D5 // Buttons
#define BUTTON_UP_PIN D6 // Buttons
#define BUTTON_SELECT_PIN D7 // Buttons

// ************************************************
// Com WIFI and MQTT
// ************************************************
#define mqtt_server "192.168.1.24" //Modify as needed
// topic MQTT publish
#define publish_topic "biere/json"
// topic MQTT subscribe
#define subscribe_topic "cdebiere/cdejson"


