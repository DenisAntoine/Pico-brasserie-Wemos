// Wemos Brew user config


// ************************************************
// Com WIFI and MQTT
// ************************************************
//#define wifi_ssid "TP-LINK_24A4" //Modify as needed
//#define wifi_password "08111692" //Modify as needed

#define wifi_ssid "Livebox-4455" //Modify as needed
#define wifi_password "FifineEtMikkie" //Modify as needed

#define mqtt_server "192.168.1.24" //Modify as needed


// topic MQTT publish
#define opState_topic "bier/opState"
#define autoStep_topic "bier/autoStep"
#define timerStep_topic "bier/timerStep"
#define temperature_topic "bier/temperature"
#define setpoint_topic "bier/setpoint"
#define pctchauf_topic "bier/pctchauf"
#define pwm_topic "bier/pwm"
#define sT1_topic "bier/StepT1"
#define sT2_topic "bier/StepT2"
#define sT3_topic "bier/StepT3"
#define sT4_topic "bier/StepT4"
#define sd1_topic "bier/Stepd1"
#define sd2_topic "bier/Stepd2"
#define sd3_topic "bier/Stepd3"
#define sd4_topic "bier/Stepd4"
#define Kp_topic "bier/Kp"
#define Ki_topic "bier/Ki"
#define Kd_topic "bier/Kd"

// topic MQTT subscribe
#define cde_setpoint_topic "cmdbier/Setpoint"
#define cde_pwm_topic "cmdbier/Pwm"
#define cde_Kp_topic "cmdbier/Kp"
#define cde_Ki_topic "cmdbier/Ki"
#define cde_Kd_topic "cmdbier/Kd"
#define cde_sT1_topic "cmdbier/StepT1"
#define cde_sT2_topic "cmdbier/StepT2"
#define cde_sT3_topic "cmdbier/StepT3"
#define cde_sT4_topic "cmdbier/StepT4"
#define cde_sd1_topic "cmdbier/Stepd1"
#define cde_sd2_topic "cmdbier/Stepd2"
#define cde_sd3_topic "cmdbier/Stepd3"
#define cde_sd4_topic "cmdbier/Stepd4"
