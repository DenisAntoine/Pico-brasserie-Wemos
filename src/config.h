// Wemos Brew user config


// ************************************************
// Com WIFI and MQTT
// ************************************************
//#define wifi_ssid "TP-LINK_24A4" //Modify as needed
//#define wifi_password "08111692" //Modify as needed

//#define wifi_ssid "Livebox-4455" //Modify as needed
//#define wifi_password "FifineEtMikkie" //Modify as needed

#define mqtt_server "192.168.1.24" //Modify as needed


// topic MQTT publish
/*#define opState_topic "biere/opState"
#define autoStep_topic "biere/autoStep"
#define timerStep_topic "biere/timerStep"
#define temperature_topic "biere/temperature"
#define setpoint_topic "biere/setpoint"
#define pctchauf_topic "biere/pctchauf"
#define pwm_topic "biere/pwm"
#define sT1_topic "biere/StepT1"
#define sT2_topic "biere/StepT2"
#define sT3_topic "biere/StepT3"
#define sT4_topic "biere/StepT4"
#define sd1_topic "biere/Stepd1"
#define sd2_topic "biere/Stepd2"
#define sd3_topic "biere/Stepd3"
#define sd4_topic "biere/Stepd4"
#define Kp_topic "biere/Kp"
#define Ki_topic "biere/Ki"
#define Kd_topic "biere/Kd"*/

#define publish_topic "biere/json"

// topic MQTT subscribe
/*#define cde_setpoint_topic "biere/cdeSetpoint"
#define cde_pwm_topic "biere/cdePwm"
#define cde_Kp_topic "biere/cdeKp"
#define cde_Ki_topic "biere/cdeKi"
#define cde_Kd_topic "biere/cdeKd"
#define cde_sT1_topic "biere/cdeStepT1"
#define cde_sT2_topic "biere/cdeStepT2"
#define cde_sT3_topic "biere/cdeStepT3"
#define cde_sT4_topic "biere/cdeStepT4"
#define cde_sd1_topic "biere/cdeStepd1"
#define cde_sd2_topic "biere/cdeStepd2"
#define cde_sd3_topic "biere/cdeStepd3"
#define cde_sd4_topic "biere/cdeStepd4"*/

#define subscribe_topic "cdebiere/cdejson"
