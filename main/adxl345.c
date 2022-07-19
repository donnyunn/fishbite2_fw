#include "adxl345.h"

static const char *TAG = "adxl345";

#define GPIO_FOR_INT1     5
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_FOR_INT1)
#define ESP_INTR_FLAG_DEFAULT 0

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ADXL345_SENSOR_ADDR CONFIG_ADXL345_ADDR   /*!< slave address for ADXL345 sensor */
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

bool status;					// Set When Error Exists 
uint8_t error_code;				// Initial State
double gains[3];				// Counts to Gs

#define ADXL345_TO_READ (6)      // Number of Bytes Read - Two Bytes Per Axis
uint8_t _buff[ADXL345_TO_READ];		//	6 Bytes Buffer

void ADXL_ISR();
void powerOn();
void readAccel(int* x, int* y, int* z);
void get_Gxyz(double *xyz);

void setTapThreshold(int tapThreshold);
int getTapThreshold();
void setAxisGains(double *_gains);
void getAxisGains(double *_gains);
void setAxisOffset(int x, int y, int z);
void getAxisOffset(int* x, int* y, int*z);
void setTapDuration(int tapDuration);
int getTapDuration();
void setDoubleTapLatency(int doubleTapLatency);
int getDoubleTapLatency();
void setDoubleTapWindow(int doubleTapWindow);
int getDoubleTapWindow();
void setActivityThreshold(int activityThreshold);
int getActivityThreshold();
void setInactivityThreshold(int inactivityThreshold);
int getInactivityThreshold();
void setTimeInactivity(int timeInactivity);
int getTimeInactivity();
void setFreeFallThreshold(int freeFallthreshold);
int getFreeFallThreshold();
void setFreeFallDuration(int freeFallDuration);
int getFreeFallDuration();

bool isActivityXEnabled();
bool isActivityYEnabled();
bool isActivityZEnabled();
bool isInactivityXEnabled();
bool isInactivityYEnabled();
bool isInactivityZEnabled();
bool isActivityAc();
bool isInactivityAc();
void setActivityAc(bool state);
void setInactivityAc(bool state);

bool getSuppressBit();
void setSuppressBit(bool state);
bool isTapDetectionOnX();
void setTapDetectionOnX(bool state);
bool isTapDetectionOnY();
void setTapDetectionOnY(bool state);
bool isTapDetectionOnZ();
void setTapDetectionOnZ(bool state);
void setTapDetectionOnXYZ(bool stateX, bool stateY, bool stateZ);

void setActivityX(bool state);
void setActivityY(bool state);
void setActivityZ(bool state);
void setActivityXYZ(bool stateX, bool stateY, bool stateZ);
void setInactivityX(bool state);
void setInactivityY(bool state);
void setInactivityZ(bool state);
void setInactivityXYZ(bool stateX, bool stateY, bool stateZ);

bool isActivitySourceOnX();
bool isActivitySourceOnY();
bool isActivitySourceOnZ();
bool isTapSourceOnX();
bool isTapSourceOnY();
bool isTapSourceOnZ();
bool isAsleep();

bool isLowPower();
void setLowPower(bool state);
double getRate();
void setRate(double rate);
void set_bw(uint8_t bw_code);
uint8_t get_bw_code();  

bool triggered(uint8_t interrupts, int mask);

uint8_t getInterruptSource();
bool getInterruptSourceBit(uint8_t interruptBit);
bool getInterruptMapping(uint8_t interruptBit);
void setInterruptMapping(uint8_t interruptBit, bool interruptPin);
bool isInterruptEnabled(uint8_t interruptBit);
void setInterrupt(uint8_t interruptBit, bool state);
void setImportantInterruptMapping(int single_tap, int double_tap, int free_fall, int activity, int inactivity);
void InactivityINT(bool status);
void ActivityINT(bool status);
void FreeFallINT(bool status);
void doubleTapINT(bool status);
void singleTapINT(bool status);

void getRangeSetting(uint8_t* rangeSetting);
void setRangeSetting(int val);
bool getSelfTestBit();
void setSelfTestBit(bool selfTestBit);
bool getSpiBit();
void setSpiBit(bool spiBit);
bool getInterruptLevelBit();
void setInterruptLevelBit(bool interruptLevelBit);
bool getFullResBit();
void setFullResBit(bool fullResBit);
bool getJustifyBit();
void setJustifyBit(bool justifyBit);
void printAllRegister();
void writeTo(uint8_t address, uint8_t val);
void readFrom(uint8_t address, int num, uint8_t *buff);
void setRegisterBit(uint8_t regAdress, int bitPos, bool state);
bool getRegisterBit(uint8_t regAdress, int bitPos);  

void print_byte(uint8_t val);

/* ESP Peripheral Functions */
static xQueueHandle adxl_evt_queue = NULL;
static xQueueHandle gpio_evt_queue = NULL;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void adxl345_int_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            ADXL_ISR();
        }
    }
}

void gpio_int_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(adxl345_int_task, "adxl345_int_task", 2048, NULL, 10, NULL);
    
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_FOR_INT1, gpio_isr_handler, (void*) GPIO_FOR_INT1);
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t adxl345_write(uint8_t reg, uint8_t *buf, int len)
{
    esp_err_t ret;
    uint8_t tx[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    tx[0] = ADXL345_SENSOR_ADDR << 1 | WRITE_BIT;
    tx[1] = reg;
    i2c_master_write_byte(cmd, tx[0], ACK_CHECK_EN);
    i2c_master_write_byte(cmd, tx[1], ACK_CHECK_EN);
    for (int i = 0; i < len; i++) {
        i2c_master_write_byte(cmd, buf[i], ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "I2C Timeout");
    } else if (ret != ESP_OK) {
        ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t adxl345_read(uint8_t reg, uint8_t *buf, int len)
{
    esp_err_t ret;
    uint8_t tx[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    tx[0] = ADXL345_SENSOR_ADDR << 1 | WRITE_BIT;
    tx[1] = reg;
    i2c_master_write_byte(cmd, tx[0], ACK_CHECK_EN);
    i2c_master_write_byte(cmd, tx[1], ACK_CHECK_EN);
    i2c_master_start(cmd);
    tx[0] = ADXL345_SENSOR_ADDR << 1 | READ_BIT;
    i2c_master_write_byte(cmd, tx[0], ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "I2C Timeout");
    } else if (ret != ESP_OK) {
        ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
    }
    return ret;
}

/* Utility Functions */
int constrain(int val, int min, int max)
{
    if (val > max) 
        return max;
    else if (val < min) 
        return min;
    else 
        return val;
}

void writeTo(uint8_t reg, uint8_t data)
{
    uint8_t tx[1];
    tx[0] = data;
    adxl345_write(reg, tx, 1);
}

void readFrom(uint8_t reg, int num, uint8_t *buff)
{
    adxl345_read(reg, buff, num);
}

/**************************** START UP ******************************/
/*                                                                  */
bool adxl345_knock(void)
{
	uint8_t _val;
	readFrom(ADXL345_DEVID, 1, &_val);
	printf("adxl345 id: 0x%02X\n", _val);
	return (_val == 0xE5);
}

void adxl345_init(void)
{
    adxl_evt_queue = xQueueCreate(10, sizeof(uint32_t));

	status = ADXL345_OK;
	error_code = ADXL345_NO_ERROR;
	gains[0] = 0.00376390;		// Original gain 0.00376390
	gains[1] = 0.00376009;		// Original gain 0.00376009
	gains[2] = 0.00349265;		// Original gain 0.00349265

    gpio_int_init();

    powerOn();
    setRangeSetting(16);

    set_bw(ADXL345_LOW_POWER | ADXL345_BW_12_5);

    setActivityXYZ(1, 1, 1);
    setActivityThreshold(32);

    setInactivityXYZ(1, 1, 1);
    setInactivityThreshold(32);
    setTimeInactivity(2);

    setTapDetectionOnXYZ(1, 1, 1);

    setTapThreshold(32);
    setTapDuration(15);
    setDoubleTapLatency(80);
    setDoubleTapWindow(200);

    setFreeFallThreshold(7);
    setFreeFallDuration(30);

    InactivityINT(1);
    ActivityINT(1);
    FreeFallINT(1);
    doubleTapINT(1);
    singleTapINT(1);

    setInterruptMapping(ADXL345_INT_DATA_READY_BIT, ADXL345_INT2_PIN);
    setInterruptMapping(ADXL345_INT_SINGLE_TAP_BIT, ADXL345_INT1_PIN);
    setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT, ADXL345_INT1_PIN);
    setInterruptMapping(ADXL345_INT_ACTIVITY_BIT, ADXL345_INT1_PIN);
    setInterruptMapping(ADXL345_INT_INACTIVITY_BIT, ADXL345_INT1_PIN);
    setInterruptMapping(ADXL345_INT_FREE_FALL_BIT, ADXL345_INT1_PIN);
    setInterruptMapping(ADXL345_INT_WATERMARK_BIT, ADXL345_INT2_PIN);
    setInterruptMapping(ADXL345_INT_OVERRUNY_BIT, ADXL345_INT2_PIN);
    setInterruptLevelBit(1);
}

void powerOn(void)
{
	//ADXL345 TURN ON
	//writeTo(ADXL345_POWER_CTL, 0);	// Wakeup
	//writeTo(ADXL345_POWER_CTL, 16);	// Auto_Sleep
	writeTo(ADXL345_POWER_CTL, 0b00101000);	// Measure
}

/**************************** USER FUNC *****************************/
/*                                                                  */
void i2c_init(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
}

void adxl345_test(void)
{
    int x, y, z;
    readAccel(&x, &y, &z);
    //printf("%d, %d, %d\n", x, y, z);
    ADXL_ISR();
}

void adxl345_sleep(void)
{
	gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    writeTo(ADXL345_POWER_CTL, 0b00010111); // sleep
    i2c_driver_delete(I2C_MASTER_NUM);
}

adxl_evt_t adxl345_getEventQueue(void)
{
    adxl_evt_t ret = ADXL345_EVT_NULL;
    int evt_num;
    if (xQueueReceive(adxl_evt_queue, &evt_num, 1000 / portTICK_PERIOD_MS)) {
        ret = (adxl_evt_t)evt_num;
    }
    return ret;
}

/********************* ISR *********************/
/* Look for Interrupts and Triggered Action    */
void ADXL_ISR() {
  
  // getInterruptSource clears all triggered actions after returning value
  // Do not call again until you need to recheck for triggered actions
  uint8_t interrupts = getInterruptSource();
  int evt_num;
  
  // Free Fall Detection
  if(triggered(interrupts, ADXL345_FREE_FALL)){
    //printf("*** FREE FALL ***\n");
    //add code here to do when free fall is sensed
     evt_num = ADXL345_EVT_FREEFALL;
     xQueueSendFromISR(adxl_evt_queue, &evt_num, NULL);
  } 
  
  // Inactivity
  if(triggered(interrupts, ADXL345_INACTIVITY)){
    //printf("*** INACTIVITY ***\n");
     //add code here to do when inactivity is sensed
     evt_num = ADXL345_EVT_INACTION;
     xQueueSendFromISR(adxl_evt_queue, &evt_num, NULL);
  }
  
  // Activity
  if(triggered(interrupts, ADXL345_ACTIVITY)){
    //printf("*** ACTIVITY ***\n"); 
     //add code here to do when activity is sensed
     evt_num = ADXL345_EVT_ACTION;
     xQueueSendFromISR(adxl_evt_queue, &evt_num, NULL);
  }
  
  // Double Tap Detection
  if(triggered(interrupts, ADXL345_DOUBLE_TAP)){
    //printf("*** DOUBLE TAP ***\n");
     //add code here to do when a 2X tap is sensed
     evt_num = ADXL345_EVT_DOUBLETAP;
     xQueueSendFromISR(adxl_evt_queue, &evt_num, NULL);
  }
  
  // Tap Detection
  if(triggered(interrupts, ADXL345_SINGLE_TAP)){
    //printf("*** TAP ***\n");
     //add code here to do when a tap is sensed
     evt_num = ADXL345_EVT_TAP;
     xQueueSendFromISR(adxl_evt_queue, &evt_num, NULL);
  } 
}

/*********************** READING ACCELERATION ***********************/
/*    Reads Acceleration into Three Variables:  x, y and z          */
void readAccel(int *x, int *y, int *z) {
	readFrom(ADXL345_DATAX0, ADXL345_TO_READ, _buff);	// Read Accel Data from ADXL345

	// Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
	*x = (int16_t)((((int)_buff[1]) << 8) | _buff[0]);
	*y = (int16_t)((((int)_buff[3]) << 8) | _buff[2]);
	*z = (int16_t)((((int)_buff[5]) << 8) | _buff[4]);
}

void get_Gxyz(double *xyz){
	int i;
	int xyz_int[3];
	readAccel(&(xyz_int[0]), &(xyz_int[1]), &(xyz_int[2]));
	for(i=0; i<3; i++){
		xyz[i] = xyz_int[i] * gains[i];
	}
}

/*************************** RANGE SETTING **************************/
/*          ACCEPTABLE VALUES: 2g, 4g, 8g, 16g ~ GET & SET          */
void getRangeSetting(uint8_t* rangeSetting) {
	uint8_t _b;
	readFrom(ADXL345_DATA_FORMAT, 1, &_b);
	*rangeSetting = _b & 0b00000011;
}

void setRangeSetting(int val) {
	uint8_t _s;
	uint8_t _b;

	switch (val) {
		case 2:
			_s = 0b00000000;
			break;
		case 4:
			_s = 0b00000001;
			break;
		case 8:
			_s = 0b00000010;
			break;
		case 16:
			_s = 0b00000011;
			break;
		default:
			_s = 0b00000000;
	}
	readFrom(ADXL345_DATA_FORMAT, 1, &_b);
	_s |= (_b & 0b11101100);
	writeTo(ADXL345_DATA_FORMAT, _s);
}

/*************************** SELF_TEST BIT **************************/
/*                            ~ GET & SET                           */
bool getSelfTestBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 7);
}

// If Set (1) Self-Test Applied. Electrostatic Force exerted on the sensor
//  causing a shift in the output data.
// If Set (0) Self-Test Disabled.
void setSelfTestBit(bool selfTestBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
}

/*************************** SPI BIT STATE **************************/
/*                           ~ GET & SET                            */
bool getSpiBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 6);
}

// If Set (1) Puts Device in 3-wire Mode
// If Set (0) Puts Device in 4-wire SPI Mode
void setSpiBit(bool spiBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 6, spiBit);
}

/*********************** INT_INVERT BIT STATE ***********************/
/*                           ~ GET & SET                            */
bool getInterruptLevelBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 5);
}

// If Set (0) Sets the Interrupts to Active HIGH
// If Set (1) Sets the Interrupts to Active LOW
void setInterruptLevelBit(bool interruptLevelBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 5, interruptLevelBit);
}

/************************* FULL_RES BIT STATE ***********************/
/*                           ~ GET & SET                            */
bool getFullResBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 3);
}

// If Set (1) Device is in Full Resolution Mode: Output Resolution Increase with G Range
//  Set by the Range Bits to Maintain a 4mg/LSB Scale Factor
// If Set (0) Device is in 10-bit Mode: Range Bits Determine Maximum G Range
//  And Scale Factor
void setFullResBit(bool fullResBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
}

/*************************** JUSTIFY BIT STATE **************************/
/*                           ~ GET & SET                            */
bool getJustifyBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 2);
}

// If Set (1) Selects the Left Justified Mode
// If Set (0) Selects Right Justified Mode with Sign Extension
void setJustifyBit(bool justifyBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
}

/*********************** THRESH_TAP BYTE VALUE **********************/
/*                          ~ SET & GET                             */
// Should Set Between 0 and 255
// Scale Factor is 62.5 mg/LSB
// A Value of 0 May Result in Undesirable Behavior
void setTapThreshold(int tapThreshold) {
	tapThreshold = constrain(tapThreshold,0,255);
	uint8_t _b = (uint8_t)tapThreshold;
	writeTo(ADXL345_THRESH_TAP, _b);
}

// Return Value Between 0 and 255
// Scale Factor is 62.5 mg/LSB
int getTapThreshold() {
	uint8_t _b;
	readFrom(ADXL345_THRESH_TAP, 1, &_b);
	return (int)_b;
}

/****************** GAIN FOR EACH AXIS IN Gs / COUNT *****************/
/*                           ~ SET & GET                            */
void setAxisGains(double *_gains){
	int i;
	for(i = 0; i < 3; i++){
		gains[i] = _gains[i];
	}
}
void getAxisGains(double *_gains){
	int i;
	for(i = 0; i < 3; i++){
		_gains[i] = gains[i];
	}
}

/********************* OFSX, OFSY and OFSZ BYTES ********************/
/*                           ~ SET & GET                            */
// OFSX, OFSY and OFSZ: User Offset Adjustments in Twos Complement Format
// Scale Factor of 15.6mg/LSB
void setAxisOffset(int x, int y, int z) {
	writeTo(ADXL345_OFSX, (uint8_t)(x));
	writeTo(ADXL345_OFSY, (uint8_t)(y));
	writeTo(ADXL345_OFSZ, (uint8_t)(z));
}

void getAxisOffset(int* x, int* y, int*z) {
	uint8_t _b;
	readFrom(ADXL345_OFSX, 1, &_b);
	*x = (int) (_b);
	readFrom(ADXL345_OFSY, 1, &_b);
	*y = (int) (_b);
	readFrom(ADXL345_OFSZ, 1, &_b);
	*z = (int) (_b);
}

/****************************** DUR BYTE ****************************/
/*                           ~ SET & GET                            */
// DUR Byte: Contains an Unsigned Time Value Representing the Max Time
//  that an Event must be Above the THRESH_TAP Threshold to qualify
//  as a Tap Event
// The scale factor is 625µs/LSB
// Value of 0 Disables the Tap/Double Tap Funcitons. Max value is 255.
void setTapDuration(int tapDuration) {
	tapDuration = constrain(tapDuration,0,255);
	uint8_t _b = (uint8_t) (tapDuration);
	writeTo(ADXL345_DUR, _b);
}

int getTapDuration() {
	uint8_t _b;
	readFrom(ADXL345_DUR, 1, &_b);
	return (int) (_b);
}

/************************** LATENT REGISTER *************************/
/*                           ~ SET & GET                            */
// Contains Unsigned Time Value Representing the Wait Time from the Detection
//  of a Tap Event to the Start of the Time Window (defined by the Window
//  Register) during which a possible Second Tap Even can be Detected.
// Scale Factor is 1.25ms/LSB.
// A Value of 0 Disables the Double Tap Function.
// It Accepts a Maximum Value of 255.
void setDoubleTapLatency(int doubleTapLatency) {
	uint8_t _b = (uint8_t) (doubleTapLatency);
	writeTo(ADXL345_LATENT, _b);
}

int getDoubleTapLatency() {
	uint8_t _b;
	readFrom(ADXL345_LATENT, 1, &_b);
	return (int) (_b);
}

/************************** WINDOW REGISTER *************************/
/*                           ~ SET & GET                            */
// Contains an Unsigned Time Value Representing the Amount of Time
//  After the Expiration of the Latency Time (determined by Latent register)
//  During which a Second Valid Tape can Begin.
// Scale Factor is 1.25ms/LSB.
// Value of 0 Disables the Double Tap Function.
// It Accepts a Maximum Value of 255.
void setDoubleTapWindow(int doubleTapWindow) {
	doubleTapWindow = constrain(doubleTapWindow,0,255);
	uint8_t _b = (uint8_t) (doubleTapWindow);
	writeTo(ADXL345_WINDOW, _b);
}

int getDoubleTapWindow() {
	uint8_t _b;
	readFrom(ADXL345_WINDOW, 1, &_b);
	return (int) (_b);
}

/*********************** THRESH_ACT REGISTER ************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Activity.
// Data Format is Unsigned, so the Magnitude of the Activity Event is Compared
//  with the Value is Compared with the Value in the THRESH_ACT Register.
// The Scale Factor is 62.5mg/LSB.
// Value of 0 may Result in Undesirable Behavior if the Activity Interrupt Enabled.
// It Accepts a Maximum Value of 255.
void setActivityThreshold(int activityThreshold) {
	activityThreshold = constrain(activityThreshold,0,255);
	uint8_t _b = (uint8_t) (activityThreshold);
	writeTo(ADXL345_THRESH_ACT, _b);
}

// Gets the THRESH_ACT byte
int getActivityThreshold() {
	uint8_t _b;
	readFrom(ADXL345_THRESH_ACT, 1, &_b);
	return (int) (_b);
}

/********************** THRESH_INACT REGISTER ***********************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Inactivity.
// The Data Format is Unsigned, so the Magnitude of the INactivity Event is
//  Compared with the value in the THRESH_INACT Register.
// Scale Factor is 62.5mg/LSB.
// Value of 0 May Result in Undesirable Behavior if the Inactivity Interrupt Enabled.
// It Accepts a Maximum Value of 255.
void setInactivityThreshold(int inactivityThreshold) {
	inactivityThreshold = constrain(inactivityThreshold,0,255);
	uint8_t _b = (uint8_t) (inactivityThreshold);
	writeTo(ADXL345_THRESH_INACT, _b);
}

int getInactivityThreshold() {
	uint8_t _b;
	readFrom(ADXL345_THRESH_INACT, 1, &_b);
	return (int) (_b);
}

/*********************** TIME_INACT RESIGER *************************/
/*                          ~ SET & GET                             */
// Contains an Unsigned Time Value Representing the Amount of Time that
//  Acceleration must be Less Than the Value in the THRESH_INACT Register
//  for Inactivity to be Declared.
// Uses Filtered Output Data* unlike other Interrupt Functions
// Scale Factor is 1sec/LSB.
// Value Must Be Between 0 and 255.
void setTimeInactivity(int timeInactivity) {
	timeInactivity = constrain(timeInactivity,0,255);
	uint8_t _b = (uint8_t) (timeInactivity);
	writeTo(ADXL345_TIME_INACT, _b);
}

int getTimeInactivity() {
	uint8_t _b;
	readFrom(ADXL345_TIME_INACT, 1, &_b);
	return (int) (_b);
}

/*********************** THRESH_FF Register *************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value, in Unsigned Format, for Free-Fall Detection
// The Acceleration on all Axes is Compared with the Value in THRES_FF to
//  Determine if a Free-Fall Event Occurred.
// Scale Factor is 62.5mg/LSB.
// Value of 0 May Result in Undesirable Behavior if the Free-Fall interrupt Enabled.
// Accepts a Maximum Value of 255.
void setFreeFallThreshold(int freeFallThreshold) {
	freeFallThreshold = constrain(freeFallThreshold,0,255);
	uint8_t _b = (uint8_t) (freeFallThreshold);
	writeTo(ADXL345_THRESH_FF, _b);
}

int getFreeFallThreshold() {
	uint8_t _b;
	readFrom(ADXL345_THRESH_FF, 1, &_b);
	return (int) (_b);
}

/************************ TIME_FF Register **************************/
/*                          ~ SET & GET                             */
// Stores an Unsigned Time Value Representing the Minimum Time that the Value
//  of all Axes must be Less Than THRES_FF to Generate a Free-Fall Interrupt.
// Scale Factor is 5ms/LSB.
// Value of 0 May Result in Undesirable Behavior if the Free-Fall Interrupt Enabled.
// Accepts a Maximum Value of 255.
void setFreeFallDuration(int freeFallDuration) {
	freeFallDuration = constrain(freeFallDuration,0,255);
	uint8_t _b = (uint8_t) (freeFallDuration);
	writeTo(ADXL345_TIME_FF, _b);
}

int getFreeFallDuration() {
	uint8_t _b;
	readFrom(ADXL345_TIME_FF, 1, &_b);
	return (int) (_b);
}

/************************** ACTIVITY BITS ***************************/
/*                                                                  */
bool isActivityXEnabled() {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 6);
}
bool isActivityYEnabled() {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 5);
}
bool isActivityZEnabled() {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 4);
}
bool isInactivityXEnabled() {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 2);
}
bool isInactivityYEnabled() {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 1);
}
bool isInactivityZEnabled() {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 0);
}

void setActivityX(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state);
}
void setActivityY(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state);
}
void setActivityZ(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state);
}
void setActivityXYZ(bool stateX, bool stateY, bool stateZ) {
	setActivityX(stateX);
	setActivityY(stateY);
	setActivityZ(stateZ);
}
void setInactivityX(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 2, state);
}
void setInactivityY(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 1, state);
}
void setInactivityZ(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 0, state);
}
void setInactivityXYZ(bool stateX, bool stateY, bool stateZ) {
	setInactivityX(stateX);
	setInactivityY(stateY);
	setInactivityZ(stateZ);
}

bool isActivityAc() {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 7);
}
bool isInactivityAc(){
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 3);
}

void setActivityAc(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 7, state);
}
void setInactivityAc(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 3, state);
}

/************************* SUPPRESS BITS ****************************/
/*                                                                  */
bool getSuppressBit(){
	return getRegisterBit(ADXL345_TAP_AXES, 3);
}
void setSuppressBit(bool state) {
	setRegisterBit(ADXL345_TAP_AXES, 3, state);
}

/**************************** TAP BITS ******************************/
/*                                                                  */
bool isTapDetectionOnX(){
	return getRegisterBit(ADXL345_TAP_AXES, 2);
}
void setTapDetectionOnX(bool state) {
	setRegisterBit(ADXL345_TAP_AXES, 2, state);
}
bool isTapDetectionOnY(){
	return getRegisterBit(ADXL345_TAP_AXES, 1);
}
void setTapDetectionOnY(bool state) {
	setRegisterBit(ADXL345_TAP_AXES, 1, state);
}
bool isTapDetectionOnZ(){
	return getRegisterBit(ADXL345_TAP_AXES, 0);
}
void setTapDetectionOnZ(bool state) {
	setRegisterBit(ADXL345_TAP_AXES, 0, state);
}

void setTapDetectionOnXYZ(bool stateX, bool stateY, bool stateZ) {
	setTapDetectionOnX(stateX);
	setTapDetectionOnY(stateY);
	setTapDetectionOnZ(stateZ);
}

bool isActivitySourceOnX(){
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 6);
}
bool isActivitySourceOnY(){
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 5);
}
bool isActivitySourceOnZ(){
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 4);
}

bool isTapSourceOnX(){
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 2);
}
bool isTapSourceOnY(){
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 1);
}
bool isTapSourceOnZ(){
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 0);
}

/*************************** ASLEEP BIT *****************************/
/*                                                                  */
bool isAsleep(){
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 3);
}

/************************** LOW POWER BIT ***************************/
/*                                                                  */
bool isLowPower(){
	return getRegisterBit(ADXL345_BW_RATE, 4);
}
void setLowPower(bool state) {
	setRegisterBit(ADXL345_BW_RATE, 4, state);
}

/*************************** RATE BITS ******************************/
/*                                                                  */
double getRate(){
	uint8_t _b;
	readFrom(ADXL345_BW_RATE, 1, &_b);
	_b &= 0b00001111;
	return (pow(2,((int) _b)-6)) * 6.25;
}

void setRate(double rate){
	uint8_t _b,_s;
	int v = (int) (rate / 6.25);
	int r = 0;
	while (v >>= 1)
	{
		r++;
	}
	if (r <= 9) {
		readFrom(ADXL345_BW_RATE, 1, &_b);
		_s = (uint8_t) (r + 6) | (_b & 0b11110000);
		writeTo(ADXL345_BW_RATE, _s);
	}
}

/*************************** BANDWIDTH ******************************/
/*                          ~ SET & GET                             */
void set_bw(uint8_t bw_code){
	if((bw_code < ADXL345_BW_0_05) || (bw_code > ADXL345_BW_1600)){
		status = false;
		error_code = ADXL345_BAD_ARG;
	}
	else{
		writeTo(ADXL345_BW_RATE, bw_code);
	}
}

uint8_t get_bw_code(){
	uint8_t bw_code;
	readFrom(ADXL345_BW_RATE, 1, &bw_code);
	return bw_code;
}

/************************* TRIGGER CHECK  ***************************/
/*                                                                  */
// Check if Action was Triggered in Interrupts
// Example triggered(interrupts, ADXL345_SINGLE_TAP);
bool triggered(uint8_t interrupts, int mask){
	return ((interrupts >> mask) & 1);
}

/*
 ADXL345_DATA_READY
 ADXL345_SINGLE_TAP
 ADXL345_DOUBLE_TAP
 ADXL345_ACTIVITY
 ADXL345_INACTIVITY
 ADXL345_FREE_FALL
 ADXL345_WATERMARK
 ADXL345_OVERRUNY
 */


uint8_t getInterruptSource() {
	uint8_t _b;
	readFrom(ADXL345_INT_SOURCE, 1, &_b);
	return _b;
}

bool getInterruptSourceBit(uint8_t interruptBit) {
	return getRegisterBit(ADXL345_INT_SOURCE,interruptBit);
}

bool getInterruptMapping(uint8_t interruptBit) {
	return getRegisterBit(ADXL345_INT_MAP,interruptBit);
}

/*********************** INTERRUPT MAPPING **************************/
/*         Set the Mapping of an Interrupt to pin1 or pin2          */
// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
void setInterruptMapping(uint8_t interruptBit, bool interruptPin) {
	setRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
}

void setImportantInterruptMapping(int single_tap, int double_tap, int free_fall, int activity, int inactivity) {
	if(single_tap == 1) {
		setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );}
	else if(single_tap == 2) {
		setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT2_PIN );}

	if(double_tap == 1) {
		setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );}
	else if(double_tap == 2) {
		setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT2_PIN );}

	if(free_fall == 1) {
		setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,   ADXL345_INT1_PIN );}
	else if(free_fall == 2) {
		setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,   ADXL345_INT2_PIN );}

	if(activity == 1) {
		setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,   ADXL345_INT1_PIN );}
	else if(activity == 2) {
		setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,   ADXL345_INT2_PIN );}

	if(inactivity == 1) {
		setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );}
	else if(inactivity == 2) {
		setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT2_PIN );}
}

bool isInterruptEnabled(uint8_t interruptBit) {
	return getRegisterBit(ADXL345_INT_ENABLE,interruptBit);
}

void setInterrupt(uint8_t interruptBit, bool state) {
	setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
}

void singleTapINT(bool status) {
	if(status) {
		setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
	}
	else {
		setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 0);
	}
}
void doubleTapINT(bool status) {
	if(status) {
		setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
	}
	else {
		setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 0);
	}
}
void FreeFallINT(bool status) {
	if(status) {
		setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
	}
	else {
		setInterrupt( ADXL345_INT_FREE_FALL_BIT,  0);
	}
}
void ActivityINT(bool status) {
	if(status) {
		setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
	}
	else {
		setInterrupt( ADXL345_INT_ACTIVITY_BIT,   0);
	}
}
void InactivityINT(bool status) {
	if(status) {
		setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
	}
	else {
		setInterrupt( ADXL345_INT_INACTIVITY_BIT, 0);
	}
}

void setRegisterBit(uint8_t regAdress, int bitPos, bool state) {
	uint8_t _b;
	readFrom(regAdress, 1, &_b);
	if (state) {
		_b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.
	}
	else {
		_b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
	}
	writeTo(regAdress, _b);
}

bool getRegisterBit(uint8_t regAdress, int bitPos) {
	uint8_t _b;
	readFrom(regAdress, 1, &_b);
	return ((_b >> bitPos) & 1);
}

/********************************************************************/
/*                                                                  */
// Print Register Values to Serial Output =
// Can be used to Manually Check the Current Configuration of Device
void printAllRegister() {
	uint8_t _b;
	printf("0x00: ");
	readFrom(0x00, 1, &_b);
	print_byte(_b);
	printf("\n");
	int i;
	for (i=29;i<=57;i++){
		printf("0x%02x: ", i);
		readFrom(i, 1, &_b);
		print_byte(_b);
		printf("\n");
	}
}

void print_byte(uint8_t val){
	int i;
	printf("B");
	for(i=7; i>=0; i--){
		printf("%d", val >> i & 1);
	}
}
