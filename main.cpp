#include "mbed.h"
#include "DriveMotor.h"
#include "RotaryEncoder.h"
#include <cstdio>

#define SHOULDER_ACCELERATION 0.3
#define SHOULDER_DOWN_SPEED 0.9
#define CENTER_ACCELERATION 0.4
#define CURTAIN_ACCELERATION 0.3
#define HELPER_ACCELERATION 0.6

SPI SPI_bus(PA_7, PA_6, PA_5);
DigitalOut SPI_head(PB_8);

RawSerial PC(USBTX, USBRX, 115200);
RawSerial XBee(PC_10, PC_11, 230400);

DriveMotor M_shoulder(PB_1, PB_6, 0.5);
DriveMotor M_helper(PB_15, PA_10);
DriveMotor M_hindfoot(PB_14, PA_9);
DriveMotor M_center(PB_13, PA_8);
DriveMotor M_rolling(PA_11, PB_4);

PwmOut S_tail(PC_8);

RotaryEncoder RE_shoulder(PA_14, PA_13);
// RotaryEncoder RE_center(PH_1, PH_0);
RotaryEncoder RE_center(PC_3, PC_2);

DigitalIn SW_shoulder_upper(PA_4);
DigitalIn SW_helper_upper(PB_0);
DigitalIn SW_helper_lower(PC_1);
DigitalIn SW_shoulder_lower(PC_0);
DigitalIn SW_hindfoot_lower(PC_9);

DigitalOut LED_MAIN(PB_7);

void beyondAuto(void);
void liftShoulder(void);
void liftHelper(void);
void liftCenterAuto(int center_target_height);
void liftCenter(void);
void liftHindfoot(void);
void moveTail(void);
inline void receiveSignal(void);
inline void decordSignal(void);
inline void checkcheck_sum(void);
inline void sendSignal(int format);

int extended_sign_pool = 0x00;
int chin_sign_pool = 0x00;
int neck_rx_sign_pool = 0x00;
int neck_ry_sign_pool = 0x00;
int neck_rz_sign_pool = 0x00;

int extended_sign = 0x00;
int chin_sign = 0x00;
int neck_rx_sign = 0x00;
int neck_ry_sign = 0x00;
int neck_rz_sign = 0x00;
int check_sum_sign = 0x00;
int reply_sign = 0x0000;

int advanced_flag = 0;
int auto_start = 0;
int lift_direction = 0;
int helper_direction = 0;
int encoder_lock = 0;

int check_sum_correct = 0;

int auto_count = 0;

double helper_target_speed = 0.0;
double shoulder_target_speed = 0.0;
double center_target_speed = 0.0;
double hindfoot_target_speed = 0.0;

void initRobot(void) {
    NVIC_SetPriority(ADC_IRQn, 0);
    // XBee.format(8, SerialBase::Odd);
    XBee.attach(&receiveSignal, SerialBase::RxIrq);
    SPI_bus.format(16, 1);
    SPI_bus.frequency(20000000);
    M_shoulder.setFrequency(1500);
    M_shoulder.stop();
    M_helper.setFrequency(936);
    M_helper.stop();
    M_hindfoot.setFrequency(936);
    M_hindfoot.stop();
    M_center.setFrequency(936);
    M_center.stop();
    M_rolling.setFrequency(936);
    M_rolling.stop();
    RE_shoulder.Reset();
    RE_center.Reset();
    // RE_helper.Reset();
    SW_shoulder_upper.mode(PullUp);
    SW_shoulder_lower.mode(PullUp);
    SW_hindfoot_lower.mode(PullUp);
    SW_helper_upper.mode(PullUp);
    SW_helper_lower.mode(PullUp);
    S_tail.period_ms(20);
    S_tail.pulsewidth_us(1450);
}

int main() {

    initRobot();

    while(true) {
        wait_us(500);

        sendSignal(0);

        if(reply_sign == 0x00FF) {
            LED_MAIN = 1;
        }
        else {
            LED_MAIN = 0;
        }

        if(!encoder_lock) {
            moveTail();
        }

        hindfoot_target_speed = 0.0;
        center_target_speed = 0.0;
        shoulder_target_speed = 0.0;
        helper_target_speed = 0.0;
        double helper_rolling_speed = 0.0;

        if(advanced_flag) {
            hindfoot_target_speed = -lift_direction * 0.7;
            center_target_speed = -helper_direction * 0.5;
            auto_count = 0;
        }
        else {
            shoulder_target_speed = lift_direction;
            helper_target_speed = -helper_direction;
        }

        if(auto_start) {
            if(shoulder_target_speed == -1 && RE_shoulder.Get_Count() > -10) {
                beyondAuto(); 
            }
            else {
                helper_rolling_speed = 0.5;
            }
        }

        liftShoulder();
        liftCenter();
        liftHindfoot();
        liftHelper();
        M_rolling.drive(helper_rolling_speed);

        PC.printf("\033[H");
        PC.printf("EN_shoulder:\t%05d\r\nshoulder:\t%05.2lf\r\nEN_center:\t%05d\r\ncenter: \t%05.2lf\r\nhelper: \t%05.2lf\r\ncurtain:\t%05.2lf",RE_shoulder.Get_Count(),M_shoulder.read(),RE_center.Get_Count(),M_center.read(),M_helper.read(),M_hindfoot.read());
        // PC.printf("center : %5d\r\n", RE_center.Get_Count());
        // PC.printf("shoulder : %5d\r\n", RE_shoulder.Get_Count());

        // PC.printf("\r\n%d\r\n",signal_sel);
        // PC.printf("shoulder: %lf\r\n", M_shoulder.readPWM());
    }
}

void beyondAuto(void) {
    int flag = 0;
    encoder_lock = 1;

    // PC.printf("\033[H%5d", count);

    if(auto_count < 10) {
        RE_center.Reset();
    }
    else if(auto_count < 150) {
        liftCenterAuto(900);
    }
    else if(auto_count < 350) {
        liftCenterAuto(-10);
    }
    else if(auto_count < 450) {
        hindfoot_target_speed = -0.7;
    }
    else if(auto_count < 550) {
        hindfoot_target_speed = 0.0;
    }
    else if(auto_count < 650) {
        hindfoot_target_speed = 0.7;
    }
    else if(auto_count < 750) {
        hindfoot_target_speed = 0.0;
    }
    else if(auto_count > 750) {
        auto_count = 0;
    }
    auto_count++;
}

void liftShoulder(void) {
    double current_speed = M_shoulder.read();
    current_speed += (shoulder_target_speed - current_speed) * SHOULDER_ACCELERATION;
    
    if((RE_shoulder.Get_Count() > -10 || SW_shoulder_lower.read() == 0) && current_speed < 0) {
        current_speed = 0;
    }
    
    if(SW_shoulder_upper.read() == 0 && current_speed > 0) {
        current_speed = 0;
    }
    
    if(current_speed < 0) {
        current_speed *= SHOULDER_DOWN_SPEED;
    } 
    M_shoulder.drive(current_speed);

    // PC.printf("%d, %lf, %lf\r\n", RE_shoulder.Get_Count(), shoulder_target_speed, M_shoulder.read());
}

void liftHelper(void) {
    double current_speed = M_helper.read();
    current_speed += (helper_target_speed - current_speed) * HELPER_ACCELERATION;

    if(SW_helper_upper.read() == 0 && current_speed < 0) {
        current_speed = 0;
    }
    else if(SW_helper_lower.read() == 0 && current_speed > 0) {
        current_speed = 0;
    }

    M_helper.drive(current_speed);
    // PC.printf("%03.2lf\r\n", M_helper.read());
}

void liftCenterAuto(int center_target_height) {
    double current_height = RE_center.Get_Count();
    
    if(center_target_height - 30 > current_height) center_target_speed = -0.8;
    else if(center_target_height + 30 < current_height) center_target_speed = 0.8;
    else center_target_speed = 0;
}

void liftCenter(void) {
    double current_speed = M_center.read();
    current_speed += (center_target_speed - current_speed) * CENTER_ACCELERATION;
    M_center.drive(current_speed);
}

void liftHindfoot(void) {
    double current_speed = M_hindfoot.read();
    current_speed += (hindfoot_target_speed - current_speed) * CURTAIN_ACCELERATION;
    
    if(SW_hindfoot_lower.read() == 0 && current_speed > 0) {
        current_speed = 0;
    }

    M_hindfoot.drive(current_speed);
}

void moveTail(void) {
    static int count = 0;
    if(count < 800) {
        S_tail.pulsewidth_us(1000);
    }
    else if(count < 1600) {
        S_tail.pulsewidth_us(1900);
    }
    count = (++count) % 1600;
}

inline void receiveSignal(void) {
    int read_sign = XBee.getc();
    static int octets = 0;
    static int check_sum = 0x00;

    if(read_sign == 0xFF) {
        octets = 0;
    }
    if (read_sign >= 0){
        switch(octets) {
            case 0: check_sum = 0x00; ++octets; break;
            case 1: extended_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 2: chin_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 3: neck_ry_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 4: neck_rx_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 5: neck_rz_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 6: 
                check_sum_sign = read_sign;
                if(((check_sum % 0x100) == read_sign) || ((check_sum == 0xFF) && (read_sign == 0xFE))) {
                    check_sum_correct = 1;
                    extended_sign = extended_sign_pool;
                    chin_sign = chin_sign_pool;
                    neck_rx_sign = neck_rx_sign_pool;
                    neck_ry_sign = neck_ry_sign_pool;
                    neck_rz_sign = neck_rz_sign_pool;
                }
                else {
                    check_sum_correct = 0;
                }
                octets = 0; 
                break;
        }
    }  

    decordSignal();
}

inline void decordSignal(void) {
    advanced_flag = (extended_sign >> 7) % 0b10;
    auto_start = (extended_sign >> 6) % 0b10;
    lift_direction = (extended_sign >> 4) % 0b100;
    helper_direction = (extended_sign >> 2) % 0b100;
    encoder_lock = (extended_sign >> 1) % 0b10;

    if(lift_direction == 2) lift_direction = 1;
    else if(lift_direction == 1) lift_direction = -1;
    else lift_direction = 0;
    
    if(helper_direction == 2) helper_direction = 1;
    else if(helper_direction == 1) helper_direction = -1;
    else helper_direction = 0;

    // PC.printf("%#02X\r\n", extended_sign);
    // PC.printf("\033[H0x%02X%02X%02X%02X", extended_sign, chin_sign, neck_rx_sign, neck_ry_sign);
    // PC.printf("%d, %d, %d, %d, %d\r\n", advanced_flag, auto_start, lift_direction, helper_direction, encoder_lock);
}

inline void sendSignal(int format) {
    int send_sign;
    switch(format) {
        case 0:
            send_sign = 0xFF00;
            break;
        case 1:
            send_sign = chin_sign + 0x0200;
            break;
        case 2:
            send_sign = neck_rx_sign + 0x0300;
            break;
        case 3:
            send_sign = neck_ry_sign + 0x0400;
            break;
        case 4:
            send_sign = neck_ry_sign + 0x0500;
            break;
    }
    SPI_head = 0;
    reply_sign = SPI_bus.write(send_sign);
    // PC.printf("send: %#04X\r\n", send_sign);
    SPI_head = 1;
}
