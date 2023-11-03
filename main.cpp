#include "mbed.h"
#include "DriveMotor.h"
#include "RotaryEncoder.h"
#include <cstdio>

#define SHOULDER_ACCELERATION 0.3
#define SHOULDER_BACK_LIMIT 0.7
#define SHOULDER_DOWN_SPEED 0.6
#define CENTER_ACCELERATION 0.6
#define CURTAIN_ACCELERATION 0.8
#define HELPER_ACCELERATION 0.8

SPI SPI_bus(PA_7, PA_6, PA_5);
DigitalOut SPI_head(PB_8);

RawSerial PC(USBTX, USBRX, 115200);
RawSerial XBee(PC_10, PC_11, 230400);

DriveMotor M_lifting(PB_1, PB_6, 0.5);
DriveMotor M_helper(PB_15, PA_10);
DriveMotor M_curtain(PB_14, PA_9);
DriveMotor M_center(PB_13, PA_8);

PwmOut S_tail(PC_8);

RotaryEncoder RE_lifting(PA_14, PA_13);
// RotaryEncoder RE_center(PH_1, PH_0);
RotaryEncoder RE_center(PC_3, PC_2);

DigitalIn SW_shoulder_upper(PA_4);
DigitalIn SW_helper_upper(PB_0);
DigitalIn SW_helper_lower(PC_1);
// DigitalIn SW4(PC_0);

DigitalOut LED_MAIN(PB_7);

int beyondAuto(void);
void liftShoulder(void);
void liftShoulderAuto(int flag);
void liftHelper(void);
void liftCenter(void);
void liftCenterManual(void);
void breakCurtain(void);
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

int helper_flag = 0;
int auto_start = 0;
int auto_start_old = 0;
int lift_direction = 0;
int curtain_break = 0;
int encoder_lock = 0;

int check_sum_correct = 0;

int lifting_target_height = 0;
int center_target_height = 0;
double helper_target_speed = 0.0;
double lifting_target_speed = 0.0;
double center_target_speed = 0.0;
double curtain_target_speed = 0.0;

void initRobot(void) {
    NVIC_SetPriority(ADC_IRQn, 0);
    // XBee.format(8, SerialBase::Odd);
    XBee.attach(&receiveSignal, SerialBase::RxIrq);
    SPI_bus.format(16, 1);
    SPI_bus.frequency(20000000);
    M_lifting.setFrequency(1500);
    M_lifting.stop();
    M_helper.setFrequency(936);
    M_helper.stop();
    M_curtain.setFrequency(936);
    M_curtain.stop();
    M_center.setFrequency(936);
    M_center.stop();
    RE_lifting.Reset();
    RE_center.Reset();
    // RE_helper.Reset();
    SW_shoulder_upper.mode(PullUp);
    SW_helper_upper.mode(PullUp);
    SW_helper_lower.mode(PullUp);
    S_tail.period_ms(20);
    S_tail.pulsewidth_us(1450);
}

int main() {
    int shoulder_auto_flag = 0;
    int signal_sel = 1;

    initRobot();

    while(true) {
        wait_us(500);
        shoulder_auto_flag = auto_start;

        if(reply_sign == 0x00FF) {
            LED_MAIN = 1;
        }
        else {
            LED_MAIN = 0;
        }

        if(!encoder_lock) {
            sendSignal(signal_sel);
            moveTail();
        }
        else {
            sendSignal(0);
        }

        if(shoulder_auto_flag) {
            liftShoulderAuto(beyondAuto());
        }
        else {
            lifting_target_speed = lift_direction;     
            center_target_height = RE_center.Get_Count();  
            liftShoulder(); 
        }

        if(helper_flag) {
            if(curtain_break != 0) {
                center_target_speed = -curtain_break;
            }
            else {
                center_target_speed = 0.0;
            }
            liftCenterManual();
            center_target_height = RE_center.Get_Count();
        }
        else {
            helper_target_speed = -curtain_break;
            center_target_speed = 0.0;
            liftCenter();
        }
        liftHelper();
        breakCurtain();

        // PC.printf("\033[H");
        // PC.printf("EN_shoulder:\t%05d\r\nshoulder:\t%05.2lf\r\ncenter: \t%05.2lf\r\nhelper: \t%05.2lf\r\ncurtain:\t%05.2lf",RE_lifting.Get_Count(),M_lifting.read(),M_center.read(),M_helper.read(),M_curtain.read());
        // PC.printf("center : %5d\r\n", RE_center.Get_Count());
        // PC.printf("lifting : %5d\r\n", RE_lifting.Get_Count());

        signal_sel = (signal_sel % 3) + 1;
        //↑絶対にコメントアウトするな

        // PC.printf("\r\n%d\r\n",signal_sel);
        // PC.printf("lifting: %lf\r\n", M_lifting.readPWM());
    }
}

int beyondAuto(void) {
    static int count = 0;
    int flag = 0;
    encoder_lock = 1;

    // PC.printf("\033[H%5d", count);

    if(helper_flag == 1) {
        count = 0;
        RE_center.Reset();
    }
    else {
        if(count > 0 && count < 100) {
            RE_center.Reset();
        }
        else if(count < 3000) {
            center_target_height = -270;
            flag = 0;
        }
        else if(count < 6000) {
            center_target_height = 0;
            flag = 0;
        }
        else if(count < 6500) {
            lifting_target_speed = 0.0;
            center_target_height = 950;
            flag = 1;
        }
        else if(count < 12500) { 
            lifting_target_height = RE_center.Get_Count();
            center_target_height = 950;
            flag = 0;
        }
        else if(count < 20000) {
            lifting_target_height = 400;
            center_target_height = 0;
            flag = 0;
        }
        else if(count < 23500) {
            lifting_target_height = 0;
            center_target_height = 0;
            flag = 0;
        }
        else if(count < 25000) {
            center_target_height = 100;
        }
        else if(count > 25500) {
            count = 0;
            flag = 0;

        }
        count++;
    }
    return flag;
}

void liftShoulder(void) {
    double current_speed = M_lifting.read();
    current_speed += (lifting_target_speed - current_speed) * SHOULDER_ACCELERATION;
    
    if(!helper_flag) {
        if(RE_lifting.Get_Count() > -10 && current_speed < 0) {
            current_speed = 0;
        }
    }

    if(SW_shoulder_upper.read() == 0 && current_speed > 0) {
        current_speed = 0;
    }
    
    if(current_speed < 0) {
        if(!helper_flag) {
            current_speed *= SHOULDER_DOWN_SPEED;
        }
    } 
    M_lifting.drive(current_speed);

    // PC.printf("%d, %lf, %lf\r\n", RE_lifting.Get_Count(), lifting_target_speed, M_lifting.read());
}

void liftShoulderAuto(int flag) {
    double current_height = RE_lifting.Get_Count();
    double target_speed;
    
    if(flag) {
        double current_speed = M_lifting.read();
        current_speed += (lifting_target_speed - current_speed) * SHOULDER_ACCELERATION;
        M_lifting.drive(current_speed);
        // PC.printf("%lf\r\n", current_speed);
    }
    else {
        if(lifting_target_height + 100 < current_height) target_speed = 0.8;
        else if(lifting_target_height - 100 > current_height) target_speed = -0.8;
        else target_speed = 0;

        double current_speed = M_lifting.read();
        current_speed += (target_speed - current_speed) * SHOULDER_ACCELERATION;
        if(current_speed < 0) current_speed *= SHOULDER_DOWN_SPEED;
        M_lifting.drive(current_speed); 
    }
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

void liftCenter(void) {
    double current_height = RE_center.Get_Count();
    double current_speed = M_center.read();
    
    if(center_target_height - 10 > current_height) center_target_speed = 1.0;
    else if(center_target_height + 10 < current_height) center_target_speed = -1.0;
    else center_target_speed = 0;

    current_speed += (center_target_speed - current_speed) * CENTER_ACCELERATION;
    M_center.drive(current_speed); 
}

void liftCenterManual(void) {
    double current_speed = M_center.read();
    current_speed += (center_target_speed - current_speed) * CENTER_ACCELERATION;
    M_center.drive(current_speed);
}

void breakCurtain(void) {
    double current_speed = M_curtain.read();
    current_speed += (curtain_target_speed - current_speed) * CURTAIN_ACCELERATION;
    M_curtain.drive(current_speed);
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
            case 3: neck_rx_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 4: neck_ry_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
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
    auto_start_old = auto_start;

    helper_flag = (extended_sign >> 7) % 0b10;
    auto_start = (extended_sign >> 6) % 0b10;
    lift_direction = (extended_sign >> 4) % 0b100;
    curtain_break = (extended_sign >> 2) % 0b100;
    encoder_lock = (extended_sign >> 1) % 0b10;

    if(lift_direction == 2) lift_direction = 1;
    else if(lift_direction == 1) lift_direction = -1;
    else lift_direction = 0;
    
    if(curtain_break == 2) curtain_break = 1;
    else if(curtain_break == 1) curtain_break = -1;
    else curtain_break = 0;

    // PC.printf("%#02X\r\n", extended_sign);
    PC.printf("\033[H0x%02X%02X%02X%02X, %d", extended_sign, chin_sign, neck_rx_sign, neck_ry_sign, SW_helper_lower.read());
    // PC.printf("%d, %d, %d, %d, %d\r\n", helper_flag, auto_start, lift_direction, curtain_break, encoder_lock);
}

inline void sendSignal(int format) {
    int send_sign;
    switch(format) {
        case 0:
            send_sign = 0x0000;
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
