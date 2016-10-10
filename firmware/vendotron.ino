// This #include statement was automatically added by the Particle IDE.
#include "AccelStepperSpark/AccelStepperSpark.h"

// This #include statement was automatically added by the Particle IDE.
#include "Arduino/Arduino.h"

// This #include statement was automatically added by the Particle IDE.
#include "neopixel/neopixel.h"

SYSTEM_MODE(MANUAL);


#define STEP_FACTOR                 8 // stepper motor microstepping value
#define STEPS_PER_ROTATION          800

#define STEPPER_ENA_OUTPUT          A4

#define FORWARD_INPUT               D5
#define REVERSE_INPUT               D6

#define BROWSE_DISTANCE             50

#define NORMAL_SPEED                50
#define NORMAL_ACCEL                10


// homing stuff (all in steps)
#define HOME_INPUT                  D7
#define HOME_SEARCH_SPEED           50
#define HOME_SEARCH_ACCEL           10
#define HOME_LATCH_BACKOFF         -50
#define HOME_LATCH_SPEED            25
#define HOME_LATCH_ACCEL            5
#define HOME_LATCH_FINAL            -20

#define DOOR_1_PIN                  WKP
#define DOOR_1_POS_CLOSE            30
#define DOOR_1_POS_OPEN             90

#define DOOR_2_PIN                  D0
#define DOOR_2_POS_CLOSE            120
#define DOOR_2_POS_OPEN             30

#define DOOR_3_PIN                  D1
#define DOOR_3_POS_CLOSE            120
#define DOOR_3_POS_OPEN             30

#define DOOR_4_PIN                  D2
#define DOOR_4_POS_CLOSE            120
#define DOOR_4_POS_OPEN             30

#define DOOR_5_PIN                  D3
#define DOOR_5_POS_CLOSE            120
#define DOOR_5_POS_OPEN             30

#define DOOR_OPEN_TIME              8000


// globals
volatile bool at_home = false;
bool forward = true;

Servo door1; 
Servo door2;
Servo door3;
Servo door4;
Servo door5;

String active_row;

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, A1, A2);

void setup() {  
    
    Particle.connect();
    
    Particle.function("home", do_home);
    Particle.function("motor", motor_mode);
    Particle.function("rotate", rotate_to);
    Particle.function("step", step_to);
    Particle.function("current", current_pos);
    Particle.function("open", open);
    Particle.function("dispense", dispense);
    
    pinMode(HOME_INPUT, INPUT_PULLUP);
    pinMode(FORWARD_INPUT, INPUT_PULLUP);
    pinMode(REVERSE_INPUT, INPUT_PULLUP);
    pinMode(STEPPER_ENA_OUTPUT, OUTPUT);
    
    attachInterrupt(HOME_INPUT, home_triggered, FALLING);
    
    active_row = NULL;
    
    door1.attach(DOOR_1_PIN);
    door1.write(DOOR_1_POS_CLOSE);
    
    door2.attach(DOOR_2_PIN);
    door2.write(DOOR_2_POS_CLOSE);
    
    door3.attach(DOOR_3_PIN);
    door3.write(DOOR_3_POS_CLOSE);
    
    door4.attach(DOOR_4_PIN);
    door4.write(DOOR_4_POS_CLOSE);
    
    door5.attach(DOOR_5_PIN);
    door5.write(DOOR_5_POS_CLOSE);
  
    motor_enable();
    
    stepper.stop();
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(steps(NORMAL_SPEED));
    stepper.setAcceleration(steps(NORMAL_ACCEL));
}


// cloud functions

int motor_mode(String mode) {
    if (mode == "enable") {
        motor_enable();
    } else if (mode == "disable") {
        motor_disable();
    }
}

int rotate_to(String angle) {
    do_rotate(angle.toInt());
}

int step_to(String s) {
    
    int raw_steps = steps(s.toInt());
    
    Particle.publish("step_to", s);
    stepper.runToNewPosition(raw_steps);
}

int current_pos(String arg) {
    return stepper.currentPosition();
}

int open(String door) {
    if (door == "1") {
        door1.write(DOOR_1_POS_OPEN);
        delay(DOOR_OPEN_TIME);
        door1.write(DOOR_1_POS_CLOSE);
        delay(100);
    }
    return 0;
}

int dispense(String slot) {
    do_rotate(70);
    delay(40);
    active_row = "1";
}


// interrupt handlers

void home_triggered() {
    at_home = true;
}


// app functions

int steps(int steps) {
    return steps * STEP_FACTOR;
}

void do_rotate(int angle) {
    
    int target = steps(angle_to_steps(angle));
    int current = stepper.currentPosition();
    int delta = target - current;
    
    Particle.publish("do_rotate", String::format("%d/%d/%d/%d", angle, target, current, delta));

    if (abs(delta) > steps(STEPS_PER_ROTATION)/2) {
        stepper.moveTo(target - steps(STEPS_PER_ROTATION));
    } else {
        stepper.moveTo(target);
    }
}

void motor_enable() {
    digitalWrite(STEPPER_ENA_OUTPUT, LOW);
}

void motor_disable() {
    digitalWrite(STEPPER_ENA_OUTPUT, HIGH);
}


int angle_to_steps(int angle) {
    return (STEPS_PER_ROTATION/360.0)*angle;
}

int do_home(String cmd) {
    
    at_home = false;
    
    stepper.stop();
    stepper.setCurrentPosition(0);

    // first we search
    stepper.setMaxSpeed(steps(HOME_SEARCH_SPEED));
    stepper.setAcceleration(steps(HOME_SEARCH_ACCEL));
    stepper.moveTo(steps(STEPS_PER_ROTATION*2)); // move around twice
    while (!at_home) {
        if (!stepper.run()) {
            Particle.publish("home_search_failed");
            return 1;
        }
    }

    Particle.process();

    
    // then move off
    stepper.stop();
    stepper.setCurrentPosition(0);
    stepper.runToNewPosition(steps(HOME_LATCH_BACKOFF));

    Particle.process();

    at_home = false; // look for home again

    // then latch slowly
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(steps(HOME_LATCH_SPEED));
    stepper.setAcceleration(steps(HOME_LATCH_ACCEL));
    stepper.moveTo(steps(STEPS_PER_ROTATION*2)); // move around twice
    while (!at_home) {
        if (!stepper.run()) {
            Particle.publish("home_latch_failed");
            return 1;
        }
    }
    
    stepper.stop();
    stepper.setCurrentPosition(0);
    
    Particle.process();

    stepper.runToNewPosition(steps(HOME_LATCH_FINAL));
    
    stepper.stop();
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(steps(NORMAL_SPEED));
    stepper.setAcceleration(steps(NORMAL_ACCEL));
    
   Particle.publish("home_complete");
   return 0;
}


void loop() {
    
    if (!Particle.connected()) {
        Particle.connect();
    }
    
    if (digitalRead(FORWARD_INPUT) == LOW) {
        stepper.moveTo(stepper.currentPosition() + steps(BROWSE_DISTANCE));
    }
    
    if (digitalRead(REVERSE_INPUT) == LOW) {
        stepper.moveTo(stepper.currentPosition() - steps(BROWSE_DISTANCE));
    }

    stepper.run();
    
    // if after move is complete, we have a position greater than 1 rotation then 
    // we need to recalculate the current position
    if (stepper.distanceToGo() == 0) {
        if (stepper.currentPosition() < 0) {
            stepper.setCurrentPosition(steps(STEPS_PER_ROTATION) + stepper.currentPosition());
        }
        if (stepper.currentPosition() > steps(STEPS_PER_ROTATION)) {
            stepper.setCurrentPosition(stepper.currentPosition() - steps(STEPS_PER_ROTATION));
        }
        
        Particle.process();
    }
    
    // are we there yet?
    if (stepper.currentPosition() == stepper.targetPosition()) {
           if (active_row != NULL) {
            open(active_row);
            active_row = NULL;
        }
    }
    

    // only talk to particle cloud while not moving the stepper to avoid stepper delays
    // because this is based on speed, it should/could still talk to particle cloud
    // while an active target is set
    // another possibility here is to also do particle updates while stepper.distanceToGo() != 0 && stepper.speed() != stepper.maxSpeed()
    // if (stepper.currentPosition() == stepper.targetPosition()) {
    //     Particle.process();
    // }
}
