#include <SimpleFOC.h>

void config_motor_pid(BLDCMotor *motor) {
    motor->PID_current_q.P = 0.02;
    motor->PID_current_q.I = 10.0;
    motor->PID_current_q.D = 0.0;
    motor->PID_current_q.limit = 5.0;
    motor->PID_current_q.output_ramp = 1e6;
    motor->LPF_current_q.Tf = 0.01;

    motor->PID_current_d.P = 0.02;
    motor->PID_current_d.I = 10.0;
    motor->PID_current_d.D = 0.0;
    motor->PID_current_d.limit = 5.0;
    motor->PID_current_q.output_ramp = 1e6;
    motor->LPF_current_d.Tf = 0.01;

    motor->velocity_index_search = 1;
    motor->voltage_sensor_align = 0.8;

    motor->PID_velocity.P = 2.5;
    motor->PID_velocity.I = 75;
    motor->PID_velocity.D = 0.02;
    motor->PID_velocity.limit = 5.0;
    motor->PID_velocity.output_ramp = 2.5e4;
    motor->LPF_velocity.Tf = 0.02;

    motor->P_angle.P = 35.0;
    motor->P_angle.I = 0;
    motor->P_angle.D = 1.5;
    motor->P_angle.output_ramp = 1200;
}
