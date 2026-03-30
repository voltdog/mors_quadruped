#ifndef _csv_maintainer_hpp_
#define _csv_maintainer_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include "system_functions.hpp"
#include <yaml-cpp/yaml.h>
#include "structs.hpp"
#include "CSVWriter.h"
#include <filesystem>
#include <ctime>

using namespace std;
using namespace Eigen;
using namespace YAML;

class CSVMaintainer
{
    public:
        explicit CSVMaintainer(bool debug_mode = false);
        ~CSVMaintainer();

        void init();
        
        void write_servo_state(double time, ServoData& servo_state);
        void write_servo_state_filt(double time, ServoData& servo_state_filt);
        void write_servo_cmd(double time, ServoData& servo_cmd);
        void write_mpc_cmd(double time, RobotData& robot_cmd);
        void write_wbc_cmd(double time, LegData& leg_cmd, RobotData& robot_cmd);
        void write_imu_data(double time, ImuData& imu_data);
        void write_enable(double time, bool leg_controller_enable, bool leg_controller_reset,
                                        bool locomotion_enable, bool locomotion_reset,
                                        bool action_ctr_enable, bool action_ctr_reset);
        void write_odometry(double time, Odometry& odometry);
        void write_robot_state(double t, RobotData& body_state, LegData& leg_state);
        void write_robot_state_check(double t, RobotData& body_state_check, LegData& leg_state_check);
        void write_robot_cmd(double t, RobotData& body_cmd);
        void write_phase_sig(double t, Vector4i& phase, Vector4d& phi);
        void write_contact_states(double t, vector<bool>& contact_states);

    private:
        void create_csv(CSVWriter &csv, const vector<string> &head, string &addr);
        void set_vector(VectorXd &data, int size, CSVWriter &csv);
        void set_vector(Vector3d &data, CSVWriter &csv);
        void set_vector(Vector4d &data, CSVWriter &csv);
        void set_vector(Vector4i &data, CSVWriter &csv);
        void set_vector(vector<bool> &data, int size, CSVWriter &csv);
        // vector<string> servo_state_head;

        CSVWriter servo_state_csv, servo_cmd_csv, imu_data_csv, mpc_cmd_csv, wbc_cmd_csv, grf_cmd_csv;
        CSVWriter enable_csv, odometry_csv, contact_sensor_csv;
        CSVWriter servo_state_filt_csv;
        CSVWriter robot_state_csv, robot_state_check_csv, robot_cmd_csv;
        CSVWriter phase_sig_csv;

        string log_folder;
        string servo_state_addr, servo_cmd_addr, imu_data_addr, mpc_cmd_addr, wbc_cmd_addr, grf_cmd_addr;
        string enable_addr, odometry_addr, contact_sensor_addr;
        string servo_state_filt_addr, robot_state_addr, robot_state_check_addr, robot_cmd_addr, phase_sig_addr;
        bool debug_mode;

};

#endif //_csv_maintainer_hpp_
