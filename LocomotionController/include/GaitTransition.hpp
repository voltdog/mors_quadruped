#ifndef SIMPLE_GAIT_TRANSITION_HPP
#define SIMPLE_GAIT_TRANSITION_HPP

#include <boost/numeric/odeint.hpp>
#include <cmath>
#include <vector>


using namespace std;


#define    SWING         0
#define    STANCE        1
#define    LATE_CONTACT  2
#define    EARLY_CONTACT 3

class GaitTransition {
public:
    GaitTransition();

    void set_transition_duration(double delta_T_gt);
    void set_gait_params(const double& t_st,
                        const double& t_sw,
                        const std::vector<double>& phase_offsets,
                        const double& stride_height);

    void make_transition(double t, double& t_st_out,
                        double& t_sw_out,
                        std::vector<double>& phase_offsets_out,
                        double& stride_height_out);

private:
    double saturation(double x, double y);
    double t_saturation;
    double t_st, pre_t_st, t_st_old;
    double t_sw, pre_t_sw, t_sw_old;
    double stride_height, pre_stride_height, stride_height_old;
    std::vector<double> phase_offsets, pre_phase_offsets, phase_offsets_old;

    bool standing;

    double t_gt, delta_T_gt;

    bool transisting;

};

#endif // SIMPLE_GAIT_TRANSITION_HPP