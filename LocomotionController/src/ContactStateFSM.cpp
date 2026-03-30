#include "ContactStateFSM.hpp"

// Конструктор
ContactStateFSM::ContactStateFSM(double start_td_detecting)
    : start_td_detecting(start_td_detecting), state(4, STANCE) {}//, phi_pre(4, 0.0) {}

// Метод step
std::vector<int> ContactStateFSM::step(const std::vector<bool>& contact_flag, const std::vector<double>& phi, const std::vector<int>& des_leg_state) 
{
    for (int i = 0; i < 4; ++i) {
        if (state[i] == SWING) {
            if (contact_flag[i] == true && des_leg_state[i] == SWING && phi[i] > start_td_detecting) {
                state[i] = EARLY_CONTACT;
            }
            if (contact_flag[i] == false && des_leg_state[i] == STANCE) {
                state[i] = LATE_CONTACT; 
            }
            if (contact_flag[i] == true and des_leg_state[i] == STANCE) {
                    state[i] = STANCE;
            }
        } else if (state[i] == STANCE) {
            if (des_leg_state[i] == SWING) {
                state[i] = SWING;
            }
        } else if (state[i] == LATE_CONTACT) {
            if (contact_flag[i] == true) {
                state[i] = STANCE;
            }
        } else if (state[i] == EARLY_CONTACT) {
            if (des_leg_state[i] == STANCE) {
                state[i] = STANCE;
            }
        }

        // Обновляем предыдущее значение фазы
        // phi_pre[i] = phi[i];
    }

    return state;
}