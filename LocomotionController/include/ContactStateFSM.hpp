#ifndef CONTACTSTATE_FSM_HPP
#define CONTACTSTATE_FSM_HPP

#include <vector>
#include "structs.hpp"

class ContactStateFSM {
public:
    // Константы состояний
    // static constexpr int SWING = 0;
    // static constexpr int STANCE = 1;
    // static constexpr int LATE_CONTACT = 2;
    // static constexpr int EARLY_CONTACT = 3;

    // Конструктор
    explicit ContactStateFSM(double start_td_detecting);

    // Метод для выполнения шага
    std::vector<int> step(const std::vector<bool>& contact_flag, const std::vector<double>& phi, const std::vector<int>& des_leg_state) ;

private:
    double start_td_detecting; // Параметр для обнаружения начала фазы
    std::vector<int> state;    // Текущие состояния для каждой ноги
    std::vector<double> phi_pre; // Предыдущие значения фаз
};

#endif // CONTACTSTATE_FSM_HPP