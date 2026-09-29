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
    // contact_debounce    - сколько подряд "подтверждённых" касаний нужно, чтобы залатчить ранний контакт
    // vz_contact_thresh   - порог |vz| стопы (м/с): выше него касание считается ложным (стопа ещё летит)
    explicit ContactStateFSM(double start_td_detecting,
                             int contact_debounce = 3,
                             double vz_contact_thresh = 0.1);

    // Метод для выполнения шага
    // foot_vz - вертикальная скорость стопы в МСК на ногу (порядок r1, l1, r2, l2)
    std::vector<int> step(const std::vector<bool>& contact_flag,
                          const std::vector<double>& phi,
                          const std::vector<int>& des_leg_state,
                          const std::vector<double>& foot_vz);

private:
    double start_td_detecting;     // Параметр для обнаружения начала фазы
    int contact_debounce_;         // Порог дебаунса (число подряд идущих подтверждённых касаний)
    double vz_contact_thresh_;     // Порог |vz| стопы для признания касания, м/с
    std::vector<int> state;        // Текущие состояния для каждой ноги
    std::vector<int> contact_count_; // Счётчик дебаунса на ногу
    std::vector<double> phi_pre;   // Предыдущие значения фаз
};

#endif // CONTACTSTATE_FSM_HPP