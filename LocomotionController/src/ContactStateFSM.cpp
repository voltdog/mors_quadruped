#include "ContactStateFSM.hpp"
#include <cmath>
#include <algorithm>

// Конструктор
ContactStateFSM::ContactStateFSM(double start_td_detecting,
                                 int contact_debounce,
                                 double vz_contact_thresh)
    : start_td_detecting(start_td_detecting),
      contact_debounce_(contact_debounce),
      vz_contact_thresh_(vz_contact_thresh),
      state(4, STANCE),
      contact_count_(4, 0) {}//, phi_pre(4, 0.0) {}

// Метод step
std::vector<int> ContactStateFSM::step(const std::vector<bool>& contact_flag,
                                       const std::vector<double>& phi,
                                       const std::vector<int>& des_leg_state,
                                       const std::vector<double>& foot_vz)
{
    for (int i = 0; i < 4; ++i) {
        // "Подтверждённое" касание: наблюдатель силы сработал И стопа почти не движется по
        // вертикали. Это отсекает ложные всплески оценки силы в фазе переноса, когда стопа
        // ещё летит вниз (|vz| велик), а контакта с поверхностью фактически нет.
        bool confirmed_contact = contact_flag[i] && (std::fabs(foot_vz[i]) < vz_contact_thresh_);
        // cout << vz_contact_thresh_ << endl;

        // Дебаунс: считаем подряд идущие подтверждённые касания; единичный шумовой выброс
        // не успевает накопить счётчик и не приводит к ложному раннему контакту.
        if (confirmed_contact)
            contact_count_[i] = std::min(contact_count_[i] + 1, contact_debounce_);
        else
            contact_count_[i] = 0;

        bool debounced_contact = confirmed_contact;// (contact_count_[i] >= contact_debounce_);

        if (state[i] == SWING) {
            // Ранний контакт латчим только по устойчивому подтверждённому касанию.
            if (des_leg_state[i] == SWING) {
                if (debounced_contact && phi[i] > start_td_detecting) {
                    state[i] = EARLY_CONTACT;
                }
            } else if (des_leg_state[i] == STANCE) {
                // Scheduled touchdown тоже проходит через фильтр скорости, чтобы сырой
                // всплеск GRF не переводил ногу в STANCE при большой |vz|. Отсев здесь
                // безопасен: LATE_CONTACT выходит по сырому флагу, поэтому реальное касание
                // подтвердится на следующем такте.
                state[i] = debounced_contact ? STANCE : LATE_CONTACT;
            }
        } else if (state[i] == STANCE) {
            if (des_leg_state[i] == SWING) {
                state[i] = SWING;
            }
        } else if (state[i] == LATE_CONTACT) {
            // Выход по СЫРОМУ флагу контакта, без фильтра по |vz|. В LATE_CONTACT нога уже
            // прошла плановый touchdown, и SwingTrajectoryGenerator продавливает её вниз со
            // скоростью dz_near_ground. Требование "стопа почти неподвижна" здесь образует
            // положительную обратную связь: пока нога не в STANCE, её продолжают опускать,
            // из-за чего |vz| только растёт и условие выхода не выполняется никогда.
            // На повороте это защёлкивается намертво (|omega x r| стопы > порога всё время),
            // нога уходит на 10 см под опору и корпус теряет устойчивость.
            if (contact_flag[i]) {
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
