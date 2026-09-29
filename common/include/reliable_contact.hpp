#pragma once

class ReliableContact
{
public:
    ReliableContact() = default;

    // s_i   - нормированная фаза ноги в диапазоне [0, 1].
    // state - текущее состояние ноги (STANCE/SWING/LATE_CONTACT/EARLY_CONTACT).
    // Возвращает gamma_i в диапазоне [0, 1].
    [[nodiscard]] float get_trust_coefficient(double s_i, int state) const noexcept;
};
