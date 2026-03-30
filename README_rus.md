# MORS Quadruped Robot Control

Репозиторий содержит базовый стек управления робособакой [МОРС](https://docs.voltbro.ru/mors/), использующий MPC/WBIC-контроллер. Для симуляции используется [MuJoCo](https://mujoco.org/). Задавать команды можно с помощью ROS2-интерфейса keyboard_teleop.

[![Watch the video](https://i9.ytimg.com/vi_webp/28EshOERJ94/mqdefault.webp?v=69bc15a6&sqp=CNTQms4G&rs=AOn4CLBik6T9q3Eg3SajVOnM9THA6gmlKw)](https://youtu.be/28EshOERJ94?si=7QsEtfh_oUpAAv3s)

Алгоритм управления основан на следующих публикациях:

Di Carlo, Jared, et al. "Dynamic locomotion in the mit cheetah 3 through convex model-predictive control." 2018 IEEE/RSJ international conference on intelligent robots and systems (IROS). IEEE, 2018. [Link](https://dspace.mit.edu/handle/1721.1/138000)

Kim, Donghyun, et al. "Highly dynamic quadruped locomotion via whole-body impulse control and model predictive control." arXiv preprint arXiv:1909.06586, 2019. [Link](https://arxiv.org/abs/1909.06586)


## Структура проекта

```text
.
├── common
├── config
├── lcm_msgs
├── LocomotionController
├── MorsLogger
├── ros_ws/src/mors_keyboard_control
├── ros_ws/src/robot_mode_controller
├── ros_ws/src/mors_ros_msgs
├── Simulator
├── start_controller.sh
└── install.sh
```

## Что за что отвечает

- `common` - общие C++ типы, вспомогательные функции, модели ног и URDF.
- `config` - YAML-конфиги контроллера, симуляции, аварийных ограничений и каналов.
- `lcm_msgs` - `.lcm` описания сообщений и генерация LCM-типов (`lcm_gen.sh`).
- `LocomotionController` - основной C++ контроллер (`locomotionControllerMPC`).
- `MorsLogger` - C++ логгер телеметрии (`mors_logger`).
- `ros_ws/src/mors_ros_msgs` - ROS 2 интерфейсы (`GaitParams.msg`, `RobotCmd.srv`).
- `ros_ws/src/robot_mode_controller` - ROS 2 узел режимов/действий.
- `ros_ws/src/mors_keyboard_control` - ROS 2 узел управления с клавиатуры.
- `Simulator` - MuJoCo-симулятор с LCM обменом.
- `start_controller.sh` - сценарий запуска основных компонентов.
- `install.sh` - установка зависимостей, сборка и настройка окружения.

## Требования

- Ubuntu 24.x
- ROS 2 Jazzy (`/opt/ros/jazzy`)
- `sudo` доступ

## Быстрый старт

### Установка

```bash
chmod +x install.sh start_controller.sh
./install.sh
source ~/.bashrc
```

### Запуск

Запуск симулятора вместе с контроллером шагания:

```bash
./start_controller.sh --sim
```

Запуск вместе с логгером:

```bash
./start_controller.sh --sim --log
```

После запуска обязательно дождитесь вывода в консоль:
```
[LocomotionController]: Started
```

После этого введите во втором терминале:

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ros2 run mors_keyboard_control mors_keyboard_control
```

## Управление с клавиатуры

Основные клавиши:

- `Enter` - встать / лечь.
- `Space` - переключение `STANDING_MODE` / `LOCOMOTION_MODE`.
- `W/S` - движение вперед/назад.
- `Q/E` - движение влево/вправо.
- `A/D` - поворот.
- `1..9` - максимальная скорость от `0.1` до `0.9`.
- `Arrow Up/Down` - изменение высоты корпуса.
- `Ctrl + Arrow Up/Down` - изменение `t_sw`.
- `Shift + Arrow Up/Down` - изменение высоты шага.

## Просмотр логов

Если вы используете ключ --log при запуске робота, то во время выполнения программы включается модуль MorsLogger и начинает постоянную запись данных из всех lcm-каналов в csv-файлы в папку mors_logs. 
Для просмотра графиков удобно пользоваться [plotjuggler](https://github.com/facontidavide/PlotJuggler).

## Конфигурация

Все файлы конфигурации находятся в папке `config`.
Список основных файлов: 
- Параметры MPC-контроллера - `stance_controller_mpc.yaml`
- Параметры swing-контроллера - `swing_controller.yaml`
- Параметры WBIC - `wbic.yaml`
- Параматры симуляции - `simulation.yaml`
- Физические параметры робота - `robot_config.yaml`
- Максимально/минимальные допустимые углы суставов - `emergency.yaml`

Остальные параметры трогать не стоит.

Путь к конфигам задается переменной `CONFIGPATH` (ее автоматически настраивает файл `install.sh`)

## Смена окружения робота

За тип окружения отвечает параметр `scene` в файле `config\simulation.yaml`. Вы  можете выбрать следующие окружения:

- `flat`
- `stairs`
- `patch`
- `boxes`
- `ramp`
- `boards`

Поэкспериментируйте с разными окружениями и параметрами движения с помощью горячих клавиш и посмотрите, как робот преодолевает различные препятствия.

![Environments](./pictures/environments.png)

## Ручная пересборка (при необходимости)

```bash
cd lcm_msgs
bash lcm_gen.sh

source /opt/ros/jazzy/setup.bash
cd ../ros_ws
colcon build --symlink-install --packages-select mors_ros_msgs robot_mode_controller mors_keyboard_control
cd ..

cmake -S LocomotionController -B LocomotionController/build -DCMAKE_BUILD_TYPE=Release
cmake --build LocomotionController/build -j"$(nproc)"

cmake -S MorsLogger -B MorsLogger/build -DCMAKE_BUILD_TYPE=Release
cmake --build MorsLogger/build -j"$(nproc)"
```

## Публикации

При использовании этой работы в академическом контексте, пожалуйста, сошлитесь на одну из следующих публикаций:


- Budanov V., Danilov V., Kapytov D., Klimov K. (2025). MORS: BLDC BASED SMALL SIZED QUADRUPED ROBOT. Journal of Computer and System Sciences International. no. 3, pp.152-176 DOI: 10.7868/S3034644425030146

- В. М. Буданов, В. А. Данилов, Д. В. Капытов, and К. В. Климов. Малогабаритный четырехногий шагающий робот на базе бесколлекторных моторов. Известия Российской академии наук. Теория и системы управления, (3):152–176, 2025. 

- К. В. Климов, Д. В. Капытов, В. А. Данилов, and А. А. Романов. Разработка конструкции компактной шагающей машины на электрических приводах для исследовательских задач. Известия высших учебных заведений. Машиностроение, 11(788), 2025.