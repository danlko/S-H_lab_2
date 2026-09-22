# Лабораторна робота №2 — Gazebo та ArduPilot SITL

ROS 2-пакет для лабораторної роботи з моделювання польоту квадрокоптера в Gazebo та ArduPilot SITL. Проєкт запускається у Docker-контейнері `ardupilot_ros`; вихідний код пакета змонтований з WSL, тому зберігається після пересоздання контейнера.

## Що реалізовано

- `fpv_lab2 flight_test` — перевірка готовності, GUIDED, arm, зліт на 2 м, зависання 30 секунд і автоматична посадка.
- `fpv_lab2 custom_flight` — зліт, політ до точки варіанта 19, утримання позиції та автоматична посадка.
- Варіант 19 задає зміщення від фактичної стартової позиції: `x_target = x0 - 8.2 м`, `y_target = y0 + 0.4 м`.
- Для ArduPilot DDS використовується namespace `/ap/v1`.

## Структура

```text
.
├── Dockerfile
├── docker-compose.yml
├── start_sim.sh
└── fpv_labs/
    └── src/
        └── fpv_lab2/
            ├── package.xml
            ├── setup.py
            └── fpv_lab2/
                ├── flight_common.py
                ├── flight_test.py
                └── custom_flight.py
```

`docker-compose.yml` монтує `./fpv_labs` у контейнер як `/home/ros/ros2_ws/src/fpv_labs`. Тому редагуй і зберігай вихідні файли у цьому репозиторії, а не лише всередині контейнера.

## Перший запуск

Потрібні WSL/Linux, Docker Compose та доступний Docker daemon.

### Термінал 1 — WSL, корінь репозиторію

```bash
cd /home/danylo/labs
docker compose build ardupilot_ros
./start_sim.sh
```

`start_sim.sh` запускає контейнер і Gazebo/ArduPilot SITL. Залиш цей термінал відкритим.

### Термінал 2 — увійти в контейнер і зібрати ROS 2-пакет

```bash
cd /home/danylo/labs
docker compose exec ardupilot_ros bash
```

Далі виконуй команди **всередині контейнера**:

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros2_ws/install/setup.bash
cd /home/ros/ros2_ws
colcon build --symlink-install --packages-select fpv_lab2
source install/setup.bash
ros2 pkg executables fpv_lab2
```

У списку мають бути `flight_test` і `custom_flight`.

## Запуск вузлів

Запускай вузли **по одному**, дочекавшись автоматичної посадки перед наступним запуском.

```bash
ros2 run fpv_lab2 flight_test
```

Після успішного завершення `flight_test`:

```bash
ros2 run fpv_lab2 custom_flight
```

Під час `custom_flight` журнал показує стартові координати, ціль, підтвердження набору висоти, досягнення цілі та перехід до посадки.

## Перезапуск і зупинка

Після перезапуску контейнера або редагування Python-коду пакет можна перебудувати так:

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros2_ws/install/setup.bash
cd /home/ros/ros2_ws
colcon build --symlink-install --packages-select fpv_lab2
source install/setup.bash
```

Щоб зупинити симуляцію, натисни `Ctrl+C` у терміналі з `start_sim.sh`, а потім у WSL з кореня репозиторію виконай:

```bash
docker compose down
```

## Збереження в GitHub

У репозиторій додаються вихідний код, Docker-файли та інструкція. `build/`, `install/` і `log/` ігноруються як згенеровані артефакти; пакет відновлюється командою `colcon build`.

Перед здачею лабораторної додай у звіт журнали запуску, координати до та після польоту, похибку позиціонування і скриншоти початкового та кінцевого положень.
