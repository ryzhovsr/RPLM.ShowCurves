# RPLM.ShowCurves

Команда, интегрируемая в САПР "Сарус", которая помогает визуализировать и строить сопряжённые кривые на основе входных данных (контрольные точки, узлы, весовые коэффициенты и т.д.).

## Что умеет

- Загружает наборы NURBS-кривых из текстового файла.
- Создаёт объекты данных кривых с проверкой на всевозможные ошибки.
- Отображает исходные и результирующие кривые в сцене.

## Стек

- **C++17 / CMake**
- **Qt5** (`Core`, `Gui`, `Widgets`)

## Структура проекта

```text
RPLM.ShowCurves/
├─ RPLM.ShowCurves/              # CMake-проект плагина
│  ├─ src/Include/               # Заголовки
│  ├─ src/Source/                # Реализация
│  ├─ src/Resources/             # Ресурсы и локализация
│  └─ make_vs*_x64.cmd           # Скрипты сборки под Visual Studio
└─ Examples/                     # Примеры входных данных
```

## Быстрый старт

### Вариант 1: готовые скрипты (Windows)

Из директории `RPLM.ShowCurves/RPLM.ShowCurves`:

```bat
make_vs22_x64.cmd -d
```

Полезные флаги:

- `-p` — только генерация проекта
- `-c` — очистка и полная пересборка
- `-r` / `-d` / `-rwdi` — конфигурация (`Release` / `Debug` / `RelWithDebInfo`)
- `-i` — установка после сборки

### Вариант 2: через CMake вручную

```bash
cmake -S RPLM.ShowCurves -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Debug
```

## Формат входного файла кривых

Пример (см. `Examples/2 Кривые Безье 5 степени/SourceCurves.txt`):

```txt
Degree: 5
IsPeriodic: false
Control Points[6]:
0.0, 0.0, 0.0
...
Weights[6]:
1, 1, 1, 1, 1, 1
Knots[12]:
0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1
```

Допускается несколько кривых в одном файле, разделённых пустой строкой.

## Автоматическая инициализация модуля в системе
В RPLM.Applications.ini добавить:
```
[RPLM.CAD.ShowCurves]
path=RPLM.CAD.ShowCurves.dll
description=Сопряжение кривых
start=true
```