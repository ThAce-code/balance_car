# Repository Guidelines

## Project Structure & Module Organization

- `balance_car.ioc`: STM32CubeMX project file (regenerate HAL/FreeRTOS code from here).
- `Core/`: CubeMX-generated application and HAL init (`Core/Src`, `Core/Inc`). Keep edits inside `/* USER CODE BEGIN */` blocks.
- `APP/`: User/application modules (preferred place for new features). Examples: `APP/led.c`, `APP/scheduler.c`.
- `Drivers/`, `Middlewares/`: Vendor libraries (CMSIS, HAL, FreeRTOS). Avoid modifying.
- `doc/`: Design notes and plans (e.g., `doc/plan_v2.md`, `doc/imu_plan.md`, `doc/basic_infro.md`).
- `cmake/`: Toolchain and CubeMX CMake integration.

## Build, Test, and Development Commands

Prereq: install ARM GCC (`arm-none-eabi-gcc`) and ensure it’s on `PATH`.

- Configure: `cmake --preset Debug` (or `Release`) creates `build/<preset>`.
- Build: `cmake --build --preset Debug`.
- Clean: delete `build/Debug` (or run your IDE clean).

Flashing/debugging depends on your probe/tool (ST-Link/OpenOCD). Add local instructions in `doc/` if needed.

## Coding Style & Naming Conventions

- Language: C11 (see top-level `CMakeLists.txt`).
- Indentation: follow existing files (CubeMX/HAL uses 2 spaces in generated code; keep `APP/` consistent within each file).
- Naming: new modules in `APP/` as `module_name.c/.h`; FreeRTOS tasks as `task_<name>.c/.h` if you adopt that pattern.
- Do not reformat auto-generated code; keep changes minimal and focused.

## Testing Guidelines

No automated test framework is set up in this repo yet. If you add tests, keep them host-buildable where possible and document how to run them in `doc/`.

## Commit & Pull Request Guidelines

- Commits: use short, imperative messages; Conventional Commit prefixes are welcome (e.g., `chore:`, `feat:`, `fix:`).
- PRs: include a brief description, relevant board/hardware assumptions, and how to reproduce/verify (build command + runtime check).

## Notes for Contributors

- When changing pins/peripherals, update both `balance_car.ioc` and `doc/basic_infro.md`.
- For control/RTOS changes, keep `doc/plan_v2.md` aligned with implementation.
