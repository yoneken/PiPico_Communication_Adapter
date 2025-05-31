# Raspberry Pi Pico Adapter for xArm6 and SSG-48

This project provides an adapter solution using the **Raspberry Pi Pico** to bridge communication between an **xArm6 robot arm (ModBus signals)** and an **SSG-48 end-effector (CAN signals)**.

We leverage existing open-source projects to facilitate this integration:

* **ModBus communication** utilizes the [Modbus-PI-Pico-FreeRTOS](https://github.com/alejoseb/Modbus-PI-Pico-FreeRTOS) project.
* **CAN communication** is handled by the [can2040](https://github.com/KevinOConnor/can2040) project.

---

## License Information

Due to the licenses of the upstream projects, this project is automatically licensed under **GPL-3.0**.

* `Modbus-PI-Pico-FreeRTOS` is licensed under **LGPL-2.1**.
* `can2040` is licensed under **GPL-3.0**.
