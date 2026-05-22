# PRD — Sistema de Control de Acceso y Seguridad con Monitoreo Ambiental
**Curso:** Arquitectura Computacional  
**Facultad:** Ingeniería Electrónica y Telecomunicaciones  
**Universidad:** Universidad del Cauca  

---

## 1. Descripción General

El proyecto consiste en diseñar e implementar un sistema embebido de control de acceso con cerradura inteligente para una empresa. El sistema gestiona el ingreso de personas mediante clave numérica, administra perfiles y roles de usuario, controla intentos fallidos, activa alarmas ante eventos de seguridad, y monitorea las condiciones ambientales (temperatura, luz, sonido, campo magnético) para garantizar el confort térmico y la seguridad del entorno.

El sistema está implementado sobre el microcontrolador **ATmega2560** y se organiza como una **Máquina de Estados Finitos (FSM)** con 6 estados principales.

---

## 2. Objetivos

- Controlar el acceso físico a la empresa mediante autenticación por clave numérica.
- Gestionar perfiles de usuario con diferentes niveles de acceso físico según rol.
- Detectar y responder a eventos de intrusión y accesos no autorizados.
- Monitorear condiciones ambientales y actuar para mantener el confort.
- Almacenar usuarios, claves y horarios en EEPROM de forma persistente.
- Forzar la rotación de claves tras 4 usos, impidiendo la reutilización de claves anteriores.

---

## 3. Hardware

| Componente | Función en el sistema |
|---|---|
| ATmega2560 | Microcontrolador principal |
| LCD Keypad Shield | Visualización de estado y entrada de clave numérica (reemplaza RFID) |
| Servo Motor | Simula el mecanismo de cerradura (abre/cierra la puerta) |
| Buzzer | Alarma sonora ante intentos excesivos o intrusión |
| LED | Indicador visual de estado (rojo parpadeante según modo) |
| Sound Sensor KY-037 | Detección de sonido/intrusión por micrófono |
| Analog Hall Sensor KY-035 | Monitoreo del estado de la puerta (abierta/cerrada) |
| Analog Temperature Sensor KY-013 | Lectura de temperatura ambiente |
| Photoresistor Module KY-018 | Lectura de nivel de luz ambiental |
| Potenciómetro | Ajuste de contraste del LCD |
| EEPROM (interna ATmega2560) | Almacenamiento persistente de usuarios, claves, roles y horarios |

> **Nota:** El módulo RFID presente en el diagrama FSM fue reemplazado por el teclado del LCD Keypad Shield como método de autenticación.

---

## 4. Software y Librerías

- **Lenguaje:** C/C++ (Arduino framework sobre ATmega2560)
- **Average Library:** Cálculo de promedios sobre lecturas de sensores analógicos (temperatura, luz) para suavizar valores y evitar lecturas ruidosas.
- **EEPROM.h:** Lectura y escritura persistente de usuarios, contraseñas y horarios.
- **Servo.h:** Control del servo motor para el mecanismo de cerradura.
- **LiquidCrystal.h:** Control del LCD Keypad Shield.

---

## 5. Máquina de Estados Finitos (FSM)

El sistema opera con **6 estados** y las siguientes transiciones:

### 5.1 Estados

| # | Estado | Descripción |
|---|---|---|
| 1 | **INICIO** | Estado inicial. Espera ingreso de clave por teclado. Muestra prompt en LCD. |
| 2 | **CONFIG** | Clave correcta. Valida rol, horario y perfil. Activa servo (cerradura abierta por tiempo programado). Retorna automáticamente a INICIO. |
| 3 | **BLOQUEO** | Se activa tras 3 intentos fallidos. Sistema bloqueado temporalmente (5 s). LED rojo ON=300 ms / OFF=700 ms. |
| 4 | **MONITOR INTRUSOS** | Monitorea micrófono (KY-037) y sensor Hall (KY-035). Detecta apertura de puerta sin autorización o sonido sospechoso. |
| 5 | **MONITOR AMBIENTAL** | Monitorea temperatura (KY-013) y luz (KY-018). Actúa si temperatura < 20°C o luz < 100. |
| 6 | **ALARMA** | Se activa ante intrusión o 3 alarmas consecutivas en menos de 12 s. Buzzer ON. LED rojo ON=100 ms / OFF=500 ms. Duración configurable. |

### 5.2 Transiciones principales

| Desde | Condición | Hacia |
|---|---|---|
| INICIO | Clave correcta + rol/horario válido | CONFIG |
| INICIO | Clave incorrecta (< 3 intentos) | INICIO (incrementa contador) |
| INICIO | 3 intentos fallidos | BLOQUEO |
| INICIO | Letra `#` / `*` | Acciones de menú (cambio de clave, gestión) |
| CONFIG | Tiempo de desbloqueo expirado (2 s) | INICIO |
| BLOQUEO | Tiempo de bloqueo expirado (5 s) | INICIO |
| MONITOR INTRUSOS | Intrusión detectada (puerta/micrófono) | ALARMA |
| MONITOR INTRUSOS | 3 veces sin evento | INICIO |
| MONITOR AMBIENTAL | Temp < 20°C o Luz < 100 | Acción de confort (actuador) |
| MONITOR AMBIENTAL | Tiempo expirado (4 s) | INICIO |
| ALARMA | 3 alarmas consecutivas < 12 s | Sistema bloqueado (BLOQUEO extendido) |
| ALARMA | Tiempo de alarma expirado (2 s) | INICIO |

### 5.3 Temporización de LEDs y Buzzer

| Estado | LED Rojo | Buzzer |
|---|---|---|
| BLOQUEO | ON=300 ms, OFF=700 ms | OFF |
| ALARMA | ON=100 ms, OFF=500 ms | ON (continuo) |

---

## 6. Gestión de Usuarios y Roles

### 6.1 Roles y niveles de acceso físico

| Rol | Nivel | Acceso físico |
|---|---|---|
| Seguridad | 1 | Acceso a zonas de vigilancia y entrada principal |
| Operario | 2 | Acceso a zonas de producción/trabajo |
| Coordinador | 3 | Acceso a zonas operativas + salas de reunión |
| Gerente | 4 | Acceso total a todas las áreas |

> Los diferentes roles habilitan el servo (cerradura) para diferentes zonas físicas del sistema.

### 6.2 Políticas de contraseña

- Clave numérica de **mínimo 4 dígitos**.
- Cada clave expira tras **4 usos**; el sistema obliga al usuario a cambiarla.
- La nueva clave **no puede ser igual** a ninguna de las claves anteriores (historial almacenado en EEPROM).
- El cambio de clave se gestiona desde el menú accesible con las teclas `#` / `*` del LCD Keypad.

### 6.3 Franjas horarias

- Las franjas horarias de acceso permitido por rol están **definidas en el código (hardcoded)**.
- Si un usuario intenta acceder fuera de su franja horaria, el acceso es denegado aunque la clave sea correcta.

### 6.4 Almacenamiento en EEPROM

- Usuarios, claves (con historial), roles y franjas horarias se guardan en la EEPROM interna del ATmega2560.
- La cantidad exacta de usuarios almacenables **está por definir** según el mapa de memoria EEPROM disponible (4 KB en ATmega2560).

---

## 7. Monitoreo Ambiental

El estado **MONITOR AMBIENTAL** usa la **Average Library** para promediar lecturas de los sensores y evitar falsas activaciones por ruido eléctrico.

| Variable | Sensor | Umbral de acción |
|---|---|---|
| Temperatura | KY-013 | < 20°C → activar calefacción/ventilación |
| Luz | KY-018 | < 100 (unidades ADC) → activar iluminación |
| Sonido | KY-037 | Umbral configurable → detección de intrusión |
| Campo magnético (puerta) | KY-035 | Cambio de estado → puerta abierta sin autorización |

> El número de personas presentes puede influir en los umbrales de confort térmico (a definir en implementación).

---

## 8. Visualización

La información del sistema se muestra de forma simultánea en **dos canales**:

### 8.1 LCD Keypad Shield (16x2)

| Línea | Contenido |
|---|---|
| Línea 1 | Estado del sistema: `IDLE`, `OPEN`, `ERR`, `ALR`, `BLK` |
| Línea 2 | Dígitos ingresados (enmascarados: `****`), conteo regresivo, mensajes de error, datos ambientales |

El contraste del LCD se ajusta mediante el **potenciómetro**.

### 8.2 Monitor Serial (Serial.print)

- Log completo de eventos: intentos de acceso, cambios de estado, lecturas de sensores, alertas.
- Útil para depuración y verificación durante el desarrollo.

---

## 9. Requisitos Funcionales

| ID | Requisito |
|---|---|
| RF-01 | El sistema acepta claves numéricas de mínimo 4 dígitos vía LCD Keypad. |
| RF-02 | El sistema valida la clave contra el valor almacenado en EEPROM. |
| RF-03 | Permite máximo 3 intentos fallidos antes de activar BLOQUEO. |
| RF-04 | Activa alarma sonora (buzzer) y visual (LED rojo rápido) ante intrusión o bloqueo. |
| RF-05 | El servo motor desbloquea la cerradura por un tiempo programado (2 s) y retorna al estado INICIO. |
| RF-06 | El sistema retorna automáticamente al estado INICIO tras cualquier acción completada. |
| RF-07 | Las franjas horarias de acceso están definidas por rol en el código. |
| RF-08 | El sensor Hall (KY-035) monitorea el estado físico de la puerta. |
| RF-09 | El sistema gestiona altas de usuarios y asignación de roles almacenados en EEPROM. |
| RF-10 | Las claves se renuevan obligatoriamente cada 4 usos; no se permite reutilización. |
| RF-11 | El sensor de sonido (KY-037) detecta eventos de intrusión por ruido. |
| RF-12 | Los sensores KY-013 y KY-018 monitorean temperatura y luz con promediado (Average Library). |
| RF-13 | El LCD muestra estado, dígitos enmascarados, conteo regresivo e información ambiental. |
| RF-14 | El monitor serial registra todos los eventos del sistema. |
| RF-15 | 3 alarmas consecutivas en menos de 12 s activan bloqueo extendido del sistema. |
| RF-16 | Los roles definen diferentes niveles de acceso físico a zonas de la empresa. |

---

## 10. Requisitos No Funcionales

| ID | Requisito |
|---|---|
| RNF-01 | El sistema debe responder a la entrada del teclado en menos de 200 ms. |
| RNF-02 | El código debe estar modularizado por estados del FSM. |
| RNF-03 | El código debe estar documentado con comentarios en cada función. |
| RNF-04 | La EEPROM no debe escribirse innecesariamente para preservar su vida útil. |
| RNF-05 | El sistema debe ser robusto ante lecturas ruidosas de sensores analógicos (uso de Average Library). |

---

## 11. Entregables

| Entregable | Descripción |
|---|---|
| Prototipo físico funcional | Hardware ensamblado y operativo sobre ATmega2560 |
| Código fuente documentado | Código C/C++ modular con comentarios, entregado según formato del curso |
| Informe escrito | Documento según formato provisto por el docente (pendiente de adjuntar) |

---

## 12. Aspectos Pendientes de Definición

- Número máximo de usuarios almacenables en EEPROM (depende del mapa de memoria).
- Franjas horarias específicas por rol (valores exactos a hardcodear).
- Umbral exacto de sonido para detección de intrusión (KY-037).
- Número de personas que modifica los umbrales de confort ambiental.
- Formato de informe (pendiente de entrega por parte del docente).