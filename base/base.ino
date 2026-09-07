#include <Servo.h>
#include <Arduino.h>

// =====================================================================
// ПИНЫ
// =====================================================================
#define PIN_WRIST_ROTATE  2   // кисть вокруг оси
#define PIN_SHOULDER_L    3   // левая часть руки (вверх-вниз)
#define PIN_SHOULDER_R    4   // правая часть руки (вверх-вниз)
#define PIN_ELBOW         5   // локоть (вверх-вниз)
#define PIN_WRIST_TILT    6   // кисть (вверх-вниз)
#define PIN_BASE          7   // основание руки (вокруг оси)
#define PIN_CLAW          8   // клешня (захват)

// =====================================================================
// ЛИМИТЫ УГЛОВ
// =====================================================================
#define BASE_MIN 0
#define BASE_MAX 360

#define SHOULDER_MIN 20
#define SHOULDER_MAX 150

#define ELBOW_MIN 10
#define ELBOW_MAX 170

#define WRIST_TILT_MIN 0
#define WRIST_TILT_MAX 180

#define WRIST_ROTATE_MIN 0
#define WRIST_ROTATE_MAX 180

#define CLAW_MIN 0
#define CLAW_MAX 90

// =====================================================================
// ПОЗИЦИИ
// =====================================================================
#define HOME_BASE         90
#define HOME_SHOULDER     90
#define HOME_ELBOW        90
#define HOME_WRIST_TILT   90
#define HOME_WRIST_ROTATE 90
#define HOME_CLAW         30

// "Транспортная" поза
#define TRANSPORT_BASE         90
#define TRANSPORT_SHOULDER     140
#define TRANSPORT_ELBOW        20
#define TRANSPORT_WRIST_TILT   90
#define TRANSPORT_WRIST_ROTATE 90
#define TRANSPORT_CLAW         CLAW_MAX

// =====================================================================
// НАСТРОЙКИ УПРАВЛЕНИЯ
// =====================================================================
#define JOY_DIVIDER 25       // больше = медленнее реакция на джойстик
#define CLAW_STEP 3          // шаг клешни за один пакет, пока зажата LB/RB

// Кисть (наклон и поворот) едет плавно независимо от частоты пакетов с пульта (10 Гц),
// а не скачками раз в 100 мс. WRIST_STEP_INTERVAL_MS — раз в сколько мс делаем шаг.
#define WRIST_STEP_INTERVAL_MS 2

// Наклон кисти управляется напрямую через микросекунды с дробным шагом —
// так плавнее, чем write() с целыми градусами. Больше значение = плавнее, но медленнее.
#define WRIST_TILT_PULSE_MIN 544
#define WRIST_TILT_PULSE_MAX 2400
#define WRIST_TILT_STEP_DEG 0.3f

// Правая серва плеча стоит по другую сторону оси от левой,
// поэтому должна крутиться в зеркальную сторону, иначе моторы
// тянут друг против друга (нагрев, клин).
#define SHOULDER_R_MIRRORED 1

// Расширенный диапазон импульсов для основания — даёт больше реального
// хода серве сверх паспортных 0-180° (работает не на всех сервах,
// если появится скрежет/клин на краях — верни 544, 2400).
#define BASE_PULSE_MIN 480
#define BASE_PULSE_MAX 2520

#define RX_BUFFER_SIZE 96

typedef struct
{
    int RX;
    int RY;
    int LX;
    int LY;
    int LB;
    int RB;
    int REC;
} ControlPacket;

static char rxBuffer[RX_BUFFER_SIZE];
static uint8_t rxIndex = 0;

static ControlPacket packet;
static bool packetReady = false;

Servo servo_base;
Servo servo_shoulder_l;
Servo servo_shoulder_r;
Servo servo_elbow;
Servo servo_wrist_tilt;
Servo servo_wrist_rotate;
Servo servo_claw;

int angle_base        = HOME_BASE;
int angle_shoulder     = HOME_SHOULDER;
int angle_elbow        = HOME_ELBOW;
float angle_wrist_tilt = HOME_WRIST_TILT; // дробный — для плавности
int angle_wrist_rotate = HOME_WRIST_ROTATE;
int angle_claw         = HOME_CLAW;

// Куда кисть должна прийти (обновляется пакетами), в отличие от angle_wrist_*,
// которые едут туда плавно шагом в 1° независимо от частоты пакетов.
int target_wrist_tilt   = HOME_WRIST_TILT;
int target_wrist_rotate = HOME_WRIST_ROTATE;
unsigned long lastWristStepTime = 0;


bool readIntField(const char* src, const char* key, int* value)
{
    const char* p = strstr(src, key);
    if (p == NULL) return false;

    p += strlen(key);
    *value = atoi(p);

    return true;
}

bool parsePacket(const char* line, ControlPacket* out)
{
    bool ok = true;

    ok &= readIntField(line, "RX=", &out->RX);
    ok &= readIntField(line, "RY=", &out->RY);
    ok &= readIntField(line, "LX=", &out->LX);
    ok &= readIntField(line, "LY=", &out->LY);
    ok &= readIntField(line, "LB=", &out->LB);
    ok &= readIntField(line, "RB=", &out->RB);
    ok &= readIntField(line, "REC=", &out->REC);

    return ok;
}

void serialParserUpdate(Stream* serial)
{
    while (serial->available() > 0)
    {
        char c = serial->read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            rxBuffer[rxIndex] = '\0';

            if (rxIndex > 0)
            {
                ControlPacket temp;

                if (parsePacket(rxBuffer, &temp))
                {
                    packet = temp;
                    packetReady = true;
                }
            }

            rxIndex = 0;
            continue;
        }

        if (rxIndex < RX_BUFFER_SIZE - 1)
        {
            rxBuffer[rxIndex++] = c;
        }
        else
        {
            rxIndex = 0;
        }
    }
}

bool serialPacketAvailable()
{
    return packetReady;
}

ControlPacket* serialGetPacket()
{
    packetReady = false;
    return &packet;
}

int clampAngle(int val, int lo, int hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void servo_inc(int& current, int delta, int lo, int hi)
{
    current += delta / JOY_DIVIDER;
    current = clampAngle(current, lo, hi);
}

// Сдвигает current на 1° в сторону target, если ещё не дошли
void stepToward(int& current, int target)
{
    if (current < target) current++;
    else if (current > target) current--;
}

// То же самое, но дробным шагом — для более плавного движения наклона кисти
void stepTowardFloat(float& current, int target, float stepDeg)
{
    float diff = (float)target - current;
    if (diff > stepDeg) current += stepDeg;
    else if (diff < -stepDeg) current -= stepDeg;
    else current = (float)target;
}

void applyShoulderCommand(int shoulderAngle)
{
    servo_shoulder_l.write(shoulderAngle);

#if SHOULDER_R_MIRRORED
    servo_shoulder_r.write(180 - shoulderAngle);
#else
    servo_shoulder_r.write(shoulderAngle);
#endif
}

void writeAllServos()
{
    servo_base.write(angle_base);
    applyShoulderCommand(angle_shoulder);
    servo_elbow.write(angle_elbow);
    servo_claw.write(angle_claw);
}

// Вызывается в каждой итерации loop() — двигает кисть плавно к последней
// цели, полученной от пульта, независимо от того, пришёл ли новый пакет
void updateWristSmoothing()
{
    unsigned long now = millis();
    if (now - lastWristStepTime < WRIST_STEP_INTERVAL_MS) return;
    lastWristStepTime = now;

    stepToward(angle_wrist_rotate, target_wrist_rotate);
    servo_wrist_rotate.write(angle_wrist_rotate);

    stepTowardFloat(angle_wrist_tilt, target_wrist_tilt, WRIST_TILT_STEP_DEG);
    int pulse = WRIST_TILT_PULSE_MIN +
        (int)((angle_wrist_tilt / 180.0f) * (WRIST_TILT_PULSE_MAX - WRIST_TILT_PULSE_MIN));
    servo_wrist_tilt.writeMicroseconds(pulse);
}

void goHome()
{
    angle_base        = HOME_BASE;
    angle_shoulder     = HOME_SHOULDER;
    angle_elbow        = HOME_ELBOW;
    angle_claw         = HOME_CLAW;

    target_wrist_tilt   = HOME_WRIST_TILT;
    target_wrist_rotate = HOME_WRIST_ROTATE;
}

void goTransport()
{
    angle_base        = TRANSPORT_BASE;
    angle_shoulder     = TRANSPORT_SHOULDER;
    angle_elbow        = TRANSPORT_ELBOW;
    angle_claw         = TRANSPORT_CLAW;

    target_wrist_tilt   = TRANSPORT_WRIST_TILT;
    target_wrist_rotate = TRANSPORT_WRIST_ROTATE;
}

void setup() {
    Serial.begin(115200);

    servo_base.attach(PIN_BASE, BASE_PULSE_MIN, BASE_PULSE_MAX);
    servo_shoulder_l.attach(PIN_SHOULDER_L);
    servo_shoulder_r.attach(PIN_SHOULDER_R);
    servo_elbow.attach(PIN_ELBOW);
    servo_wrist_tilt.attach(PIN_WRIST_TILT, WRIST_TILT_PULSE_MIN, WRIST_TILT_PULSE_MAX);
    servo_wrist_rotate.attach(PIN_WRIST_ROTATE);
    servo_claw.attach(PIN_CLAW);

    goHome();
    writeAllServos();
}

void loop() {

    serialParserUpdate(&Serial);
    updateWristSmoothing(); // едет плавно каждую итерацию, а не раз в 100 мс

    if (serialPacketAvailable())
    {
        ControlPacket* p = serialGetPacket();

        if (p->REC == 3)
        {
            goHome();                 // Up
        }
        else if (p->REC == 1)
        {
            // Down — стоп, ничего не меняем в этом кадре
        }
        else if (p->REC == 4)
        {
            angle_claw = CLAW_MAX;    // Left — клешня открыть полностью
        }
        else if (p->REC == 2)
        {
            angle_claw = CLAW_MIN;    // Right — клешня закрыть полностью
        }
        else if (p->REC == 5)
        {
            goTransport();            // Center — транспортная поза
        }
        else
        {
            // база: LX
            servo_inc(angle_base, -p->LX, BASE_MIN, BASE_MAX);

            // плечо + локоть вместе: LY 
            servo_inc(angle_shoulder, p->LY, SHOULDER_MIN, SHOULDER_MAX);
            servo_inc(angle_elbow, -p->LY, ELBOW_MIN, ELBOW_MAX);

            // поворот кисти: RX
            servo_inc(target_wrist_rotate, p->RX, WRIST_ROTATE_MIN, WRIST_ROTATE_MAX);

            // наклон кисти вверх-вниз: RY
            servo_inc(target_wrist_tilt, -p->RY, WRIST_TILT_MIN, WRIST_TILT_MAX);

            // клешня: держим LB (закрыть) / RB (открыть)
            if (p->RB) angle_claw = clampAngle(angle_claw + CLAW_STEP, CLAW_MIN, CLAW_MAX);
            if (p->LB) angle_claw = clampAngle(angle_claw - CLAW_STEP, CLAW_MIN, CLAW_MAX);
        }

        writeAllServos();
    }
}
