#define vel_motor_esq 10
#define vel_motor_dir 11
#define e1 8
#define e2 9
#define d1 12
#define d2 7

int trigger_frente = A4; // Front sensor trigger
int echo_frente = A5;    // Front sensor echo
int trigger_esq = A2;    // Left sensor trigger
int echo_esq = A3;       // Left sensor echo
int trigger_dir = A0;    // Right sensor trigger
int echo_dir = A1;       // Right sensor echo

void setup() {
    pinMode(trigger_frente, OUTPUT);
    pinMode(echo_frente, INPUT);
    pinMode(trigger_esq, OUTPUT);
    pinMode(echo_esq, INPUT);
    pinMode(trigger_dir, OUTPUT);
    pinMode(echo_dir, INPUT);
    pinMode(vel_motor_esq, OUTPUT);
    pinMode(vel_motor_dir, OUTPUT);
    pinMode(e1, OUTPUT);
    pinMode(e2, OUTPUT);
    pinMode(d1, OUTPUT);
    pinMode(d2, OUTPUT);
    delay(5000); // Wait for initialization
}

void loop() {
    unsigned long duracao_frente, duracao_esq, duracao_dir;
    int direita, esquerda, frente;

    // Front sensor
    digitalWrite(trigger_frente, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_frente, HIGH);
    delayMicroseconds(10); // Standard pulse width
    digitalWrite(trigger_frente, LOW);
    duracao_frente = pulseIn(echo_frente, HIGH);
    frente = duracao_frente / 29 / 2;

    // Left sensor
    digitalWrite(trigger_esq, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_esq, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_esq, LOW);
    duracao_esq = pulseIn(echo_esq, HIGH);
    esquerda = duracao_esq / 29 / 2;

    // Right sensor
    digitalWrite(trigger_dir, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_dir, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_dir, LOW);
    duracao_dir = pulseIn(echo_dir, HIGH);
    direita = duracao_dir / 29 / 2;

    // Stop motors by default
    analogWrite(vel_motor_esq, 0);
    analogWrite(vel_motor_dir, 0);
    digitalWrite(e1, LOW);
    digitalWrite(e2, LOW);
    digitalWrite(d1, LOW);
    digitalWrite(d2, LOW);

    if (frente > 8) {
        if (direita > 7 && direita < 13) { // Follow right wall
            analogWrite(vel_motor_esq, 120);
            analogWrite(vel_motor_dir, 150);
            digitalWrite(e1, HIGH);
            digitalWrite(e2, LOW);
            digitalWrite(d1, LOW);
            digitalWrite(d2, HIGH);
        } else if (direita >= 13) { // Too far from right, turn right
            analogWrite(vel_motor_esq, 255);
            analogWrite(vel_motor_dir, 60);
            digitalWrite(e1, HIGH);
            digitalWrite(e2, LOW);
            digitalWrite(d1, LOW);
            digitalWrite(d2, HIGH);
        } else if (direita <= 7) { // Too close to right, turn left
            analogWrite(vel_motor_esq, 60);
            analogWrite(vel_motor_dir, 255);
            digitalWrite(e1, HIGH);
            digitalWrite(e2, LOW);
            digitalWrite(d1, LOW);
            digitalWrite(d2, HIGH);
        }
    } else { // Front obstacle detected
        if (esquerda <= 20 && direita > 20) dir();        // Turn right
        else if (esquerda > 20 && direita > 20) dir();   // Turn right (both open)
        else if (direita <= 20 && esquerda > 20) esq();  // Turn left
        else if (direita <= 20 && esquerda <= 20) voltar(); // Reverse
    }
}

void esq() { // Turn left
    analogWrite(vel_motor_esq, 120);
    analogWrite(vel_motor_dir, 120);
    digitalWrite(e1, LOW);
    digitalWrite(e2, HIGH);
    digitalWrite(d1, LOW);
    digitalWrite(d2, HIGH);
    delay(700);
}

void dir() { // Turn right
    analogWrite(vel_motor_esq, 120);
    analogWrite(vel_motor_dir, 120);
    digitalWrite(e1, HIGH);
    digitalWrite(e2, LOW);
    digitalWrite(d1, HIGH);
    digitalWrite(d2, LOW);
    delay(800);
}

void voltar() { // Reverse
    analogWrite(vel_motor_esq, 120);
    analogWrite(vel_motor_dir, 120);
    digitalWrite(e1, HIGH);
    digitalWrite(e2, LOW);
    digitalWrite(d1, HIGH);
    digitalWrite(d2, LOW);
    delay(1200);
}
