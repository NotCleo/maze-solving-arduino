#define vel_motor_esq 10
#define vel_motor_dir 11
#define e1 8
#define e2 9
#define d1 12
#define d2 7

int trigger_frente = A4;
int echo_frente = A5;
int trigger_esq = A2;
int echo_esq = A3;
int trigger_dir = A0;
int echo_dir = A1;

void setup() {
    Serial.begin(9600);
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
    delay(5000);
    Serial.println("Setup complete, starting...");
}

void loop() {
    Serial.println("--- New Loop ---");
    unsigned long duracao_frente, duracao_esq, duracao_dir;
    int direita, esquerda, frente;

    digitalWrite(trigger_frente, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_frente, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_frente, LOW);
    duracao_frente = pulseIn(echo_frente, HIGH, 30000);
    frente = duracao_frente / 29 / 2;
    Serial.print("Frente: ");
    Serial.print(frente);
    Serial.println(" cm");

    digitalWrite(trigger_esq, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_esq, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_esq, LOW);
    duracao_esq = pulseIn(echo_esq, HIGH, 30000);
    esquerda = duracao_esq / 29 / 2;
    Serial.print("Esquerda: ");
    Serial.print(esquerda);
    Serial.println(" cm");

    digitalWrite(trigger_dir, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_dir, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_dir, LOW);
    duracao_dir = pulseIn(echo_dir, HIGH, 30000);
    direita = duracao_dir / 29 / 2;
    Serial.print("Direita: ");
    Serial.print(direita);
    Serial.println(" cm");

    analogWrite(vel_motor_esq, 0);
    analogWrite(vel_motor_dir, 0);
    digitalWrite(e1, LOW);
    digitalWrite(e2, LOW);
    digitalWrite(d1, LOW);
    digitalWrite(d2, LOW);

    if (frente > 8) {
        if (direita > 7 && direita < 13) {
            Serial.println("Following wall: Forward");
            analogWrite(vel_motor_esq, 120);
            analogWrite(vel_motor_dir, 150);
            digitalWrite(e1, HIGH);
            digitalWrite(e2, LOW);
            digitalWrite(d1, LOW);
            digitalWrite(d2, HIGH);
        } else if (direita >= 13) {
            Serial.println("Too far from right: Turn right");
            analogWrite(vel_motor_esq, 255);
            analogWrite(vel_motor_dir, 60);
            digitalWrite(e1, HIGH);
            digitalWrite(e2, LOW);
            digitalWrite(d1, LOW);
            digitalWrite(d2, HIGH);
        } else if (direita <= 7) {
            Serial.println("Too close to right: Turn left");
            analogWrite(vel_motor_esq, 60);
            analogWrite(vel_motor_dir, 255);
            digitalWrite(e1, HIGH);
            digitalWrite(e2, LOW);
            digitalWrite(d1, LOW);
            digitalWrite(d2, HIGH);
        }
    } else {
        if (esquerda <= 20 && direita > 20) {
            Serial.println("Left blocked: Turn right");
            dir();
        } else if (esquerda > 20 && direita > 20) {
            Serial.println("Both open: Turn right");
            dir();
        } else if (direita <= 20 && esquerda > 20) {
            Serial.println("Right blocked: Turn left");
            esq();
        } else if (direita <= 20 && esquerda <= 20) {
            Serial.println("All blocked: Reverse");
            voltar();
        }
    }
}

void esq() {
    Serial.println("Turning left");
    analogWrite(vel_motor_esq, 120);
    analogWrite(vel_motor_dir, 120);
    digitalWrite(e1, LOW);
    digitalWrite(e2, HIGH);
    digitalWrite(d1, LOW);
    digitalWrite(d2, HIGH);
    delay(700);
}

void dir() {
    Serial.println("Turning right");
    analogWrite(vel_motor_esq, 120);
    analogWrite(vel_motor_dir, 120);
    digitalWrite(e1, HIGH);
    digitalWrite(e2, LOW);
    digitalWrite(d1, HIGH);
    digitalWrite(d2, LOW);
    delay(800);
}

void voltar() {
    Serial.println("Reversing");
    analogWrite(vel_motor_esq, 120);
    analogWrite(vel_motor_dir, 120);
    digitalWrite(e1, HIGH);
    digitalWrite(e2, LOW);
    digitalWrite(d1, HIGH);
    digitalWrite(d2, LOW);
    delay(1200);
}
