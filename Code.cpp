// --------------------- Definicoes de Pinos/Variaveis ---------------------

// Define os pinos de IO
#define pwm 3
#define feedback A5
#define pot A4

int val = 0; // Variavel que armazena temporariamente valores lidos
float tensao = 0; // Tensao no pino feedback
float erro = 0; // Diferença do setpoint e a tensao medida
float setpoint = 0; // Valor variavel a partir do potenciometro

// Ganhos P, I & D
float kp = 300;
float ki = 20;
float kd = 300;

// Integral, Proporcional e Derivativo
float I;
float P;
float D;

float Ianterior = 0; // Acumula o valor do componente integral
float duty = 0; // Duty cycle para o controle PWM
float T = 0.01; // Periodo do sistema atualiza a cada 0.01 segundo (10 ms)
float erro_anterior = 0; // Usado para calcular a derivada

// ----------------------- Definicoes de IO, comunicacao e PWM -----------------------

void setup() 
{
  pinMode(pwm, OUTPUT); // Define o pino pwm como saída
  pinMode(feedback, INPUT); // Define o pino feedback como entrada
  pinMode(pot, INPUT); // Define o pino pot como entrada
  Serial.begin(9600); // Inicia a comunicação serial para debug
  analogWrite(pwm, 120); // Inicia a saída PWM com duty cycle intermediário (120/255)
}

// ----------------------------------------------------------------- Loop Principal -----------------------------------------------------------------

void loop() 
{
    
    tensao = ((analogRead(feedback) * 5.0)/1024.0) * 2.0; // Mede o feedback do sistema e converte o valor para uma escala de 5V multiplicada por 2
    setpoint = ((analogRead(pot)*5.0)/1024.0)*2.0;  // Le o setpoint definido pelo potenciometro
    erro = setpoint - tensao; // Cálculo do erro

    P = erro * kp; // Responde ao erro atual
    I = ki*(erro)*T + Ianterior;  // Soma dos erros ao longo do tempo para corrigir desvios acumulados
    D = kd * ((erro - erro_anterior)/T); // Considera a taxa de variação do erro, respondendo rapidamente a mudancas
    Ianterior = I; // Pega o valor atual de I e atribui a variavel

    // P = Proporcional | I = Integral | D = Derivativo
    duty = P; 
    //duty = P + I;
    //duty = P + I + D;

    // Limita duty de 0 a 200 para evitar a saturação do PWM
    if(duty > 200)
    {
      duty = 200;
    }
    if(duty < 0)
    {
      duty = 0;
    }
    erro_anterior = erro;

    // Envia o valor calculado para o pino pwm
    analogWrite(pwm, duty);
    delay(10);
}
