#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "lcd.h"
#include "touch.h"
#include "string.h"
#include "math.h"

int color[12] = { WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY };
uint16_t rain;//����＾�� ��
uint16_t gas;//���⼾�� ��
uint16_t vib;//�������� �� 
uint16_t light;//�������� ��
int reserveFlag = 0;//������ �������� ǥ��
int reserveTime = 0;//�����带 ������ �ð�
int modTime = 0;//������ 1, 2, 3 ���� 10��, 20��, 30��
int timeCount = 0;//������ ������ 1�ʾ� ī��Ʈ

uint8_t temparature;//�½������� �µ���
uint8_t humidity;//�½������� ������

void RCC_Configure(void);
void GPIO_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void ADC_Configure(void);
void speaker_on();
int magnetic();
int magnetic2();
void open();
void close();
void TemHum_read();
void comtophone(void);
void on(void);

void RCC_Configure(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //port C enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // ADC enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); //port D enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //portA enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//USART1 clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);// USART2 clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //portB enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//Timer2 enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//Timer3 enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //Alternative functions I/O enable
}

void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PC0�� ����� ���� A0 ����(ä�� 10) 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //�Ƴ��α� ���� �о���� ���� ��带 GPIO_Mode_AIN�� ����
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PC1�� ���� ���� A0 ����(ä�� 11) 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //�Ƴ��α� ���� �о���� ���� ��带 GPIO_Mode_AIN�� ����
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//PC2�� ���� ���� A0 ����(ä�� 12) 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //�Ƴ��α� ���� �о���� ���� ��带 GPIO_Mode_AIN�� ����
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PC3�� �������� ����(ä�� 13) 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //�Ƴ��α� ���� �о���� ���� ��带 GPIO_Mode_AIN�� ����
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PD0�� ���׳�ƽ ���� ����(�����̴��ʿ� �޸�)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //input
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//���׳�ƽ ����2 ����(�������ʿ� �޸�)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //input
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PD5�� ���� ����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //���͵���̹�  ����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//PC8->IN1
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//PC9->IN2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PC10->IN3
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PC11->IN4
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    //���͵���̹� ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//�׽�Ʈ�� led
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//�׽�Ʈ�� led
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* UART1 pin setting */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //Alternate function output push-pull
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PA10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //input with pull down
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* UART2 pin setting */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //Alternate function output push-pull
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //PA3
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //input with pull down
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //�½�������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;

    // Enable the USART1 peripheral
    USART_Cmd(USART1, ENABLE);

    //Initialize the USART1
    USART_Init(USART1, &USART1_InitStructure);

    //Enable the USART1 RX interrupts
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;

    // Enable the USART2 peripheral
    USART_Cmd(USART2, ENABLE);

    //Initialize the USART2
    USART_Init(USART2, &USART2_InitStructure);

    //Enable the USART2 RX interrupts
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //MODE 
    ADC_InitStructure.ADC_ScanConvMode = ENABLE; //conversion scan enable
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // continuous conversion enable
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //externeral trigger
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //data alignment set
    ADC_InitStructure.ADC_NbrOfChannel = 4; //channel�� ��

    ADC_Init(ADC1, &ADC_InitStructure);
    // gpio port�� pin�� ���ε� channel �Ҵ� 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_7Cycles5); //����＾��
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_7Cycles5); //���⼾��
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_7Cycles5); //��������
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_7Cycles5); //��������

    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//interrupt enable
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1); //calibration (reduce error value ) reset
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1); //calibration start
    while (ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

void speaker_on() {//����Ŀ on
    GPIO_SetBits(GPIOD, GPIO_Pin_5);
}
void speaker_off() {//����Ŀ off
    GPIO_ResetBits(GPIOD, GPIO_Pin_5);
}
void fan_on() {//ȯǳ�� on
    GPIO_SetBits(GPIOC, GPIO_Pin_10);//IN3 high
    GPIO_ResetBits(GPIOC, GPIO_Pin_11);//IN4 low
    //fan_flag=1;
}

void fan_off() {//ȯǳ�� off
    GPIO_ResetBits(GPIOC, GPIO_Pin_10);//IN3 low
    //fan_flag=0;
}

int magnetic() {//���׳�ƽ ����. ������ ����
    if (~GPIOD->IDR & GPIO_Pin_0) {//���� ��������
        return 1;
    }
    else//���� ���������� 
        return 0;
}

int magnetic2() {//���׳�ƽ ����2
    if (~GPIOD->IDR & GPIO_Pin_12) {//�� �ݴ����� ��������
        return 1;
    }
    else {//�� �ݴ����� ����������
        return 0;
    }
}

void open() {//������ �Լ�
    while (magnetic2() == 0) {//�� �ݴ����� �����ִ� ����
      //���� ����
        GPIO_SetBits(GPIOC, GPIO_Pin_8);//IN1 high
        GPIO_ResetBits(GPIOC, GPIO_Pin_9);//IN2 low
    }
    //���� off
    GPIO_ResetBits(GPIOC, GPIO_Pin_8);//IN1 low
    GPIO_ResetBits(GPIOD, GPIO_Pin_5);//buzzer off-> ���� ������ speaker�� �۵��ϰ� ���� �����µ�, ���� ������ ���� ���� ���� ����� �������� �� �� ���ߵ��� ��
}

void close() {//���ݴ� �Լ� 
    while (magnetic() == 0) {//���� �����ִµ���
      //���� ����
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);//IN1 low
        GPIO_SetBits(GPIOC, GPIO_Pin_9);//IN2 high
    }
    //���� off
    GPIO_SetBits(GPIOC, GPIO_Pin_8);//IN1 high
}


void on() {//test�� led on
    GPIO_SetBits(GPIOD, GPIO_Pin_2);
}

void ADC1_2_IRQHandler(void) {
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) { //ADC_IT_EOC(END OF CONVERTION)�� SET�� �����̸� value���� ����

        rain = ADC_GetConversionValue(ADC1);
        gas = ADC_GetConversionValue(ADC1);
        vib = ADC_GetConversionValue(ADC1);
        light = ADC_GetConversionValue(ADC1);

        if (rain > 1000 && rain <= 2300) {//�� ������
            if (magnetic() == 0) {//�� ���ȴ��� Ȯ��(���׳�ƽ ����)
                close();//�� �������� �ݱ�
            }
        }
        if (gas <= 1200 && gas >= 900) {//���� �����ϸ�
            if (temparature > 10) {//60�� �̻��̸� ������ ����Ŀ ����//�ÿ��� ���� 10���� ��������
                speaker_on();//�溸�� �︲
                if (magnetic() == 1)//�� ����������
                    open();//�� ����
            }
        }
        if (vib <= 50) {//���� �����ϸ�
            speaker_on();//�溸�� �︲
            comtophone();//��������� �˸� ����
        }
    }
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
}

void sendDataUART1(uint16_t data) {
    USART_SendData(USART1, data);
}

void sendDataUART2(uint16_t data) {
    USART_SendData(USART2, data);
}

char warningWord[1] = "!";//���
void comtophone(void) {//��� ���� ��� ���� 
       //bluetooth ���� �����Ͽ� �ڵ���ȭ�鿡 ���  
    for (int i = 0; i < 1; i++) {
        sendDataUART2(warningWord[i]);
    }
}


void USART1_IRQHandler(void) {
    uint16_t word;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART1);
        sendDataUART2(word);
        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}


void USART2_IRQHandler(void) {
    uint16_t word;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        word = USART_ReceiveData(USART2); //�ڵ������� �Է��� ���ڰ� bluetooth ����� ���� ����� ����

        if (word == 'o') {//o�Է½� �� ����
            open();//�� ����
        }
        else if (word == 'c') {//c�Է½� �� ����
            close();//�� ����
        }
        if (word == '1') {//������1
          //Ÿ�̸� 10�ʸ��� ȯǳ��, ������
            reserveFlag = 1;//������ ������
            reserveTime = 60;//������ ���� �ð� 60��
            modTime = 10;//10�ʸ��� ����
        }
        if (word == '2') {//������2
            reserveFlag = 1;//������ ������
            reserveTime = 60;//������ ���۽ð� 60��
            modTime = 20;//20�ʸ��� ����
        }
        if (word == '3') {//������3
            reserveFlag = 1;//������ ������
            reserveTime = 60;//������ ���۽ð� 60��
            modTime = 30;//30�ʸ��� ����
        }
        if (word == 'b') {//buzzer off
            speaker_off();//����Ŀ ����
        }

    }
}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        if (reserveFlag == 1) { //�����尡 �����Ǹ� 
            timeCount++; //������ �ð� ī��Ʈ
        }
    }
}

void TIM_Configure(void) {

    TIM_TimeBaseInitTypeDef TIM3_TimeBaseStructure; //1�� timer->������ ī��Ʈ��
    TIM_TimeBaseInitTypeDef TIM2_TimBaseStructure; //�½������� �б��

    TIM3_TimeBaseStructure.TIM_Period = 10000;
    TIM3_TimeBaseStructure.TIM_Prescaler = 7200;
    //1/72MHz * 7200 * 10000 = 1s
    TIM3_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM3_TimeBaseStructure);
    TIM_Cmd(TIM3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //interrupt enable

    TIM2_TimBaseStructure.TIM_Period = 84000000 - 1;
    TIM2_TimBaseStructure.TIM_Prescaler = 84;
    TIM2_TimBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM2_TimBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM2_TimBaseStructure);
    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // UART1
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // UART2
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_EnableIRQ(ADC1_2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void TemHum_read() {//�½������� �д� �Լ�

    int i, j, temp;
    uint8_t data[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    GPIO_InitTypeDef GPIO_InitStructure;

    // �½��� ���� �� �ʱ�ȭ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // DHT ������ �ʱ� 18ms ���� �Է��� �����ؾ� ��
    // 18ms�� ��ٸ�
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    TIM2->CNT = 0;
    while ((TIM2->CNT) <= 18000);

    // ���� 20-40us���� HIGH �Է��� ��� ��
  // 40us�� ��ٸ���, �ʱ�ȭ ��
    GPIO_SetBits(GPIOB, GPIO_Pin_6);
    TIM2->CNT = 0;
    while ((TIM2->CNT) <= 40);

    // Data�� �޾ƿ��� ���� Input ���� �����Ѵ�
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // DHT11�� ACK�� Timeout�� count�Ѵ�
    // DHT11�� ��ȣ�� ��� 80us���� �����Ǿ�� ��
    //timeout�߻��� return
    TIM2->CNT = 0;
    while (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)) {
        if (TIM2->CNT > 100)
            return;
    }

    TIM2->CNT = 0;
    while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)) {
        if (TIM2->CNT > 100)
            return;
    }

    // 40��Ʈ(8*5)�� �о�´�
    for (j = 0; j < 5; ++j) {
        for (i = 0; i < 8; ++i) {

            // 50us���� LOW�� �����Ѵ�
            while (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6));

            // Timer Count ����
            TIM_SetCounter(TIM2, 0);

            // 26-28us���� HiGH ��ȣ�� ������ 0����, 70us���� HIGH ��ȣ�� ������ 1�� ����Ѵ�
            while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6));


            // ���� �������� ���� �ð��� �޾ƿ´�
            temp = TIM_GetCounter(TIM2);

            //shift 0
            data[j] = data[j] << 1;

            // 40us���� ���� ũ��(26-28us�� �ƴϸ�)  �ش� �迭�� ���� 1�� �ȴ�
            if (temp > 40)
                data[j] = data[j] + 1;
        }
    }

    // �µ����� data[2]��, �������� data[0]�� ����ȴ�
    temparature = data[2];
    humidity = data[0];

    return;
}

int main() {
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    USART1_Init();
    USART2_Init();
    NVIC_Configure();
    TIM_Configure();
    LCD_Init();

    while (1) {

        if (reserveFlag == 1) {//�����尡 �����Ǿ��� ��
            if (timeCount % modTime == 0 && magnetic() == 1) { //����ð��� �Ǿ��� ���� ����������
                open(); //���� ����
            }
            else if (timeCount % modTime == 0 && magnetic() == 0) { //����ð��� �Ǿ��� ���� ����������
                close(); //���� �ݴ´�.
            }
            if (timeCount > reserveTime) {//����ð��� �Ѿ��
                reserveFlag = 0; //flag reset
                timeCount = 0; //�ð� �ʱ�ȭ
            }
        }

        printf("rain= %d, gas=%d, vib=%d, light=%d\n", rain, gas, vib, light);

        TemHum_read();

        //printf("TEMPRATURE %d - HUMIDITY %d\n", temparature, humidity);
        LCD_ShowString(85, 20, "Temperature", BLACK, WHITE);
        LCD_ShowNum(85, 40, temparature, 10, BLACK, WHITE);

        if (humidity < 40) {//���� ���, 40% �����̸� ����
            LCD_ShowString(85, 60, "Humidity: Dry   ", BLACK, WHITE);
        }
        else if (humidity < 60) {//40%~60%�̸� ����
            LCD_ShowString(85, 60, "Humidity: Proper", BLACK, WHITE);
        }
        else {//60% �̻��̸� ����
            LCD_ShowString(85, 60, "Humidity: Wet   ", BLACK, WHITE);
        }

        //���������� ����/��ο����� ���
        if (light > 3000)
            LCD_ShowString(85, 80, "Light: Bright", BLACK, WHITE);
        else
            LCD_ShowString(85, 80, "Light: Dark  ", BLACK, WHITE);
    }
}
