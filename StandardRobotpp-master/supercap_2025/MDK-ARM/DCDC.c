
#include <DCDC.h>

uint16_t adc[4];

CurrentLoop currentloop;
DCDCStage dcdcstage;
DCDCMode dcdcmode;

#define Ki 399.9681f
#define Kp 0.024248f
#define MaxIntegral 0.9f

#define MAX_OUTPUT_CURRENT 16.0f

#define CAP_MAX_VOLTAGE 18.5f

FDCAN_TxHeaderTypeDef TxHeader1;

uint8_t Automode_Stage = 0;
int16_t Power_Buffer[2] = {60, 60};

uint8_t cap_overVoltage_state = 0;
uint8_t cap_lowVoltage_state = 0;
uint8_t Bus_Charge_state = 0;

float charge_energy;

float Power_Buffer_nocalibration;

void CAN_Init()
{
    TxHeader1.Identifier = 0x300;          // CAN ID
    TxHeader1.IdType = FDCAN_STANDARD_ID;  // ��׼ID
    TxHeader1.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader1.DataLength = FDCAN_DLC_BYTES_8;  // ���ͳ��ȣ�8byte
    TxHeader1.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    TxHeader1.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader1.FDFormat = FDCAN_FD_CAN;  // CANFD
    TxHeader1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader1.MessageMarker = 0;

    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x1AA;  //����ID
    sFilterConfig.FilterID2 = 0x7FF;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan2);
}

void Cap_Init()
{
    //Ӳ���ϵ�ȴ�
    HAL_Delay(100);
    //pid��ʼ��

    //���ʻ�·����

    PID_Init(&dcdcmode.autoMode.powerPID, 15.0f, 0.0015f, 0, 100.0f, 120.0f, 120.0f);  //��������pid
    PID_Init(&dcdcmode.autoMode.Bus_currentPID, 1.5, 0.18, 0, 35.0f, 40.0f, 40.0f);    //���ߵ���PID
    PID_Init(&dcdcmode.autoMode.ChargePID, 4, 0, 0.000, 2, 40.0f, 40.0f);  //�������� ˫���������
    PID_SetDeadzone(&dcdcmode.autoMode.powerPID, 0);
    PID_SetDeadzone(&dcdcmode.autoMode.Bus_currentPID, 0);

    dcdcmode.autoMode.powerLimit = 30;
    //ͨ��1k
    HAL_TIM_Base_Start_IT(&htim6);

    //hrtim��ʼ��
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);  //ͨ����
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);               //�����Ӷ�ʱ��B
    __HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_TIM_IT_UPD);
    //adc��ʼ��
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
    HAL_ADCEx_InjectedStart(&hadc1);
    //��������10k 50%ռ�ձ�
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 50);
    //���ʲ������㼰���ʱջ� 5k
    HAL_TIM_Base_Start_IT(&htim1);

    CAN_Init();
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef * hadc)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
    adc[0] = hadc->Instance->JDR1;
    adc[1] = hadc->Instance->JDR2;
    adc[2] = hadc->Instance->JDR3;
    adc[3] = hadc->Instance->JDR4;

    currentloop.current = (adc[0] - 2035) * 0.01831f;

    if (dcdcmode.mode != dcdcmode.lastMode) {
        if (dcdcmode.lastMode == TURNOFF) {
            currentloop.chargeLoop.u[1] = currentloop.targetCurrent - currentloop.current;
            currentloop.chargeLoop.u[0] = currentloop.chargeLoop.u[1];
            currentloop.chargeLoop.y[1] = dcdcstage.CapVoltage / dcdcstage.BUSVoltage *
                                          (1.054f - dcdcstage.CapVoltage * 0.025f);
            currentloop.chargeLoop.y[0] = currentloop.chargeLoop.y[1];
            currentloop.output = currentloop.chargeLoop.y[0];
            currentloop.OutPutPi.integral = dcdcstage.CapVoltage / dcdcstage.BUSVoltage *
                                            (1.054f - dcdcstage.CapVoltage * 0.025f);
        } else if (dcdcmode.lastMode == OUTPUT && dcdcmode.mode == CHARGE) {
            currentloop.chargeLoop.u[1] = currentloop.targetCurrent - currentloop.current;
            currentloop.chargeLoop.u[0] = currentloop.chargeLoop.u[1];
            currentloop.chargeLoop.y[1] = currentloop.output;
            currentloop.chargeLoop.y[0] = currentloop.chargeLoop.y[1];
        } else if (dcdcmode.lastMode == CHARGE && dcdcmode.mode == OUTPUT) {
            currentloop.OutPutPi.integral = 1 - currentloop.output;
        }
    }
    //���type2��·
    if (dcdcmode.mode == CHARGE) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

        currentloop.chargeLoop.u[0] = currentloop.chargeLoop.u[1];
        currentloop.chargeLoop.u[1] = currentloop.targetCurrent - currentloop.current;
        currentloop.chargeLoop.y[0] = currentloop.chargeLoop.y[1];
        currentloop.chargeLoop.y[1] = currentloop.output;
        currentloop.output =
            0.01613f * currentloop.chargeLoop.u[1] - 0.01574f * currentloop.chargeLoop.u[0] +
            1.394f * currentloop.chargeLoop.y[1] - 0.3937f * currentloop.chargeLoop.y[0];

        if (currentloop.output > 1) currentloop.output = 1;
        if (currentloop.output < 0) currentloop.output = 0;

        hhrtim1.Instance->sTimerxRegs[1].CMP1xR = currentloop.output * 20000;
        hhrtim1.Instance->sTimerxRegs[1].CMP2xR = 10880 + currentloop.output * 10880;
        dcdcmode.lastMode = CHARGE;
    }
    //�ŵ�pi��·
    if (dcdcmode.mode == OUTPUT) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

        currentloop.error = (currentloop.targetCurrent + currentloop.current);
        currentloop.OutPutPi.integral += currentloop.error * Ki * 0.000004f;
        if (currentloop.OutPutPi.integral > MaxIntegral)
            currentloop.OutPutPi.integral = MaxIntegral;
        if (currentloop.OutPutPi.integral < 0) currentloop.OutPutPi.integral = 0;
        currentloop.output = currentloop.OutPutPi.integral + currentloop.error * Kp;

        if (currentloop.output > 1) currentloop.output = 1;
        if (currentloop.output < 0) currentloop.output = 0;
        currentloop.output = 1 - currentloop.output;
        hhrtim1.Instance->sTimerxRegs[1].CMP1xR = currentloop.output * 20000;
        hhrtim1.Instance->sTimerxRegs[1].CMP2xR = 10880 + currentloop.output * 10880;
        dcdcmode.lastMode = OUTPUT;
    }
}

uint8_t TxData[8] = {0};

void CAN_Transmit()
{
    int16_t MAXpower = (uint8_t)(dcdcstage.MAXpower);
    int16_t CapVoltage = (int16_t)(dcdcstage.CapVoltage * 100);
    int16_t L_Current = (int16_t)(currentloop.current * 100);
    int16_t Bus_Power = (int16_t)(dcdcstage.BUSCurrent * 100);
    //״̬����
    TxData[0] = (uint8_t)dcdcmode.mode;  //�Ƿ���
    TxData[1] = (uint8_t)dcdcmode.autoMode.stage;
    //�������ѹ
    TxData[2] = (uint8_t)(CapVoltage);
    TxData[3] = (uint8_t)(CapVoltage >> 8);
    //��е���
    TxData[4] = (uint8_t)(L_Current);
    TxData[5] = (uint8_t)(L_Current >> 8);
    //���߹���
    TxData[6] = (uint8_t)(Bus_Power);
    TxData[7] = (uint8_t)(Bus_Power >> 8);

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader1, TxData);
}
//���ݳ�ѹ ��ѹ״̬����
float target;
float target_Bus_Current;
void DCDC_CALLBACK()
{
    //adc����
    dcdcstage.BUSCurrent = (adc[1] - 2042) * 0.007324f;
    dcdcstage.CapVoltage = adc[2] * 0.008059f;
    dcdcstage.BUSVoltage = adc[3] * 0.008059f;
    dcdcstage.MAXpower = dcdcstage.CapVoltage * MAX_OUTPUT_CURRENT * 0.9f;
    //������������
    dcdcstage.BUSPower = dcdcstage.BUSCurrent * dcdcstage.BUSVoltage + 2.95f;
    dcdcmode.autoMode.powerBuffer += (dcdcmode.autoMode.powerLimit - dcdcstage.BUSPower) * 0.0002f;
    LIMIT(dcdcmode.autoMode.powerBuffer, 0, 60);
    //ĸ�ߵ�ѹ���ߵ���ֱ�ӹض��Ա�������
    if (dcdcstage.BUSVoltage > 28.0f) {
        dcdcmode.mode = TURNOFF;
        dcdcmode.autoMode.stage = 0;
    }
    //���Զ�ģʽ�µĵ�ѹ����
    if (dcdcstage.CapVoltage < 4.0f && dcdcmode.mode != CHARGE && dcdcmode.autoMode.stage != 1) {
        dcdcmode.mode = TURNOFF;
        dcdcmode.autoMode.stage = 0;
    }
    //���̶ϵ�ʱ�����ݹض� ���˳��Զ�ģʽ
    if (dcdcstage.BUSVoltage < 19.0f || dcdcstage.BUSVoltage > 28.0f) {
        dcdcmode.mode = TURNOFF;
        dcdcmode.autoMode.stage = 0;
    } else {
        dcdcmode.autoMode.stage = Automode_Stage;
    }

    //״̬�������� ����һ�������Է�ֹ����ģʽ��������
    if (dcdcstage.CapVoltage > CAP_MAX_VOLTAGE) cap_overVoltage_state = 1;
    if (dcdcstage.CapVoltage < CAP_MAX_VOLTAGE - 0.3f) cap_overVoltage_state = 0;
    if (dcdcstage.CapVoltage < 4.0f) cap_lowVoltage_state = 1;
    if (dcdcstage.CapVoltage > 4.5f) cap_lowVoltage_state = 0;
    //��������PID����
    PID_SingleCalc(&dcdcmode.autoMode.powerPID, 32.0f, dcdcmode.autoMode.powerBuffer);
    target_Bus_Current =
        (dcdcmode.autoMode.powerLimit - dcdcmode.autoMode.powerPID.output) / dcdcstage.BUSVoltage;
    if (target_Bus_Current <= 0.1) target_Bus_Current = 0.1;
    PID_SingleCalc(
        &dcdcmode.autoMode.Bus_currentPID, target_Bus_Current,
        dcdcstage.BUSCurrent);  //dcdcmode.autoMode.powerPID.output*dcdcstage.BUSVoltage
                                //������PID���� ��ֹ�ٽ��ٽ��ѹʱ�����������µ��ݶ˵�ѹͻ��
    PID_SingleCalc(&dcdcmode.autoMode.ChargePID, CAP_MAX_VOLTAGE, dcdcstage.CapVoltage);

    //�Զ�ģʽ
    if (dcdcmode.autoMode.stage == 1 && dcdcstage.BUSVoltage <= 28.0f) {
        if (dcdcmode.autoMode.lastStage != dcdcmode.autoMode.stage)  //�ж�auto�Ƿ��г�
        {
            //			dcdcmode.autoMode.powerBuffer = 30.0f;
            PID_Clear(&dcdcmode.autoMode.powerPID);
        }
        //������ݴ��ڳ�ѹ���ѹ״̬
        if (cap_overVoltage_state == 1 || cap_lowVoltage_state == 1) {
            if (cap_overVoltage_state == 1 && dcdcmode.autoMode.Bus_currentPID.output >= 0) {
                dcdcmode.mode = TURNOFF;
            } else if (cap_overVoltage_state == 1 && dcdcmode.autoMode.Bus_currentPID.output < 0) {
                currentloop.targetCurrent = -dcdcmode.autoMode.Bus_currentPID.output;
                dcdcmode.mode = OUTPUT;
            }

            if (cap_lowVoltage_state == 1 && dcdcmode.autoMode.Bus_currentPID.output <= 0) {
                dcdcmode.mode = TURNOFF;
            } else if (cap_lowVoltage_state == 1 && dcdcmode.autoMode.Bus_currentPID.output > 0) {
                currentloop.targetCurrent = dcdcmode.autoMode.Bus_currentPID.output;
                dcdcmode.mode = CHARGE;
            }
        } else  //�ǳ�ѹ����auto�߼�
        {
            if (dcdcmode.autoMode.Bus_currentPID.output > 0)  //charge
            {
                if (dcdcstage.CapVoltage >= 17.5f &&
                    dcdcmode.autoMode.ChargePID.output <= dcdcmode.autoMode.Bus_currentPID.output) {
                    currentloop.targetCurrent = dcdcmode.autoMode.ChargePID.output;
                    dcdcmode.mode = CHARGE;
                } else {
                    currentloop.targetCurrent = dcdcmode.autoMode.Bus_currentPID.output;
                    dcdcmode.mode = CHARGE;
                }
            }

            if (dcdcmode.autoMode.Bus_currentPID.output < 0)  //output
            {
                currentloop.targetCurrent = -dcdcmode.autoMode.Bus_currentPID.output;
                dcdcmode.mode = OUTPUT;
            }
            dcdcmode.autoMode.lastStage = 1;
        }
    }

    else {
        if (cap_overVoltage_state == 1) dcdcmode.mode = TURNOFF;
        //		if(cap_lowVoltage_state == 1)dcdcmode.mode = TURNOFF;
    }
    //
    if (dcdcmode.autoMode.stage == 0) {
        if (dcdcmode.autoMode.lastStage == 1) {
            dcdcmode.mode = TURNOFF;
        }
        dcdcmode.autoMode.lastStage = 0;
    }

    //��·�ر�
    if (dcdcmode.mode == TURNOFF) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
        hhrtim1.Instance->sTimerxRegs[1].CMP1xR = 0;
        hhrtim1.Instance->sTimerxRegs[1].CMP2xR = 96;
        dcdcmode.lastMode = TURNOFF;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)  //
{
    if (htim == &htim1) DCDC_CALLBACK();
    if (htim == &htim6) {
        CAN_Transmit();
    }
}

FDCAN_RxHeaderTypeDef RxHeader;  // ����������յ�������֡ͷ����Ϣ
uint8_t RxData[8] = {0};         // ��������������ݶ�����

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef * hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == &hfdcan2)  // �ж���hfdcan1���ж�
    {
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)  // �ж���FIFO0_NEW_MESSAGE�ص�
        {
            HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData);
            if (RxHeader.Identifier == 0x1AA) {
                // �ӽ��ն����ж�ȡ����֡
                Power_Buffer[0] = (int16_t)(RxData[0] | (RxData[1] << 8));
                //dcdcmode.autoMode.powerBuffer = Power_Buffer[0];
                dcdcmode.autoMode.powerLimit = (int16_t)(RxData[2] | (RxData[3] << 8));
                Automode_Stage = RxData[4];
                //target = RxData[5] / 100.f;
            }
        }
    }
}
