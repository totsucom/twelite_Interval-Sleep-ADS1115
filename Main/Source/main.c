/*
 * 16bitADコンバーターADS1115をI2Cで接続して接続されたPt100を用いて温度測定
 * 省電力化のためPt100抵抗部への電源供給はトランジスタをスイッチとして用いている
 * 送信毎にスリープする
 *
 * 別途回路図を参照のこと
 */
#include <string.h>         // C 標準ライブラリ用
#include <AppHardwareApi.h>
#include "utils.h"
#include "ToCoNet.h"
#include "serial.h"         // シリアル用
#include "sprintf.h"        // SPRINTF 用
#include "ToCoNet_mod_prototype.h" // ToCoNet モジュール定義(無線で使う)
#include "SMBus.h"

#define SLEEP_INTERVAL 60000        // スリープ時間[ms]=送信間隔
#define RAM_OFF TRUE                // スリープでRAMをOFFするか (1μAほど節約)

#define PWR_PIN   9                 // デジタル出力4でRTD部(抵抗部分)への電源供給スイッチにしている

#define UART_BAUD 115200 	        // シリアルのボーレート
//#define _DEBUG

static tsFILE sSerStream;           // シリアル用ストリーム
static tsSerialPortSetup sSerPort;  // シリアルポートデスクリプタ

// ToCoNet 用パラメータ
#define APP_ID   0x67721122
#define CHANNEL  15

// ADS1115定数
#define ADS1115_ADDRESS        (0x48)      // ADDRﾋﾟﾝ=>GND
#define ADS1115_CONVERSION_REG (0x00)
#define ADS1115_CONFIG_REG     (0x01)
//      入力方法
#define ADS1115_MUX_AIN1_AIN0 (0b000<<4)   // 差分入力
#define ADS1115_MUX_AIN3_AIN0 (0b001<<4)
#define ADS1115_MUX_AIN3_AIN1 (0b010<<4)
#define ADS1115_MUX_AIN3_AIN2 (0b011<<4)
#define ADS1115_MUX_AIN0_GND  (0b100<<4)   // シングル入力
#define ADS1115_MUX_AIN1_GND  (0b101<<4)
#define ADS1115_MUX_AIN2_GND  (0b110<<4)
#define ADS1115_MUX_AIN3_GND  (0b111<<4)
//      電圧スケール(プラスマイナス)
#define ADS1115_FS_6144V      (0b000<<1)    // ±6.144V
#define ADS1115_FS_4096V      (0b001<<1)
#define ADS1115_FS_2048V      (0b010<<1)
#define ADS1115_FS_1024V      (0b011<<1)
#define ADS1115_FS_0512V      (0b100<<1)
#define ADS1115_FS_0256V      (0b101<<1)
//      電圧スケール(サンプリングレート)
#define ADS1115_SFS_8         (0b000<<5)
#define ADS1115_SFS_16        (0b001<<5)
#define ADS1115_SFS_32        (0b010<<5)
#define ADS1115_SFS_64        (0b011<<5)
#define ADS1115_SFS_128       (0b100<<5)    // default
#define ADS1115_SFS_250       (0b101<<5)
#define ADS1115_SFS_475       (0b110<<5)
#define ADS1115_SFS_860       (0b111<<5)


// 状態定義
typedef enum
{
    //組み込み済みの状態
    //E_STATE_IDLE        //初期状態
    //E_STATE_RUNNING     //実行中
    //E_STATE_FINISHED    //完了

    //ユーザー定義の状態
    E_STATE_APP_BASE = ToCoNet_STATE_APP_BASE, //enum開始番号の決定のために必要
    E_STATE_APP_WAIT_CONVERSION_1,  //ADC変換待ち1
    E_STATE_APP_WAIT_CONVERSION_2,  //ADC変換待ち2
    E_STATE_APP_WAIT_CONVERSION_3,  //ADC変換待ち3
    E_STATE_APP_WAIT_TX,            //送信完了待ち
    E_STATE_APP_SLEEP               //スリープ移行
} teStateApp;

static uint32 u32Seq;               // 送信パケットのシーケンス番号


// デバッグメッセージ出力用
#ifdef _DEBUG
#define debug(...) vfPrintf(&sSerStream, LB __VA_ARGS__)
#else
#define debug(...)
#endif

// デバッグ出力用に UART を初期化
static void vSerialInit() {
    static uint8 au8SerialTxBuffer[96];
    static uint8 au8SerialRxBuffer[32];

    sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
    sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
    sSerPort.u32BaudRate = UART_BAUD;
    sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
    sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
    sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
    sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
    sSerPort.u8SerialPort = E_AHI_UART_0;
    sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
    SERIAL_vInit(&sSerPort);

    sSerStream.bPutChar = SERIAL_bTxChar;
    sSerStream.u8Device = E_AHI_UART_0;
}

// ハードウェア初期化
static void vInitHardware()
{
#ifdef _DEBUG
	// デバッグ出力用
	vSerialInit();
	ToCoNet_vDebugInit(&sSerStream);
	ToCoNet_vDebugLevel(0);
#endif

    // IOポート初期化
    vPortAsOutput(PWR_PIN);
    vPortSetLo(PWR_PIN);

    // I2C初期化
    vSMBusInit(I2C_CLOCK_100KHZ);
}

// ブロードキャスト送信を実行
static bool_t sendBroadcast(const char *p)
{
    tsTxDataApp tsTx;
    memset(&tsTx, 0, sizeof(tsTxDataApp));

    tsTx.u32SrcAddr = ToCoNet_u32GetSerial();//チップのS/N
    tsTx.u32DstAddr = TOCONET_MAC_ADDR_BROADCAST;

    tsTx.bAckReq = FALSE;
    tsTx.u8Retry = 0x02; // 送信失敗時は 2回再送
    tsTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA;
    tsTx.u16RetryDur = 4;   // 再送間隔[ms]
    tsTx.u16DelayMax = 16;  // 送信開始タイミングにブレを作る(最大16ms)

#if RAM_OFF
    //RAMをOFFする場合
    u32Seq = ToCoNet_u32GetRand();  //u32Seqのカウンタは保持されないので乱数を使う(テストでは毎回違う値を指していたのでOK)
    tsTx.u8CbId = u32Seq & 0xFF;
    tsTx.u8Seq = u32Seq & 0xFF;
    //※これでも乱数が重複するとうまくいかないので受信側でも工夫が必要
    //　Test07 の cbToCoNet_vRxEvent() を参照のこと
#else
    //ToCoNet_vSleep()でRAMをONのままスリープする場合のコード
    u32Seq++; //u32Seqのカウンタは保持される
    tsTx.u8CbId = u32Seq & 0xFF;
    tsTx.u8Seq = u32Seq & 0xFF;
#endif

    // 与えられた文字列を送信
    tsTx.u8Len = strlen(p);
    memcpy(tsTx.auData, p, tsTx.u8Len);

    // 送信
    return ToCoNet_bMacTxReq(&tsTx);
}

// AD変換開始指示
// どのピンを使用するか、ADS1115_MUX_xxx で指示する
// 電圧レンジを、ADS1115_FS_xxx で指示する
void ADS1115start(uint8 u8ADS1115_MUX, uint8 u8ADS115_FS)
{
    uint8 reg[2];
    reg[0] = 0x80                   //start conversion
            | u8ADS1115_MUX
            | u8ADS115_FS           //voltage scale
            | 0x01;                 //single-shot
    reg[1] = ADS1115_SFS_128        //sampling speed
            | 0x03;                 //default
    bSMBusWrite(ADS1115_ADDRESS, ADS1115_CONFIG_REG, 2, reg);
}

// AD変換が完了したか？
bool_t ADS1115conversionReady()
{
    //I2C通信でいえることだが、
    //bSMBusRandomRead()を使うとうまくいかない。(自分の理解不足?)
    //bSMBusWrite()でレジスタを指定し、bSMBusSequentialRead()で結果を読むようにする。

    uint8 reg[2];
    bSMBusWrite(ADS1115_ADDRESS, ADS1115_CONFIG_REG, 0, NULL);
    bSMBusSequentialRead(ADS1115_ADDRESS, 2, reg);
    return (reg[0] & 0x80);
}

// AD変換の結果を得る
int16 ADS1115getValue()
{
    int16 val;
    bSMBusWrite(ADS1115_ADDRESS, ADS1115_CONVERSION_REG, 0, NULL);
    bSMBusSequentialRead(ADS1115_ADDRESS, 2, (uint8 *)&val);
    return val;
}

//Pt100抵抗値から温度を計算する
float OhmToTemp(float ohm)
{
    //-30.0 to 159.0℃
    static float tbl[] = {
        88.22,  88.62,	89.01,	89.40,	89.80,	90.19,	90.59,	90.98,	91.37,	91.77,
        92.16,  92.55,	92.95,	93.34,	93.73,	94.12,	94.52,	94.91,	95.30,	95.69,
        96.09,	96.48,	96.87,	97.26,	97.65,	98.04,	98.44,	98.83,	99.22,	99.61,
        100.00,	100.39,	100.78,	101.17,	101.56,	101.95,	102.34,	102.73,	103.12,	103.51,
        103.90,	104.29,	104.68,	105.07,	105.46,	105.85,	106.24,	106.63,	107.02,	107.40,
        107.79,	108.18,	108.57,	108.96,	109.35,	109.73,	110.12,	110.51,	110.90,	111.29,
        111.67,	112.06,	112.45,	112.83,	113.22,	113.61,	114.00,	114.38,	114.77,	115.15,
        115.54,	115.93,	116.31,	116.70,	117.08,	117.47,	117.86,	118.24,	118.63,	119.01,
        119.40,	119.78,	120.17,	120.55,	120.94,	121.32,	121.71,	122.09,	122.47,	122.86,
        123.24,	123.63,	124.01,	124.39,	124.78,	125.16,	125.54,	125.93,	126.31,	126.69,
        127.08,	127.46,	127.84,	128.22,	128.61,	128.99,	129.37,	129.75,	130.13,	130.52,
        130.90,	131.28,	131.66,	132.04,	132.42,	132.80,	133.18,	133.57,	133.95,	134.33,
        134.71,	135.09,	135.47,	135.85,	136.23,	136.61,	136.99,	137.37,	137.75,	138.13,
        138.51,	138.88,	139.26,	139.64,	140.02,	140.40,	140.78,	141.16,	141.54,	141.91,
        142.29,	142.67,	143.05,	143.43,	143.80,	144.18,	144.56,	144.94,	145.31,	145.69,
        146.07,	146.44,	146.82,	147.20,	147.57,	147.95,	148.33,	148.70,	149.08,	149.46,
        149.83,	150.21,	150.58,	150.96,	151.33,	151.71,	152.08,	152.46,	152.83,	153.21,
        153.58,	153.96,	154.33,	154.71,	155.08,	155.46,	155.83,	156.20,	156.58,	156.95,
        157.33,	157.70,	158.07,	158.45,	158.82,	159.19,	159.56,	159.94,	160.31,	160.68
    };

    // 低い温度から順に検索する低能検索w
    int i;
    int n = sizeof(tbl) / sizeof(tbl[0]);
    for(i = 0; i < n; i++) {
        if(tbl[i] > ohm) {
            if(i == 0) {
                return -1000.0f; // 抵抗値が範囲外
            } else {
                // 抵抗値の中間部分は線形補完で計算
                return (ohm - tbl[i-1]) / (tbl[i] - tbl[i-1]) + (i - 31);
            }
        }
    }
    return 1000.0f; //抵抗値が範囲外
}

// ユーザ定義のイベントハンドラ
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg)
{
    static int16 i16AdcVoltage[3];
    static uint32 u32ConvTime_ms[3];

    switch (pEv->eState) {
    // アイドル状態
    case E_STATE_IDLE:
        if (eEvent == E_EVENT_START_UP) { // 起動時
            debug("** ADS1115+RTD Test **");

            // 測定モードへ移行
            ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
            ToCoNet_Event_Process(E_ORDER_KICK, TRUE, vProcessEvCore);
        }
        break;

    // 稼働状態
    case E_STATE_RUNNING:
        if (eEvent == E_ORDER_KICK) {

            // Pt100部分に電源供給
            vPortSetHi(PWR_PIN);

            // I2Cのスピードに比べてトランジスタの立ち上がりの方が高速なので
            // 待ち時間は不要(のはず)

            // 最初のAD変換を指示(基準抵抗の電圧)
            ADS1115start(ADS1115_MUX_AIN3_AIN1, ADS1115_FS_2048V);
            u32ConvTime_ms[0] = u32TickCount_ms;//開始時間を記憶

            // 変換待ち1に状態変更
            ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_CONVERSION_1);
        }
        break;

    // 変換待ち状態 1
    case E_STATE_APP_WAIT_CONVERSION_1:;

        // 変換できたか
        if(ADS1115conversionReady()) {

            u32ConvTime_ms[0] = u32TickCount_ms - u32ConvTime_ms[0];//かかった時間を計算

            // 基準抵抗の電圧値を取得
            i16AdcVoltage[0] = ADS1115getValue();

            // ２つ目のAD変換を指示(Pt100センサの電圧)
            ADS1115start(ADS1115_MUX_AIN1_AIN0, ADS1115_FS_2048V);
            u32ConvTime_ms[1] = u32TickCount_ms;//開始時間を記憶

            // 変換待ち2に状態変更
            ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_CONVERSION_2);
        }
        break;

    // 変換待ち状態 2
    case E_STATE_APP_WAIT_CONVERSION_2:;

        // 変換できたか
        if(ADS1115conversionReady()) {

            u32ConvTime_ms[1] = u32TickCount_ms - u32ConvTime_ms[1];//かかった時間を計算

            // Pt100部分への電源供給を停止
            vPortSetLo(PWR_PIN);

            // Pt100センサの電圧値を取得
            i16AdcVoltage[1] = ADS1115getValue();

            // ３つ目のAD変換を指示(電源電圧)
            ADS1115start(ADS1115_MUX_AIN0_GND, ADS1115_FS_4096V);
            u32ConvTime_ms[2] = u32TickCount_ms;//開始時間を記憶

            // 変換待ち3に状態変更
            ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_CONVERSION_3);
        }
        break;

    // 変換待ち状態 3
    case E_STATE_APP_WAIT_CONVERSION_3:;

        // 変換できたか
        if(ADS1115conversionReady()) {

            u32ConvTime_ms[2] = u32TickCount_ms - u32ConvTime_ms[2];//かかった時間を計算

            // 電源電圧値を取得
            i16AdcVoltage[2] = ADS1115getValue();

            char buf[50], *p = &buf[0];
            sprintf(p, "P%04d ", (int)(4096.0f * (float)i16AdcVoltage[2] / 32767.0f));
            p += strlen(p);

            // Pt100センサ、基準抵抗の電圧値が正しいか？
            if(i16AdcVoltage[0] == 32767 || i16AdcVoltage[0] == -32768 || i16AdcVoltage[0] == 0 ||
               i16AdcVoltage[1] == 32767 || i16AdcVoltage[1] == -32768 || i16AdcVoltage[1] == 0) {

                sprintf(p, "ERR");
            } else {

                float v0 = 2.048f * (float)i16AdcVoltage[0] / 32767.0f; //基準抵抗電圧
                float v1 = 2.048f * (float)i16AdcVoltage[1] / 32767.0f; //Pt100電圧
                float ohm = 100.0f * v1 / v0; //Pt100抵抗値
                float temp = OhmToTemp(ohm);  //Pt100温度

                if(temp < -100.0f || temp > 200.0f) {
                    sprintf(p, "ERR");
                } else {
                    sprintf(p, "S:%04u T:%+05d",
                        (uint)u32ConvTime_ms[0]+(uint)u32ConvTime_ms[1]+(uint)u32ConvTime_ms[2],
                        (int)(temp*100.0f));
                }
            }

            // 無線送信
            sendBroadcast(buf);
#ifdef _DEBUG
            debug("%s", buf);
#endif

            // 送信待ち
            ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_TX);
        }
        break;

    // 送信完了待ち状態
    case E_STATE_APP_WAIT_TX:
        if (eEvent == E_ORDER_KICK) {
            // 変換終了でスリープへ移行
            ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP);
        }
        break;

    // Sleep への移行状態
    case E_STATE_APP_SLEEP:
        // vSleep は必ず E_EVENT_NEW_STATE 内など１回のみ呼び出される場所で呼び出す
        if (eEvent == E_EVENT_NEW_STATE) {

#ifdef _DEBUG
            debug("Sleeping...Time waking up=%ums", u32TickCount_ms);
            WAIT_UART_OUTPUT(E_AHI_UART_0); // UART 出力の完了を待つ
#endif

            // RTD部分への電源供給を停止(念のため駄目押し)
            vPortSetLo(PWR_PIN);

            //スリープ
            //RAMオンの場合1uAほど余計に電力を消費
            ToCoNet_vSleep(
                E_AHI_WAKE_TIMER_0,
                SLEEP_INTERVAL - u32TickCount_ms,   //起動からの経過時間を差し引いてスリープ時間とする
                TRUE,                               //TRUE:インターバル
                RAM_OFF);                           //TRUE:RAMオフ ※u32Seqなどの変数は忘れられる
            while(1);
        }
        break;

    default:
        break;
    }
}

// メインループ無限ループではなく割り込みなどの発生を起点として呼び出されます
void cbToCoNet_vMain(void)
{
}

// パケット送信完了時
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus)
{
    //送信完了。このあとvProcessEvCore()内でE_STATE_APP_WAIT_TXが実行される
    debug(">> SENT %s seq=%u", bStatus ? "OK" : "NG", u32Seq);
    ToCoNet_Event_Process(E_ORDER_KICK, 0, vProcessEvCore);
}

// パケット受信時
void cbToCoNet_vRxEvent(tsRxDataApp *pRx)
{
}

// 電源オンによるスタート
void cbAppColdStart(bool_t bAfterAhiInit)
{
	//static uint8 u8WkState;
	if (!bAfterAhiInit) {
        // 必要モジュール登録手続き
        ToCoNet_REG_MOD_ALL();
	} else {
        // 電源電圧が 2.0V まで下降しても稼働を継続
        vAHI_BrownOutConfigure(0, FALSE, FALSE, FALSE, FALSE);

        // SPRINTF 初期化
        SPRINTF_vInit128();

        // ToCoNet パラメータ
        //memset (&sToCoNet_AppContext, 0x00, sizeof(sToCoNet_AppContext));
        sToCoNet_AppContext.u32AppId = APP_ID;
        sToCoNet_AppContext.u8Channel = CHANNEL;
        sToCoNet_AppContext.bRxOnIdle = FALSE;  //送信のみなので、受信回路は開かない
        //sToCoNet_AppContext.u8TxPower =
        u32Seq = 0;

        // ユーザ定義のイベントハンドラを登録
        ToCoNet_Event_Register_State_Machine(vProcessEvCore);

        // ハードウェア初期化
        vInitHardware();

        // MAC 層開始
        ToCoNet_vMacStart();    //無線機能を使わない場合はこれを呼ばない
	}
}

// スリープからの復帰
void cbAppWarmStart(bool_t bAfterAhiInit)
{
	if (!bAfterAhiInit) {
        // 起床要因のチェック (u32AHI_Init の前に呼ぶ)
        //if(u32AHI_DioWakeStatus() & (1UL << DI1)) {
        //    // この場合、ボタンによる起床であった
        //}
    } else {
        vAHI_BrownOutConfigure(0, FALSE, FALSE, FALSE, FALSE);
        vInitHardware(bAfterAhiInit);
        ToCoNet_vMacStart();
     }
}

// ネットワークイベント発生時
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg)
{
	switch(eEvent) {
	default:
		break;
	}
}


// ハードウェア割り込み発生後（遅延呼び出し）
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
    switch (u32DeviceId) {
    case E_AHI_DEVICE_TICK_TIMER:
    	break;

    default:
    	break;
    }
}

// ハードウェア割り込み発生時
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
	return FALSE;
}
